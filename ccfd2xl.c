/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ccfd2xl.c -  CAN XL CiA 611-1 CC/FD to XL converter
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/sockios.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "cia-611-1.h"
#include "printframe.h"
#include "dlc_convert.h"

#define DEFAULT_PRIO_ID 0x222

extern int optind, opterr, optopt;

void print_usage(char *prg)
{
	fprintf(stderr, "%s - CAN XL CiA 611-1 CC/FD to XL converter\n\n", prg);
	fprintf(stderr, "Usage: %s [options] <src_if> <dst_if>\n", prg);
	fprintf(stderr, "Options:\n");
	fprintf(stderr, "         -3               (use SDT 0x03)\n");
	fprintf(stderr, "         -v               (verbose)\n");
}

int copy_aligned32(__u32 *dest, __u32 *src, unsigned int len)
{
	int len32;
	int i;

	if (!len)
		return 0;

	len32 = len>>2; /* byte -> uint32 : divide by 4 */
	if (len & 3) /* round up if needed */
		len32++;

	for (i = 0; i < len32; i++)
		dest[i] = src[i];

	return len32<<2; /* return copied bytes */
}

int get_cc_dlc(struct can_frame *cf)
{
	if (cf->len == CAN_MAX_DLEN &&
	    cf->len8_dlc > CAN_MAX_DLEN &&
	    cf->len8_dlc <= CAN_MAX_RAW_DLC)
		return cf->len8_dlc; /* 0x9 .. 0xF */

	return cf->len; /* 0x0 .. 0x8 */
}

int fd2xl_sdt03(struct canfd_frame *fdf, struct canxl_frame *xlf)
{
	unsigned char pci = 0;
	unsigned char ubuf[68] = { 0 }; /* pci + data + padding */
	int i;

	xlf->af = fdf->can_id & 0x1FFFFFFFU;
	xlf->sdt = 0x03; /* SDT 0x03 CC/FD tunneling */

	/* handle IDE flag */
	if (fdf->can_id & CAN_EFF_FLAG)
		xlf->af |= IDE;

	xlf->af |= FDF;

	/* paranoia check if we have a proper CAN FD length value */
	if (fdf->len != can_fd_dlc2len(can_fd_len2dlc(fdf->len)))
		return 1;

	/* copy data to unaligned buffer with 1 byte pci offset */
	if (fdf->len) {
		/* copy aligned source data to alignment shift buffer */
		copy_aligned32((__u32 *)&ubuf[4], (__u32 *)fdf->data, fdf->len);

		/* shift the data to make space for the PCI in buf[0] */
		for (i = 0; i < fdf->len; i++)
			ubuf[i + 1] = ubuf[i + 4];
	}

	/* convert length value to CAN FD DLC */
	pci = can_fd_len2dlc(fdf->len);

	/* handle BRS flag */
	if (fdf->flags & CANFD_BRS)
		pci |= BRS3;

	/* handle ESI flag */
	if (fdf->flags & CANFD_ESI)
		pci |= ESI;

	ubuf[0] = pci;

	xlf->len = fdf->len + 1; /* data length + pci length */

	/* copy the aligned data */
	copy_aligned32((__u32 *)xlf->data, (__u32 *)ubuf, xlf->len);

	return 0; /* no error */
}

int cc2xl_sdt03(struct can_frame *cf, struct canxl_frame *xlf)
{
	unsigned char pci = 0;
	unsigned char ubuf[68] = { 0 }; /* pci + data + padding */
	int i;

	xlf->af = cf->can_id & 0x1FFFFFFFU;
	xlf->sdt = 0x03; /* SDT 0x03 CC/FD tunneling */

	/* handle IDE flag */
	if (cf->can_id & CAN_EFF_FLAG)
		xlf->af |= IDE;

	pci = get_cc_dlc(cf); /* get raw DLC value */
	ubuf[0] = pci; /* no CAN FD bits, so the PCI is done */

	/* handle RTR frames */
	if (cf->can_id & CAN_RTR_FLAG) {
		xlf->af |= RTR;
		xlf->len = 1; /* XL DLC = 0 */

		/* copy only the aligned pci */
		copy_aligned32((__u32 *)xlf->data, (__u32 *)ubuf, xlf->len);

		return 0; /* no error */
	}

	/* copy data to unaligned buffer with 1 byte pci offset */
	if (cf->len) {
		/* copy aligned source data to alignment shift buffer */
		copy_aligned32((__u32 *)&ubuf[4], (__u32 *)cf->data, cf->len);

		/* shift the data to make space for the PCI in buf[0] */
		for (i = 0; i < cf->len; i++)
			ubuf[i + 1] = ubuf[i + 4];
	}

	xlf->len = cf->len + 1;

	ubuf[0] = pci;

	/* copy the aligned data */
	copy_aligned32((__u32 *)xlf->data, (__u32 *)ubuf, xlf->len);

	return 0; /* no error */
}

int cc2xl_sdt06(struct can_frame *cf, struct canxl_frame *xlf)
{
	__u32 cc_dlc = get_cc_dlc(cf); /* get raw DLC value */

	xlf->sdt = 0x06; /* SDT 0x06 CC tunneling */
	xlf->af = cf->can_id & 0x1FFFFFFFU;

	/* handle IDE flag */
	if (cf->can_id & CAN_EFF_FLAG)
		xlf->af |= IDE;

	/* handle RTR frames */
	if (cf->can_id & CAN_RTR_FLAG) {
		xlf->af |= RTR;
		xlf->len = 1; /* XL DLC = 0 */

		/* always copy the DLC into the single data byte */
		copy_aligned32((__u32 *)&xlf->data[0], &cc_dlc, 4);

		return 0; /* no error */
	}

	/* handle zero data length */
	if (cf->len == 0) {
		xlf->af |= ZDLC;
		xlf->len = 1; /* XL DLC = 0 */

		return 0; /* no error */
	}

	/* copy the aligned data */
	copy_aligned32((__u32 *)xlf->data, (__u32 *)cf->data, cf->len);
	xlf->len = cf->len; /* 0x1 .. 0x8 */

	/* handle optional DLC > 8 values */
	if (cc_dlc > CAN_MAX_DLEN) {
		/* store behind the 8 byte of data */
		copy_aligned32((__u32 *)&xlf->data[8], &cc_dlc, 4);
		xlf->len++;
	}

	return 0; /* no error */
}

int fd2xl_sdt07(struct canfd_frame *fdf, struct canxl_frame *xlf)
{
	xlf->sdt = 0x07; /* SDT 0x07 FD tunneling */
	xlf->af = fdf->can_id & 0x1FFFFFFFU;

	/* handle IDE flag */
	if (fdf->can_id & CAN_EFF_FLAG)
		xlf->af |= IDE;

	/* handle BRS flag */
	if (fdf->flags & CANFD_BRS)
		xlf->af |= BRS7;

	/* handle zero data length */
	if (fdf->len == 0) {
		xlf->af |= ZDLC;
		xlf->len = 1; /* XL DLC = 0 */

		return 0; /* no error */
	}

	/* copy the aligned data */
	copy_aligned32((__u32 *)xlf->data, (__u32 *)fdf->data, fdf->len);
	xlf->len = fdf->len; /* 0x1 .. 0x40 */

	return 0; /* no error */
}

int main(int argc, char **argv)
{
	int opt;
	canid_t prio_id = DEFAULT_PRIO_ID;
	int use_sdt03 = 0;
	int verbose = 0;

	int src, dst;
	struct sockaddr_can addr;
	struct canxl_frame xlf;
	struct canfd_frame fdf;

	int nbytes, ret;
	int sockopt = 1;
	struct timeval tv;

	while ((opt = getopt(argc, argv, "3vh?")) != -1) {
		switch (opt) {

		case 'v':
			verbose = 1;
			break;

		case '3':
			use_sdt03 = 1;
			break;

		case '?':
		case 'h':
		default:
			print_usage(basename(argv[0]));
			return 1;
			break;
		}
	}

	/* src_if and dst_if are two mandatory parameters */
	if (argc - optind != 2) {
		print_usage(basename(argv[0]));
		exit(0);
	}

	/* src_if */
	if (strlen(argv[optind]) >= IFNAMSIZ) {
		printf("Name of src CAN device '%s' is too long!\n\n",
		       argv[optind]);
		return 1;
	}

	/* dst_if */
	if (strlen(argv[optind + 1]) >= IFNAMSIZ) {
		printf("Name of dst CAN device '%s' is too long!\n\n",
		       argv[optind]);
		return 1;
	}

	/* open src socket */
	src = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (src < 0) {
		perror("src socket");
		return 1;
	}
	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(argv[optind]);

	/* enable CAN FD frames */
	ret = setsockopt(src, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
			 &sockopt, sizeof(sockopt));
	if (ret < 0) {
		perror("src sockopt CAN_RAW_XL_FRAMES");
		exit(1);
	}

	if (bind(src, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* open dst socket */
	dst = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dst < 0) {
		perror("dst socket");
		return 1;
	}
	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(argv[optind + 1]);

	/* enable CAN XL frames */
	ret = setsockopt(dst, SOL_CAN_RAW, CAN_RAW_XL_FRAMES,
			 &sockopt, sizeof(sockopt));
	if (ret < 0) {
		perror("dst sockopt CAN_RAW_XL_FRAMES");
		exit(1);
	}

	if (bind(dst, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* main loop */
	while (1) {

		/* read source CAN CC/FD frame */
		nbytes = read(src, &fdf, sizeof(struct canfd_frame));
		if (nbytes < 0) {
			perror("read");
			return 1;
		}

		if (nbytes != CAN_MTU && nbytes != CANFD_MTU) {
			fprintf(stderr, "read: no CC/FD CAN frame\n");
			return 1;
		}

		if (verbose) {
			if (ioctl(src, SIOCGSTAMP, &tv) < 0) {
				perror("SIOCGSTAMP");
				return 1;
			}

			/* print timestamp and device name */
			printf("(%ld.%06ld) %s ", tv.tv_sec, tv.tv_usec,
			       argv[optind]);

			if (nbytes == CAN_MTU)
				printccframe((struct can_frame *)&fdf);
			else
				printfdframe(&fdf);

		}

		memset(&xlf, 0, sizeof(xlf));
		xlf.prio = prio_id;
		xlf.flags = CANXL_XLF;

		if (use_sdt03) {
			/* split into two functions to compare the effort */
			if (nbytes == CAN_MTU)
				cc2xl_sdt03((struct can_frame *)&fdf, &xlf);
			else
				fd2xl_sdt03(&fdf, &xlf);
		} else {
			if (nbytes == CAN_MTU)
				cc2xl_sdt06((struct can_frame *)&fdf, &xlf);
			else
				fd2xl_sdt07(&fdf, &xlf);
		}

		if (verbose) {
			if (ioctl(src, SIOCGSTAMP, &tv) < 0) {
				perror("SIOCGSTAMP");
				return 1;
			}

			/* print timestamp and device name */
			printf("(%ld.%06ld) %s ", tv.tv_sec, tv.tv_usec,
			       argv[optind + 1]);

			printxlframe(&xlf);
		}

		/* send CAN XL frame */
		nbytes = write(dst, &xlf, CANXL_HDR_SIZE + xlf.len);
		if (nbytes != CANXL_HDR_SIZE + xlf.len) {
			printf("nbytes = %d\n", nbytes);
			perror("forward src canxl_frame");
			exit(1);
		}

	} /* while (1) */

	close(src);
	close(dst);

	return 0;
}
