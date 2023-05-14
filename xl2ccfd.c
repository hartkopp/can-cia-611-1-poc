/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * xl2ccfd.c - CAN XL CiA 611-1 XL to CC/FD converter
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

extern int optind, opterr, optopt;

void print_usage(char *prg)
{
	fprintf(stderr, "%s - CAN XL CiA 611-1 XL to CC/FD converter\n\n", prg);
	fprintf(stderr, "Usage: %s [options] <src_if> <dst_if>\n", prg);
	fprintf(stderr, "Options:\n");
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

void set_cc_dlc(struct can_frame *cf, __u8 dlc)
{
	/* dlc &= 0x0F was already done by the caller */

	cf->len = dlc; /* raw DLC 0x0 .. 0xF */

	/* add SocketCAN DLC > 8 representation */
	if (cf->len > CAN_MAX_DLEN) {
		cf->len8_dlc = cf->len;
		cf->len = CAN_MAX_DLEN;
	}
}

int sdt03(struct canxl_frame *xlf, struct canfd_frame *fdf)
{
	struct can_frame *cf = (struct can_frame *)fdf;
	__u32 pci = (__u32)xlf->data[0]; /* read first LE __u32 word */
	unsigned char ubuf[68] = { 0 }; /* pci + data + padding */
	int i;

	fdf->can_id = xlf->af & 0x1FFFFFFFU;

	if (xlf->af & IDE)
		cf->can_id |= CAN_EFF_FLAG;

	if (xlf->af & FDF) {
		fdf->flags = CANFD_FDF;

		if (xlf->af & RTR)
			return 0; /* spec -> invalid */

		if (pci & BRS3)
			fdf->flags |= CANFD_BRS;
			
		if (pci & ESI)
			fdf->flags |= CANFD_ESI;

		/* make pci the dlc value */
		pci &= 0xF;

		/* no data -> we are done as len is initialized to zero */
		if (!pci)
			return CANFD_MTU;

		/* get length from DLC value */
		fdf->len = can_fd_dlc2len(pci);

		/* the DLC has to fit the CAN XL DLC (length + pci len) */
		if (xlf->len != fdf->len + 1)
			return 0; /* spec -> invalid */

		/* copy aligned source data to alignment shift buffer */
		copy_aligned32((__u32 *)ubuf, (__u32 *)xlf->data, xlf->len);

		/* shift the data by one byte to cut the PCI in buf[0] */
		for (i = 0; i < fdf->len; i++)
			ubuf[i] = ubuf[i + 1];

		/* copy aligned destination data from alignment shift buffer */
		copy_aligned32((__u32 *)fdf->data, (__u32 *)ubuf, fdf->len);

		return CANFD_MTU;
	}

	/* Classical CAN */

	/* make pci the dlc value */
	pci &= 0xF;

	if (xlf->af & RTR) {
		if (xlf->len != 1)
			return 0; /* spec -> invalid */

		cf->can_id |= CAN_RTR_FLAG;
		set_cc_dlc(cf, pci);

		return CAN_MTU;
	}

	/* no data -> we are done as len is initialized to zero */
	if (!pci)
		return CAN_MTU;

	set_cc_dlc(cf, pci);

	if (xlf->len > CAN_MAX_DLEN + 1)
		return 0;

	/* the DLC has to fit the CAN XL DLC (length + pci len) */
	if (xlf->len != cf->len + 1)
		return 0;

	/* copy aligned source data to alignment shift buffer */
	copy_aligned32((__u32 *)ubuf, (__u32 *)xlf->data, xlf->len);

	/* shift the data by one byte to cut the PCI in buf[0] */
	for (i = 0; i < cf->len; i++)
		ubuf[i] = ubuf[i + 1];

	/* copy aligned destination data from alignment shift buffer */
	copy_aligned32((__u32 *)cf->data, (__u32 *)ubuf, cf->len);

	return CAN_MTU;
}

int sdt06(struct canxl_frame *xlf, struct can_frame *cf)
{
	cf->can_id = xlf->af & 0x1FFFFFFFU;

	if (xlf->af & IDE)
		cf->can_id |= CAN_EFF_FLAG;

	if (xlf->af & RTR) {
		cf->can_id |= CAN_RTR_FLAG;

		set_cc_dlc(cf, xlf->data[0] & 0x0F);

		return CAN_MTU;
	}

	/* no data -> we are done as len is initialized to zero */
	if (xlf->af & ZDLC)
		return CAN_MTU;

	/* check if we have a proper CAN length value */
	if (xlf->len > CAN_MAX_DLEN + 1)
		return 0; /* spec -> invalid */

	if (xlf->len <= CAN_MAX_DLEN) {
		/* standard CC frame with 0 .. 8 bytes */
		cf->len = xlf->len;
	} else {
		/* get raw DLC > 8 value from behind the 8 bytes data */
		if (xlf->data[8] & 0x0F <= CAN_MAX_DLEN)
			return 0; /* spec -> invalid */

		/* valid DLC values 0x9 .. 0xF */
		set_cc_dlc(cf, xlf->data[8] & 0x0F);
	}

	/* copy the aligned data */
	copy_aligned32((__u32 *)cf->data, (__u32 *)xlf->data, cf->len);

	return CAN_MTU;
}

int sdt07(struct canxl_frame *xlf, struct canfd_frame *fdf)
{
	fdf->can_id = xlf->af & 0x1FFFFFFFU;
	fdf->flags = CANFD_FDF;

	if (xlf->af & IDE)
		fdf->can_id |= CAN_EFF_FLAG;

	if (xlf->af & BRS7)
		fdf->flags |= CANFD_BRS;

	/* no data -> we are done as len is initialized to zero */
	if (xlf->af & ZDLC)
		return CANFD_MTU;

	/* paranoia check if we have a proper CAN FD length value */
	if (xlf->len != can_fd_dlc2len(can_fd_len2dlc(xlf->len)))
		return 0;

	fdf->len = xlf->len; /* 0x1 .. 0x40 */

	/* copy the aligned data */
	copy_aligned32((__u32 *)fdf->data, (__u32 *)xlf->data, xlf->len);

	return CANFD_MTU;
}

int main(int argc, char **argv)
{
	int opt;
	int mtu = 0;
	int verbose = 0;

	int src, dst;
	struct sockaddr_can addr;
	struct canxl_frame xlf;
	struct canfd_frame fdf;

	int nbytes, ret;
	int sockopt = 1;
	struct timeval tv;

	while ((opt = getopt(argc, argv, "vh?")) != -1) {
		switch (opt) {

		case 'v':
			verbose = 1;
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

	/* enable CAN XL frames */
	ret = setsockopt(src, SOL_CAN_RAW, CAN_RAW_XL_FRAMES,
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

	/* enable CAN FD frames */
	ret = setsockopt(dst, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
			 &sockopt, sizeof(sockopt));
	if (ret < 0) {
		perror("dst sockopt CAN_RAW_FD_FRAMES");
		exit(1);
	}

	if (bind(dst, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 1;
	}

	/* main loop */
	while (1) {

		/* read source CAN XL frame */
		nbytes = read(src, &xlf, sizeof(struct canxl_frame));
		if (nbytes < 0) {
			perror("read");
			return 1;
		}

		if (nbytes < CANXL_HDR_SIZE + CANXL_MIN_DLEN) {
			fprintf(stderr, "read: no CAN frame\n");
			return 1;
		}

		if (!(xlf.flags & CANXL_XLF)) {
			fprintf(stderr, "read: no CAN XL frame flag\n");
			return 1;
		}

		if (nbytes != CANXL_HDR_SIZE + xlf.len) {
			printf("nbytes = %d\n", nbytes);
			fprintf(stderr, "read: no CAN XL frame len\n");
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

			printxlframe(&xlf);
		}

		memset(&fdf, 0, sizeof(fdf));

		switch (xlf.sdt) {
		case 0x03:
			mtu = sdt03(&xlf, &fdf);
			break;

		case 0x06:
			mtu = sdt06(&xlf, (struct can_frame *)&fdf);
			break;

		case 0x07:
			mtu = sdt07(&xlf, &fdf);
			break;

		default:
			printf("unhandled SDT 0x%02X\n", xlf.sdt);
		}

		if (!mtu) {
			fprintf(stderr, "conversion error for SDT 0x%02X\n",
				xlf.sdt);
			return 1;
		}

		if (verbose) {
			if (ioctl(src, SIOCGSTAMP, &tv) < 0) {
				perror("SIOCGSTAMP");
				return 1;
			}

			/* print timestamp and device name */
			printf("(%ld.%06ld) %s ", tv.tv_sec, tv.tv_usec,
			       argv[optind + 1]);

			if (mtu == CAN_MTU)
				printccframe((struct can_frame *)&fdf);
			else
				printfdframe(&fdf);

		}

		/* send CAN CC/FD frame */
		nbytes = write(dst, &fdf, mtu);
		if (nbytes != mtu) {
			printf("nbytes = %d\n", nbytes);
			perror("send CAN CC/FD frame");
			exit(1);
		}

	} /* while (1) */

	close(src);
	close(dst);

	return 0;
}
