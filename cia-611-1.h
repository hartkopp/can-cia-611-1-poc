/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * cia-611-1.h - CAN CiA 611-1 definitions
 *
 */

#ifndef CIA_611_1_H
#define CIA_611_1_H

#include <linux/types.h>

/* all SDTs */
#define RTR  0x40000000U /* AF: Classical CAN RTR */
#define IDE  0x20000000U /* AF: Identifier Extension 11 -> 29 bit */

/* SDT 0x06 / 0x07 */
#define ZDLC 0x80000000U /* AF: Zero DLC */
#define BRS7 0x40000000U /* AF: Zero DLC */

/* SDT 0x03 */
#define FDF  0x80000000U /* AF: FD Frame for SDT 0x03 */
#define ESI  0x20 /* PCI: Error State Indicator */
#define BRS3  0x10 /* PCI: Bit Rate Setting */

#endif /* CIA_611_1_H */
