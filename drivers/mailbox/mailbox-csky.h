/*
 * mailbox driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Charles Lu <chongzhi_lu@c-sky.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MAILBOX_CSKY_H
#define __MAILBOX_CSKY_H

#define CSKY_MBOX_DEV_ID0		0
#define CSKY_MBOX_DEV_ID1		1

#define CSKY_MBOX_CHAN_0		0
#define CSKY_MBOX_DIRECTION_TX		0
#define CSKY_MBOX_DIRECTION_RX		1

#define CSKY_MBOX_MAX_CHAN		1
#define CSKY_MBOX_MAX_MESSAGE_LENGTH	64	/* u32 x 16 */
#define CSKY_MBOX_MAX_DATA_LENGTH	(CSKY_MBOX_MAX_MESSAGE_LENGTH - 4)

/* Interrupt Generate Register (R/W)
 * Write 1 into it, enable interrupt to client immediately.
 * Write 0 into it, disable interrupt to client immediately.
 */
#define CSKY_MBOX_INTGR		0x00

/* Interrupt Clear Register (W1C)
 * Write 1 into it, clear the
 */
#define CSKY_MBOX_INTCR		0x04

/* Interrupt Mask Register (R/W)
 * 1 means mask interrupt bit
 * 0 means unmask interrupt bit
 */
#define CSKY_MBOX_INTMR		0x08

/* Interrupt Register Status Register (RO)
 * Read 1 means has interrupt
 * Read 0 means no interrupt
 */
#define CSKY_MBOX_INTRSR	0x0C

/* Interrupt Masked Status Register (RO)
 * Read 1 means has interrupt on masked bit
 * Read 0 means no interrupt on masked bit
 */
#define CSKY_MBOX_INTMSR	0x10

/* Interrupt Enable (R/W)
 * 1 means interrupt enabled
 * 0 means interrupt disabled
 */
#define CSKY_MBOX_INTENB	0x14

#endif /* __MAILBOX_CSKY_H */

