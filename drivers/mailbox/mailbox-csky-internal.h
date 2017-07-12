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

#ifndef __MAILBOX_CSKY_INTERNAL_H
#define __MAILBOX_CSKY_INTERNAL_H

#include "mailbox-csky.h"

enum mbox_csky_mssg_type {
	CSKY_MBOX_MSSG_DATA = 'd',	/* Data to receiver */
	CSKY_MBOX_MSSG_ACK  = 'a',	/* ACK to sender */
};

#define MBOX_CSKY_MSSG_HEAD_LENGTH 4
/**
 * struct mbox_csky_message - Description of a message that send to mailbox
 * @mssg_type:	The message type that transfer, refer to mbox_csky_mssg_type
 * @length:	Then data length, must <= CSKY_MBOX_MAX_DATA_LENGTH
 * @reserved0:	Undefined
 * @reserved1:	Undefined
 * @data:	The transfer data. Ignore if mssg_type is CSKY_MBOX_MSSG_ACK
 */
struct mbox_csky_message {
	u8 mssg_type;
	u8 length;
	u8 reserved0;
	u8 reserved1;
	u8 data[CSKY_MBOX_MAX_DATA_LENGTH];
};

#endif /* __MAILBOX_CSKY_INTERNAL_H */

