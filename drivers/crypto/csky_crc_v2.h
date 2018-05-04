/*
 * Copyright (C) 2018 C-SKY MicroSystems Co.,Ltd.
 * Author: Huoqing Cai <huoqing_cai@c-sky.com>
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

#ifndef __CSKY_CRC_V2_H
#define __CSKY_CRC_V2_H

#include <linux/types.h>

typedef enum {
	MOD_CRC8	= 0,
	MOD_CRC16,
	MOD_CRC32
} crc_mod_e;

typedef enum {
	STD_ROHC	= 0,
	STD_MAXIM,
	STD_X25,
	STD_CCITT,
	STD_CCITT_FALSE,
	STD_XMODEM,
	STD_DNP,
	STD_USB,
	STD_IBM,
	STD_MODBUS,
    STD_PMEQ,
    STD_NONE,
    STD_ITU
} crc_std_e;

struct crc_register {
	u32 config_reg;
	u32 init_val;
	u32 xor_out;
	u32 result;
	u32 reserved[12];
	u32 new_data;
};

struct csky_crypto_crc_list {
	struct list_head dev_list;
	spinlock_t	 lock;
};

#define SEL_POLY		0x00000007
#define SEL_POLARITY	0x00000008
#define INIT_DATA		0x0000FFFF

#endif /* __CSKY_CRC_V2_H */
