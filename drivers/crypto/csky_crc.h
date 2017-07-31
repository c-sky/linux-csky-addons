/*
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Vincent Cui <xiaoxia_cui@c-sky.com>
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

#ifndef __CSKY_CRC_H
#define __CSKY_CRC_H

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
	STD_USB,
	STD_IBM,
	STD_MODBUS
} crc_std_e;

struct crc_register {
	u32 data;
	u32 sel;
	u32 init;
};

#define SEL_POLY		0x00000007
#define SEL_POLARITY	0x00000008
#define INIT_DATA		0x0000FFFF

#endif /* __CSKY_CRC_H */