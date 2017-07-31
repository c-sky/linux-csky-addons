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
#ifndef __CSKY_AES_H
#define __CSKY_AES_H


#define AES_ENDIAN	0x00000100
#define AES_ENDIAN_LT	0
#define AES_ENDIAN_BG	1

#define AES_IT_DATAINT	0x4
#define AES_IT_KEYINT	0x2
#define AES_IT_BUSY	0x1
#define AES_IT_ALL	0x7

#define AES_OPC_ENC	0x00
#define AES_OPC_DEC	0x01
#define AES_OPC_EXP	0X02

#define AES_KL_128	0
#define AES_KL_192	1
#define AES_KL_256	2

struct aes_reg {
	uint32_t datain[4];	/* Data input 0~127 */
	uint32_t key[8];	/* Key 0~255        */
	uint32_t iv[4];		/* Initial Vector: 0~127 */
	uint32_t ctrl;		/* AES Control Register */
	uint32_t state;		/* AES State Register */
	uint32_t dataout[4];	/* Data Output 0~127 */
};

#endif /* __CSKY_AES_H */