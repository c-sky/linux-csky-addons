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

#ifndef __CSKY_TDES_H
#define __CSKY_TDES_H


#define TDES_ENDIAN	0x00000008
#define TDES_ENDIAN_LT	0
#define TDES_ENDIAN_BG	1

#define TDES_IT_DATAINT	0x4
#define TDES_IT_PAERR	0x2
#define TDES_IT_BUSY	0x1
#define TDES_IT_ALL	0x7

#define TDES_OPC_ENC	0x0000
#define TDES_OPC_DEC	0x0002

#define TDES_MOD_ECB	0x0000
#define TDES_MOD_CBC	0x0010

#define TDES_OPR_DES3	0x0040

#define TDES_KL_128	0
#define TDES_KL_192	1
#define TDES_KL_256	2

struct tdes_reg {
	uint32_t datain[2];	/* Data input 0~15 */
	uint32_t key[6];	/* Key 0~191 */
	uint32_t iv[2];		/* Initial Vector: 0~31 */
	uint32_t ctrl;		/* DES Control Register */
	uint32_t state;		/* DES State Register */
	uint32_t dataout[2];	/* Data Output 0~15 */
};

#endif /* __CSKY_TDES_H */
