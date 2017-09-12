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

#ifndef __CSKY_SHA_H__
#define __CSKY_SHA_H__


#define CSKY_SHA_INIT	3
#define CSKY_SHA_INT	4
#define CSKY_SHA_ENDIAN	5
#define CSKY_SHA_CALC	6
#define CSKY_SHA_DONE	0x40

typedef struct sha_reg {
	uint32_t SHA_CON;
	uint32_t SHA_INTSTATE;
	uint32_t SHA_H0L;
	uint32_t SHA_H1L;
	uint32_t SHA_H2L;
	uint32_t SHA_H3L;
	uint32_t SHA_H4L;
	uint32_t SHA_H5L;
	uint32_t SHA_H6L;
	uint32_t SHA_H7L;
	uint32_t SHA_H0H;
	uint32_t SHA_H1H;
	uint32_t SHA_H2H;
	uint32_t SHA_H3H;
	uint32_t SHA_H4H;
	uint32_t SHA_H5H;
	uint32_t SHA_H6H;
	uint32_t SHA_H7H;
	uint32_t SHA_DATA1;
	uint32_t REV[15];
	uint32_t SHA_DATA2;
} sha_reg_t;

typedef enum{
	SHA_1       = 1,
	SHA_256     = 2,
	SHA_224     = 3,
	SHA_512     = 4,
	SHA_384     = 5,
}sha_mode_t;

typedef enum{
	SHA_STATUS_HASH      = 0,
	SHA_STATUS_START     = 1,
	SHA_STATUS_UPDATE    = 2,
	SHA_STATUS_END       = 3
}sha_status_t;

typedef enum{
	SHA_BIG_ENDIAN    = 0,
	SHA_LITTLE_ENDIAN = 1
}sha_endian_t;

#endif

