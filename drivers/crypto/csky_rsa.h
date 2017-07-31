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

#ifndef __CSKY_RSA_H
#define __CSKY_RSA_H

struct rsa_reg {
	uint32_t rsa_mwid;
	uint32_t rsa_dwid;
	uint32_t rsa_bwid;
	uint32_t rsa_ctrl;
	uint32_t rsa_rst;
	uint32_t rsa_lp_cnt;
	uint32_t rsa_q0;
	uint32_t rsa_q1;
	uint32_t rsa_isr;
	uint32_t rsa_imr;
	uint32_t rev1[54];
	uint32_t rsa_rfm;
	uint32_t rev2[63];
	uint32_t rsa_rfd;
	uint32_t rev3[63];
	uint32_t rsa_rfc;
	uint32_t rev4[63];
	uint32_t rsa_rfb;
	uint32_t rev5[63];
	uint32_t rsa_rfr;
};

#define RSA_KEY_LEN	1024

#define BN_MAX_BITS	((RSA_KEY_LEN << 1) + 32)
#define BN_MAX_BYTES	((BN_MAX_BITS + 7) >> 3)
#define BN_MAX_WORDS	((BN_MAX_BYTES + 3) >> 2)

#define MAX_RSA_LP_CNT	10000

#define UINT32_TO_UINT64(data)	\
	((uint64_t)(((uint64_t)(data)) & 0x00000000ffffffffU))
#define UINT64L_TO_UINT32(data)	\
	((uint32_t)(((uint64_t)(data)) & 0x00000000ffffffffU))
#define UINT64H_TO_UINT32(data)	\
	((uint32_t)((((uint64_t)(data)) >> 32) & 0x00000000ffffffffU))

#define PKCS1_PADDING	0x01
#define NO_PADDING	0x02

#define MD5_PADDING	0x00
#define SHA1_PADDING	0x01

#define MD5_HASH_SZ	16
#define SHA1_HASH_SZ	20

typedef struct bignum {
	uint32_t pdata[BN_MAX_WORDS];
	uint32_t words;
} bignum_t;

#endif /* __CSKY_RSA_H */
