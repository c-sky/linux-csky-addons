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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/mpi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <crypto/akcipher.h>
#include <crypto/algapi.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include "csky_rsa.h"

#define RSA_FLAGS_BUSY		BIT(0)

#define RSA_FLAGS_SIGN		BIT(8)
#define RSA_FLAGS_VERIFY	BIT(9)
#define RSA_FLAGS_ENC		BIT(10)
#define RSA_FLAGS_DEC		BIT(11)

#define RSA_FLAGS_OPR_MASK	(RSA_FLAGS_SIGN | RSA_FLAGS_VERIFY | \
				 RSA_FLAGS_ENC | RSA_FLAGS_DEC)
#define CSKY_RSA_QUEUE_LENGTH	10

struct csky_rsa_dev;

struct rsa_key_obj {
	uint8_t *n;
	uint8_t *e;
	uint8_t *d;
	uint32_t n_len;
	uint32_t e_len;
	uint32_t d_len;
};

struct csky_rsa_base_ctx {
	struct csky_rsa_dev *dd;
	struct rsa_key_obj key;
};

struct csky_rsa_ctx {
	struct csky_rsa_base_ctx base;
};

struct csky_rsa_reqctx {
	unsigned long dummy;
};

struct csky_rsa_dev {
	struct list_head		 list;
	struct crypto_async_request	*areq;
	struct csky_rsa_base_ctx	*ctx;
	struct device			*dev;
	struct rsa_reg __iomem		*reg_base;
	struct tasklet_struct		done_task;

	struct crypto_queue		queue;
	unsigned long			flags;
	spinlock_t			lock;

	void *				buf;
	uint32_t			buflen;
	struct scatterlist		*real_dst;
};

struct csky_rsa_drv {
	struct list_head dev_list;
	spinlock_t	 lock;
};

static struct csky_rsa_drv csky_rsa = {
	.dev_list = LIST_HEAD_INIT(csky_rsa.dev_list),
	.lock	  = __SPIN_LOCK_UNLOCKED(csky_rsa.lock),
};

static inline void csky_rsa_clear_int(struct csky_rsa_dev *dd)
{
	writel_relaxed(0xffff, &dd->reg_base->rsa_isr);
	writel_relaxed(0x0000, &dd->reg_base->rsa_imr);
}

static inline void csky_rsa_setm_width(struct csky_rsa_dev *dd, uint32_t width)
{
	writel_relaxed(width, &dd->reg_base->rsa_mwid);
}

static inline void csky_rsa_setd_width(struct csky_rsa_dev *dd, uint32_t width)
{
	writel_relaxed(width, &dd->reg_base->rsa_dwid);
}

static inline void csky_rsa_setb_width(struct csky_rsa_dev *dd, uint32_t width)
{
	writel_relaxed(width, &dd->reg_base->rsa_bwid);
}

static inline void csky_rsa_cal_q(struct csky_rsa_dev *dd)
{
	writel_relaxed(0x6, &dd->reg_base->rsa_ctrl);
}

static inline void csky_rsa_opr_start(struct csky_rsa_dev *dd)
{
	writel_relaxed(0x3, &dd->reg_base->rsa_ctrl);
}

static inline void csky_rsa_opr_reset(struct csky_rsa_dev *dd)
{
	uint32_t tmp;

	writel_relaxed(0x8, &dd->reg_base->rsa_ctrl);
	tmp = readl_relaxed(&dd->reg_base->rsa_rst);
	tmp |= 0x1;
	writel_relaxed(tmp, &dd->reg_base->rsa_rst);
	while (readl_relaxed(&dd->reg_base->rsa_rst));
}

static inline uint32_t csky_rsa_loop_cnt(struct csky_rsa_dev *dd)
{
	return readl_relaxed(&dd->reg_base->rsa_lp_cnt);
}

static inline uint32_t csky_rsa_cal_q_done(struct csky_rsa_dev *dd)
{
	return (readl_relaxed(&dd->reg_base->rsa_isr) >> 5) & 0x1;
}

static inline uint32_t csky_rsa_opr_done(struct csky_rsa_dev *dd)
{
	return readl_relaxed(&dd->reg_base->rsa_isr) & 0x1;
}

static inline uint32_t csky_rsa_raise_exception(struct csky_rsa_dev *dd)
{
	return readl_relaxed(&dd->reg_base->rsa_isr) & 0x1E;
}

static inline uint32_t csky_rsa_loadm(struct csky_rsa_dev *dd, uint32_t *data,
				      uint32_t length)
{
	uint32_t i;
	uint32_t baseaddr;

	baseaddr = (uint32_t)&dd->reg_base->rsa_rfm;
	for(i = 0; i < length; i++) {
		writel_relaxed(data[i], (void *)baseaddr);
		baseaddr = baseaddr + 4;
	}

	return 0;
}

static void csky_rsa_loadd(struct csky_rsa_dev *dd, uint32_t *data,
			   uint32_t length)
{
	uint32_t i;
	uint32_t baseaddr;

	baseaddr = (uint32_t)&dd->reg_base->rsa_rfd;
	for(i = 0;  i < length; i++) {
		writel_relaxed(data[i], (void *)baseaddr);
		baseaddr = baseaddr + 4;
	}
}

static void csky_rsa_loadc(struct csky_rsa_dev *dd, uint32_t *data,
			   uint32_t length)
{
	uint32_t i;
	uint32_t baseaddr;

	baseaddr = (uint32_t)&dd->reg_base->rsa_rfc;
	for(i = 1; i < length + 1; i++) {
		writel_relaxed(data[i-1], (void *)baseaddr);
		baseaddr = baseaddr + 4;
	}
}

static void csky_rsa_loadb(struct csky_rsa_dev *dd, uint32_t *data,
			   uint32_t length)
{
	uint32_t i;
	uint32_t baseaddr;

	baseaddr = (uint32_t)&dd->reg_base->rsa_rfb;
	for(i = 0; i < length; i++) {
		writel_relaxed(data[i], (void *)baseaddr);
		baseaddr = baseaddr + 4;
	}
}

static void csky_rsa_read_r(struct csky_rsa_dev *dd, uint32_t data[],
			    uint32_t length)
{
	uint32_t i;
	uint32_t baseaddr;

	baseaddr = (uint32_t)&dd->reg_base->rsa_rfr;
	for(i = 0; i < length; i++) {
		data[i] = readl_relaxed((void *)baseaddr);
		baseaddr = baseaddr + 4;
	}
}

static uint32_t get_valid_bits(const uint32_t *addr, uint32_t wordsize)
{
	uint32_t i = 0;
	uint32_t j = 0;

	for (i = wordsize; i > 0; i--) {
		if (addr[i - 1]) {
			break;
		}
	}

	for (j = 32; j > 0; j--) {
		if (addr[i - 1] & (0x1 << (j - 1))) {
			break;
		}
	}

	return ((i - 1) << 5) + j;
}

static uint32_t get_first_nonzero_words(uint32_t *a, uint32_t max_words)
{
	uint32_t i = 0;

	for (i = max_words; i > 0; i--) {
		if (a[i - 1]) {
			return i;
		}
	}
	return 0;
}

static uint32_t word_array_left_shift(uint32_t *a, uint32_t words,
				      uint32_t shift_bits, uint32_t *r)
{
	uint32_t i = 0;
	uint32_t w;
	uint32_t b;
	uint32_t tmp = 0;

	w = shift_bits >> 5;
	b = shift_bits - (w << 5);

	for (i = 0; i < w; i++) {
		r[i] = 0;
	}

	tmp = 0;
	for (i = 0; i < words; i++) {
		r[w + i] = (tmp | ((a[i] << b) & (~((0x1 << b) - 1))));
		tmp = ((a[i] >> (32 - b)) & ((0x1 << b) - 1));
	}
	r[w + i] = tmp;

	return 0;
}

static uint32_t _word_array_sub(uint32_t *a, uint32_t a_words,
				uint32_t *b, uint32_t b_words,
				uint32_t *r)
{
	uint32_t i;
	uint64_t tmp = 0;
	uint32_t borrow = 0;

	for (i = 0; i < b_words; i++) {
		tmp = UINT32_TO_UINT64(a[i]) - UINT32_TO_UINT64(b[i]) -
			  UINT32_TO_UINT64(borrow);
		r[i] = UINT64L_TO_UINT32(tmp);
		borrow = ((UINT64H_TO_UINT32(tmp) == 0) ? (0):
				  (0xffffffff - UINT64H_TO_UINT32(tmp) + 1));
	}

	for (i = b_words; i < a_words; i++) {
		tmp = UINT32_TO_UINT64(a[i]) - UINT32_TO_UINT64(borrow);
		r[i] = UINT64L_TO_UINT32(tmp);
		borrow = ((UINT64H_TO_UINT32(tmp) == 0) ? (0):
				  (0xffffffff - UINT64H_TO_UINT32(tmp) + 1));
	}

	if (borrow) {
		return -1;
	}

	return 0;
}

static uint32_t word_array_mod(uint32_t *a, uint32_t a_words,
			       uint32_t *b, uint32_t b_words,
			       uint32_t *r)
{
	uint32_t ret;
	bignum_t tmpa;
	bignum_t tmpb;
	uint32_t tmpa_valid_bits, tmpa_words;
	uint32_t tmpb_words, b_valid_bits;

	memset(&tmpa, 0, sizeof(tmpa));
	memset(&tmpb, 0, sizeof(tmpa));

	b_valid_bits = get_valid_bits(b, b_words);

	memcpy(tmpa.pdata, a, (a_words << 2));

	do {
		tmpa_words = get_first_nonzero_words(tmpa.pdata, a_words);
		tmpa_valid_bits = get_valid_bits(tmpa.pdata, tmpa_words);
		if (tmpa_valid_bits > b_valid_bits + 1) {
			memset(tmpb.pdata, 0, (a_words << 2));
			word_array_left_shift(b, b_words,
				tmpa_valid_bits - b_valid_bits - 1,
				tmpb.pdata);
			tmpb_words = get_first_nonzero_words(tmpb.pdata,
							     a_words);
			ret = _word_array_sub(tmpa.pdata, tmpa_words,
					      tmpb.pdata, tmpb_words,
					      tmpa.pdata);
		} else if (tmpa_words == b_words) {
			memcpy(r, tmpa.pdata, (tmpa_words << 2));
			ret = _word_array_sub(r, tmpa_words,
					      b, b_words,
					      tmpa.pdata);
		} else {
			ret = _word_array_sub(tmpa.pdata, tmpa_words,
					      b, b_words,
					      tmpa.pdata);
		}
	} while (ret == 0);

	return 0;
}

static void convert_byte_array(uint8_t *in, uint8_t *out, uint32_t len)
{
	uint8_t tmp;
	uint32_t idx, round = len >> 1;

	for (idx = 0; idx < round; idx++) {
		tmp = *(in + idx);
		*(out + idx) = *(in + len - 1 - idx);
		*(out + len - 1 - idx) = tmp;
	}

	if (len & 0x1) {
		*(out + round) = *(in + round);
	}
}

static void convert_buf_to_bndata(const uint8_t *src, uint32_t src_bytes,
				  uint32_t *dst, uint32_t dst_words)
{
	memset(dst, 0, dst_words << 2);
	convert_byte_array((uint8_t *)src, (uint8_t *)dst, src_bytes);
}

static void convert_bndata_to_buf(const uint32_t *src, uint32_t src_words,
				  uint8_t *dst, uint32_t dst_bytes)
{
	memset(dst, 0, dst_bytes);
	convert_byte_array((uint8_t *)src, (uint8_t *)dst, dst_bytes);
}


static uint32_t sw_exptmod_2_2m(const uint32_t *modulus, uint32_t words,
				uint32_t *tmp_c)
{
	uint32_t ret;
	uint32_t m_valid_bits;
	uint32_t data1 = 0;
	bignum_t tmp;

	memset(&tmp, 0, sizeof(bignum_t));

	m_valid_bits = (words << 5);

	data1 = 0x1;
	word_array_left_shift(&data1, 1, (m_valid_bits << 1), tmp.pdata);
	tmp.words = get_first_nonzero_words(tmp.pdata, words*2 + 1);

	ret = word_array_mod(tmp.pdata, tmp.words,
			     (uint32_t *)modulus, words, tmp_c);
	if (ret != 0) {
		return ret;
	}
	return 0;
}

static uint32_t csky_rsa_exptmod_1024(struct csky_rsa_dev *dd,
				      const uint32_t *modulus,
				      const uint32_t *exponent,
				      const uint32_t *base,
				      uint32_t *out)
{
	uint32_t tmp_c[32];
	uint32_t ret;

	if ((NULL == modulus) || (NULL == exponent) || (NULL == base) ||
	    (NULL == out)) {
		return 1;
	}

	ret = sw_exptmod_2_2m(modulus, 32, tmp_c);
	if (ret != 0) {
		return ret;
	}

	/* reset for safe */
	csky_rsa_opr_reset(dd);
	/* clear and disable int */
	csky_rsa_clear_int(dd);
	/* set m */
	csky_rsa_setm_width(dd, 32 >> 1);
	csky_rsa_loadm(dd, (uint32_t *)modulus, 32);
	/* set d */
	csky_rsa_setd_width(dd, get_valid_bits(exponent, 32) - 1);
	csky_rsa_loadd(dd, (uint32_t *)exponent, 32);
	/* set b */
	csky_rsa_setb_width(dd, 32 >> 1);
	csky_rsa_loadb(dd, (uint32_t *)base, 32);
	/* set c */
	csky_rsa_loadc(dd, tmp_c, 32);

	csky_rsa_cal_q(dd);
	while(!csky_rsa_cal_q_done(dd) && (!csky_rsa_raise_exception(dd)));

	if (!csky_rsa_raise_exception(dd)) {
		csky_rsa_opr_start(dd);
		while((!csky_rsa_opr_done(dd)) &&
			(csky_rsa_loop_cnt(dd) < MAX_RSA_LP_CNT) &&
			(!csky_rsa_raise_exception(dd)));

		if ((csky_rsa_loop_cnt(dd) >= MAX_RSA_LP_CNT)
			|| csky_rsa_raise_exception(dd)) {
			ret = 1;
		} else {
			csky_rsa_read_r(dd, out, 32);
		}
	} else {
		ret = 1;
	}

	csky_rsa_opr_reset(dd);

	return ret;
}

static const uint8_t der_sha1_t[] = {
	0x30, 0x21,
	0x30, 0x09,
	0x06, 0x05, 0x2b, 0x0e, 0x03, 0x02, 0x1a,
	0x05, 0x00,
	0x04, 0x14 };

static const uint8_t der_md5_t[] = {
	0x30, 0x20, /* type Sequence, length 0x20 (32) */
	0x30, 0x0c, /* type Sequence, length 0x09 */
	0x06, 0x08, /* type OID, length 0x05 */
	0x2a, 0x86, 0x48, 0x86, 0xF7, 0x0D, 0x02, 0x05, /* id-md5 */
	0x05, 0x00, /* NULL */
	0x04, 0x10  /* Octet string, length 0x10 (16), followed by md5 hash */
};

static uint32_t RSA_padding_add_PKCS1_sha1_emsa_1024(const uint8_t *dgst,
						     uint8_t *out,
						     uint32_t *outlen,
						     uint32_t type)

{
	uint32_t i;
	uint8_t *p;
	uint32_t modulus_len;
	uint8_t *der;
	uint32_t der_len;
	uint32_t hashlen;
	uint32_t pslen;

	if (type == MD5_PADDING) {
		der	= (uint8_t *)der_md5_t;
		der_len = sizeof(der_md5_t);
		hashlen = MD5_HASH_SZ;
	} else if (type == SHA1_PADDING) {
		der	= (uint8_t *)der_sha1_t;
		der_len = sizeof(der_sha1_t);
		hashlen = SHA1_HASH_SZ;
	} else {
		der	= (uint8_t *)der_md5_t;
		der_len = sizeof(der_md5_t);
		hashlen = MD5_HASH_SZ;
	}

	modulus_len = 1024 >> 3;

	if (*outlen < modulus_len) {
		*outlen = modulus_len;
		return -1;
	}


	p = (uint8_t *)out;

	*(p++) = 0x00;
	*(p++) = 0x01;

	/* pad out with 0xff data */
	pslen = modulus_len - 3 - der_len - hashlen;

	for (i = 0; i < pslen; i++) {
		p[i] = 0xff; /* PS */
	}

	p += pslen;
	*(p++) = 0x0;

	for (i = 0; i < der_len; i++) {
		p[i] = der[i];
	}
	p += der_len;

	for (i = 0; i < hashlen; i++) {
		p[i] = dgst[i];
	}

	*outlen = modulus_len;
	return 0;
}

static uint32_t RSA_padding_check_PKCS1_type_emsa(const uint8_t *dgst,
						  const uint8_t *in,
						  const uint32_t inlen,
						  uint8_t *is_valid,
						  uint32_t type)
{
	uint32_t i;
	uint32_t ret;
	const uint8_t *p;
	uint8_t *der;
	uint32_t der_len;
	uint32_t hashlen;
	uint32_t pslen;
	uint32_t modulus_len;

	if (type == MD5_PADDING) {
		der	= (uint8_t *)der_md5_t;
		der_len = sizeof(der_md5_t);
		hashlen = MD5_HASH_SZ;
	} else if (type == SHA1_PADDING) {
		der	= (uint8_t *)der_sha1_t;
		der_len = sizeof(der_sha1_t);
		hashlen = SHA1_HASH_SZ;
	} else {
		der	= (uint8_t *)der_md5_t;
		der_len = sizeof(der_md5_t);
		hashlen = MD5_HASH_SZ;
	}

	modulus_len = RSA_KEY_LEN >> 3;

	if (inlen != modulus_len) {
		return -1;
	}

	*is_valid = 0;

	pslen = modulus_len - 3 - der_len - hashlen;
	p = in;
	p++;

	if (*(p) != 0x01) {
		ret = -1;
		goto _verify_fail;
	}
	p++;

	/* scan PS */
	for (i = 0; i < pslen; i++) {
		if (*(p + i) != 0xff) {
			ret = -1;
			goto _verify_fail;
		}
	}
	p += pslen;

	if ((*p) != 0x00) {
		ret = -1;
		goto _verify_fail;
	}
	p++;

	/* scan t */
	for (i = 0; i < der_len; i++) {
		if (*(p + i) != der[i]) {
			ret = -1;
			goto _verify_fail;
		}
	}
	p += der_len;

	for (i = 0; i < hashlen; i++) {
		if (*(p + i) != dgst[i]) {
			ret = -1;
			goto _verify_fail;
		}
	}

	*is_valid = 1;
	ret = 0;

_verify_fail:

	return ret;
}

static uint32_t RSA_ES_padding_add_PKCS1_emsa_1024(const uint8_t *dgst,
						   uint32_t dgstlen,
						   uint8_t *out,
						   uint32_t *outlen,
						   uint32_t padding)
{
	uint32_t i;
	uint8_t *p;
	uint32_t modulus_len;
	uint32_t pslen;

	modulus_len = RSA_KEY_LEN >> 3;

	if (*outlen < modulus_len) {
		*outlen = modulus_len;
		return 1;
	}

	p = (uint8_t *)out;

	*(p++) = 0x00;
	*(p++) = 0x02;

	/* pad out with 0xff data */
	pslen = modulus_len - 3 - dgstlen;

	for (i = 0; i < pslen; i++) {
		p[i] = 0xff; /* PS */
	}

	p += pslen;
	*(p++) = 0x0;

	for (i = 0; i < dgstlen; i++) {
		p[i] = dgst[i];
	}

	*outlen = modulus_len;

	return 0;
}

static uint32_t RSA_ES_padding_check_PKCS1_type_emsa(uint8_t *out,
						     uint32_t *out_size,
						     uint8_t *src,
						     uint32_t src_size,
						     uint32_t padding)
{
	uint32_t i;
	uint8_t *p;
	uint32_t modulus_len;
	uint32_t pslen;

	modulus_len = RSA_KEY_LEN >> 3;

	if (src_size < modulus_len) {
		return 1;
	}

	p = (uint8_t *)src;
	*(p++) = 0x00;

	if (padding == PKCS1_PADDING) {
		if (*(p++) != 0x02) {
			return 1;
		}
	} else {
		if (*(p++) != 0x01) {
			return 1;
		}
	}

	pslen = src_size - 2;

	while (pslen--) {
		if (*(p++) == 0x0) {
			break;
		}
	}

	for (i = 0; i < pslen; i++) {
		 out[i] = p[i];
	}

	*out_size = pslen;

	return 0;
}

int rsa_encrypt(struct csky_rsa_dev *dd, uint8_t *n, uint8_t *e,
		uint8_t *src, uint32_t src_size,
		uint8_t *out, uint32_t *out_size,
		uint32_t padding)
{
	uint32_t ret;
	uint32_t tmp_n[32];
	uint32_t tmp_e[32];
	uint32_t tmp_src_padded[32];
	uint32_t tmp_out[32];
	uint32_t keywords = 0, keybytes = 0;
	uint32_t tmp_src_padded_len = 0;

	keywords = RSA_KEY_LEN >> 5;
	keybytes = (keywords << 2);

	convert_buf_to_bndata(n, keybytes, tmp_n, keywords);
	convert_buf_to_bndata(e, keybytes, tmp_e, keywords);

	tmp_src_padded_len = keybytes;
	if (padding == PKCS1_PADDING) {
		ret = RSA_ES_padding_add_PKCS1_emsa_1024(
			(const uint8_t *)src, src_size,
			(uint8_t *)tmp_src_padded,
			&tmp_src_padded_len, padding);
		if (ret != 0) {
			return ret;
		}
		convert_byte_array(
			(uint8_t *)tmp_src_padded,
			(uint8_t *)tmp_src_padded,
			tmp_src_padded_len);
	} else {
		convert_byte_array(
			(uint8_t *)src,
			(uint8_t *)tmp_src_padded,
			tmp_src_padded_len);
	}

	ret = csky_rsa_exptmod_1024(dd, tmp_n, tmp_e, tmp_src_padded, tmp_out);
	if (ret != 0) {
		return ret;
	}

	convert_bndata_to_buf(tmp_out, keywords, out, keybytes);
	*out_size = keybytes;

	return ret;
}

int rsa_decrypt(struct csky_rsa_dev *dd,
		uint8_t *n, uint8_t *d,
		uint8_t *src, uint32_t src_size,
		uint8_t *out, uint32_t *out_size,
		uint32_t padding)
{
	uint32_t ret;
	uint32_t tmp_n[32];
	uint32_t tmp_d[32];
	uint32_t tmp_dst_padded[32];
	uint32_t tmp_sig[32];
	uint32_t keywords = 0, keybytes = 0;

	keywords = RSA_KEY_LEN >> 5;
	keybytes = (keywords << 2);

	convert_buf_to_bndata(n, keybytes, tmp_n, keywords);
	convert_buf_to_bndata(d, keybytes, tmp_d, keywords);
	convert_buf_to_bndata(src, src_size, tmp_sig, keywords);

	ret = csky_rsa_exptmod_1024(dd, tmp_n, tmp_d, tmp_sig, tmp_dst_padded);
	if (ret != 0) {
		return ret;
	}

	convert_byte_array((uint8_t *)tmp_dst_padded,
			   (uint8_t *)tmp_dst_padded, keybytes);

	ret = RSA_ES_padding_check_PKCS1_type_emsa(out, out_size,
						   (uint8_t *)tmp_dst_padded,
						   keybytes, padding);

	return ret;
}

int rsa_sign(struct csky_rsa_dev *dd,
	     uint8_t *n, uint8_t *d,
	     uint8_t *src, uint32_t src_size,
	     uint8_t *signature, uint32_t *sig_size, uint32_t type)
{
	uint32_t ret;
	uint32_t tmp_n[32];
	uint32_t tmp_d[32];
	uint32_t tmp_src_padded[32];
	uint32_t tmp_sig[32];
	uint32_t keywords = 0, keybytes = 0;
	uint32_t tmp_src_padded_len = 0;

	keywords = RSA_KEY_LEN >> 5;
	keybytes = (keywords << 2);

	convert_buf_to_bndata(n, keybytes, tmp_n, keywords);
	convert_buf_to_bndata(d, keybytes, tmp_d, keywords);

	tmp_src_padded_len = keybytes;
	ret = RSA_padding_add_PKCS1_sha1_emsa_1024((const uint8_t *)src,
						   (uint8_t *)tmp_src_padded,
						   &tmp_src_padded_len, type);
	if (ret != 0) {
		return ret;
	}

	convert_byte_array((uint8_t *)tmp_src_padded,
			   (uint8_t *)tmp_src_padded,
			   keybytes);

	ret = csky_rsa_exptmod_1024(dd, tmp_n, tmp_d, tmp_src_padded, tmp_sig);
	if (ret != 0) {
		return ret;
	}

	convert_bndata_to_buf(tmp_sig, keywords, signature, keybytes);
	*sig_size = keybytes;

	return 0;
}

int rsa_verify(struct csky_rsa_dev *dd,
	       uint8_t *n, uint8_t *e,
	       uint8_t *src, uint32_t src_size,
	       uint8_t *signature, uint32_t sig_size,
	       uint8_t *result, uint32_t type)
{
	uint32_t ret;
	uint32_t tmp_n[32];
	uint32_t tmp_e[32];
	uint32_t tmp_dst_padded[32];
	uint32_t tmp_sig[32];
	uint32_t keywords = 0, keybytes = 0;

	*result = 0;

	keywords = RSA_KEY_LEN >> 5;
	keybytes = (keywords << 2);

	convert_buf_to_bndata(n, keybytes, tmp_n, keywords);
	convert_buf_to_bndata(e, keybytes, tmp_e, keywords);
	convert_buf_to_bndata(signature, sig_size, tmp_sig, keywords);

	ret = csky_rsa_exptmod_1024(dd, tmp_n, tmp_e, tmp_sig, tmp_dst_padded);
	if (ret != 0) {
		return ret;
	}

	convert_byte_array((uint8_t *)tmp_dst_padded,
			   (uint8_t *)tmp_dst_padded, keybytes);
	ret = RSA_padding_check_PKCS1_type_emsa(
		src,
		(const uint8_t *)tmp_dst_padded,
		keybytes,
		result,
		type);

	return ret;
}

static struct csky_rsa_dev *csky_rsa_find_dev(struct csky_rsa_base_ctx *ctx)
{
	struct csky_rsa_dev *rsa_dd = NULL;
	struct csky_rsa_dev *tmp;

	spin_lock_bh(&csky_rsa.lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &csky_rsa.dev_list, list) {
			rsa_dd = tmp;
			break;
		}
		ctx->dd = rsa_dd;
	} else {
		rsa_dd = ctx->dd;
	}
	spin_unlock_bh(&csky_rsa.lock);

	return rsa_dd;
}

static inline struct akcipher_request *akcipher_request_cast(
	struct crypto_async_request *req)
{
	return container_of(req, struct akcipher_request, base);
}

static inline int csky_rsa_complete(struct csky_rsa_dev *dd, int err)
{
	dd->flags &= ~RSA_FLAGS_BUSY;
	dd->flags &= ~RSA_FLAGS_OPR_MASK;
	dd->areq->complete(dd->areq, err);

	tasklet_schedule(&dd->done_task);

	return err;
}

static int csky_rsa_handle(struct csky_rsa_dev *dd)
{
	struct csky_rsa_base_ctx *ctx = dd->ctx;
	struct rsa_key_obj *pkey = &ctx->key;
	int err = 0;
	uint8_t tmpdata[128];
	uint32_t tmplen = 0;
	uint8_t sign = 0;

	if (dd->flags & RSA_FLAGS_ENC) {
		rsa_encrypt(dd, pkey->n, pkey->e, dd->buf, dd->buflen, dd->buf,
			    &dd->buflen, PKCS1_PADDING);
		if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
					 dd->buf, dd->buflen))
			err = -EINVAL;

	} else if (dd->flags && RSA_FLAGS_DEC) {
		rsa_decrypt(dd, pkey->n, pkey->d, dd->buf, dd->buflen,
			    (uint8_t *)tmpdata, &tmplen, PKCS1_PADDING);
		memset(dd->buf, 0, pkey->n_len);
		memcpy(dd->buf+pkey->n_len-tmplen, tmpdata, tmplen);
		if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
					 dd->buf, pkey->n_len))
			err = -EINVAL;

	} else if (dd->flags && RSA_FLAGS_SIGN) {
		rsa_sign(dd, pkey->n, pkey->d, dd->buf, dd->buflen, dd->buf,
				&pkey->n_len,MD5_PADDING);
		if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
					 dd->buf, pkey->n_len))
			err = -EINVAL;
	} else if (dd->flags && RSA_FLAGS_VERIFY) {
		sg_copy_to_buffer(dd->real_dst, sg_nents(dd->real_dst),
				  (void *)tmpdata, pkey->n_len);
		rsa_verify(dd, pkey->n, pkey->e, dd->buf, dd->buflen,
			   (uint8_t *)&tmpdata, pkey->n_len,
			   &sign, MD5_PADDING);
		if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
		    (void *)&sign, 1))
			err = -EINVAL;
	} else
		err = -EINVAL;

	csky_rsa_complete(dd, err);

	return err;
}

static int csky_rsa_handle_queue(struct csky_rsa_dev *dd,
				 struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	struct csky_rsa_base_ctx	*ctx;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&dd->queue, new_areq);
	if (dd->flags & RSA_FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	areq = crypto_dequeue_request(&dd->queue);
	if (areq)
		dd->flags |= RSA_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx	 = crypto_tfm_ctx(areq->tfm);
	dd->areq = areq;
	dd->ctx  = ctx;

	return csky_rsa_handle(dd);
}

static int csky_rsa_check_key_length(unsigned int len)
{
	switch (len) {
	case 1024:
		return 0;
	}

	return -EINVAL;
}

static int csky_rsa_enc(struct akcipher_request *req)
{
	struct csky_rsa_base_ctx *ctx;
	struct csky_rsa_dev	 *dd;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	dd  = csky_rsa_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	dd->flags |= RSA_FLAGS_ENC;
	dd->buflen = req->src_len;
	dd->real_dst = req->dst;
	sg_copy_to_buffer(req->src, sg_nents(req->src), dd->buf, req->src_len);

	return csky_rsa_handle_queue(dd, &req->base);
}

static int csky_rsa_dec(struct akcipher_request *req)
{
	struct csky_rsa_base_ctx *ctx;
	struct csky_rsa_dev	 *dd;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	dd  = csky_rsa_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	dd->flags |= RSA_FLAGS_DEC;
	dd->buflen = req->src_len;
	dd->real_dst = req->dst;
	sg_copy_to_buffer(req->src, sg_nents(req->src), dd->buf, req->src_len);

	return csky_rsa_handle_queue(dd, &req->base);
}

static int csky_rsa_sign(struct akcipher_request *req)
{
	struct csky_rsa_base_ctx *ctx;
	struct csky_rsa_dev	 *dd;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	dd  = csky_rsa_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	dd->flags |= RSA_FLAGS_SIGN;
	dd->buflen = req->src_len;
	dd->real_dst = req->dst;
	sg_copy_to_buffer(req->src, sg_nents(req->src), dd->buf, req->src_len);

	return csky_rsa_handle_queue(dd, &req->base);
}

static int csky_rsa_verify(struct akcipher_request *req)
{
	struct csky_rsa_base_ctx *ctx;
	struct csky_rsa_dev	 *dd;

	ctx = akcipher_tfm_ctx(crypto_akcipher_reqtfm(req));
	dd  = csky_rsa_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	dd->flags |= RSA_FLAGS_VERIFY;
	dd->buflen = req->src_len;
	dd->real_dst = req->dst;
	sg_copy_to_buffer(req->src, sg_nents(req->src), dd->buf, req->src_len);

	return csky_rsa_handle_queue(dd, &req->base);
}

static int csky_rsa_set_priv_key(struct crypto_akcipher *tfm, const void *key,
				 unsigned int keylen)
{
	struct csky_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key_obj *pkey = &ctx->base.key;
	struct rsa_key raw_key = {0};
	int ret;

	ret = rsa_parse_priv_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	pkey->n_len = raw_key.n_sz;
	memset(pkey->n, 0, pkey->n_len);
	memcpy(pkey->n, raw_key.n, raw_key.n_sz);

	pkey->d_len = raw_key.d_sz;
	memset(pkey->d, 0, pkey->n_len);
	memcpy(pkey->d, raw_key.d, raw_key.d_sz);

	pkey->e_len = raw_key.e_sz;
	memset(pkey->e, 0, pkey->n_len);
	memcpy(&pkey->e[pkey->n_len - raw_key.e_sz], raw_key.e, raw_key.e_sz);

	if (csky_rsa_check_key_length(pkey->n_len << 3)) {
		return -EINVAL;
	}

	return 0;
}

static int csky_rsa_set_pub_key(struct crypto_akcipher *tfm, const void *key,
				unsigned int keylen)
{
	struct csky_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key_obj *pkey = &ctx->base.key;
	struct rsa_key raw_key = {0};
	int ret;

	ret = rsa_parse_pub_key(&raw_key, key, keylen);
	if (ret)
		return ret;

	pkey->n_len = raw_key.n_sz;
	memset(pkey->n, 0, pkey->n_len);
	memcpy(pkey->n, raw_key.n, raw_key.n_sz);

	pkey->e_len = raw_key.e_sz;
	memset(pkey->e, 0, pkey->n_len);
	memcpy(&pkey->e[pkey->n_len - raw_key.e_sz], raw_key.e, raw_key.e_sz);

	if (csky_rsa_check_key_length(pkey->n_len << 3)) {
		return -EINVAL;
	}

	return 0;
}

static int csky_rsa_max_size(struct crypto_akcipher *tfm)
{
	struct csky_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key_obj *pkey = &ctx->base.key;

	return pkey->n_len;
}

static int csky_rsa_init(struct crypto_akcipher *tfm)
{
	struct csky_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key_obj *pkey = &ctx->base.key;

	pkey->n = (void *)__get_free_pages(GFP_KERNEL, 1);
	if (!pkey->n)
		goto _out_n;
	pkey->e = (void *)__get_free_pages(GFP_KERNEL, 1);
	if (!pkey->e)
		goto _out_e;
	pkey->d = (void *)__get_free_pages(GFP_KERNEL, 1);
	if (!pkey->d)
		goto _out_d;

	return 0;

_out_d:
	free_page((unsigned long)pkey->e);
_out_e:
	free_page((unsigned long)pkey->n);
_out_n:
	return -ENOMEM;
}

static void csky_rsa_exit(struct crypto_akcipher *tfm)
{
	struct csky_rsa_ctx *ctx = akcipher_tfm_ctx(tfm);
	struct rsa_key_obj *pkey = &ctx->base.key;

	free_page((unsigned long)pkey->n);
	free_page((unsigned long)pkey->d);
	free_page((unsigned long)pkey->e);
}

static struct akcipher_alg rsa_algs[] = {
	{
		.encrypt	= csky_rsa_enc,
		.decrypt	= csky_rsa_dec,
		.sign		= csky_rsa_sign,
		.verify		= csky_rsa_verify,
		.set_priv_key	= csky_rsa_set_priv_key,
		.set_pub_key	= csky_rsa_set_pub_key,
		.max_size	= csky_rsa_max_size,
		.init		= csky_rsa_init,
		.exit		= csky_rsa_exit,
		.reqsize	= sizeof(struct csky_rsa_reqctx),
		.base = {
			.cra_name	 = "rsa-c",
			.cra_driver_name = "csky-rsa",
			.cra_priority	 = 100,
			.cra_module	 = THIS_MODULE,
			.cra_ctxsize	 = sizeof(struct csky_rsa_ctx),
		},
	},
};

static void csky_rsa_done_task(unsigned long data)
{
	struct csky_rsa_dev *dd = (struct csky_rsa_dev *)data;

	csky_rsa_handle_queue(dd, NULL);
}

static void csky_rsa_unregister_algs(struct csky_rsa_dev *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rsa_algs); i++)
		crypto_unregister_akcipher(&rsa_algs[i]);
}

static int csky_rsa_register_algs(struct csky_rsa_dev *dd)
{
	int err, i, j;

	for (i = 0; i < ARRAY_SIZE(rsa_algs); i++) {
		err = crypto_register_akcipher(&rsa_algs[i]);
		if (err) {
			for (j = 0; j < i; j++)
				crypto_unregister_akcipher(&rsa_algs[j]);
			return err;
		}
	}

	return 0;
}

static int csky_rsa_probe(struct platform_device *pdev)
{
	struct csky_rsa_dev *rsa_dd;
	struct device	    *dev = &pdev->dev;
	struct resource	    *rsa_res;
	int err;

	rsa_dd = devm_kzalloc(&pdev->dev, sizeof(*rsa_dd), GFP_KERNEL);
	if (rsa_dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		err = -ENOMEM;
		goto rsa_dd_err;
	}

	rsa_dd->dev = dev;

	platform_set_drvdata(pdev, rsa_dd);

	INIT_LIST_HEAD(&rsa_dd->list);
	spin_lock_init(&rsa_dd->lock);

	tasklet_init(&rsa_dd->done_task,
		     csky_rsa_done_task, (unsigned long)rsa_dd);

	crypto_init_queue(&rsa_dd->queue, CSKY_RSA_QUEUE_LENGTH);

	rsa_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rsa_res) {
		err = -ENODEV;
		goto res_err;
	}

	rsa_dd->reg_base = devm_ioremap_resource(dev, rsa_res);
	if (IS_ERR(rsa_dd->reg_base)) {
		err = PTR_ERR(rsa_dd->reg_base);
		goto res_err;
	}

	spin_lock(&csky_rsa.lock);
	list_add_tail(&rsa_dd->list, &csky_rsa.dev_list);
	spin_unlock(&csky_rsa.lock);

	rsa_dd->buf = (void *)__get_free_pages(GFP_KERNEL, 1);
	if (!rsa_dd->buf)
		goto err_algs;

	err = csky_rsa_register_algs(rsa_dd);
	if (err)
		goto err_algs;

	dev_info(dev, "CSKY RSA Driver Initialized\n");

	return 0;

err_algs:
	spin_lock(&csky_rsa.lock);
	list_del(&rsa_dd->list);
	spin_unlock(&csky_rsa.lock);
res_err:
	tasklet_kill(&rsa_dd->done_task);
rsa_dd_err:

	return err;
}

static int csky_rsa_remove(struct platform_device *pdev)
{
	static struct csky_rsa_dev *rsa_dd;

	rsa_dd = platform_get_drvdata(pdev);
	if (!rsa_dd)
		return -ENODEV;

	spin_lock(&csky_rsa.lock);
	list_del(&rsa_dd->list);
	spin_unlock(&csky_rsa.lock);

	tasklet_kill(&rsa_dd->done_task);
	csky_rsa_unregister_algs(rsa_dd);
	free_page((unsigned long)rsa_dd->buf);

	return 0;
}

static const struct of_device_id csky_rsa_dt_ids[] = {
	{ .compatible = "csky,csky-rsa" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, csky_rsa_dt_ids);

static struct platform_driver csky_rsa_driver = {
	.probe	 = csky_rsa_probe,
	.remove	 = csky_rsa_remove,
	.driver	 = {
		.name   = "csky_rsa",
		.of_match_table = of_match_ptr(csky_rsa_dt_ids),
	},
};

module_platform_driver(csky_rsa_driver);

MODULE_DESCRIPTION("CSKY RSA hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Cui <xiaoxia_cui@c-sky.com>");
