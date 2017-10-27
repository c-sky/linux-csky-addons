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
#include <linux/device.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <crypto/scatterwalk.h>
#include <crypto/algapi.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include "csky_sha.h"

/* SHA flags */
#define SHA_FLAGS_BUSY		BIT(0)
#define SHA_FLAGS_OUTPUT_READY  BIT(1)
#define SHA_FLAGS_INIT		BIT(2)
#define SHA_FLAGS_CPU		BIT(3)

#define SHA_FLAGS_FINUP		BIT(16)
#define SHA_FLAGS_FINAL		BIT(17)
#define SHA_FLAGS_ALGO_MASK	GENMASK(22, 18)
#define SHA_FLAGS_SHA1		BIT(18)
#define SHA_FLAGS_SHA224	BIT(19)
#define SHA_FLAGS_SHA256	BIT(20)
#define SHA_FLAGS_SHA384	BIT(21)
#define SHA_FLAGS_SHA512	BIT(22)
#define SHA_FLAGS_ERROR		BIT(23)
#define SHA_FLAGS_PAD		BIT(24)

#define SHA_OP_UPDATE		1
#define SHA_OP_FINAL		2

#define SHA_BUFFER_LEN		(PAGE_SIZE / 16)

#define CSKY_SHA_QUEUE_LENGTH	10

struct csky_sha_dev;

struct csky_sha_reqctx {
	struct csky_sha_dev *dd;
	unsigned long	     flags;
	unsigned long	     op;

	uint8_t	 digest[SHA512_DIGEST_SIZE] __aligned(sizeof(u32));
	uint64_t digcnt;
	size_t	 bufcnt;
	size_t	 buflen;
	size_t	 block_size;

	uint32_t endian_flag;
	size_t	 last_left;

	struct scatterlist *sg;
	unsigned int	    offset;
	unsigned int	    total;

	uint8_t  buffer[SHA_BUFFER_LEN+SHA512_BLOCK_SIZE] __aligned(sizeof(u32));

};

struct csky_sha_ctx {
	struct csky_sha_dev *dd;
};

struct csky_sha_dev {
	struct list_head	 list;
	struct device		*dev;
	struct sha_reg __iomem  *io_base;
	spinlock_t		 lock;
	struct tasklet_struct	 done_task;

	unsigned long		 flags;
	struct crypto_queue	 queue;
	struct ahash_request	 *req;
};

struct csky_sha_drv {
	struct list_head dev_list;
	spinlock_t	 lock;
};

static struct csky_sha_drv csky_sha = {
	.dev_list = LIST_HEAD_INIT(csky_sha.dev_list),
	.lock 	  = __SPIN_LOCK_UNLOCKED(csky_sha.lock),
};

static inline void csky_sha_set_mode(struct csky_sha_dev *dd, sha_mode_t mode)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->io_base->SHA_CON);
	tmp  = mode;
	writel_relaxed(tmp, &dd->io_base->SHA_CON);
}

static inline void csky_sha_enable_init(struct csky_sha_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->io_base->SHA_CON);
	tmp |= 1 << CSKY_SHA_INIT;
	writel_relaxed(tmp, &dd->io_base->SHA_CON);
}

static inline void csky_sha_enable_calc(struct csky_sha_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->io_base->SHA_CON);
	tmp |= 1 << CSKY_SHA_CALC;
	writel_relaxed(tmp, &dd->io_base->SHA_CON);
}

static inline void csky_sha_enable_int(struct csky_sha_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->io_base->SHA_CON);
	tmp |= 1 << CSKY_SHA_INT;
	writel_relaxed(tmp, &dd->io_base->SHA_CON);
}

static inline void csky_sha_message_done(struct csky_sha_dev *dd)
{
	while((readl_relaxed(&dd->io_base->SHA_CON) & CSKY_SHA_DONE) != 0);
}

static inline void csky_sha_set_endian(struct csky_sha_dev *dd,
					sha_endian_t mode)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->io_base->SHA_CON);
	tmp |= mode << CSKY_SHA_ENDIAN;
	writel_relaxed(tmp, &dd->io_base->SHA_CON);
}

static inline void csky_sha_input_data(struct csky_sha_dev *dd,
					uint32_t *data, uint32_t length)
{
	uint32_t i;
	uint32_t input_data = (uint32_t)&dd->io_base->SHA_DATA1;
	uint32_t tmp;
	for (i=0; i<length; i++) {
	#ifdef __LITTLE_ENDIAN
		tmp = *(uint32_t *)(data + i);
		*(uint32_t *)(data + i) = (((tmp & 0xff000000) >> 24) | \
					   ((tmp & 0x00ff0000) >> 8)  | \
					   ((tmp & 0x0000ff00) << 8)  | \
					   ((tmp & 0x000000ff) << 24));
	#endif
		writel_relaxed(*(data + i), (void *)input_data);
		input_data += 4;
	}
}

static inline void csky_sha_reverse_order(uint8_t *pdata, int length)
{
	uint32_t wlen = length >> 2;
	uint32_t result = 0;
	uint32_t tmp_data[SHA512_DIGEST_SIZE>>2];
	int i = 0;

	memcpy((void *)tmp_data, (void *)pdata, length);
	for (i = 0; i < wlen; i++) {
		result = (((tmp_data[i] & 0xff000000) >> 24) | \
			  ((tmp_data[i] & 0x00ff0000) >> 8)  | \
			  ((tmp_data[i] & 0x0000ff00) << 8)  | \
			  ((tmp_data[i] & 0x000000ff) << 24));

		tmp_data[i] = result;
	}
	memcpy((void *)pdata, (void *)tmp_data, length);
}

static inline void csky_sha_get_data(struct csky_sha_dev *dd,
				     uint32_t *data, uint32_t size)
{
	uint32_t result_l = (uint32_t)&dd->io_base->SHA_H0L;
	uint32_t result_h = (uint32_t)&dd->io_base->SHA_H0H;
	uint32_t i;

	if (size >= (SHA384_DIGEST_SIZE/4)) {
		for (i = 0; i < size/2; i++) {
		    data[i << 1]       = readl_relaxed((void *)result_h);
		    data[(i << 1) + 1] = readl_relaxed((void *)result_l);
		    result_l += 4;
		    result_h += 4;
		}
	} else {
		for (i = 0; i < size; i++) {
			data[i] = readl_relaxed((void *)result_l);
			result_l += 4;
		}
	}

	csky_sha_reverse_order((uint8_t *)data, size  << 2);
}

static void csky_sha_start(struct csky_sha_reqctx *ctx, sha_mode_t mode)
{
	struct csky_sha_dev *dd = ctx->dd;

	csky_sha_set_mode(dd, mode);
#ifdef __LITTLE_ENDIAN
	csky_sha_set_endian(dd, SHA_LITTLE_ENDIAN);
#else
	csky_sha_set_endian(dd, SHA_BIG_ENDIAN);
#endif
	csky_sha_enable_init(dd);
}

static size_t csky_sha_append_sg(struct csky_sha_reqctx *ctx)
{
	size_t count;

	while ((ctx->bufcnt < ctx->buflen) && ctx->total) {
		count = min(ctx->sg->length - ctx->offset, ctx->total);
		count = min(count, ctx->buflen - ctx->bufcnt);

		if (count <= 0) {
			if ((ctx->sg->length == 0) && !sg_is_last(ctx->sg)) {
				ctx->sg = sg_next(ctx->sg);
				continue;
			} else
				break;
		}

		scatterwalk_map_and_copy(ctx->buffer + ctx->bufcnt, ctx->sg,
					 ctx->offset, count, 0);

		ctx->bufcnt += count;
		ctx->offset += count;
		ctx->total  -= count;

		if (ctx->offset == ctx->sg->length) {
			ctx->sg = sg_next(ctx->sg);
			if (ctx->sg)
				ctx->offset = 0;
			else
				ctx->total = 0;
		}
	}

	return 0;
}

static void csky_sha_flush_padding(struct csky_sha_reqctx *ctx,
				   uint8_t *pad_buf, uint32_t pad_size)
{
	uint32_t block_size = ctx->block_size;
	uint32_t left_len;
	uint32_t left = ctx->digcnt & (block_size - 1);
	uint32_t fill = block_size - left;
	uint32_t len  = pad_size;
	uint32_t i;

	if ((block_size == SHA512_BLOCK_SIZE) ||
		(block_size == SHA384_BLOCK_SIZE))
		left_len = len & 0x7F;
	else
		left_len = len & 0x3F;

	ctx->bufcnt += len;

	if (len >= fill) {
		memcpy((void *)(ctx->buffer + left), pad_buf, fill);
		if ((len - fill) >= block_size)
			memcpy((void *)(ctx->buffer + left + fill),
				&pad_buf[fill], len - fill);
	}
}

static void csky_sha_fill_padding(struct csky_sha_reqctx *ctx)
{
	uint32_t block_size = ctx->block_size;
	uint32_t total_length = ctx->digcnt << 3;
	uint32_t last = ctx->digcnt & (block_size - 1);
	uint32_t pad_rsvr = (ctx->block_size >= SHA384_BLOCK_SIZE) ? 16 : 8;
	uint32_t padn = (last < block_size - pad_rsvr) ?
			(block_size - last) : (block_size * 2 - last);
	uint32_t left = ctx->digcnt & 0x3;
	uint8_t  temp[4];
	uint8_t  sha_padding[SHA512_BLOCK_SIZE*2] = {0};
	uint32_t i;

	for (i = 0;  i < 4; i++)
		temp[i] = (total_length >> (8 * i)) & 0xff;

	memset(sha_padding, 0x0, SHA512_BLOCK_SIZE*2);

	sha_padding[0] = 0x80;

	for (i = 0; i < 4; i++)
		sha_padding[padn - 4 + i] = temp[3 - i];

	csky_sha_flush_padding(ctx, sha_padding, padn);

	ctx->flags |= SHA_FLAGS_PAD;

}

static int csky_sha_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct csky_sha_ctx *tctx = crypto_ahash_ctx(tfm);
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	struct csky_sha_dev *dd = NULL;
	struct csky_sha_dev *tmp;
	sha_mode_t mode;

	spin_lock_bh(&csky_sha.lock);
	if (!tctx->dd) {
		list_for_each_entry(tmp, &csky_sha.dev_list, list) {
			dd = tmp;
			break;
		}
		tctx->dd = dd;
	} else
		dd = tctx->dd;
	spin_unlock_bh(&csky_sha.lock);

	ctx->dd = dd;
	ctx->flags = 0;

	dev_dbg(dd->dev, "digest size: %d\n", crypto_ahash_digestsize(tfm));

	switch (crypto_ahash_digestsize(tfm)) {
	case SHA1_DIGEST_SIZE:
		ctx->flags 	|= SHA_FLAGS_SHA1;
		ctx->block_size  = SHA1_BLOCK_SIZE;
		mode 		 = SHA_1;
		break;
	case SHA224_DIGEST_SIZE:
		ctx->flags 	|= SHA_FLAGS_SHA224;
		ctx->block_size  = SHA224_BLOCK_SIZE;
		mode 		 = SHA_224;
		break;
	case SHA256_DIGEST_SIZE:
		ctx->flags 	|= SHA_FLAGS_SHA256;
		ctx->block_size  = SHA256_BLOCK_SIZE;
		mode 		 = SHA_256;
		break;
	case SHA384_DIGEST_SIZE:
		ctx->flags 	|= SHA_FLAGS_SHA384;
		ctx->block_size  = SHA384_BLOCK_SIZE;
		mode 		 = SHA_384;
		break;
	case SHA512_DIGEST_SIZE:
		ctx->flags 	|= SHA_FLAGS_SHA512;
		ctx->block_size  = SHA512_BLOCK_SIZE;
		mode 		 = SHA_512;
		break;
	default:
		return -EINVAL;
		break;
	}

	ctx->bufcnt = 0;
	ctx->digcnt = 0;
	ctx->buflen = ctx->block_size;
	ctx->last_left = 0;
	ctx->total  = 0;

	csky_sha_start(ctx, mode);

	return 0;
}

static int csky_sha_xmit_cpu(struct csky_sha_dev *dd, const uint8_t *buf,
			     size_t length, int final)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(dd->req);
	int count, len32;

	dev_dbg(dd->dev, "xmit_cpu: digcnt: 0x%llx, length: %d, final: %d\n",
		 ctx->digcnt, length, final);

	if (final)
		dd->flags |= SHA_FLAGS_FINAL;

	len32 = DIV_ROUND_UP(length, sizeof(u32));

	dd->flags |= SHA_FLAGS_CPU;

	for (count = 0; count < length; count += ctx->block_size) {
		csky_sha_input_data(dd, (uint32_t *)&ctx->buffer[count],
				    ctx->block_size >> 2);
		csky_sha_enable_calc(dd);
		csky_sha_message_done(dd);
	}

	return 0;
}

static int csky_sha_update_req(struct csky_sha_dev *dd)
{
	struct ahash_request   *req = dd->req;
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	int err = 0;
	int bufcnt;
	uint32_t last_total = 0;

	if (ctx->flags & SHA_FLAGS_FINUP) {
		ctx->digcnt += ctx->total;
		while (ctx->total >= ctx->buflen) {
			csky_sha_append_sg(ctx);
			bufcnt = ctx->bufcnt;
			ctx->bufcnt = 0;
			err = csky_sha_xmit_cpu(dd, ctx->buffer, bufcnt, 0);
			if (err != 0)
				return err;
		}
	} else {
		while ((ctx->total + ctx->bufcnt) >= ctx->buflen) {
			last_total = ctx->total;
			csky_sha_append_sg(ctx);
			bufcnt = ctx->bufcnt;
			ctx->bufcnt = 0;
			ctx->digcnt += bufcnt?(last_total-ctx->total):bufcnt;
			last_total = ctx->total;
			err = csky_sha_xmit_cpu(dd, ctx->buffer, bufcnt, 0);
			if (err != 0)
				return err;
		}

		if (ctx->total > 0) {
			ctx->digcnt += ctx->total;
			csky_sha_append_sg(ctx);
		}
	}

	return err;
}

static int csky_sha_final_req(struct csky_sha_dev *dd)
{
	struct ahash_request   *req = dd->req;
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	int err = 0;
	int bufcnt;

	if (ctx->total != 0)
		csky_sha_append_sg(ctx);

	csky_sha_fill_padding(ctx);

	bufcnt = ctx->bufcnt;
	ctx->bufcnt = 0;
	err = csky_sha_xmit_cpu(dd, ctx->buffer, bufcnt, 1);

	dd->flags |= SHA_FLAGS_OUTPUT_READY;

	tasklet_schedule(&dd->done_task);

	return err;
}

static void csky_sha_copy_hash(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	uint32_t *hash = (uint32_t *)ctx->digest;
	uint32_t hashsize;

	switch (ctx->flags & SHA_FLAGS_ALGO_MASK) {
	case SHA_FLAGS_SHA1:
		hashsize = SHA1_DIGEST_SIZE;
		break;
	case SHA_FLAGS_SHA224:
		hashsize = SHA224_DIGEST_SIZE;
		break;
	case SHA_FLAGS_SHA256:
		hashsize = SHA256_DIGEST_SIZE;
		break;
	case SHA_FLAGS_SHA384:
		hashsize = SHA384_DIGEST_SIZE;
		break;
	case SHA_FLAGS_SHA512:
		hashsize = SHA512_DIGEST_SIZE;
		break;
	default:
		hashsize = SHA1_DIGEST_SIZE;
		break;
	}

	csky_sha_get_data(ctx->dd, hash, hashsize >> 2);
}

static void csky_sha_copy_ready_hash(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);

	if (!req->result)
		return;

	if (ctx->flags & SHA_FLAGS_SHA1)
		memcpy(req->result, ctx->digest, SHA1_DIGEST_SIZE);
	else if (ctx->flags & SHA_FLAGS_SHA224)
		memcpy(req->result, ctx->digest, SHA224_DIGEST_SIZE);
	else if (ctx->flags & SHA_FLAGS_SHA256)
		memcpy(req->result, ctx->digest, SHA256_DIGEST_SIZE);
	else if (ctx->flags & SHA_FLAGS_SHA384)
		memcpy(req->result, ctx->digest, SHA384_DIGEST_SIZE);
	else
		memcpy(req->result, ctx->digest, SHA512_DIGEST_SIZE);
}

static int csky_sha_finish(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	struct csky_sha_dev 	*dd = ctx->dd;

	csky_sha_copy_ready_hash(req);

	dev_dbg(dd->dev, "digcnt: 0x%llx, bufcnt: %d\n", ctx->digcnt, ctx->bufcnt);

	return 0;
}

static void csky_sha_finish_req(struct ahash_request *req, int err)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	struct csky_sha_dev 	*dd = ctx->dd;

	if (!err) {
		csky_sha_copy_hash(req);
		if (SHA_FLAGS_FINAL & dd->flags)
			err = csky_sha_finish(req);
	} else
		ctx->flags |= SHA_FLAGS_ERROR;

	dd->flags &= ~(SHA_FLAGS_BUSY | SHA_FLAGS_FINAL | SHA_FLAGS_CPU |
		       SHA_FLAGS_OUTPUT_READY);

	if (req->base.complete)
		req->base.complete(&req->base, err);
}

static int csky_sha_handle_queue(struct csky_sha_dev *dd,
				 struct ahash_request *req)
{
	struct crypto_async_request *async_req, *backlog;
	struct csky_sha_reqctx *ctx;
	unsigned long flags;
	int err = 0, ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&dd->queue, req);

	if (SHA_FLAGS_BUSY & dd->flags) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}

	backlog = crypto_get_backlog(&dd->queue);
	async_req = crypto_dequeue_request(&dd->queue);
	if (async_req)
		dd->flags |= SHA_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req = ahash_request_cast(async_req);
	dd->req = req;
	ctx = ahash_request_ctx(req);

	dev_dbg(dd->dev, "handling new req, op: %lu, nbytes: %d\n",
						ctx->op, req->nbytes);

	if (ctx->op == SHA_OP_UPDATE) {
		err = csky_sha_update_req(dd);
		if (err != -EINPROGRESS && (ctx->flags & SHA_FLAGS_FINUP))
			err = csky_sha_final_req(dd);
		else {
			dd->flags &= ~SHA_FLAGS_BUSY;
			return 0;
		}
	} else if (ctx->op == SHA_OP_FINAL)
		err = csky_sha_final_req(dd);

	if (err != -EINPROGRESS)
		csky_sha_finish_req(req, err);

	dev_dbg(dd->dev, "exit, err: %d\n", err);

	return ret;
}

static int csky_sha_enqueue(struct ahash_request *req, unsigned int op)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	struct csky_sha_ctx   *tctx = crypto_tfm_ctx(req->base.tfm);
	struct csky_sha_dev     *dd = tctx->dd;

	ctx->op = op;

	return csky_sha_handle_queue(dd, req);
}

static int csky_sha_update(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);

	if (!req->nbytes)
		return 0;

	ctx->total  = req->nbytes;
	ctx->sg	    = req->src;
	ctx->offset = 0;

	if (ctx->flags & SHA_FLAGS_FINUP)
		ctx->flags |= SHA_FLAGS_CPU;
	else if (ctx->bufcnt + ctx->total < ctx->buflen) {
		ctx->digcnt += ctx->total;
		csky_sha_append_sg(ctx);
		return 0;
	}

	return csky_sha_enqueue(req, SHA_OP_UPDATE);
}

static int csky_sha_final(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);

	ctx->flags |= SHA_FLAGS_FINUP;

	if (ctx->flags & SHA_FLAGS_ERROR)
		return 0;

	return csky_sha_enqueue(req, SHA_OP_FINAL);
}

static int csky_sha_finup(struct ahash_request *req)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);
	int err1, err2;

	ctx->flags |= SHA_FLAGS_FINUP;

	err1 = csky_sha_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	err2 = csky_sha_final(req);

	return err1 ?: err2;
}

static int csky_sha_digest(struct ahash_request *req)
{
	return csky_sha_init(req) ?: csky_sha_finup(req);
}

static int csky_sha_export(struct ahash_request *req, void *out)
{
	const struct csky_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(out, ctx, sizeof(*ctx));
	return 0;
}

static int csky_sha_import(struct ahash_request *req, const void *in)
{
	struct csky_sha_reqctx *ctx = ahash_request_ctx(req);

	memcpy(ctx, in, sizeof(*ctx));
	return 0;
}

static int csky_sha_cra_init(struct crypto_tfm *tfm)
{
	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_sha_reqctx));

	return 0;
}

static struct ahash_alg sha_1_256_algs[] = {
	{
		.init	= csky_sha_init,
		.update	= csky_sha_update,
		.final	= csky_sha_final,
		.finup	= csky_sha_finup,
		.digest = csky_sha_digest,
		.export = csky_sha_export,
		.import = csky_sha_import,
		.halg = {
			.digestsize = SHA1_DIGEST_SIZE,
			.statesize  = sizeof(struct csky_sha_reqctx),
			.base = {
				.cra_name	 = "sha1",
				.cra_driver_name = "csky-sha1",
				.cra_priority	 = 100,
				.cra_flags	 = CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA1_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct csky_sha_ctx),
				.cra_alignmask	 = 0,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = csky_sha_cra_init,
			}
		}
	},
	{
		.init	= csky_sha_init,
		.update	= csky_sha_update,
		.final	= csky_sha_final,
		.finup	= csky_sha_finup,
		.digest	= csky_sha_digest,
		.export	= csky_sha_export,
		.import	= csky_sha_import,
		.halg = {
			.digestsize = SHA256_DIGEST_SIZE,
			.statesize  = sizeof(struct csky_sha_reqctx),
			.base = {
				.cra_name	 = "sha256",
				.cra_driver_name = "csky-sha256",
				.cra_priority	 = 100,
				.cra_flags	 = CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA256_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct csky_sha_ctx),
				.cra_alignmask	 = 0,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = csky_sha_cra_init,
			}
		}
	},
};

static struct ahash_alg sha_224_alg = {
	.init   = csky_sha_init,
	.update	= csky_sha_update,
	.final	= csky_sha_final,
	.finup	= csky_sha_finup,
	.digest	= csky_sha_digest,
	.export	= csky_sha_export,
	.import	= csky_sha_import,
	.halg = {
		.digestsize = SHA224_DIGEST_SIZE,
		.statesize  = sizeof(struct csky_sha_reqctx),
		.base   = {
			.cra_name	 = "sha224",
			.cra_driver_name = "csky-sha224",
			.cra_priority	 = 100,
			.cra_flags	 = CRYPTO_ALG_ASYNC,
			.cra_blocksize	 = SHA224_BLOCK_SIZE,
			.cra_ctxsize     = sizeof(struct csky_sha_ctx),
			.cra_alignmask	 = 0,
			.cra_module	 = THIS_MODULE,
			.cra_init	 = csky_sha_cra_init,
		}
	}
};

static struct ahash_alg sha_384_512_algs[] = {
	{
		.init	= csky_sha_init,
		.update	= csky_sha_update,
		.final  = csky_sha_final,
		.finup	= csky_sha_finup,
		.digest	= csky_sha_digest,
		.export	= csky_sha_export,
		.import	= csky_sha_import,
		.halg = {
			.digestsize = SHA384_DIGEST_SIZE,
			.statesize  = sizeof(struct csky_sha_reqctx),
			.base = {
				.cra_name	 = "sha384",
				.cra_driver_name = "csky-sha384",
				.cra_priority	 = 100,
				.cra_flags	 = CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA384_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct csky_sha_ctx),
				.cra_alignmask	 = 0x3,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = csky_sha_cra_init,
			}
		}
	},
	{
		.init   = csky_sha_init,
		.update	= csky_sha_update,
		.final	= csky_sha_final,
		.finup  = csky_sha_finup,
		.digest	= csky_sha_digest,
		.export	= csky_sha_export,
		.import	= csky_sha_import,
		.halg = {
			.digestsize = SHA512_DIGEST_SIZE,
			.statesize  = sizeof(struct csky_sha_reqctx),
			.base = {
				.cra_name	 = "sha512",
				.cra_driver_name = "csky-sha512",
				.cra_priority	 = 100,
				.cra_flags	 = CRYPTO_ALG_ASYNC,
				.cra_blocksize	 = SHA512_BLOCK_SIZE,
				.cra_ctxsize	 = sizeof(struct csky_sha_ctx),
				.cra_alignmask	 = 0x3,
				.cra_module	 = THIS_MODULE,
				.cra_init	 = csky_sha_cra_init,
			}
		}
	},
};

static void csky_sha_done_task(unsigned long data)
{
	struct csky_sha_dev *dd = (struct csky_sha_dev *)data;

	if (SHA_FLAGS_CPU & dd->flags) {
		if (SHA_FLAGS_OUTPUT_READY & dd->flags)
			dd->flags &= ~SHA_FLAGS_OUTPUT_READY;
	}

	return;
}

static void csky_sha_unregister_algs(struct csky_sha_dev *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sha_1_256_algs); i++)
		crypto_unregister_ahash(&sha_1_256_algs[i]);

	crypto_unregister_ahash(&sha_224_alg);

	for (i = 0; i < ARRAY_SIZE(sha_384_512_algs); i++)
		crypto_unregister_ahash(&sha_384_512_algs[i]);
}

static int csky_sha_register_algs(struct csky_sha_dev *dd)
{
	int err, i, j;

	for (i = 0; i < ARRAY_SIZE(sha_1_256_algs); i++) {
		err = crypto_register_ahash(&sha_1_256_algs[i]);
		if (err)
			goto err_sha_1_256_algs;
	}

	err = crypto_register_ahash(&sha_224_alg);
	if (err)
		goto err_sha_224_algs;

	for (i = 0; i < ARRAY_SIZE(sha_384_512_algs); i++) {
		err = crypto_register_ahash(&sha_384_512_algs[i]);
		if (err)
			goto err_sha_384_512_algs;
	}

	return 0;

err_sha_384_512_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&sha_384_512_algs[j]);
	crypto_unregister_ahash(&sha_224_alg);

err_sha_224_algs:
	i = ARRAY_SIZE(sha_1_256_algs);
err_sha_1_256_algs:
	for (j = 0; j < i; j++)
		crypto_unregister_ahash(&sha_1_256_algs[j]);

	return err;
}

static int csky_sha_probe(struct platform_device *pdev)
{
	struct csky_sha_dev *sha_dd;
	struct device *dev = &pdev->dev;
	struct resource *sha_res;
	int err;

	sha_dd = devm_kzalloc(&pdev->dev, sizeof(*sha_dd), GFP_KERNEL);
	if (sha_dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		err = -ENOMEM;
		goto sha_dd_err;
	}

	sha_dd->dev = dev;

	platform_set_drvdata(pdev, sha_dd);

	INIT_LIST_HEAD(&sha_dd->list);
	spin_lock_init(&sha_dd->lock);

	tasklet_init(&sha_dd->done_task, csky_sha_done_task,
					(unsigned long)sha_dd);

	crypto_init_queue(&sha_dd->queue, CSKY_SHA_QUEUE_LENGTH);

	sha_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!sha_res) {
		dev_err(dev, "no MEM resource info\n");
		err = -ENODEV;
		goto res_err;
	}

	sha_dd->io_base = devm_ioremap_resource(&pdev->dev, sha_res);
	if (IS_ERR(sha_dd->io_base)) {
		dev_err(dev, "can't ioremap\n");
		err = PTR_ERR(sha_dd->io_base);
		goto res_err;
	}

	spin_lock(&csky_sha.lock);
	list_add_tail(&sha_dd->list, &csky_sha.dev_list);
	spin_unlock(&csky_sha.lock);

	err = csky_sha_register_algs(sha_dd);
	if (err)
		goto err_algs;

	dev_info(dev, "CSKY SHA Driver Initialized\n");

	return 0;

err_algs:
	spin_lock(&csky_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&csky_sha.lock);

res_err:
	tasklet_kill(&sha_dd->done_task);
sha_dd_err:
	dev_err(dev, "initialization failed.\n");

	return err;
}

static int csky_sha_remove(struct platform_device *pdev)
{
	static struct csky_sha_dev *sha_dd;

	sha_dd = platform_get_drvdata(pdev);
	if (!sha_dd)
		return -ENODEV;

	spin_lock(&csky_sha.lock);
	list_del(&sha_dd->list);
	spin_unlock(&csky_sha.lock);

	csky_sha_unregister_algs(sha_dd);

	tasklet_kill(&sha_dd->done_task);

	return 0;
}

static const struct of_device_id csky_sha_dt_ids[] = {
	{ .compatible = "csky,sha-v2" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, csky_sha_dt_ids);

static struct platform_driver csky_sha_driver = {
	.probe	  = csky_sha_probe,
	.remove	 = csky_sha_remove,
	.driver	 = {
		.name = "csky-sha",
		.of_match_table = of_match_ptr(csky_sha_dt_ids),
	},
};

module_platform_driver(csky_sha_driver);

MODULE_DESCRIPTION("CSKY SHA (1/256/224/384/512) hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Cui <xiaoxia_cui@c-sky.com>");
