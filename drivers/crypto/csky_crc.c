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

#include <linux/err.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/crypto.h>
#include <linux/cryptohash.h>
#include <crypto/algapi.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <asm/unaligned.h>
#include "csky_crc.h"

#define CRC_CCRYPTO_QUEUE_LENGTH	5

#define CHKSUM_BLOCK_SIZE		4
#define CHKSUM32_DIGEST_SIZE		4
#define CHKSUM16_DIGEST_SIZE		2
#define CHKSUM8_DIGEST_SIZE		1
#define CHKSUM_DIGEST_SIZE		CHKSUM32_DIGEST_SIZE

#define CRC_CRYPTO_STATE_UPDATE		1
#define CRC_CRYPTO_STATE_FINALUPDATE	2
#define CRC_CRYPTO_STATE_FINISH		3

struct csky_crc_reqctx {
	u32 dummy;
};

struct csky_crypto_crc {
	struct list_head		list;
	struct device			*dev;
	spinlock_t			lock;

	struct crc_register __iomem	*regs;
	struct ahash_request		*req;
	struct tasklet_struct		done_task;
	struct crypto_queue		queue;

	u8				busy;
};

static struct csky_crypto_crc_list {
	struct list_head dev_list;
	spinlock_t	 lock;
};

static struct csky_crypto_crc_list crc_list = {
	.dev_list = LIST_HEAD_INIT(crc_list.dev_list),
	.lock	  = __SPIN_LOCK_UNLOCKED(crc_list.lock),
};

struct csky_crypto_crc_reqctx {
	struct csky_crypto_crc *crc;

	u32	total;
	size_t	bufnext_len;
	u8	bufnext[CHKSUM_DIGEST_SIZE];
	u8	flag;
};

struct csky_crypto_crc_ctx {
	struct csky_crypto_crc *crc;

	u32	key;
	u32	sel;
	crc_mod_e mod;
	crc_std_e std;
};

static struct scatterlist *sg_get(struct scatterlist *sg_list,
				  unsigned int nents,
				  unsigned int index)
{
	struct scatterlist *sg = NULL;
	int i;

	for_each_sg(sg_list, sg, nents, i)
		if (i == index)
			break;

	return sg;
}

static int csky_crypto_crc_init_hw(struct csky_crypto_crc *crc,
				   struct csky_crypto_crc_ctx *ctx)
{
	writel(ctx->sel, &crc->regs->sel);
	writel(ctx->key, &crc->regs->init);

	return 0;
}

static int csky_crypto_crc_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct csky_crypto_crc_ctx *crc_ctx = crypto_ahash_ctx(tfm);
	struct csky_crypto_crc_reqctx *ctx  = ahash_request_ctx(req);
	struct csky_crypto_crc *crc;

	spin_lock_bh(&crc_list.lock);
	list_for_each_entry(crc, &crc_list.dev_list, list) {
		crc_ctx->crc = crc;
		break;
	}
	spin_unlock_bh(&crc_list.lock);

	ctx->crc	 = crc;
	ctx->bufnext_len = 0;
	ctx->total	 = 0;
	ctx->flag	 = 0;

	return csky_crypto_crc_init_hw(crc, crc_ctx);
}

static int csky_crypto_crc_handle_sg(struct csky_crypto_crc *crc)
{
	struct ahash_request *req = crc->req;
	struct csky_crypto_crc_reqctx *ctx = ahash_request_ctx(req);
	void  *sg_src;
	size_t sg_len;
	struct scatterlist *sg;
	int nsg, i, j;

	nsg = sg_nents(req->src);
	for (i = 0; i < nsg; i++) {
		sg = sg_get(req->src, nsg, i);
		sg_src = sg_virt(sg);
		sg_len = sg_dma_len(sg);

		if (ctx->bufnext_len + sg_len <  CHKSUM_DIGEST_SIZE) {
			memcpy(ctx->bufnext + ctx->bufnext_len,
				sg_src, sg_len);
			ctx->bufnext_len += sg_len;
			continue;
		}

		memcpy(ctx->bufnext + ctx->bufnext_len, sg_src,
		       CHKSUM_DIGEST_SIZE - ctx->bufnext_len);
		writel(*(u32 *)ctx->bufnext, &crc->regs->data);

		sg_src += CHKSUM_DIGEST_SIZE - ctx->bufnext_len;
		sg_len -= CHKSUM_DIGEST_SIZE - ctx->bufnext_len;

		for (j = 0; j < sg_len / CHKSUM_DIGEST_SIZE; j++) {
			memcpy(ctx->bufnext, sg_src, CHKSUM_DIGEST_SIZE);
			writel(*(u32 *)ctx->bufnext, &crc->regs->data);
			sg_src += CHKSUM_DIGEST_SIZE;
		}

		ctx->bufnext_len = sg_len % CHKSUM_DIGEST_SIZE;
		memcpy(ctx->bufnext, sg_src, ctx->bufnext_len);
	}
	return 0;
}

static int csky_crypto_crc_handle(struct csky_crypto_crc *crc)
{
	struct ahash_request *req = crc->req;
	struct csky_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	if (ctx->flag == CRC_CRYPTO_STATE_FINISH) {
		memset(ctx->bufnext + ctx->bufnext_len, 0,
			CHKSUM_DIGEST_SIZE - ctx->bufnext_len);
		writel(*(u32 *)ctx->bufnext, &crc->regs->data);
	} else if (ctx->flag == CRC_CRYPTO_STATE_FINALUPDATE) {
		if (ctx->bufnext_len + req->nbytes < CHKSUM_DIGEST_SIZE) {
			memcpy(ctx->bufnext + ctx->bufnext_len,
				sg_virt(req->src), req->nbytes);
			ctx->bufnext_len += req->nbytes;

			memset(ctx->bufnext + ctx->bufnext_len, 0,
				CHKSUM_DIGEST_SIZE - ctx->bufnext_len);
			writel(*(u32 *)ctx->bufnext, &crc->regs->data);
		} else {
			csky_crypto_crc_handle_sg(crc);
			if (ctx->bufnext_len) {
				memset(ctx->bufnext + ctx->bufnext_len, 0,
					CHKSUM_DIGEST_SIZE - ctx->bufnext_len);
				writel(*(u32 *)ctx->bufnext, &crc->regs->data);
			}
		}
	} else if (ctx->flag == CRC_CRYPTO_STATE_UPDATE) {
		csky_crypto_crc_handle_sg(crc);
	} else {
		return -EINVAL;
	}

	put_unaligned_le32(readl(&crc->regs->data), req->result);

	crc->busy = 0;
	if (req->base.complete)
		req->base.complete(&req->base, 0);

	tasklet_schedule(&crc->done_task);

	return 0;
}

static int csky_crypto_crc_handle_queue(struct csky_crypto_crc *crc,
					struct ahash_request *req)
{
	struct crypto_async_request   *async_req, *backlog;
	struct csky_crypto_crc_reqctx *ctx;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&crc->lock, flags);
	if (req)
		ret = ahash_enqueue_request(&crc->queue, req);
	if (crc->busy) {
		spin_unlock_irqrestore(&crc->lock, flags);
		return ret;
	}
	backlog   = crypto_get_backlog(&crc->queue);
	async_req = crypto_dequeue_request(&crc->queue);
	if (async_req)
		crc->busy = 1;
	spin_unlock_irqrestore(&crc->lock, flags);

	if (!async_req)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	req	  = ahash_request_cast(async_req);
	crc->req = req;
	ctx	  = ahash_request_ctx(req);

	dev_dbg(crc->dev, "handling new req, flag=%u, nbytes: %d\n",
		ctx->flag, req->nbytes);

	if (ctx->flag == CRC_CRYPTO_STATE_FINISH) {
		if (ctx->bufnext_len == 0) {
			crc->busy = 0;
			return 0;
		}
	} else if (ctx->flag == CRC_CRYPTO_STATE_UPDATE) {
		if (ctx->bufnext_len + req->nbytes < CHKSUM_DIGEST_SIZE) {
			memcpy(ctx->bufnext + ctx->bufnext_len,
				sg_virt(req->src), req->nbytes);
			ctx->bufnext_len += req->nbytes;

			crc->busy = 0;
			return 0;
		}
	}

	return csky_crypto_crc_handle(crc);
}

static int csky_crypto_crc_update(struct ahash_request *req)
{
	struct csky_crypto_crc_reqctx *ctx = ahash_request_ctx(req);

	dev_dbg(ctx->crc->dev, "crc_update\n");
	if (!req->nbytes)
		return 0;

	ctx->total += req->nbytes;
	ctx->flag   = CRC_CRYPTO_STATE_UPDATE;

	return csky_crypto_crc_handle_queue(ctx->crc, req);
}

static int csky_crypto_crc_final(struct ahash_request *req)
{
	struct csky_crypto_crc_reqctx *ctx  = ahash_request_ctx(req);

	dev_dbg(ctx->crc->dev, "crc_final\n");
	ctx->flag = CRC_CRYPTO_STATE_FINISH;

	return csky_crypto_crc_handle_queue(ctx->crc, req);
}

static int csky_crypto_crc_finup(struct ahash_request *req)
{
	struct csky_crypto_crc_reqctx *ctx  = ahash_request_ctx(req);

	dev_dbg(ctx->crc->dev, "crc_finishupdate\n");

	ctx->total += req->nbytes;
	ctx->flag   = CRC_CRYPTO_STATE_FINALUPDATE;

	return csky_crypto_crc_handle_queue(ctx->crc, req);
}

static int csky_crypto_crc_digest(struct ahash_request *req)
{
	int ret;

	ret = csky_crypto_crc_init(req);
	if (ret)
		return ret;

	return csky_crypto_crc_finup(req);
}

static int csky_crypto_crc_cra_init(struct csky_crypto_crc_ctx *ctx)
{
	int ret = 0;

	if (ctx->mod == MOD_CRC16) {
		switch (ctx->std) {
		case STD_MODBUS:
			ctx->sel = 0x0;
			ctx->key = 0xFFFF;
			break;
		case STD_IBM:
			ctx->sel = 0x0;
			ctx->key = 0x0;
			break;
		case STD_MAXIM:
			ctx->sel = 0x4;
			ctx->key = 0x0;
			break;
		case STD_USB:
			ctx->sel = 0x4;
			ctx->key = 0xFFFF;
			break;
		case STD_CCITT:
			ctx->sel = 0x1;
			ctx->key = 0x0;
			break;
		case STD_X25:
			ctx->sel = 0x5;
			ctx->key = 0xFFFF;
			break;
		default:
			ret = -EINVAL;
		}
	} else if (ctx->mod == MOD_CRC8) {
		switch (ctx->std) {
		case STD_MAXIM:
			ctx->sel = 0x2;
			ctx->key = 0x0;
			break;
		case STD_ROHC:
			ctx->sel = 0x3;
			ctx->key = 0xff;
			break;
		default:
			ret = -EINVAL;
		}
	} else
		ret = -EINVAL;

	return ret;
}

static int csky_crypto_crc8maxim_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC8;
	crc_ctx->std = STD_MAXIM;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc8rohc_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC8;
	crc_ctx->std = STD_ROHC;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16ibm_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_IBM;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16maxim_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_MAXIM;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16modbus_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_MODBUS;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16usb_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_USB;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16x25_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_X25;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static int csky_crypto_crc16ccitt_cra_init(struct crypto_tfm *tfm)
{
	struct csky_crypto_crc_ctx *crc_ctx = crypto_tfm_ctx(tfm);
	int ret = 0;

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct csky_crypto_crc_reqctx));

	crc_ctx->mod = MOD_CRC16;
	crc_ctx->std = STD_CCITT;
	ret = csky_crypto_crc_cra_init(crc_ctx);
	if (ret)
		dev_err(crc_ctx->crc->dev, "crc alg argument invalid \n");

	return ret;
}

static struct ahash_alg crc_algs[] = {
	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM8_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc8_rohc",
			.cra_driver_name= "csky-crc8-rohc",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc8rohc_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM8_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc8_maxim",
			.cra_driver_name= "csky-crc8-maxim",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc8maxim_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_ibm",
			.cra_driver_name= "csky-crc16-ibm",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16ibm_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_maxim",
			.cra_driver_name= "csky-crc16-maxim",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16maxim_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_modbus",
			.cra_driver_name= "csky-crc16-modbus",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16modbus_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_usb",
			.cra_driver_name= "csky-crc16-usb",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16usb_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_ccitt",
			.cra_driver_name= "csky-crc16-ccitt",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16ccitt_cra_init
		}
	},

	{
		.init			= csky_crypto_crc_init,
		.update			= csky_crypto_crc_update,
		.final			= csky_crypto_crc_final,
		.finup			= csky_crypto_crc_finup,
		.digest			= csky_crypto_crc_digest,
		.halg.digestsize	= CHKSUM16_DIGEST_SIZE,
		.halg.statesize		= sizeof(struct csky_crc_reqctx),
		.halg.base  = {
			.cra_name	= "crc16_x25",
			.cra_driver_name= "csky-crc16-x25",
			.cra_priority	= 100,
			.cra_flags	= CRYPTO_ALG_ASYNC,
			.cra_blocksize  = CHKSUM_BLOCK_SIZE,
			.cra_ctxsize	= sizeof(struct csky_crypto_crc_ctx),
			.cra_alignmask  = 0,
			.cra_module	= THIS_MODULE,
			.cra_init	= csky_crypto_crc16x25_cra_init
		}
	},
};

static void csky_crypto_crc_done_task(unsigned long data)
{
	struct csky_crypto_crc *crc = (struct csky_crypto_crc *)data;

	csky_crypto_crc_handle_queue(crc, NULL);
}

static int csky_crypto_crc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct csky_crypto_crc *crc;
	int ret;
	int i, j;

	crc = devm_kzalloc(dev, sizeof(*crc), GFP_KERNEL);
	if (!crc) {
		dev_err(&pdev->dev, "fail to malloc csky_crypto_crc\n");
		return -ENOMEM;
	}

	crc->dev = dev;

	platform_set_drvdata(pdev, crc);

	INIT_LIST_HEAD(&crc->list);
	spin_lock_init(&crc->lock);

	tasklet_init(&crc->done_task,
		     csky_crypto_crc_done_task,
		     (unsigned long)crc);
	crypto_init_queue(&crc->queue, CRC_CCRYPTO_QUEUE_LENGTH);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	crc->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR((void *)crc->regs)) {
		ret = PTR_ERR(crc->regs);
		dev_err(&pdev->dev, "Cannot map CRC IO\n");
		goto _res_err;
	}

	spin_lock(&crc_list.lock);
	list_add(&crc->list, &crc_list.dev_list);
	spin_unlock(&crc_list.lock);

	if (list_is_singular(&crc_list.dev_list)) {
		for (i = 0; i < ARRAY_SIZE(crc_algs); i++) {
			ret = crypto_register_ahash(&crc_algs[i]);
			if (ret) {
				for (j = 0; j < i; j++) {
					crypto_unregister_ahash(&crc_algs[j]);
				}
				dev_err(&pdev->dev,
					"Can't register crypto ahash device\n");
				goto _reg_err;
			}
		}
	}

	dev_info(&pdev->dev, "CSKY CRC driver initialized\n");

	return 0;

_res_err:
	tasklet_kill(&crc->done_task);
_reg_err:
	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	spin_unlock(&crc_list.lock);

	return ret;
}

static int csky_crypto_crc_remove(struct platform_device *pdev)
{
	struct csky_crypto_crc *crc = platform_get_drvdata(pdev);
	int i;

	if (!crc)
		return -ENODEV;

	spin_lock(&crc_list.lock);
	list_del(&crc->list);
	spin_unlock(&crc_list.lock);

	for (i = 0; i < ARRAY_SIZE(crc_algs); i++)
		crypto_unregister_ahash(&crc_algs[i]);

	tasklet_kill(&crc->done_task);

	return 0;
}

static const struct of_device_id csky_crc_dt_ids[] = {
	{ .compatible = "csky,csky-crc" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, csky_crc_dt_ids);

static struct platform_driver csky_crypto_crc_driver = {
	.probe	= csky_crypto_crc_probe,
	.remove	= csky_crypto_crc_remove,
	.driver	= {
		.name = "csky-crc",
		.of_match_table = of_match_ptr(csky_crc_dt_ids),
	},
};

module_platform_driver(csky_crypto_crc_driver);

MODULE_AUTHOR("Vincent Cui <xiaoxia_cui@c-sky.com>");
MODULE_DESCRIPTION("CSKY CRC hardware crypto driver");
MODULE_LICENSE("GPL");
