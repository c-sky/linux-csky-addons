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
#include <crypto/algapi.h>
#include <crypto/des.h>
#include "csky_tdes.h"

#define CSKY_TDES_BUFFER_ORDER	2
#define CSKY_TDES_BUFFER_SIZE	(PAGE_SIZE << CSKY_TDES_BUFFER_ORDER)

#define TDES_FLAGS_ENC		BIT(0)
#define TDES_FLAGS_DEC		BIT(1)
#define TDES_FLAGS_ECB		BIT(2)
#define TDES_FLAGS_CBC		BIT(3)

#define TDES_FLAGS_INIT		BIT(8)
#define TDES_FLAGS_BUSY		BIT(9)

#define CSKY_TDES_QUEUE_LENGTH	10

#define SIZE_IN_WORDS(x) 	(x>>2)

#define HTOL(x) ((x & 0xff) << 24 | (x & 0xff00) << 8 | \
		 (x & 0xff0000) >> 8 | (x & 0xff000000) >> 24)

struct csky_tdes_dev;

struct csky_tdes_base_ctx {
	struct csky_tdes_dev *dd;
	int keylen;
	u32 key[3*DES_KEY_SIZE / sizeof(u32)];
	u32 block_size;
};

struct csky_tdes_ctx {
	struct csky_tdes_base_ctx base;
};

struct csky_tdes_ctr_ctx {
	struct csky_tdes_base_ctx base;
	u32 iv[DES_BLOCK_SIZE / sizeof(u32)];
};

struct csky_tdes_reqctx {
	unsigned long mode;
};

struct csky_tdes_dev {
	struct list_head		list;
	struct crypto_async_request	*areq;
	struct csky_tdes_base_ctx	*ctx;
	struct device			*dev;
	struct tdes_reg __iomem		*reg_base;
	struct tasklet_struct		done_task;

	struct crypto_queue 		queue;
	struct scatterlist 		*real_dst;
	unsigned long			flags;
	spinlock_t			lock;
	size_t				total;
	size_t				datalen;
	u32				*data;
	size_t				buflen;
	void				*buf;
};

struct csky_tdes_drv {
	struct list_head dev_list;
	spinlock_t	 lock;
};

static struct csky_tdes_drv csky_tdes = {
	.dev_list = LIST_HEAD_INIT(csky_tdes.dev_list),
	.lock	  = __SPIN_LOCK_UNLOCKED(csky_tdes.lock),
};

static struct csky_tdes_dev *csky_tdes_find_dev(struct csky_tdes_base_ctx *ctx)
{
	struct csky_tdes_dev *tdes_dd = NULL;
	struct csky_tdes_dev *tmp;

	spin_lock_bh(&csky_tdes.lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &csky_tdes.dev_list, list) {
			tdes_dd = tmp;
			break;
		}
		ctx->dd = tdes_dd;
	} else {
		tdes_dd = ctx->dd;
	}
	spin_unlock_bh(&csky_tdes.lock);

	return tdes_dd;
}

static inline void csky_tdes_setopcode(struct csky_tdes_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~0x0002;
	if (dd->flags & TDES_FLAGS_ENC)
		tmp |= TDES_OPC_ENC;
	else if (dd->flags & TDES_FLAGS_DEC)
		tmp |= TDES_OPC_DEC;

	tmp &= ~0x0010;
	if (dd->flags & TDES_FLAGS_CBC)
		tmp |= TDES_MOD_CBC;
	else
		tmp |= TDES_MOD_ECB;

	tmp |= TDES_OPR_DES3;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_tdes_enable(struct csky_tdes_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp |= 1;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_tdes_disable(struct csky_tdes_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~1;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_tdes_set_endian(struct csky_tdes_dev *dd,
					uint32_t endian)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	if (endian == TDES_ENDIAN_LT)
		tmp &= ~TDES_ENDIAN;
	else
		tmp |= TDES_ENDIAN;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_tdes_init(struct csky_tdes_dev *dd)
{
#ifndef USING_BIG_ENDIAN
	csky_tdes_set_endian(dd, TDES_ENDIAN_LT);
#endif
	if (!(dd->flags & TDES_FLAGS_INIT))
		dd->flags |= TDES_FLAGS_INIT;
}

static inline int csky_tdes_check_int_status(struct csky_tdes_dev *dd,
					     uint32_t flag)
{
	return (readl_relaxed(&dd->reg_base->state) & flag) ? 1 : 0;
}

static inline size_t csky_tdes_padlen(size_t len, size_t block_size)
{
	len &= block_size - 1;
	return len ? block_size - len : 0;
}

static inline void csky_tdes_in_block(struct csky_tdes_dev *dd,
				      uint32_t *data)
{
	int i;

	for (i = 0; i < SIZE_IN_WORDS(DES_BLOCK_SIZE); i ++) {
		writel_relaxed(
			HTOL(data[i]),
			&dd->reg_base->datain[SIZE_IN_WORDS(DES_BLOCK_SIZE)
					      - 1 - i]);
	}
}

static inline void csky_tdes_out_block(struct csky_tdes_dev *dd,
				       uint32_t *data)
{
	int i;

	for (i = 0; i < SIZE_IN_WORDS(DES_BLOCK_SIZE); i ++) {
		data[i] = HTOL(readl_relaxed(
			&dd->reg_base->dataout[SIZE_IN_WORDS(DES_BLOCK_SIZE)
						- 1 - i]));
	}
}

static inline int csky_tdes_complete(struct csky_tdes_dev *dd, int err)
{
	dd->flags &= ~TDES_FLAGS_BUSY;
	dd->areq->complete(dd->areq, err);

	tasklet_schedule(&dd->done_task);

	return err;
}

static int csky_tdes_engine_op(struct csky_tdes_dev *dd)
{
	int i;
	int err = 0;
	int len;

	for (i = 0; i < dd->datalen; i += DES_BLOCK_SIZE) {
		csky_tdes_in_block(dd, dd->data);

		csky_tdes_enable(dd);
		csky_tdes_check_int_status(dd, TDES_IT_BUSY);
		csky_tdes_disable(dd);

		csky_tdes_out_block(dd, dd->data);
		dd->data += SIZE_IN_WORDS(DES_BLOCK_SIZE);
	}

	if (dd->flags & TDES_FLAGS_ENC)
		len = dd->datalen;
	else if (dd->flags & TDES_FLAGS_DEC)
		len = dd->total;
	else
		return csky_tdes_complete(dd, -EINVAL);

	if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
				 dd->buf, len))
		err = -EINVAL;

	return csky_tdes_complete(dd, err);
}

static int csky_tdes_start(struct csky_tdes_dev *dd,
			   struct scatterlist *src,
			   struct scatterlist *dst,
			   size_t len)
{
	size_t padlen = csky_tdes_padlen(len, DES_BLOCK_SIZE);

	if (!(dd->flags & TDES_FLAGS_INIT)) {
		return -EACCES;
	}

	if (unlikely(len == 0))
		return -EINVAL;

	sg_copy_to_buffer(src, sg_nents(src), dd->buf, len);

	dd->real_dst = dst;
	dd->total    = len;
	dd->datalen  = len + padlen;
	dd->data     = (u32 *)dd->buf;

	return 0;
}

static int csky_tdes_set_key(struct csky_tdes_dev *dd, const uint32_t *iv)
{
	int i;
	uint32_t *key = dd->ctx->key;

	for (i = 0; i < SIZE_IN_WORDS(dd->ctx->keylen); i++) {
		if (i < 2)
			writel_relaxed(HTOL(key[i]), &dd->reg_base->key[1-i%2]);
		else if (i < 4)
			writel_relaxed(HTOL(key[i]), &dd->reg_base->key[3-i%2]);
		else
			writel_relaxed(HTOL(key[i]), &dd->reg_base->key[5-i%2]);
	}

	if (dd->flags & TDES_FLAGS_CBC) {
		for (i = 0; i < SIZE_IN_WORDS(DES_BLOCK_SIZE); i++) {
			writel_relaxed(HTOL(
				iv[i]),
				&dd->reg_base->iv[SIZE_IN_WORDS(DES_BLOCK_SIZE)
						  -1-i]);
		}
	}

	csky_tdes_setopcode(dd);

	return 0;
}

static int csky_tdes_handle(struct csky_tdes_dev *dd)
{
	struct ablkcipher_request *req  = ablkcipher_request_cast(dd->areq);
	struct csky_tdes_reqctx   *rctx = ablkcipher_request_ctx(req);
	int ret;

	dd->flags &= ~(TDES_FLAGS_ECB | TDES_FLAGS_CBC | \
				   TDES_FLAGS_ENC | TDES_FLAGS_DEC);
	dd->flags |= rctx->mode;

	csky_tdes_init(dd);
	ret = csky_tdes_start(dd, req->src, req->dst, req->nbytes);
	if (ret)
		return ret;

	ret = csky_tdes_set_key(dd, req->info);
	if (ret)
		return ret;

	ret = csky_tdes_engine_op(dd);
	return ret;
}

static int csky_tdes_handle_queue(struct csky_tdes_dev *dd,
				  struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	struct csky_tdes_base_ctx	*ctx;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&dd->queue, new_areq);
	if (dd->flags & TDES_FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	areq = crypto_dequeue_request(&dd->queue);
	if (areq)
		dd->flags |= TDES_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx	 = crypto_tfm_ctx(areq->tfm);
	dd->areq = areq;
	dd->ctx  = ctx;

	return csky_tdes_handle(dd);
}

static int csky_tdes_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct csky_tdes_base_ctx *ctx;
	struct csky_tdes_reqctx   *rctx;
	struct csky_tdes_dev	  *dd;

	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	if (!ctx)
		return -ENOMEM;

	dd  = csky_tdes_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	rctx	   = ablkcipher_request_ctx(req);
	rctx->mode = mode;

	if ((mode & TDES_FLAGS_ECB) || (mode & TDES_FLAGS_CBC))
		ctx->block_size = DES_BLOCK_SIZE;

	return csky_tdes_handle_queue(dd, &req->base);
}

static int csky_tdes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
			    unsigned int keylen)
{
	struct csky_tdes_base_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	if ((keylen != 2*DES_KEY_SIZE) && (keylen != 3*DES_KEY_SIZE)) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int csky_tdes_ecb_encrypt(struct ablkcipher_request *req)
{
	return csky_tdes_crypt(req, TDES_FLAGS_ECB | TDES_FLAGS_ENC);
}

static int csky_tdes_ecb_decrypt(struct ablkcipher_request *req)
{
	return csky_tdes_crypt(req, TDES_FLAGS_ECB | TDES_FLAGS_DEC);
}

static int csky_tdes_cbc_encrypt(struct ablkcipher_request *req)
{
	return csky_tdes_crypt(req, TDES_FLAGS_CBC | TDES_FLAGS_ENC);
}

static int csky_tdes_cbc_decrypt(struct ablkcipher_request *req)
{
	return csky_tdes_crypt(req, TDES_FLAGS_CBC | TDES_FLAGS_DEC);
}

static int csky_tdes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct csky_tdes_reqctx);

	return 0;
}

static void csky_tdes_cra_exit(struct crypto_tfm *tfm)
{

}

static struct crypto_alg tdes_algs[] = {
	{
		.cra_name	= "ecb(des3_ede)",
		.cra_driver_name= "csky-ecb-tdes",
		.cra_priority	= 200,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize	= DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct csky_tdes_ctx),
		.cra_alignmask	= 0xf,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= csky_tdes_cra_init,
		.cra_exit	= csky_tdes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.setkey		= csky_tdes_setkey,
			.encrypt	= csky_tdes_ecb_encrypt,
			.decrypt	= csky_tdes_ecb_decrypt,
		}
	},
	{
		.cra_name	= "cbc(des3_ede)",
		.cra_driver_name= "csky-cbc-tdes",
		.cra_priority	= 200,
		.cra_flags	= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize  = DES_BLOCK_SIZE,
		.cra_ctxsize	= sizeof(struct csky_tdes_ctx),
		.cra_alignmask  = 0xf,
		.cra_type	= &crypto_ablkcipher_type,
		.cra_module	= THIS_MODULE,
		.cra_init	= csky_tdes_cra_init,
		.cra_exit	= csky_tdes_cra_exit,
		.cra_u.ablkcipher = {
			.min_keysize	= 2 * DES_KEY_SIZE,
			.max_keysize	= 3 * DES_KEY_SIZE,
			.ivsize		= DES_BLOCK_SIZE,
			.setkey		= csky_tdes_setkey,
			.encrypt	= csky_tdes_cbc_encrypt,
			.decrypt	= csky_tdes_cbc_decrypt,
		}
	},
};

static int csky_tdes_buff_init(struct csky_tdes_dev *dd)
{
	dd->buf = (void *)__get_free_pages(GFP_KERNEL, CSKY_TDES_BUFFER_ORDER);
	dd->buflen = CSKY_TDES_BUFFER_SIZE;
	dd->buflen &= ~(DES_BLOCK_SIZE - 1);

	if (!dd->buf) {
		dev_err(dd->dev, "unable to alloc pages.\n");
		return -ENOMEM;
	}

	return 0;
}

static void csky_tdes_buff_cleanup(struct csky_tdes_dev *dd)
{
	if ((unsigned long)dd->buf)
		free_pages((unsigned long)dd->buf, CSKY_TDES_BUFFER_ORDER);
}

static void csky_tdes_done_task(unsigned long data)
{
	struct csky_tdes_dev *dd = (struct csky_tdes_dev *)data;

	csky_tdes_handle_queue(dd, NULL);
}

static void csky_tdes_unregister_algs(struct csky_tdes_dev *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tdes_algs); i++)
		crypto_unregister_alg(&tdes_algs[i]);
}

static int csky_tdes_register_algs(struct csky_tdes_dev *dd)
{
	int err, i, j;

	for (i = 0; i < ARRAY_SIZE(tdes_algs); i++) {
		err = crypto_register_alg(&tdes_algs[i]);
		if (err) {
			for (j = 0; j < i; j++)
				crypto_unregister_alg(&tdes_algs[j]);
			return err;
		}
	}

	return 0;
}

static int csky_tdes_probe(struct platform_device *pdev)
{
	struct csky_tdes_dev *tdes_dd;
	struct device	   *dev = &pdev->dev;
	struct resource	 *tdes_res;
	int err;

	tdes_dd = devm_kzalloc(&pdev->dev, sizeof(*tdes_dd), GFP_KERNEL);
	if (tdes_dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		err = -ENOMEM;
		goto tdes_dd_err;
	}

	tdes_dd->dev = dev;

	platform_set_drvdata(pdev, tdes_dd);

	INIT_LIST_HEAD(&tdes_dd->list);
	spin_lock_init(&tdes_dd->lock);

	tasklet_init(&tdes_dd->done_task, csky_tdes_done_task,
		     (unsigned long)tdes_dd);

	crypto_init_queue(&tdes_dd->queue, CSKY_TDES_QUEUE_LENGTH);

	tdes_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!tdes_res) {
		err = -ENODEV;
		goto res_err;
	}

	tdes_dd->reg_base = devm_ioremap_resource(dev, tdes_res);
	if (IS_ERR(tdes_dd->reg_base)) {
		err = PTR_ERR(tdes_dd->reg_base);
		goto res_err;
	}

	err = csky_tdes_buff_init(tdes_dd);
	if (err)
		goto res_err;

	spin_lock(&csky_tdes.lock);
	list_add_tail(&tdes_dd->list, &csky_tdes.dev_list);
	spin_unlock(&csky_tdes.lock);

	err = csky_tdes_register_algs(tdes_dd);
	if (err)
		goto err_algs;

	dev_info(dev, "CSKY TDES Driver Initialized\n");

	return 0;

err_algs:
	spin_lock(&csky_tdes.lock);
	list_del(&tdes_dd->list);
	spin_unlock(&csky_tdes.lock);
res_err:
	tasklet_kill(&tdes_dd->done_task);
tdes_dd_err:

	return err;
}

static int csky_tdes_remove(struct platform_device *pdev)
{
	static struct csky_tdes_dev *tdes_dd;

	tdes_dd = platform_get_drvdata(pdev);
	if (!tdes_dd)
		return -ENODEV;

	spin_lock(&csky_tdes.lock);
	list_del(&tdes_dd->list);
	spin_unlock(&csky_tdes.lock);

	csky_tdes_buff_cleanup(tdes_dd);

	tasklet_kill(&tdes_dd->done_task);
	csky_tdes_unregister_algs(tdes_dd);

	return 0;
}

static const struct of_device_id csky_tdes_dt_ids[] = {
	{ .compatible = "csky,csky-tdes" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, csky_tdes_dt_ids);

static struct platform_driver csky_tdes_driver = {
	.probe	= csky_tdes_probe,
	.remove	= csky_tdes_remove,
	.driver	= {
		.name = "csky_tdes",
		.of_match_table = of_match_ptr(csky_tdes_dt_ids),
	},
};

module_platform_driver(csky_tdes_driver);

MODULE_DESCRIPTION("CSKY TDES hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Cui <xiaoxia_cui@c-sky.com>");
