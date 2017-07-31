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
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <linux/crypto.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include "csky_aes.h"

#define CSKY_AES_BUFFER_ORDER	2
#define CSKY_AES_BUFFER_SIZE	(PAGE_SIZE << CSKY_AES_BUFFER_ORDER)

#define AES_FLAGS_ENC		BIT(0)
#define AES_FLAGS_DEC		BIT(1)
#define AES_FLAGS_ECB		BIT(2)
#define AES_FLAGS_CBC		BIT(3)

#define AES_FLAGS_INIT		BIT(8)
#define AES_FLAGS_BUSY		BIT(9)

#define CSKY_AES_QUEUE_LENGTH	10

#define SIZE_IN_WORDS(x)	(x>>2)

#define HTOL(x)			((x & 0xff) << 24 | (x & 0xff00) << 8 | \
				 (x & 0xff0000) >> 8 | (x & 0xff000000) >> 24)

struct csky_aes_dev;

struct csky_aes_base_ctx {
	struct csky_aes_dev *dd;
	int keylen;
	u32 key[AES_KEYSIZE_256 / sizeof(u32)];
	u32 block_size;
};

struct csky_aes_ctx {
	struct csky_aes_base_ctx base;
};

struct csky_aes_reqctx {
	unsigned long	mode;
};

struct csky_aes_dev {
	struct list_head		list;
	struct crypto_async_request	*areq;
	struct csky_aes_base_ctx	*ctx;
	struct device			*dev;
	struct aes_reg __iomem		*reg_base;
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

struct csky_aes_drv {
	struct list_head	dev_list;
	spinlock_t		lock;
};

static struct csky_aes_drv csky_aes = {
	.dev_list = LIST_HEAD_INIT(csky_aes.dev_list),
	.lock	  = __SPIN_LOCK_UNLOCKED(csky_aes.lock),
};

static struct csky_aes_dev *csky_aes_find_dev(struct csky_aes_base_ctx *ctx)
{
	struct csky_aes_dev *aes_dd = NULL;
	struct csky_aes_dev *tmp;

	spin_lock_bh(&csky_aes.lock);
	if (!ctx->dd) {
		list_for_each_entry(tmp, &csky_aes.dev_list, list) {
			aes_dd = tmp;
			break;
		}
		ctx->dd = aes_dd;
	} else {
		aes_dd = ctx->dd;
	}
	spin_unlock_bh(&csky_aes.lock);

	return aes_dd;
}

static inline void csky_aes_setopcode(struct csky_aes_dev *dd, uint32_t opr)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~0x00c0;
	if (opr == AES_OPC_ENC)
		tmp |= AES_OPC_ENC << 6;
	else if (opr == AES_OPC_DEC)
		tmp |= AES_OPC_DEC << 6;
	else
		tmp |= AES_OPC_EXP << 6;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_config_mode(struct csky_aes_dev *dd, int cbc_mode)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~0x0008;
	tmp |= (cbc_mode) ? (1 << 3): 0;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_set_key_length(struct csky_aes_dev *dd, int keylen)
{
	uint32_t tmp;
	uint32_t key_len;

	if (keylen == AES_KEYSIZE_128)
		key_len = AES_KL_128;
	else if (keylen == AES_KEYSIZE_192)
		key_len = AES_KL_192;
	else
		key_len = AES_KL_256;

	tmp = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~0x0030;
	tmp |= key_len << 4;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_enable(struct csky_aes_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp |= 1;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_disable(struct csky_aes_dev *dd)
{
	uint32_t tmp;

	tmp  = readl_relaxed(&dd->reg_base->ctrl);
	tmp &= ~1;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_set_endian(struct csky_aes_dev *dd,
				       uint32_t endian)
{
	uint32_t tmp;

	tmp = readl_relaxed(&dd->reg_base->ctrl);
	if (endian == AES_ENDIAN_LT)
		tmp &= ~AES_ENDIAN;
	else
		tmp |= AES_ENDIAN;
	writel_relaxed(tmp, &dd->reg_base->ctrl);
}

static inline void csky_aes_init(struct csky_aes_dev *dd)
{
#ifdef __LITTLE_ENDIAN
	csky_aes_set_endian(dd, AES_ENDIAN_LT);
#else
	csky_aes_set_endian(dd, AES_ENDIAN_BG);
#endif
	if (!(dd->flags & AES_FLAGS_INIT))
		dd->flags |= AES_FLAGS_INIT;
}

static inline int csky_aes_check_int_status(struct csky_aes_dev *dd,
					    uint32_t flag)
{
	return (readl_relaxed(&dd->reg_base->state) & flag) ? 1 : 0;
}

static inline size_t csky_aes_padlen(size_t len, size_t block_size)
{
	len &= block_size - 1;
	return len ? block_size - len : 0;
}

static inline void csky_aes_in_block(struct csky_aes_dev *dd, uint32_t *data)
{
	int i;

	for (i = 0; i < SIZE_IN_WORDS(AES_BLOCK_SIZE); i ++)
		writel_relaxed(HTOL(data[i]),
			&dd->reg_base->datain[SIZE_IN_WORDS(AES_BLOCK_SIZE)
						- 1 - i]);
}

static inline void csky_aes_out_block(struct csky_aes_dev *dd, uint32_t *data)
{
	int i;

	for (i = 0; i < SIZE_IN_WORDS(AES_BLOCK_SIZE); i ++)
		data[i] = HTOL(readl_relaxed(
			&dd->reg_base->dataout[SIZE_IN_WORDS(AES_BLOCK_SIZE)
						- 1 - i]));
}

static inline int csky_aes_complete(struct csky_aes_dev *dd, int err)
{
	dd->flags &= ~AES_FLAGS_BUSY;
	dd->areq->complete(dd->areq, err);

	tasklet_schedule(&dd->done_task);

	return err;
}

static int csky_aes_engine_op(struct csky_aes_dev *dd)
{
	int cbc_mode = dd->flags & AES_FLAGS_CBC;
	int i;
	int err = 0;
	int len;

	csky_aes_config_mode(dd, cbc_mode);
	for (i = 0; i < dd->datalen; i += AES_BLOCK_SIZE) {
		csky_aes_in_block(dd, dd->data);

		csky_aes_enable(dd);
		csky_aes_check_int_status(dd, AES_IT_BUSY);
		csky_aes_disable(dd);

		csky_aes_out_block(dd, dd->data);
		dd->data += SIZE_IN_WORDS(AES_BLOCK_SIZE);
	}

	if (dd->flags & AES_FLAGS_ENC)
		len = dd->datalen;
	else if (dd->flags & AES_FLAGS_DEC)
		len = dd->total;
	else
		return csky_aes_complete(dd, -EINVAL);

	if (!sg_copy_from_buffer(dd->real_dst, sg_nents(dd->real_dst),
							dd->buf, len))
		err = -EINVAL;

	return csky_aes_complete(dd, err);
}

static int csky_aes_start(struct csky_aes_dev *dd,
			  struct scatterlist *src,
			  struct scatterlist *dst,
			  size_t len)
{
	size_t padlen = csky_aes_padlen(len, AES_BLOCK_SIZE);

	if (!(dd->flags & AES_FLAGS_INIT)) {
		return -EACCES;
	}

	if (unlikely(len == 0))
		return -EINVAL;

	sg_copy_to_buffer(src, sg_nents(src), dd->buf, len);

	dd->real_dst= dst;
	dd->total   = len;
	dd->datalen = len + padlen;
	dd->data	= (u32 *)dd->buf;

	return 0;
}

static int csky_aes_set_key(struct csky_aes_dev *dd, const uint32_t *iv)
{
	int i;
	uint32_t *key	= dd->ctx->key;
	uint32_t  keylen = dd->ctx->keylen;

	for (i = 0; i < SIZE_IN_WORDS(dd->ctx->keylen); i++)
		writel_relaxed(HTOL(key[i]),
			&dd->reg_base->key[SIZE_IN_WORDS(dd->ctx->keylen)
					   - 1 - i]);

	csky_aes_set_key_length(dd, keylen);

	if (dd->flags & AES_FLAGS_ENC)
		csky_aes_setopcode(dd, AES_OPC_ENC);
	else if (dd->flags & AES_FLAGS_DEC) {
		csky_aes_setopcode(dd, AES_OPC_EXP);
		csky_aes_enable(dd);
		while (csky_aes_check_int_status(dd, AES_IT_KEYINT));
		csky_aes_disable(dd);
		csky_aes_setopcode(dd, AES_OPC_DEC);
	}

	if (dd->flags & AES_FLAGS_CBC) {
		for (i = 0; i < SIZE_IN_WORDS(AES_BLOCK_SIZE); i++) {
			writel_relaxed(HTOL(iv[i]),
				&dd->reg_base->iv[SIZE_IN_WORDS(AES_BLOCK_SIZE)
						  - 1 - i]);
		}
	}

	return 0;
}

static int csky_aes_handle(struct csky_aes_dev *dd)
{
	struct ablkcipher_request *req = ablkcipher_request_cast(dd->areq);
	struct csky_aes_reqctx	*rctx  = ablkcipher_request_ctx(req);
	int ret;

	dd->flags &= ~(AES_FLAGS_ECB | AES_FLAGS_CBC |
				   AES_FLAGS_ENC | AES_FLAGS_DEC);
	dd->flags |= rctx->mode;

	csky_aes_init(dd);
	ret = csky_aes_start(dd, req->src, req->dst, req->nbytes);
	if (ret)
		return ret;

	ret = csky_aes_set_key(dd, req->info);
	if (ret)
		return ret;

	ret = csky_aes_engine_op(dd);
	return ret;
}

static int csky_aes_handle_queue(struct csky_aes_dev *dd,
				 struct crypto_async_request *new_areq)
{
	struct crypto_async_request *areq, *backlog;
	struct csky_aes_base_ctx	*ctx;
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&dd->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&dd->queue, new_areq);
	if (dd->flags & AES_FLAGS_BUSY) {
		spin_unlock_irqrestore(&dd->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&dd->queue);
	areq 	= crypto_dequeue_request(&dd->queue);
	if (areq)
		dd->flags |= AES_FLAGS_BUSY;
	spin_unlock_irqrestore(&dd->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx		 = crypto_tfm_ctx(areq->tfm);
	dd->areq = areq;
	dd->ctx  = ctx;

	return csky_aes_handle(dd);
}

static int csky_aes_crypt(struct ablkcipher_request *req, unsigned long mode)
{
	struct csky_aes_base_ctx *ctx;
	struct csky_aes_reqctx   *rctx;
	struct csky_aes_dev		 *dd;

	ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	if (!ctx)
		return -ENOMEM;
	dd  = csky_aes_find_dev(ctx);
	if (!dd)
		return -ENODEV;

	rctx		= ablkcipher_request_ctx(req);
	rctx->mode  = mode;

	if ((mode & AES_FLAGS_ECB) || (mode & AES_FLAGS_CBC))
		ctx->block_size = AES_BLOCK_SIZE;

	return csky_aes_handle_queue(dd, &req->base);
}

static int csky_aes_setkey(struct crypto_ablkcipher *tfm, const u8 *key,
						   unsigned int keylen)
{
	struct csky_aes_base_ctx *ctx = crypto_ablkcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128 &&
		keylen != AES_KEYSIZE_192 &&
		keylen != AES_KEYSIZE_256) {
		crypto_ablkcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

static int csky_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	return csky_aes_crypt(req, AES_FLAGS_ECB | AES_FLAGS_ENC);
}

static int csky_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	return csky_aes_crypt(req, AES_FLAGS_ECB | AES_FLAGS_DEC);
}

static int csky_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	return csky_aes_crypt(req, AES_FLAGS_CBC | AES_FLAGS_ENC);
}

static int csky_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	return csky_aes_crypt(req, AES_FLAGS_CBC | AES_FLAGS_DEC);
}

static int csky_aes_cra_init(struct crypto_tfm *tfm)
{
	tfm->crt_ablkcipher.reqsize = sizeof(struct csky_aes_reqctx);

	return 0;
}

static void csky_aes_cra_exit(struct crypto_tfm *tfm)
{

}

static struct crypto_alg csky_aes_algs[] = {
	{
		.cra_name		= "ecb(aes)",
		.cra_driver_name	= "csky-ecb-aes",
		.cra_priority		= 200,
		.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct csky_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= csky_aes_cra_init,
		.cra_exit		= csky_aes_cra_exit,
		.cra_u.ablkcipher	= {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= csky_aes_setkey,
			.encrypt	= csky_aes_ecb_encrypt,
			.decrypt	= csky_aes_ecb_decrypt,
		}
	},
	{
		.cra_name		= "cbc(aes)",
		.cra_driver_name	= "csky-cbc-aes",
		.cra_priority		= 200,
		.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
		.cra_blocksize		= AES_BLOCK_SIZE,
		.cra_ctxsize		= sizeof(struct csky_aes_ctx),
		.cra_alignmask		= 0xf,
		.cra_type		= &crypto_ablkcipher_type,
		.cra_module		= THIS_MODULE,
		.cra_init		= csky_aes_cra_init,
		.cra_exit		= csky_aes_cra_exit,
		.cra_u.ablkcipher	= {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.ivsize		= AES_BLOCK_SIZE,
			.setkey		= csky_aes_setkey,
			.encrypt	= csky_aes_cbc_encrypt,
			.decrypt	= csky_aes_cbc_decrypt,
		}
	},

};

static int csky_aes_buff_init(struct csky_aes_dev *dd)
{
	dd->buf		= (void *)__get_free_pages(GFP_KERNEL, CSKY_AES_BUFFER_ORDER);
	dd->buflen	= CSKY_AES_BUFFER_SIZE;
	dd->buflen &= ~(AES_BLOCK_SIZE - 1);

	if (!dd->buf) {
		dev_err(dd->dev, "unable to alloc pages.\n");
		return -ENOMEM;
	}

	return 0;
}

static void csky_aes_buff_cleanup(struct csky_aes_dev *dd)
{
	if ((unsigned long)dd->buf)
		free_pages((unsigned long)dd->buf, CSKY_AES_BUFFER_ORDER);
}

static void csky_aes_done_task(unsigned long data)
{
	struct csky_aes_dev *dd = (struct csky_aes_dev *)data;

	csky_aes_handle_queue(dd, NULL);
}

static void csky_aes_unregister_algs(struct csky_aes_dev *dd)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(csky_aes_algs); i++)
		crypto_unregister_alg(&csky_aes_algs[i]);
}

static int csky_aes_register_algs(struct csky_aes_dev *dd)
{
	int err, i, j;

	for (i = 0; i < ARRAY_SIZE(csky_aes_algs); i++) {
		err = crypto_register_alg(&csky_aes_algs[i]);
		if (err) {
			for (j = 0; j < i; j++)
				crypto_unregister_alg(&csky_aes_algs[j]);
			return err;
		}
	}

	return 0;
}

static int csky_aes_probe(struct platform_device *pdev)
{
	struct csky_aes_dev *aes_dd;
	struct device *dev = &pdev->dev;
	struct resource *aes_res;
	int err;

	aes_dd = devm_kzalloc(&pdev->dev, sizeof(*aes_dd), GFP_KERNEL);
	if (aes_dd == NULL) {
		dev_err(dev, "unable to alloc data struct.\n");
		err = -ENOMEM;
		goto aes_dd_err;
	}

	aes_dd->dev = dev;

	platform_set_drvdata(pdev, aes_dd);

	INIT_LIST_HEAD(&aes_dd->list);
	spin_lock_init(&aes_dd->lock);

	tasklet_init(&aes_dd->done_task, csky_aes_done_task, (unsigned long)aes_dd);

	crypto_init_queue(&aes_dd->queue, CSKY_AES_QUEUE_LENGTH);

	aes_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!aes_res) {
		err = -ENODEV;
		goto res_err;
	}

	aes_dd->reg_base = devm_ioremap_resource(dev, aes_res);
	if (IS_ERR(aes_dd->reg_base)) {
		err = PTR_ERR(aes_dd->reg_base);
		goto res_err;
	}

	err = csky_aes_buff_init(aes_dd);
	if (err)
		goto res_err;

	spin_lock(&csky_aes.lock);
	list_add_tail(&aes_dd->list, &csky_aes.dev_list);
	spin_unlock(&csky_aes.lock);

	err = csky_aes_register_algs(aes_dd);
	if (err)
		goto err_algs;

	dev_info(dev, "CSKY AES Driver Initialized\n");

	return 0;

err_algs:
	spin_lock(&csky_aes.lock);
	list_del(&aes_dd->list);
	spin_unlock(&csky_aes.lock);
res_err:
	tasklet_kill(&aes_dd->done_task);
aes_dd_err:

	return err;
}

static int csky_aes_remove(struct platform_device *pdev)
{
	static struct csky_aes_dev *aes_dd;

	aes_dd = platform_get_drvdata(pdev);
	if (!aes_dd)
		return -ENODEV;

	spin_lock(&csky_aes.lock);
	list_del(&aes_dd->list);
	spin_unlock(&csky_aes.lock);

	csky_aes_buff_cleanup(aes_dd);

	tasklet_kill(&aes_dd->done_task);
	csky_aes_unregister_algs(aes_dd);

	return 0;
}

static const struct of_device_id csky_aes_dt_ids[] = {
	{ .compatible = "csky,csky-aes" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, csky_aes_dt_ids);

static struct platform_driver csky_aes_driver = {
	.probe	= csky_aes_probe,
	.remove	= csky_aes_remove,
	.driver	= {
		.name = "csky_aes",
		.of_match_table = of_match_ptr(csky_aes_dt_ids),
	},
};

module_platform_driver(csky_aes_driver);

MODULE_DESCRIPTION("CSKY AES hw acceleration support.");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Vincent Cui <xiaoxia_cui@c-sky.com>");
