/*
 * virtual block driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Huoqing Cai <huoqing_cai@c-sky.com>
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

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define SIMP_BLKDEV_MAJOR	82
#define VIRT_DISK_NAME		"virblk"

struct virblk_dev {
	struct device *dev;
	void __iomem *virt_base;
	size_t virt_size;

	struct clk *clk;
};

static struct request_queue *simp_blkdev_queue;
static struct gendisk *simp_blkdev_disk;

static void simp_blkdev_do_request(struct request_queue *q)
{
	struct request *req;
	struct req_iterator ri;
	struct bio_vec bvec;
	char *disk_mem;
	char *buffer;

	while ((req = blk_fetch_request(q)) != NULL) {
		struct virblk_dev *virblk = q->queuedata;
		if ((blk_rq_pos(req) << 9) + blk_rq_cur_bytes(req) > virblk->virt_size) {
			printk("bad request: block = %llu, count=%u\n",
				(unsigned long long)blk_rq_pos(req),
				blk_rq_cur_bytes(req));
			blk_end_request_all(req, -EIO);
			continue;
		}

		disk_mem = virblk->virt_base + (blk_rq_pos(req) << 9);

		switch (rq_data_dir(req)) {
		case READ:
			printk("read: %lld, %u\n", req->__sector, req->__data_len);
			rq_for_each_segment(bvec, req, ri) {
				buffer = kmap(bvec.bv_page) + bvec.bv_offset;
				memcpy(buffer, disk_mem, bvec.bv_len);
				kunmap(bvec.bv_page);
				disk_mem += bvec.bv_len;
			}
			__blk_end_request_all(req, 0);
			break;

		case WRITE:
			printk("write: %lld, %u\n", req->__sector, req->__data_len);
			rq_for_each_segment(bvec, req, ri) {
				buffer = kmap(bvec.bv_page) + bvec.bv_offset;
				memcpy(disk_mem, buffer, bvec.bv_len);
				kunmap(bvec.bv_page);
				disk_mem += bvec.bv_len;
			}
			__blk_end_request_all(req, 0);
			break;

		default:
			break;
		}
	}
}

struct block_device_operations simp_blkdev_fops = {
	.owner = THIS_MODULE,
};

static const struct of_device_id virblk_dt_ids[] = {
	{ .compatible = "virblk-csky" },
	{}
};

static int virblk_probe(struct platform_device *pdev)
{
	struct virblk_dev *virblk;
	struct resource *res;
	size_t size;
	int ret;
	int (*init_func)(void);

	virblk = devm_kzalloc(&pdev->dev, sizeof(*virblk), GFP_KERNEL);
	if (!virblk)
		return -ENOMEM;

	virblk->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(virblk->dev, "found no memory resource\n");
		return -EINVAL;
	}

	size = resource_size(res);

	if (!devm_request_mem_region(virblk->dev, res->start, size, pdev->name)) {
		dev_err(virblk->dev, "could not request region for resource\n");
		return -EBUSY;
	}

	if (of_property_read_bool(pdev->dev.of_node, "no-memory-wc"))
		virblk->virt_base = devm_ioremap(virblk->dev, res->start, size);
	else
		virblk->virt_base = devm_ioremap_wc(virblk->dev, res->start, size);

	virblk->virt_size = size;

	if (!virblk->virt_base)
		return -ENOMEM;

	virblk->clk = devm_clk_get(virblk->dev, NULL);

	if (IS_ERR(virblk->clk))
		virblk->clk = NULL;
	else
		clk_prepare_enable(virblk->clk);

	/* Init virtual block device */
	simp_blkdev_queue = blk_init_queue(simp_blkdev_do_request, NULL);
	simp_blkdev_queue->queuedata = virblk;
	if (!simp_blkdev_queue) {
		ret = -ENOMEM;
		goto err_init_queue;
	}

	simp_blkdev_disk = alloc_disk(1);
	if (!simp_blkdev_disk) {
		ret = -ENOMEM;
		goto err_alloc_disk;
	}

	strcpy(simp_blkdev_disk->disk_name, VIRT_DISK_NAME);
	simp_blkdev_disk->major = SIMP_BLKDEV_MAJOR;
	simp_blkdev_disk->first_minor = 0;
	simp_blkdev_disk->fops = &simp_blkdev_fops;
	simp_blkdev_disk->queue = simp_blkdev_queue;
	set_capacity(simp_blkdev_disk, size >> 9);
	add_disk(simp_blkdev_disk);


	platform_set_drvdata(pdev, virblk);

	init_func = of_device_get_match_data(&pdev->dev);
	if (init_func) {
		ret = init_func();
		if (ret)
			return ret;
	}

	return 0;

err_alloc_disk:
	blk_cleanup_queue(simp_blkdev_queue);
err_init_queue:

	return ret;
}

static int virblk_remove(struct platform_device *pdev)
{
	struct virblk_dev *virblk = platform_get_drvdata(pdev);

	/* Deinit virtual block device */
	del_gendisk(simp_blkdev_disk);
	put_disk(simp_blkdev_disk);
	blk_cleanup_queue(simp_blkdev_queue);

	if (virblk->clk)
		clk_disable_unprepare(virblk->clk);

	return 0;
}

static struct platform_driver virblk_driver = {
	.driver = {
		.name = "virblk",
		.of_match_table = virblk_dt_ids,
	},
	.probe = virblk_probe,
	.remove = virblk_remove,
};

static int __init virblk_init(void)
{
	return platform_driver_register(&virblk_driver);
}

module_init(virblk_init);

MODULE_AUTHOR("Huoqing Cai <huoqing_cai@c-sky.com>");
MODULE_DESCRIPTION("C-SKY virtual block");
MODULE_LICENSE("GPL v2");
