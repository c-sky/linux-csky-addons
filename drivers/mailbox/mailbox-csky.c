/*
 * mailbox driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Charles Lu <chongzhi_lu@c-sky.com>
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

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "mailbox-csky.h"
#include "mailbox-csky-internal.h"

#define DRIVER_NAME	"mailbox-csky"

/* 0x18 is register length from CSKY_MBOX_INTGR to CSKY_MBOX_INTENB */
#define MBOX_INTGR_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id ? 0 : 1) + CSKY_MBOX_INTGR)
#define MBOX_INTCR_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id) + CSKY_MBOX_INTCR)
#define MBOX_INTMR_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id) + CSKY_MBOX_INTMR)
#define MBOX_INTRSR_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id) + CSKY_MBOX_INTRSR)
#define MBOX_INTMSR_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id) + CSKY_MBOX_INTMSR)
#define MBOX_INTENB_ADDR(mbox)	\
	(mbox->base + 0x18*(mbox->dev_id) + CSKY_MBOX_INTENB)
#define MBOX_TX_MSSG_ADDR(mbox) (mbox->base + 0x18*2 + (mbox->dev_id ? 64 : 0))
#define MBOX_RX_MSSG_ADDR(mbox) (mbox->base + 0x18*2 + (mbox->dev_id ? 0 : 64))

#define TX_GENERATE_INTERRUPT(mbox)	writel(1, MBOX_INTGR_ADDR(mbox))
#define RX_CLEAR_INTERRUPT(mbox) 	writel(1, MBOX_INTCR_ADDR(mbox))
#define RX_MASK_INTERRUPT(mbox) 	writel(1, MBOX_INTMR_ADDR(mbox))
#define RX_UNMASK_INTERRUPT(mbox)	writel(0, MBOX_INTMR_ADDR(mbox))
#define RX_READ_INTERRUPT(mbox)		readl(MBOX_INTRSR_ADDR(mbox))
#define RX_READ_MASKED_INTERRUPT(mbox)	readl(MBOX_INTMSR_ADDR(mbox))
#define RX_ENABLE_INTERRUPT(mbox)	writel(1, MBOX_INTENB_ADDR(mbox))
#define RX_DISABLE_INTERRUPT(mbox)	writel(0, MBOX_INTENB_ADDR(mbox))

struct csky_mbox_chan {
	struct csky_mbox *parent;
};

struct csky_mbox {
	struct device *dev;
	int dev_id;
	struct mutex cfg_lock;
	int irq;
	void __iomem *base;
	u32 chan_num;
	struct csky_mbox_chan *mchans;
	struct mbox_chan *chans;
	struct mbox_controller controller;
};

#ifdef __LITTLE_ENDIAN
#define	BYTE0(w)	((w) & 0xFF)
#define	BYTE1(w)	(((w) >> 8) & 0xFF)
#define	BYTE2(w)	(((w) >> 16) & 0xFF)
#define	BYTE3(w)	(((w) >> 24) & 0xFF)
#else
#define	BYTE0(w)	(((w) >> 24) & 0xFF)
#define	BYTE1(w)	(((w) >> 16) & 0xFF)
#define	BYTE2(w)	(((w) >> 8) & 0xFF)
#define	BYTE3(w)	((w) & 0xFF)
#endif

static irqreturn_t csky_mbox_interrupt(int irq, void *p)
{
	struct csky_mbox *mbox = (struct csky_mbox *)p;
	struct mbox_chan *chan = &(mbox->chans[0]);
	struct mbox_message *mssg_rx =
		(struct mbox_message *)(MBOX_RX_MSSG_ADDR(mbox));

	RX_CLEAR_INTERRUPT(mbox);

	if (mssg_rx->mssg_type == CSKY_MBOX_MSSG_DATA) {
		struct mbox_message *mssg_tx =
			(struct mbox_message *)MBOX_TX_MSSG_ADDR(mbox);
#ifdef DEBUG
		u32 *data = (u32 *)mssg_rx;
		dev_info(mbox->dev, "Recv data, first 8 bytes:" \
			"%02x %02x %02x %02x %02x %02x %02x %02x\n",
			BYTE0(data[0]), BYTE1(data[0]),
			BYTE2(data[0]), BYTE3(data[0]),
			BYTE0(data[1]), BYTE1(data[1]),
			BYTE2(data[1]), BYTE3(data[1]));
#endif

		/* Receive message's data to upper */
		mbox_chan_received_data(chan, (void*)(mssg_rx));

		/* Send ACK back */
		mssg_tx->mssg_type = CSKY_MBOX_MSSG_ACK;
		TX_GENERATE_INTERRUPT(mbox);
	}
	else if (mssg_rx->mssg_type == CSKY_MBOX_MSSG_ACK) {
		mbox_chan_txdone(chan, 0);	/* Notify tx done */
	}
	else {
		dev_err(mbox->dev, "Undefined mssg_type:%02x",
			mssg_rx->mssg_type);
	}

	return IRQ_HANDLED;
}

static struct mbox_chan *csky_mbox_xlate(struct mbox_controller *controller,
					 const struct of_phandle_args *spec)
{
	struct csky_mbox *mbox = dev_get_drvdata(controller->dev);
	u32 i = spec->args[0];
	if (i > CSKY_MBOX_MAX_CHAN) {
		dev_err(mbox->dev, "Failed to get chans[%d]\n", i);
		return NULL;
	}

	return &mbox->chans[i];
}

static int csky_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct csky_mbox_chan *mchan = chan->con_priv;
	struct csky_mbox *mbox = mchan->parent;

#ifdef DEBUG
	char *bytes = (char *)data;
	dev_info(mbox->dev, "Send data, first 8 bytes:" \
		"%02x %02x %02x %02x %02x %02x %02x %02x\n",
		bytes[0], bytes[1], bytes[2], bytes[3],
		bytes[4], bytes[5], bytes[6], bytes[7]);
#endif
	TX_GENERATE_INTERRUPT(mbox);
	return 0;
}

static int csky_mbox_startup(struct mbox_chan *chan)
{
	struct csky_mbox_chan *mchan = chan->con_priv;
	struct csky_mbox *mbox = mchan->parent;

	/* enable and ummask interrupt */
	RX_ENABLE_INTERRUPT(mbox);
	RX_UNMASK_INTERRUPT(mbox);
	return 0;
}

static void csky_mbox_shutdown(struct mbox_chan *chan)
{
	struct csky_mbox_chan *mchan = chan->con_priv;
	struct csky_mbox *mbox = mchan->parent;

	/* disable interrupts */
	RX_CLEAR_INTERRUPT(mbox);
	RX_DISABLE_INTERRUPT(mbox);
	RX_MASK_INTERRUPT(mbox);
}

static const struct mbox_chan_ops csky_mbox_ops = {
	.send_data	= csky_mbox_send_data,
	.startup	= csky_mbox_startup,
	.shutdown	= csky_mbox_shutdown,
	.last_tx_done	= NULL,	/* Not needed, only txdone_poll mode needs */
	.peek_data	= NULL, /* Not needed, interrupt will handle it */
};

static int csky_mbox_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct csky_mbox *mbox;

	u32 val;
	int i, err;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->dev = dev;
	mbox->chan_num = CSKY_MBOX_MAX_CHAN;

	mbox->mchans = devm_kzalloc(dev,
		mbox->chan_num * sizeof(*mbox->mchans), GFP_KERNEL);
	if (!mbox->mchans)
		return -ENOMEM;

	mbox->chans = devm_kzalloc(dev,
		mbox->chan_num * sizeof(*mbox->chans), GFP_KERNEL);
	if (!mbox->chans)
		return -ENOMEM;

	mbox->irq = platform_get_irq(pdev, 0);
	if (mbox->irq < 0)
		return mbox->irq;

	err = device_property_read_u32(dev, "dev-id", &val);
	if (err) {
		dev_err(dev, "No 'dev_id' defined in dts\n");
		return -ENODEV;
	}
	if (val != CSKY_MBOX_DEV_ID0 && val != CSKY_MBOX_DEV_ID1) {
		dev_err(dev, "No such mailbox 'dev_id':%d\n", val);
		return -ENODEV;
	}
	mbox->dev_id = val;
	mbox->base = of_iomap(node, 0);

	err = devm_request_irq(dev, mbox->irq, csky_mbox_interrupt, 0,
				dev_name(dev), mbox);
	if (err) {
		dev_err(dev, "Failed to register a mailbox IRQ handler: %d\n",
			err);
		return -ENODEV;
	}

	mbox->controller.dev = dev;
	mbox->controller.ops = &csky_mbox_ops;
	mbox->controller.chans = &mbox->chans[0];
	mbox->controller.num_chans = mbox->chan_num;
	mbox->controller.of_xlate = csky_mbox_xlate;
	mbox->controller.txdone_irq = true;
	mbox->controller.txdone_poll = false;

	for (i = 0; i < mbox->chan_num; ++i) {
		mbox->chans[i].con_priv = &mbox->mchans[i];
		mbox->mchans[i].parent = mbox;
	}

	/* Mask and clear all interrupt vectors */
	RX_DISABLE_INTERRUPT(mbox);
	RX_MASK_INTERRUPT(mbox);
	RX_CLEAR_INTERRUPT(mbox);

	err = mbox_controller_register(&mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox-%d %d\n",
			mbox->dev_id, err);
		return err;
	}

	platform_set_drvdata(pdev, mbox);
	dev_info(dev, "Mailbox enabled\n");

	return 0;
}

static int csky_mbox_remove(struct platform_device *pdev)
{
	struct csky_mbox *mbox = platform_get_drvdata(pdev);

	if (!mbox)
		return -EINVAL;

	mbox_controller_unregister(&mbox->controller);

	return 0;
}

static const struct of_device_id csky_mbox_match[] = {
	{ .compatible = "csky,mailbox-1.0" },
	{},
};

MODULE_DEVICE_TABLE(of, csky_mbox_match);

static struct platform_driver csky_mbox_driver = {
	.probe	= csky_mbox_probe,
	.remove	= csky_mbox_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.of_match_table	= csky_mbox_match,
	},
};

module_platform_driver(csky_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("C-SKY mailbox specific functions");
MODULE_AUTHOR("Charles Lu <chongzhi_lu@csky.com>");
