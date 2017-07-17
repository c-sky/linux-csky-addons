/*
 * mailbox client driver for C-SKY's SoCs.
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

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mailbox-csky.h"
#include "mailbox-csky-internal.h"

#define MBOX_MAX_MSG_LEN	CSKY_MBOX_MAX_MESSAGE_LENGTH
#define HEXDUMP_BYTES_PER_LINE	16
#define HEXDUMP_LINE_LEN	((HEXDUMP_BYTES_PER_LINE * 4) + 2)
#define HEXDUMP_MAX_LEN		(HEXDUMP_LINE_LEN *		\
				(MBOX_MAX_MSG_LEN / HEXDUMP_BYTES_PER_LINE))

static struct dentry *root_debugfs_dir;

struct mbox_client_csky_device {
	struct device		*dev;
	void __iomem		*tx_mmio;
	void __iomem		*rx_mmio;
	struct mbox_chan	*tx_channel;
	struct mbox_chan	*rx_channel;
	char			*rx_buffer;
	struct mbox_message	*message;
	spinlock_t		lock;
};

static ssize_t mbox_client_csky_message_write(struct file *filp,
					      const char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct mbox_client_csky_device *tdev = filp->private_data;
	void *data;
	int ret;

	if (!tdev->tx_channel) {
		dev_err(tdev->dev, "Channel cannot do Tx\n");
		return -EINVAL;
	}

	if (count > CSKY_MBOX_MAX_DATA_LENGTH) {
		dev_err(tdev->dev,
			"Message length %zd greater than max allowed %d\n",
			count, CSKY_MBOX_MAX_DATA_LENGTH);
		return -EINVAL;
	}

	tdev->message = kzalloc(MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->message)
		return -ENOMEM;

	/* Fill message according to struct mbox_message format */
	tdev->message->mssg_type = CSKY_MBOX_MSSG_DATA;
	tdev->message->length = count;
	ret = copy_from_user(tdev->message->data, userbuf, count);
	if (ret) {
		ret = -EFAULT;
		goto out;
	}

	data = tdev->message;
	print_hex_dump_bytes("Client: Sending: Message: ",
		DUMP_PREFIX_ADDRESS, tdev->message, MBOX_MAX_MSG_LEN);

	ret = mbox_send_message(tdev->tx_channel, data);
	if (ret < 0)
		dev_err(tdev->dev, "Failed to send message via mailbox\n");

out:
	kfree(tdev->message);
	return ret < 0 ? ret : count;
}

static ssize_t mbox_client_csky_message_read(struct file *filp,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct mbox_client_csky_device *tdev = filp->private_data;
	unsigned long flags;
	char *touser, *ptr;
	int l = 0;
	int ret;

	touser = kzalloc(HEXDUMP_MAX_LEN + 1, GFP_KERNEL);
	if (!touser)
		return -ENOMEM;

	if (!tdev->rx_channel) {
		ret = snprintf(touser, 20, "<NO RX CAPABILITY>\n");
		ret = simple_read_from_buffer(userbuf, count, ppos,
					      touser, ret);
		goto out;
	}

	if (tdev->rx_buffer[0] == '\0') {
		ret = snprintf(touser, 9, "<EMPTY>\n");
		ret = simple_read_from_buffer(userbuf, count, ppos,
					      touser, ret);
		goto out;
	}

	spin_lock_irqsave(&tdev->lock, flags);

	ptr = tdev->rx_buffer;
	while (l < HEXDUMP_MAX_LEN) {
		hex_dump_to_buffer(ptr,
				   HEXDUMP_BYTES_PER_LINE,
				   HEXDUMP_BYTES_PER_LINE, 1, touser + l,
				   HEXDUMP_LINE_LEN, true);

		ptr += HEXDUMP_BYTES_PER_LINE;
		l += HEXDUMP_LINE_LEN;
		*(touser + (l - 1)) = '\n';
	}
	*(touser + l) = '\0';

	memset(tdev->rx_buffer, 0, MBOX_MAX_MSG_LEN);

	spin_unlock_irqrestore(&tdev->lock, flags);

	ret = simple_read_from_buffer(
		userbuf, count, ppos, touser, HEXDUMP_MAX_LEN);
out:
	kfree(touser);
	return ret;
}

static const struct file_operations mbox_client_csky_message_ops = {
	.write	= mbox_client_csky_message_write,
	.read	= mbox_client_csky_message_read,
	.open	= simple_open,
	.llseek	= generic_file_llseek,
};

static int index_names = 0;
static bool debugfs_dir_created = false;
static const char* file_names[] = {"mbox-client0", "mbox-client1"};

static int mbox_client_csky_add_debugfs(struct platform_device *pdev,
					struct mbox_client_csky_device *tdev)
{
	if (!debugfs_initialized())
		return 0;

	if (index_names > 2) {
		dev_err(&pdev->dev, "Max device index is 2\n");
		return 0;
	}

	if (!debugfs_dir_created) {
		root_debugfs_dir = debugfs_create_dir("mailbox", NULL);
		if (!root_debugfs_dir) {
			dev_err(&pdev->dev,
				"Failed to create mailbox debugfs\n");
			return -EINVAL;
		}
		debugfs_dir_created = true;
	}

	debugfs_create_file(file_names[index_names], 0600, root_debugfs_dir,
			    tdev, &mbox_client_csky_message_ops);

	index_names++;
	return 0;
}

static void mbox_client_csky_receive_message(struct mbox_client *client,
					     void *message)
{
	struct mbox_client_csky_device *tdev = dev_get_drvdata(client->dev);
	unsigned long flags;

	if (tdev->rx_mmio == NULL) {
		dev_err(client->dev, "rx_mmio is NULL\n");
		return;
	}

	spin_lock_irqsave(&tdev->lock, flags);
	memcpy_fromio(tdev->rx_buffer, tdev->rx_mmio, MBOX_MAX_MSG_LEN);
#ifdef DEBUG
	print_hex_dump_bytes("Client: Received [MMIO]: ",
			     DUMP_PREFIX_ADDRESS,
			     tdev->rx_buffer, MBOX_MAX_MSG_LEN);
#endif
	spin_unlock_irqrestore(&tdev->lock, flags);
}

static void mbox_client_csky_prepare_message(struct mbox_client *client,
					     void *message)
{
	struct mbox_client_csky_device *tdev = dev_get_drvdata(client->dev);

	if (tdev->tx_mmio) {
		memcpy_toio(tdev->tx_mmio, message, MBOX_MAX_MSG_LEN);
	}
}

static void mbox_client_csky_message_sent(struct mbox_client *client,
					  void *message, int r)
{
	if (r) {
		dev_warn(client->dev,
			 "Client: Message could not be sent: %d\n", r);
	}
	else {
#ifdef DEBUG
		dev_info(client->dev, "Client: Message sent\n");
#endif
	}
}

static struct mbox_chan *
mbox_client_csky_request_channel(struct platform_device *pdev,
				 const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->rx_callback	= mbox_client_csky_receive_message;
	client->tx_prepare	= mbox_client_csky_prepare_message;
	client->tx_done		= mbox_client_csky_message_sent;
	client->tx_block	= true;
	client->knows_txdone	= true;
	client->tx_tout		= 500;

	channel = mbox_request_channel_byname(client, name);
	if (IS_ERR(channel)) {
		devm_kfree(&pdev->dev, client);
		dev_warn(&pdev->dev, "Failed to request %s channel\n", name);
		return NULL;
	}

	return channel;
}

static int mbox_client_csky_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct mbox_client_csky_device *tdev;
	int ret;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->tx_mmio = of_iomap(node, 0);
	tdev->rx_mmio = of_iomap(node, 1);

	tdev->tx_channel = mbox_client_csky_request_channel(pdev, "channel");
	if (!tdev->tx_channel) {
		dev_err(&pdev->dev, "Request channel failed\n");
		return -EPROBE_DEFER;
	}
	/* In fact, rx_channel is same with tx_channel in C-SKY's mailbox */
	tdev->rx_channel = tdev->tx_channel;

	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);

	spin_lock_init(&tdev->lock);

	tdev->rx_buffer = devm_kzalloc(&pdev->dev,
					MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->rx_buffer)
		return -ENOMEM;

	ret = mbox_client_csky_add_debugfs(pdev, tdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int mbox_client_csky_remove(struct platform_device *pdev)
{
	struct mbox_client_csky_device *tdev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(root_debugfs_dir);

	if (tdev->tx_channel)
		mbox_free_channel(tdev->tx_channel);

	if (tdev->rx_channel && tdev->rx_channel != tdev->tx_channel)
		mbox_free_channel(tdev->rx_channel);

	return 0;
}

static const struct of_device_id mbox_client_csky_match[] = {
	{ .compatible = "csky,mailbox-client" },
	{},
};

static struct platform_driver mbox_client_csky_driver = {
	.driver = {
		.name = "csky,mailbox-client",
		.of_match_table = mbox_client_csky_match,
	},
	.probe  = mbox_client_csky_probe,
	.remove = mbox_client_csky_remove,
};
module_platform_driver(mbox_client_csky_driver);

MODULE_DESCRIPTION("CSKY Mailbox Client driver");
MODULE_AUTHOR("Charles Lu <chongzhi_lu@c-sky.com");
MODULE_LICENSE("GPL v2");
