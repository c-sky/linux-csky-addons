/*
 * TTY mailbox client driver for C-SKY's SoCs.
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
#include <linux/circ_buf.h>

#include "mailbox-csky.h"
#include "mailbox-csky-internal.h"

#define MBOX_MAX_MSG_LEN	CSKY_MBOX_MAX_MESSAGE_LENGTH
#define RX_BUF_SIZE		4096

static struct dentry *root_debugfs_dir;

struct tty_mbox_client_csky_device {
	struct device		*dev;
	void __iomem		*tx_mmio;
	void __iomem		*rx_mmio;
	struct mbox_chan	*tx_channel;
	struct mbox_chan	*rx_channel;
	char			*rx_buffer;
	uint			rx_head;	/* circular rx buffer head */
	uint			rx_tail;	/* circular rx buffer tail */
	struct mbox_message	*message;
	spinlock_t		lock;
};

static ssize_t tty_mbox_client_csky_message_write(struct file *filp,
						  const char __user *userbuf,
						  size_t count, loff_t *ppos)
{
	struct tty_mbox_client_csky_device *tdev = filp->private_data;
	void *data;
	uint sent_len = 0;
	int ret;

	if (!tdev->tx_channel) {
		dev_err(tdev->dev, "Channel cannot do Tx\n");
		return -EINVAL;
	}

	if (tdev->message == NULL) {
		tdev->message = devm_kzalloc(tdev->dev,
					     MBOX_MAX_MSG_LEN, GFP_KERNEL);
		if (tdev->message == NULL)
			return -ENOMEM;
	}

	while (sent_len < count) {
		uint tx_len =
			((count - sent_len) < CSKY_MBOX_MAX_DATA_LENGTH) ?
			(count - sent_len) : CSKY_MBOX_MAX_DATA_LENGTH;

		tdev->message->mssg_type = CSKY_MBOX_MSSG_DATA;
		tdev->message->length = tx_len;
		ret = copy_from_user(tdev->message->data,
				     &(userbuf[sent_len]),
				     MBOX_CSKY_MSSG_HEAD_LENGTH + tx_len);

#ifdef DEBUG
		print_hex_dump_bytes("Client: Sending: Message: ",
			DUMP_PREFIX_ADDRESS, tdev->message, MBOX_MAX_MSG_LEN);
#endif
		data = tdev->message;
		ret = mbox_send_message(tdev->tx_channel, data);
		if (ret < 0) {
			dev_err(tdev->dev, "Failed to send message\n");
			return -EIO;
		}
		sent_len += tx_len;
	}

	return sent_len;
}

static ssize_t tty_mbox_client_csky_message_read(struct file *filp,
						 char __user *userbuf,
						 size_t count, loff_t *ppos)
{
	struct tty_mbox_client_csky_device *tdev = filp->private_data;
	uint read_length, rx_buffer_length, rx_cnt_to_end;
	int ret;

#ifdef DEBUG
	char touser_debug[32];
#endif

	if (count == 0) {
		return 0;
	}

	if (!tdev->rx_channel) {
#ifdef DEBUG
		ret = snprintf(touser_debug, 20, "<NO RX CAPABILITY>\n");
		ret = simple_read_from_buffer(userbuf, count, ppos,
					      touser_debug, ret);
		return ret;
#else
		dev_err(tdev->dev, "NO RX CAPABILITY\n");
		return -ENXIO;
#endif
	}

	rx_buffer_length = CIRC_CNT(tdev->rx_head, tdev->rx_tail, RX_BUF_SIZE);
	if (rx_buffer_length == 0) {
#ifdef DEBUG
		ret = snprintf(touser_debug, 9, "<EMPTY>\n");
		ret = simple_read_from_buffer(userbuf, count, ppos,
					      touser_debug, ret);
		return ret;
#else
		return 0;
#endif
	}

	read_length = min(count, rx_buffer_length);

	rx_cnt_to_end = CIRC_CNT_TO_END(tdev->rx_head,
					tdev->rx_tail,
					RX_BUF_SIZE);
	if (rx_cnt_to_end >= read_length) {
		/* Copy once */
		ret = copy_to_user(userbuf,
				   &(tdev->rx_buffer[tdev->rx_tail]),
				   read_length);
		if (ret != 0) {
			return -EFAULT;
		}
		tdev->rx_tail += read_length;
		return read_length;
	}
	else {
		/* Copy twice */
		ret = copy_to_user(&(userbuf[0]),
				   &(tdev->rx_buffer[tdev->rx_tail]),
				   rx_cnt_to_end);
		if (ret != 0) {
			return -EFAULT;
		}

		ret = copy_to_user(&(userbuf[rx_cnt_to_end]),
				   &(tdev->rx_buffer[0]),
				   read_length - rx_cnt_to_end);
		if (ret != 0) {
			return -EFAULT;
		}

		tdev->rx_tail = read_length - rx_cnt_to_end;
		return read_length;
	}
}

static const struct file_operations tty_mbox_client_csky_message_ops = {
	.write	= tty_mbox_client_csky_message_write,
	.read	= tty_mbox_client_csky_message_read,
	.open	= simple_open,
	.llseek	= generic_file_llseek,
};

static int index_names = 0;
static bool debugfs_dir_created = false;
static const char* file_names[] = {"ttym-client0", "ttym-client1"};

static int
tty_mbox_client_csky_add_debugfs(struct platform_device *pdev,
				 struct tty_mbox_client_csky_device *tdev)
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
			    tdev, &tty_mbox_client_csky_message_ops);

	index_names++;
	return 0;
}

static void tty_mbox_client_csky_receive_message(struct mbox_client *client,
						 void *message)
{
	struct tty_mbox_client_csky_device *tdev =
		dev_get_drvdata(client->dev);
	unsigned long flags;
	struct mbox_message mssg;
	uint rx_buffer_space, copy_len;

	if (tdev->rx_mmio == NULL) {
		dev_err(client->dev, "rx_mmio is NULL\n");
		return;
	}

	spin_lock_irqsave(&tdev->lock, flags);
	memcpy(&mssg, (struct mbox_message*)(tdev->rx_mmio), sizeof(mssg));
	rx_buffer_space = CIRC_SPACE(tdev->rx_head,
				     tdev->rx_tail,
				     RX_BUF_SIZE);

	copy_len = (rx_buffer_space < mssg.length) ?
		   rx_buffer_space : mssg.length;

	if (copy_len) {
		uint space_to_end = CIRC_SPACE_TO_END(tdev->rx_head,
						      tdev->rx_tail,
						      RX_BUF_SIZE);

		if (copy_len <= space_to_end) {
			/* head to end has enough space, copy once */
			memcpy(tdev->rx_buffer + tdev->rx_head,
			       &(mssg.data[0]),
			       copy_len);
			tdev->rx_head = (tdev->rx_head + copy_len) &
					(RX_BUF_SIZE - 1);
		}
		else {
			/* head to end has not enough space, copy twice */
			uint len_from_buffer = copy_len - space_to_end;
			memcpy(tdev->rx_buffer + tdev->rx_head,
			       &(mssg.data[0]),
			       space_to_end);

			memcpy(tdev->rx_buffer,
			       &(mssg.data[space_to_end]),
			       len_from_buffer);
			tdev->rx_head = (len_from_buffer) & (RX_BUF_SIZE - 1);
		}
	}
	spin_unlock_irqrestore(&tdev->lock, flags);
}

static void tty_mbox_client_csky_prepare_message(struct mbox_client *client,
						 void *message)
{
	struct tty_mbox_client_csky_device *tdev =
		dev_get_drvdata(client->dev);

	if (tdev->tx_mmio) {
		memcpy_toio(tdev->tx_mmio, message, MBOX_MAX_MSG_LEN);
	}
}

static void tty_mbox_client_csky_message_sent(struct mbox_client *client,
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
tty_mbox_client_csky_request_channel(struct platform_device *pdev,
				     const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->rx_callback	= tty_mbox_client_csky_receive_message;
	client->tx_prepare	= tty_mbox_client_csky_prepare_message;
	client->tx_done		= tty_mbox_client_csky_message_sent;
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

static int tty_mbox_client_csky_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct tty_mbox_client_csky_device *tdev;
	int ret;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->tx_mmio = of_iomap(node, 0);
	tdev->rx_mmio = of_iomap(node, 1);

	tdev->tx_channel = tty_mbox_client_csky_request_channel(pdev,
								"channel");
	if (!tdev->tx_channel) {
		dev_err(&pdev->dev, "Request channel failed\n");
		return -EPROBE_DEFER;
	}
	/* In fact, rx_channel is same with tx_channel in C-SKY's mailbox */
	tdev->rx_channel = tdev->tx_channel;

	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);

	spin_lock_init(&tdev->lock);

	tdev->rx_buffer = devm_kzalloc(&pdev->dev, RX_BUF_SIZE, GFP_KERNEL);
	if (!tdev->rx_buffer)
		return -ENOMEM;
	tdev->rx_head = 0;
	tdev->rx_tail = 0;

	ret = tty_mbox_client_csky_add_debugfs(pdev, tdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int tty_mbox_client_csky_remove(struct platform_device *pdev)
{
	struct tty_mbox_client_csky_device *tdev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(root_debugfs_dir);

	if (tdev->tx_channel)
		mbox_free_channel(tdev->tx_channel);

	if (tdev->rx_channel && tdev->rx_channel != tdev->tx_channel)
		mbox_free_channel(tdev->rx_channel);

	if (tdev->message)
		devm_kfree(tdev->dev, tdev->message);
	return 0;
}

static const struct of_device_id tty_mbox_client_csky_match[] = {
	{ .compatible = "csky,tty-mailbox-client" },
	{},
};

static struct platform_driver tty_mbox_client_csky_driver = {
	.driver = {
		.name = "csky,tty-mailbox-client",
		.of_match_table = tty_mbox_client_csky_match,
	},
	.probe  = tty_mbox_client_csky_probe,
	.remove = tty_mbox_client_csky_remove,
};
module_platform_driver(tty_mbox_client_csky_driver);

MODULE_DESCRIPTION("CSKY TTY Mailbox Client driver");
MODULE_AUTHOR("Charles Lu <chongzhi_lu@c-sky.com");
MODULE_LICENSE("GPL v2");
