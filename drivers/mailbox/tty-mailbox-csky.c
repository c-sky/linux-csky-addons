/*
 * TTY based on C-SKY mailbox driver for C-SKY's SoCs.
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
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/circ_buf.h>

#include "mailbox-csky.h"
#include "mailbox-csky-internal.h"

#define MBOX_MAX_MSG_LEN	CSKY_MBOX_MAX_MESSAGE_LENGTH
#define TTY_MBOX_DRIVER_NAME	"ttym"
#define TTY_MBOX_MAJOR		0	/* Let kernel decides */
#define TTY_MBOX_MAX_CONSOLES	1	/* Only 1 supported, don't try more */

#define BUF_SIZE	2048	/* This must be a power of two */

static const struct of_device_id csky_ttym_match[];

struct ttym_data {
	struct device		*dev;
	spinlock_t		lock;		/* lock when send buffer */
	struct mutex		mtx;		/* unlock when recv data */

	struct tty_driver	*tty_driver;
	struct tty_port		tty_port;

	bool			rx_throttle;
	void __iomem		*tx_mmio;
	void __iomem		*rx_mmio;
	struct mbox_chan	*tx_channel;
	struct mbox_chan	*rx_channel;
	struct mbox_message	*tx_buffer;
	char			*rx_buffer;
	bool			tx_sent;	/* Tx message has been sent */

#if 0
	u8			buf[BUF_SIZE];	/* transmit circular buffer */
	u8			head;		/* circular buffer head */
	u8			tail;		/* circular buffer tail */
#endif
};

static struct tty_driver *s_ttym_driver;
static struct ttym_data  *s_ttym_data;		/* only one ttym supported */

static int ttym_open(struct tty_struct *ttys, struct file *filp)
{
	struct ttym_data *ttymd = s_ttym_data;

	if (!ttymd->dev)
		return -ENODEV;

	return tty_port_open(&ttymd->tty_port, ttys, filp);
}

static void ttym_close(struct tty_struct *ttys, struct file *filp)
{
	struct ttym_data *ttymd = s_ttym_data;

	if (ttymd->dev)
		tty_port_close(&ttymd->tty_port, ttys, filp);
}

/*
 * This function is called when the tty layer has data for us send.
 */
static int ttym_write(struct tty_struct *ttys, const unsigned char *s,
		      int count)
{
	struct ttym_data *ttymd = ttys->driver_data;
	uint copy_len;
	uint written = 0;
	int ret;

	if (!ttymd->tx_channel) {
		dev_err(ttymd->dev, "Channel cannot do Tx\n");
		return -EINVAL;
	}

	if (!ttymd->tx_sent) {
		dev_warn(ttymd->dev, "Tx message has not sent yet\n");
		return 0;
	}

	written = (count <= CSKY_MBOX_MAX_DATA_LENGTH) ?
		count : CSKY_MBOX_MAX_DATA_LENGTH;

	ttymd->tx_buffer->mssg_type = CSKY_MBOX_MSSG_DATA;
	ttymd->tx_buffer->length = written;	/* Payload len without head */
	/* Align copy_len with 4, because IO only accept 4Bytes aligned data */
	copy_len = (MBOX_CSKY_MSSG_HEAD_LENGTH + written + 3) & 0xFFFFFFFC;
	memcpy(ttymd->tx_buffer->data, s, copy_len);

#ifdef DEBUG
	print_hex_dump_bytes("Client: Sending: Message: ",
		DUMP_PREFIX_ADDRESS, ttymd->tx_buffer, MBOX_MAX_MSG_LEN);
#endif

	ret = mbox_send_message(ttymd->tx_channel, ttymd->tx_buffer);
	if (ret < 0)
		dev_err(ttymd->dev, "Failed to send message via mailbox\n");

	return ret < 0 ? ret : written;
}

static int ttym_write_room(struct tty_struct *ttys)
{
	struct ttym_data *ttymd = ttys->driver_data;
	int count = ttymd->tx_sent ? CSKY_MBOX_MAX_DATA_LENGTH : 0;

	return count;
}

static void ttym_throttle(struct tty_struct *ttys)
{
	struct ttym_data *ttymd = ttys->driver_data;
	ttymd->rx_throttle = true;
}

static void ttym_unthrottle(struct tty_struct *ttys)
{
	struct ttym_data *ttymd = ttys->driver_data;
	ttymd->rx_throttle = false;
}

static void ttym_hangup(struct tty_struct *ttys)
{
	struct ttym_data *ttymd = ttys->driver_data;
	tty_port_hangup(&ttymd->tty_port);
}

/*
 * TTY driver operations
 *
 * If we could ask the hypervisor how much data is still in the TX buffer, or
 * at least how big the TX buffers are, then we could implement the
 * .wait_until_sent and .chars_in_buffer functions.
 */
static const struct tty_operations ttym_ops = {
	.open		= ttym_open,
	.close		= ttym_close,
	.write		= ttym_write,
	.write_room	= ttym_write_room,
	.throttle	= ttym_throttle,
	.unthrottle	= ttym_unthrottle,
	.hangup		= ttym_hangup,
};

static void csky_ttym_mbox_client_receive_message(struct mbox_client *client,
						  void *message)
{
	struct ttym_data *ttymd = dev_get_drvdata(client->dev);
	struct mbox_message *mssg = (struct mbox_message *)ttymd->rx_buffer;
	int count, ret;
	ulong flags;

	if (ttymd->rx_throttle) {
		return;
	}

	if (ttymd->rx_mmio == NULL) {
		dev_err(client->dev, "rx_mmio is NULL\n");
		return;
	}

	spin_lock_irqsave(&ttymd->lock, flags);
	memcpy_fromio(ttymd->rx_buffer, ttymd->rx_mmio, MBOX_MAX_MSG_LEN);
	spin_unlock_irqrestore(&ttymd->lock, flags);


#ifdef DEBUG
	print_hex_dump_bytes("Client: Received: ", DUMP_PREFIX_ADDRESS,
			     ttymd->rx_buffer, MBOX_MAX_MSG_LEN);
#endif

	count = tty_buffer_request_room(&ttymd->tty_port, mssg->length);
	if (count != mssg->length) {
		dev_err(client->dev, "tty buffer request %dB room failed\n",
			mssg->length);
		return;
	}
	ret = tty_insert_flip_string(&ttymd->tty_port, mssg->data, count);
	if (ret != count) {
		dev_err(client->dev, "Insert %dB but copied %dB\n",
			count, ret);
	}

	tty_flip_buffer_push(&ttymd->tty_port);
}

static void csky_ttym_mbox_client_prepare_message(struct mbox_client *client,
						  void *message)
{
	struct ttym_data *ttymd = dev_get_drvdata(client->dev);

	if (ttymd->tx_mmio) {
		memcpy_toio(ttymd->tx_mmio, message, MBOX_MAX_MSG_LEN);
	}
}

static void csky_ttym_mbox_client_message_sent(struct mbox_client *client,
					       void *message, int r)
{
	struct ttym_data *ttymd = dev_get_drvdata(client->dev);
	if (r) {
		dev_warn(client->dev,
			 "Client: Message could not be sent: %d\n", r);
	}
	else {
#ifdef DEBUG
		dev_info(client->dev, "Client: Message sent\n");
#endif
	}
	ttymd->tx_sent = true;	/* Set to be true anyway */
	tty_port_tty_wakeup(&ttymd->tty_port);
}

static struct mbox_chan *
csky_ttym_mbox_client_request_channel(struct platform_device *pdev,
				      const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->rx_callback	= csky_ttym_mbox_client_receive_message;
	client->tx_prepare	= csky_ttym_mbox_client_prepare_message;
	client->tx_done		= csky_ttym_mbox_client_message_sent;
	client->tx_block	= true;
	client->knows_txdone	= true;
	client->tx_tout		= 1000;

	channel = mbox_request_channel_byname(client, name);
	if (IS_ERR(channel)) {
		devm_kfree(&pdev->dev, client);
		dev_warn(&pdev->dev, "Failed to request %s channel\n", name);
		return NULL;
	}

	return channel;
}

/*
 * initialize the TTY port
 *
 * This function will only be called once, no matter how many times
 * csky_ttym_open() is called.  That's why we register the ISR here, and also
 * why we initialize tty_struct-related variables here.
 */
static int csky_ttym_port_activate(struct tty_port *port,
				   struct tty_struct *ttys)
{
	struct ttym_data *ttymd = container_of(port,
					       struct ttym_data, tty_port);
	ttys->driver_data = ttymd;
	dev_info(ttymd->dev, "ttym port activated\n");
	return 0;
}

/*
 * The port is being closed by the last user.
 * Do any hardware specific stuff here
 */
static void csky_ttym_port_shutdown(struct tty_port *port)
{
	struct ttym_data *ttymd = container_of(port,
					       struct ttym_data, tty_port);
	dev_info(ttymd->dev, "ttym port shutdown\n");
}

static const struct tty_port_operations csky_ttym_port_ops = {
	.activate = csky_ttym_port_activate,
	.shutdown = csky_ttym_port_shutdown,
};

static int csky_ttym_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	uint count = 0;

	int ret;

	/* Count the number of byte channels */
	for_each_compatible_node(np, NULL, csky_ttym_match[0].compatible)
				 count++;

	if (count == 0 || count > TTY_MBOX_MAX_CONSOLES) {
		dev_err(dev, "Max %d mtty support, but now %d\n",
			TTY_MBOX_MAX_CONSOLES, count);
		return -ENODEV;
	}

	/* Allocate memorys */
	s_ttym_driver = alloc_tty_driver(1);
	if (!s_ttym_driver) {
		ret = -ENOMEM;
		goto error;
	}

	s_ttym_data = devm_kzalloc(&pdev->dev,
				   sizeof(struct ttym_data), GFP_KERNEL);
	if (!s_ttym_data) {
		ret = -ENOMEM;
		goto error;
	}

	s_ttym_data->tx_buffer = devm_kzalloc(&pdev->dev,
					      MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!s_ttym_data->tx_buffer) {
		ret = -ENOMEM;
		goto error;
	}

	s_ttym_data->rx_buffer = devm_kzalloc(&pdev->dev,
					      MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!s_ttym_data->rx_buffer) {
		ret = -ENOMEM;
		goto error;
	}

	/* Initialize the tty driver */
	s_ttym_driver->owner = THIS_MODULE;
	s_ttym_driver->driver_name = TTY_MBOX_DRIVER_NAME;
	s_ttym_driver->name = "ttym";
	s_ttym_driver->major = TTY_MBOX_MAJOR,
	s_ttym_driver->type = TTY_DRIVER_TYPE_CONSOLE,
	s_ttym_driver->subtype = SYSTEM_TYPE_CONSOLE,
	s_ttym_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV,
	s_ttym_driver->init_termios = tty_std_termios;
	tty_set_operations(s_ttym_driver, &ttym_ops);

	ret = tty_register_driver(s_ttym_driver);
	if (ret) {
		dev_info(dev, "Could not register ttym driver, ret=%d\n", ret);
		goto error;
	}

	/* Initialize the s_ttym_data */
	spin_lock_init(&s_ttym_data->lock);
	mutex_init(&s_ttym_data->mtx);
	s_ttym_data->tty_driver = s_ttym_driver;

	tty_port_init(&s_ttym_data->tty_port);
	s_ttym_data->tty_port.ops = &csky_ttym_port_ops;
	s_ttym_data->dev = tty_port_register_device(&s_ttym_data->tty_port,
		s_ttym_driver, 0, &pdev->dev);
	if (IS_ERR(s_ttym_data->dev)) {
		ret = PTR_ERR(s_ttym_data->dev);
		dev_err(&pdev->dev, "could not register ttym (ret=%i)\n", ret);
		goto error;
	}

	s_ttym_data->rx_throttle = false;
	s_ttym_data->tx_mmio = of_iomap(node, 0);
	s_ttym_data->rx_mmio = of_iomap(node, 1);
	s_ttym_data->tx_channel =
		csky_ttym_mbox_client_request_channel(pdev, "channel");
	if (!s_ttym_data->tx_channel) {
		dev_err(&pdev->dev, "Request maiblox channel failed\n");
		return -EPROBE_DEFER;
	}
	/* In fact, rx_channel is same with tx_channel in C-SKY's mailbox */
	s_ttym_data->rx_channel = s_ttym_data->tx_channel;
	s_ttym_data->tx_sent = true;

	dev_set_drvdata(&pdev->dev, s_ttym_data);

	dev_info(dev, "tty based on mailbox enabled\n");
	return 0;

error:
	if (s_ttym_driver) {
		tty_unregister_driver(s_ttym_driver);
		put_tty_driver(s_ttym_driver);
	}

	tty_port_destroy(&s_ttym_data->tty_port);
	devm_kfree(&pdev->dev, s_ttym_data->tx_buffer);
	devm_kfree(&pdev->dev, s_ttym_data->rx_buffer);
	devm_kfree(&pdev->dev, s_ttym_data);
	s_ttym_data = NULL;

	return ret;
}

static int csky_ttym_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	tty_unregister_device(s_ttym_driver, 0);
	tty_unregister_driver(s_ttym_driver);
	put_tty_driver(s_ttym_driver);

	if(s_ttym_data->tty_port.count)
		csky_ttym_port_shutdown(&s_ttym_data->tty_port);
	tty_port_destroy(&s_ttym_data->tty_port);
	kfree(s_ttym_data);
	s_ttym_data = NULL;

	dev_info(dev, "tty based on mailbox removed\n");
	return 0;
}

static const struct of_device_id csky_ttym_match[] = {
	{ .compatible = "csky,tty_mailbox" },
	{},
};

MODULE_DEVICE_TABLE(of, csky_ttym_match);

static struct platform_driver csky_ttym_driver = {
	.probe	= csky_ttym_probe,
	.remove	= csky_ttym_remove,
	.driver	= {
		.name	= TTY_MBOX_DRIVER_NAME,
		.of_match_table	= csky_ttym_match,
	},
};

module_platform_driver(csky_ttym_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TTY based on C-SKY mailbox specific functions");
MODULE_AUTHOR("Charles Lu <chongzhi_lu@csky.com>");
