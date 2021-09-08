// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
//
// AMD SPI controller driver
//
// Copyright (c) 2020, Advanced Micro Devices, Inc.
//
// Author: Sanjay R Mehta <sanju.mehta@amd.com>

#include <linux/acpi.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>

#define AMD_SPI_CTRL0_REG	0x00
#define AMD_SPI_EXEC_CMD	BIT(16)
#define AMD_SPI_FIFO_CLEAR	BIT(20)
#define AMD_SPI_BUSY		BIT(31)
#define AMD_SPI_ENABLE_REG	0x20


#define AMD_SPI_DUMMY_CYCL_REG		0x32
#define AMD_SPI_OPCODE_REG		0x45
#define AMD_SPI_CMD_TRIGGER_REG		0x47
#define AMD_SPI_TRIGGER_CMD		BIT(7)

#define AMD_SPI_OPCODE_MASK	0xFF

#define AMD_SPI_ALT_CS_REG	0x1D
#define AMD_SPI_ALT_CS_MASK	GENMASK(1, 0)

#define AMD_SPI_FIFO_BASE	0x80
#define AMD_SPI_TX_COUNT_REG	0x48
#define AMD_SPI_RX_COUNT_REG	0x4B
#define AMD_SPI_STATUS_REG	0x4C

#define AMD_SPI_FIFO_SIZE	70
#define AMD_SPI_MEM_SIZE	200

/* M_CMD OP codes for SPI */
#define AMD_SPI_XFER_TX		1
#define AMD_SPI_XFER_RX		2

struct amd_spi {
	void __iomem *io_remap_addr;
	unsigned long io_base_addr;
	u32 rom_addr;
	struct list_head rbuf_head;
	const struct amd_spi_devtype_data *devtype_data;
	struct spi_device *spi_dev;
	struct spi_master *master;
};

struct amd_spi_devtype_data {
	u8 version;
	int (*exec_op)(struct amd_spi *amd_spi);
	void (*set_op)(struct amd_spi *amd_spi, u8 cmd_opcode);
	int (*busy_wait)(struct amd_spi *amd_spi);
};

struct amd_spi_read_buffer {
	struct list_head node;
	u8 *buf;
	u8 len;
};

static inline u8 amd_spi_readreg8(struct amd_spi *amd_spi, int idx)
{
	return ioread8((u8 __iomem *)amd_spi->io_remap_addr + idx);
}

static inline void amd_spi_writereg8(struct amd_spi *amd_spi, int idx, u8 val)
{
	iowrite8(val, ((u8 __iomem *)amd_spi->io_remap_addr + idx));
}

static void amd_spi_setclear_reg8(struct amd_spi *amd_spi, int idx, u8 set, u8 clear)
{
	u8 tmp = amd_spi_readreg8(amd_spi, idx);

	tmp = (tmp & ~clear) | set;
	amd_spi_writereg8(amd_spi, idx, tmp);
}

static inline u32 amd_spi_readreg32(struct amd_spi *amd_spi, int idx)
{
	return ioread32((u8 __iomem *)amd_spi->io_remap_addr + idx);
}

static inline void amd_spi_writereg32(struct amd_spi *amd_spi, int idx, u32 val)
{
	iowrite32(val, ((u8 __iomem *)amd_spi->io_remap_addr + idx));
}

static inline void amd_spi_setclear_reg32(struct amd_spi *amd_spi, int idx, u32 set, u32 clear)
{
	u32 tmp = amd_spi_readreg32(amd_spi, idx);

	tmp = (tmp & ~clear) | set;
	amd_spi_writereg32(amd_spi, idx, tmp);
}

static inline void amd_spi_clear_chip(struct amd_spi *amd_spi, u8 chip_select)
{
	amd_spi_writereg8(amd_spi, AMD_SPI_ALT_CS_REG, chip_select & ~AMD_SPI_ALT_CS_MASK);
}

static void amd_spi_select_chip(struct amd_spi *amd_spi, u8 cs)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_ALT_CS_REG, cs, AMD_SPI_ALT_CS_MASK);
}

static void amd_spi_clear_fifo_ptr(struct amd_spi *amd_spi)
{
	amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, AMD_SPI_FIFO_CLEAR, AMD_SPI_FIFO_CLEAR);
}

static void amd_spi_set_opcode_v1(struct amd_spi *amd_spi, u8 cmd_opcode)
{
	amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, cmd_opcode, AMD_SPI_OPCODE_MASK);
}

static void amd_spi_set_opcode_v2(struct amd_spi *amd_spi, u8 cmd_opcode)
{
	amd_spi_writereg8(amd_spi, AMD_SPI_OPCODE_REG, cmd_opcode);
}

static inline void amd_spi_set_rx_count(struct amd_spi *amd_spi, u8 rx_count)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_RX_COUNT_REG, rx_count, 0xff);
}

static inline void amd_spi_set_tx_count(struct amd_spi *amd_spi, u8 tx_count)
{
	amd_spi_setclear_reg8(amd_spi, AMD_SPI_TX_COUNT_REG, tx_count, 0xff);
}

static int amd_spi_busy_wait_v1(struct amd_spi *amd_spi)
{
	int timeout = 100000;

	/* poll for SPI bus to become idle */
	while (amd_spi_readreg32(amd_spi, AMD_SPI_CTRL0_REG) & AMD_SPI_BUSY) {
		usleep_range(10, 20);
		if (timeout-- < 0)
			return -ETIMEDOUT;
	}

	return 0;
}

static int amd_spi_execute_opcode_v1(struct amd_spi *amd_spi)
{
	int ret;

	ret = amd_spi_busy_wait_v1(amd_spi);
	if (ret)
		return ret;

	/* Set ExecuteOpCode bit in the CTRL0 register */
	amd_spi_setclear_reg32(amd_spi, AMD_SPI_CTRL0_REG, AMD_SPI_EXEC_CMD, AMD_SPI_EXEC_CMD);

	return 0;
}

static int amd_spi_busy_wait_v2(struct amd_spi *amd_spi)
{
	int timeout = 100000;

	while (amd_spi_readreg32(amd_spi, AMD_SPI_STATUS_REG) & AMD_SPI_BUSY) {
		usleep_range(10, 20);
		if (timeout-- < 0)
			return -ETIMEDOUT;
	}

	return 0;
}

static int amd_spi_execute_opcode_v2(struct amd_spi *amd_spi)
{
	int ret;

	ret = amd_spi_busy_wait_v2(amd_spi);
	if (ret)
		return ret;

	amd_spi_setclear_reg8(amd_spi, AMD_SPI_CMD_TRIGGER_REG, AMD_SPI_TRIGGER_CMD,
			      AMD_SPI_TRIGGER_CMD);

	return 0;
}

static void amd_spi_clear_list(struct amd_spi *amd_spi)
{
	struct amd_spi_read_buffer *rbuf, *tmp;

	list_for_each_entry_safe(rbuf, tmp, &amd_spi->rbuf_head, node) {
		list_del(&rbuf->node);
		kfree(rbuf);
	}
}

static int amd_spi_master_setup(struct spi_device *spi)
{
	struct amd_spi *amd_spi = spi_master_get_devdata(spi->master);

	amd_spi_clear_fifo_ptr(amd_spi);

	return 0;
}

static inline void amd_spi_set_opcode(struct amd_spi *amd_spi, u8 opcode)
{
	amd_spi->devtype_data->set_op(amd_spi, opcode);
}

static inline int amd_spi_busy_wait(struct amd_spi *amd_spi)
{
	return amd_spi->devtype_data->busy_wait(amd_spi);
}

static inline int amd_spi_execute_opcode(struct amd_spi *amd_spi)
{
	return amd_spi->devtype_data->exec_op(amd_spi);
}

static int amd_spi_transfer(struct amd_spi *amd_spi, u8 opcode, u8 tx_len, u8 rx_len, u8 fifo_pos)
{
	struct amd_spi_read_buffer *rbuf;
	struct list_head *p;
	int ret, i;

	amd_spi_set_opcode(amd_spi, opcode);
	amd_spi_set_tx_count(amd_spi, tx_len);
	amd_spi_set_rx_count(amd_spi, rx_len);

	ret = amd_spi_execute_opcode(amd_spi);
	if (ret)
		return ret;

	if (!list_empty(&amd_spi->rbuf_head)) {
		ret = amd_spi_busy_wait(amd_spi);
		if (ret)
			return ret;
		list_for_each(p, &amd_spi->rbuf_head) {
			rbuf = list_entry(p, struct amd_spi_read_buffer, node);
			for (i = 0; i < rbuf->len; i++)
				rbuf->buf[i] = amd_spi_readreg8(amd_spi, fifo_pos++);
		}
		amd_spi_clear_list(amd_spi);
	}
	return 0;
 }

/* amd_spi_master_transfer expects a spi_message with no more than
 * AMD_SPI_FIFO_SIZE and no TX after a RX in the same CS The CS can not
 * be held between two amd_spi_execute_opcode so fill the FIFO with all
 * transfers until the first RX transfer
 */
static int amd_spi_transfer_one_message(struct spi_controller *ctrl, struct spi_message *msg)
{
	struct amd_spi *amd_spi = spi_master_get_devdata(ctrl);
	u8 tx_len = 0, rx_len = 0, opcode = 0, fifo_pos = AMD_SPI_FIFO_BASE;
	struct amd_spi_read_buffer *rbuf;
	struct spi_transfer *xfer;
	u8 *tx_buf;
	int ret, i;

	amd_spi_select_chip(amd_spi, msg->spi->chip_select);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		if (xfer->tx_buf) {
			tx_buf = (u8 *)xfer->tx_buf;
			if (!tx_len) {
				opcode = tx_buf[0];
				xfer->len--;
				tx_buf++;
			}

			tx_len += xfer->len;
			for (i = 0; i < xfer->len; i++)
				amd_spi_writereg8(amd_spi, fifo_pos++, tx_buf[i]);
		}
		if (xfer->rx_buf) {
			rx_len += xfer->len;
			rbuf = kmalloc(sizeof(*rbuf), GFP_KERNEL);
			if (!rbuf) {
				ret = -ENOMEM;
				goto complete;
			}
			rbuf->buf = (u8 *)xfer->rx_buf;
			rbuf->len = xfer->len;
			list_add(&rbuf->node, &amd_spi->rbuf_head);
		}

		if (xfer->cs_change) {
			ret = amd_spi_transfer(amd_spi, opcode, tx_len, rx_len, fifo_pos);
			if (ret)
				goto complete;

			msg->actual_length += rx_len;
			if (tx_len)
				msg->actual_length += tx_len + 1;

			fifo_pos = AMD_SPI_FIFO_BASE;
			opcode = 0;
			tx_len = 0;
			rx_len = 0;
		}
	}

	if (tx_len || rx_len) {
		ret = amd_spi_transfer(amd_spi, opcode, tx_len, rx_len, fifo_pos);
		if (ret)
			goto complete;

		msg->actual_length += rx_len;
		if (tx_len)
			msg->actual_length += tx_len + 1;
	}
	ret = 0;

complete:
	if (!list_empty(&amd_spi->rbuf_head))
		amd_spi_clear_list(amd_spi);
	/* complete the transaction */
	msg->status = ret;

	if (amd_spi->devtype_data->version)
		amd_spi_clear_chip(amd_spi, msg->spi->chip_select);

	spi_finalize_current_message(ctrl);

	return ret;
}

static size_t amd_spi_max_transfer_size(struct spi_device *spi)
{
	return AMD_SPI_FIFO_SIZE;
}

static int amd_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_master *master;
	struct amd_spi *amd_spi;
	int err = 0;

	/* Allocate storage for spi_master and driver private data */
	master = spi_alloc_master(dev, sizeof(struct amd_spi));
	if (!master) {
		dev_err(dev, "Error allocating SPI master\n");
		return -ENOMEM;
	}

	amd_spi = spi_master_get_devdata(master);
	amd_spi->io_remap_addr = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(amd_spi->io_remap_addr)) {
		err = PTR_ERR(amd_spi->io_remap_addr);
		dev_err(dev, "error %d ioremap of SPI registers failed\n", err);
		goto err_free_master;
	}
	dev_dbg(dev, "io_remap_address: %p\n", amd_spi->io_remap_addr);

	amd_spi->devtype_data = device_get_match_data(dev);
	if (!amd_spi->devtype_data) {
		err = -ENODEV;
		goto err_free_master;
	}

	/* Initialize the spi_master fields */
	master->bus_num = 0;
	master->num_chipselect = 4;
	master->mode_bits = 0;
	master->flags = SPI_MASTER_HALF_DUPLEX | SPI_CONTROLLER_NO_TX_RX_CS;
	master->setup = amd_spi_master_setup;
	master->max_transfer_size = amd_spi_max_transfer_size;
	master->max_message_size = amd_spi_max_transfer_size;
	master->transfer_one_message = amd_spi_transfer_one_message;

	INIT_LIST_HEAD(&amd_spi->rbuf_head);

	/* Register the controller with SPI framework */
	err = devm_spi_register_master(dev, master);
	if (err) {
		dev_err(dev, "error %d registering SPI controller\n", err);
		goto err_free_master;
	}

	return 0;

err_free_master:
	spi_master_put(master);

	return err;
}

static const struct amd_spi_devtype_data spi_v1 = {
	.exec_op	= amd_spi_execute_opcode_v1,
	.set_op		= amd_spi_set_opcode_v1,
	.busy_wait	= amd_spi_busy_wait_v1,
};

static const struct amd_spi_devtype_data spi_v2 = {
	.version	= 1,
	.exec_op	= amd_spi_execute_opcode_v2,
	.set_op		= amd_spi_set_opcode_v2,
	.busy_wait	= amd_spi_busy_wait_v2,
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id spi_acpi_match[] = {
	{ "AMDI0061", .driver_data = (kernel_ulong_t) &spi_v1 },
	{ "AMDI0062", .driver_data = (kernel_ulong_t) &spi_v2 },
	{},
};
MODULE_DEVICE_TABLE(acpi, spi_acpi_match);
#endif

static struct platform_driver amd_spi_driver = {
	.driver = {
		.name = "amd_spi",
		.acpi_match_table = ACPI_PTR(spi_acpi_match),
	},
	.probe = amd_spi_probe,
};

module_platform_driver(amd_spi_driver);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Sanjay Mehta <sanju.mehta@amd.com>");
MODULE_DESCRIPTION("AMD SPI Master Controller Driver");
