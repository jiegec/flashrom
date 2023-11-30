/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2018 Luc Verhaegen <libv@skynet.be>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * This adds support for the ATI/AMD Radeon SPI interfaces.
 */

#include <stdlib.h>
#include "programmer.h"
#include "hwaccess_physmap.h"
#include "platform/pci.h"

/* improve readability */
#define mmio_read(reg) pci_mmio_readl(device->bar + (reg))
#define mmio_write(reg, val) pci_mmio_writel((val), device->bar + (reg))

struct ati_spi_pci_private;
struct ati_spi_data {
	struct pci_dev *dev;
	uint8_t *bar;
	const struct ati_spi_pci_private *private;
	void *private_data;
};

struct ati_spi_pci_private {
	int io_bar;

	int (*save) (struct ati_spi_data *data);
	int (*restore) (struct ati_spi_data *data);
	int (*enable) (struct ati_spi_data *data);

	struct spi_master *master;
};

#define R600_GENERAL_PWRMGT		0x0618

#define R600_LOWER_GPIO_ENABLE		0x0710
#define R600_CTXSW_VID_LOWER_GPIO_CNTL	0x0718
#define R600_HIGH_VID_LOWER_GPIO_CNTL	0x071c
#define R600_MEDIUM_VID_LOWER_GPIO_CNTL	0x0720
#define R600_LOW_VID_LOWER_GPIO_CNTL	0x0724

#define R600_ROM_CNTL			0x1600

#define R600_GPIOPAD_MASK		0x1798
#define R600_GPIOPAD_A			0x179C
#define R600_GPIOPAD_EN			0x17A0

struct r600_spi_data {
	uint32_t reg_general_pwrmgt;
	uint32_t reg_lower_gpio_enable;
	uint32_t reg_ctxsw_vid_lower_gpio_cntl;
	uint32_t reg_high_vid_lower_gpio_cntl;
	uint32_t reg_medium_vid_lower_gpio_cntl;
	uint32_t reg_low_vid_lower_gpio_cntl;

	uint32_t reg_rom_cntl;
	uint32_t reg_gpiopad_mask;
	uint32_t reg_gpiopad_a;
	uint32_t reg_gpiopad_en;
};

/*
 * Save for later restore.
 */
static int r600_spi_save(struct ati_spi_data *device)
{
	struct r600_spi_data *data;

	msg_pdbg("%s();\n", __func__);

	if (device->private_data) {
		msg_perr("%s: device->private_data is already assigned.\n",
			 __func__);
		return -1;
	}

	data = calloc(1, sizeof(struct r600_spi_data));

	data->reg_general_pwrmgt = mmio_read(R600_GENERAL_PWRMGT);

	data->reg_lower_gpio_enable = mmio_read(R600_LOWER_GPIO_ENABLE);
	data->reg_ctxsw_vid_lower_gpio_cntl =
		mmio_read(R600_CTXSW_VID_LOWER_GPIO_CNTL);
	data->reg_high_vid_lower_gpio_cntl =
		mmio_read(R600_HIGH_VID_LOWER_GPIO_CNTL);
	data->reg_medium_vid_lower_gpio_cntl =
		mmio_read(R600_MEDIUM_VID_LOWER_GPIO_CNTL);
	data->reg_low_vid_lower_gpio_cntl =
		mmio_read(R600_LOW_VID_LOWER_GPIO_CNTL);

	data->reg_rom_cntl = mmio_read(R600_ROM_CNTL);

	data->reg_gpiopad_mask = mmio_read(R600_GPIOPAD_MASK);
	data->reg_gpiopad_a = mmio_read(R600_GPIOPAD_A);
	data->reg_gpiopad_en = mmio_read(R600_GPIOPAD_EN);

	device->private_data = data;

	return 0;
}

/*
 * Restore saved registers, in the order of enable writes.
 */
static int r600_spi_restore(struct ati_spi_data *device)
{
	struct r600_spi_data *data = device->private_data;

	msg_pdbg("%s();\n", __func__);

	if (!data) {
		msg_perr("%s: device->private_data is not assigned.\n",
			 __func__);
		return -1;
	}

	mmio_write(R600_ROM_CNTL, data->reg_rom_cntl);

	mmio_write(R600_GPIOPAD_A, data->reg_gpiopad_a);
	mmio_write(R600_GPIOPAD_EN, data->reg_gpiopad_en);
	mmio_write(R600_GPIOPAD_MASK, data->reg_gpiopad_mask);

	mmio_write(R600_GENERAL_PWRMGT, data->reg_general_pwrmgt);

	mmio_write(R600_CTXSW_VID_LOWER_GPIO_CNTL,
		   data->reg_ctxsw_vid_lower_gpio_cntl);
	mmio_write(R600_HIGH_VID_LOWER_GPIO_CNTL,
		   data->reg_high_vid_lower_gpio_cntl);
	mmio_write(R600_MEDIUM_VID_LOWER_GPIO_CNTL,
		   data->reg_medium_vid_lower_gpio_cntl);
	mmio_write(R600_LOW_VID_LOWER_GPIO_CNTL,
		   data->reg_low_vid_lower_gpio_cntl);

	mmio_write(R600_LOWER_GPIO_ENABLE, data->reg_lower_gpio_enable);

	free(data);
	device->private_data = NULL;

	return 0;
}

static const struct ati_spi_pci_private r600_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
};

struct ati_spi_pci_match {
	uint16_t vendor_id;
	uint16_t device_id;
	const struct ati_spi_pci_private *private;
};

const struct ati_spi_pci_match ati_spi_pci_devices[] = {
	{0x1002, 0x958D, &r600_spi_pci_private},
	{},
};

static const struct dev_entry devs_ati_spi[] = {
	{0x1002, 0x958D, NT, "AMD", "RV630 GL [FireGL V3600]" },
	{},
};

/*
 *
 */
static int ati_spi_shutdown(void *par_data)
{
	struct ati_spi_data *data = par_data;

	data->private->restore(data);

	free(par_data);
	return 0;
}

static int ati_spi_init(const struct programmer_cfg *cfg)
{
	struct pci_dev *dev = NULL;
	int i;
	uint8_t *bar;
	const struct ati_spi_pci_private *private;

	dev = pcidev_init(cfg, devs_ati_spi, PCI_BASE_ADDRESS_0);
	if (!dev)
		return 1;

	for (i = 0; ati_spi_pci_devices[i].private != NULL; i++)
		if ((dev->vendor_id == ati_spi_pci_devices[i].vendor_id) &&
			(dev->device_id == ati_spi_pci_devices[i].device_id))
			break;
	if (ati_spi_pci_devices[i].private == NULL)
		return 1;
	private = ati_spi_pci_devices[i].private;

	uintptr_t io_base_addr = pcidev_readbar(dev, private->io_bar);
	if (!io_base_addr)
		return 1;

	msg_pinfo("Detected ATI SPI I/O base address: 0x%"PRIxPTR".\n", io_base_addr);

	bar = rphysmap("ATI SPI", io_base_addr, 0x4000);
	if (bar == ERROR_PTR)
		return 1;

	struct ati_spi_data *data = calloc(1, sizeof(*data));
	if (!data) {
		msg_perr("Unable to allocate space for ati_spi_data\n");
		return 1;
	}
	data->dev = dev;
	data->bar = bar;
	data->private = private;

	register_shutdown(ati_spi_shutdown, data);

	int ret = private->save(data);
	if (ret)
		return ret;

	ret = private->enable(data);
	if (ret)
		return ret;

	if (register_spi_master(private->master, data))
		return 1;

	return 0;
}

const struct programmer_entry programmer_ati_spi = {
	.name			= "ati_spi",
	.type			= PCI,
	.devs.dev		= devs_ati_spi,
	.init			= ati_spi_init,
};
