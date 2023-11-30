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
#define mmio_read_byte(reg) pci_mmio_readb(device->bar + (reg))
#define mmio_write(reg, val) pci_mmio_writel((val), device->bar + (reg))
#define mmio_mask(reg, val, mask) pci_mmio_maskl((val), (mask), device->bar + (reg))
static void pci_mmio_maskl(uint32_t value, uint32_t mask, uint8_t *addr)
{
	uint32_t temp;

	temp = pci_mmio_readl(addr);
	temp &= ~mask;
	temp |= value & mask;
	pci_mmio_writel(temp, addr);
}

enum ati_spi_type {
	ATI_SPI_TYPE_R600 = 1,
	ATI_SPI_TYPE_RV730,
	ATI_SPI_TYPE_EVERGREEN,
	ATI_SPI_TYPE_NORTHERN_ISLAND,
	ATI_SPI_TYPE_SOUTHERN_ISLAND,
	ATI_SPI_TYPE_BONAIRE, /* First of sea island type spi interface */
	ATI_SPI_TYPE_HAWAII,
};

struct ati_spi_pci_private;
struct ati_spi_data {
	struct pci_dev *dev;
	uint8_t *bar;
	const struct ati_spi_pci_private *private;
	void *private_data;
};

struct ati_spi_pci_private {
	int io_bar;

	enum ati_spi_type type;

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
#define R600_PAGE_MIRROR_CNTL		0x1604
#define R600_ROM_SW_CNTL		0x1618
#define R600_ROM_SW_STATUS		0x161C
#define R600_ROM_SW_COMMAND		0x1620
#define R600_ROM_SW_DATA_0x00		0x1624
/* ... */
#define R600_ROM_SW_DATA_0xFC		0x1720
#define R600_ROM_SW_DATA(off)		(R600_ROM_SW_DATA_0x00 + (off))

#define R600_GPIOPAD_MASK		0x1798
#define R600_GPIOPAD_A			0x179C
#define R600_GPIOPAD_EN			0x17A0

#define R600_ROM_SW_STATUS_LOOP_COUNT 1000

#define R600_SPI_TRANSFER_SIZE 0x100

struct r600_spi_data {
	uint32_t reg_general_pwrmgt;
	uint32_t reg_lower_gpio_enable;
	uint32_t reg_ctxsw_vid_lower_gpio_cntl;
	uint32_t reg_high_vid_lower_gpio_cntl;
	uint32_t reg_medium_vid_lower_gpio_cntl;
	uint32_t reg_low_vid_lower_gpio_cntl;

	uint32_t reg_rom_cntl;
	uint32_t reg_page_mirror_cntl;
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
	data->reg_page_mirror_cntl = mmio_read(R600_PAGE_MIRROR_CNTL);

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

	mmio_write(R600_PAGE_MIRROR_CNTL, data->reg_page_mirror_cntl);

	free(data);
	device->private_data = NULL;

	return 0;
}

/*
 * Enable SPI Access.
 */
static int r600_spi_enable(struct ati_spi_data *device)
{
	const struct ati_spi_pci_private *private = device->private;
	int i;

	msg_pdbg("%s();\n", __func__);

	if (private->type == ATI_SPI_TYPE_RV730)
		/* As below, but also set the (unused?) pcie clk divider */
		mmio_mask(R600_ROM_CNTL, 0x19000002, 0xFF000002);
	else
		/* software enable clock gating and set sck divider to 1 */
		mmio_mask(R600_ROM_CNTL, 0x10000002, 0xF0000002);

	if (private->type == ATI_SPI_TYPE_NORTHERN_ISLAND) {
		/*
		 * Probably some other gpio lines...
		 * These are not restored by ATIs own tool.
		 */
		mmio_mask(0x64A0, 0x100, 0x100);
		mmio_mask(0x64A8, 0x100, 0x100);
		mmio_mask(0x64A4, 0x100, 0x100);
	}

	/* set gpio7,8,9 low */
	mmio_mask(R600_GPIOPAD_A, 0, 0x0700);
	/* gpio7 is input, gpio8/9 are output */
	mmio_mask(R600_GPIOPAD_EN, 0x0600, 0x0700);
	/* only allow software control on gpio7,8,9 */
	mmio_mask(R600_GPIOPAD_MASK, 0x0700, 0x0700);

	/* disable open drain pads */
	mmio_mask(R600_GENERAL_PWRMGT, 0, 0x0800);

	if ((private->type == ATI_SPI_TYPE_R600) ||
	    (private->type == ATI_SPI_TYPE_RV730) ||
	    (private->type == ATI_SPI_TYPE_EVERGREEN) ||
	    (private->type == ATI_SPI_TYPE_NORTHERN_ISLAND)) {
		mmio_mask(R600_CTXSW_VID_LOWER_GPIO_CNTL, 0, 0x0400);
		mmio_mask(R600_HIGH_VID_LOWER_GPIO_CNTL, 0, 0x0400);
		mmio_mask(R600_MEDIUM_VID_LOWER_GPIO_CNTL, 0, 0x0400);
		mmio_mask(R600_LOW_VID_LOWER_GPIO_CNTL, 0, 0x0400);
	}

	if ((private->type == ATI_SPI_TYPE_R600) ||
	    (private->type == ATI_SPI_TYPE_RV730))
		mmio_mask(R600_LOWER_GPIO_ENABLE, 0x0400, 0x0400);

	default_delay(1000);

	mmio_mask(R600_GPIOPAD_MASK, 0, 0x700);
	mmio_mask(R600_GPIOPAD_EN, 0, 0x700);
	mmio_mask(R600_GPIOPAD_A, 0, 0x00080000);

	/* page mirror usage */
	mmio_mask(R600_PAGE_MIRROR_CNTL, 0x04000000, 0x0C000000);

	if (mmio_read(R600_ROM_SW_STATUS)) {
		for (i = 0; i < R600_ROM_SW_STATUS_LOOP_COUNT; i++) {
			mmio_write(R600_ROM_SW_STATUS, 0);
			default_delay(1000);
			if (!mmio_read(R600_ROM_SW_STATUS))
				break;
		}

		if (i == R600_ROM_SW_STATUS_LOOP_COUNT) {
			msg_perr("%s: failed to clear R600_ROM_SW_STATUS\n",
				 __func__);
			return -1;
		}
	}

	return 0;
}

/*
 *
 */
static int r600_spi_command(const struct flashctx *flash,
		 unsigned int writecnt, unsigned int readcnt,
		 const unsigned char *writearr, unsigned char *readarr)
{
	const struct spi_master spi_master = flash->mst->spi;
	struct ati_spi_data *device =
		(struct ati_spi_data *) spi_master.data;
	uint32_t command, control;
	unsigned int i, command_size;

	msg_pdbg("%s(%p(%p), %d, %d, %p (0x%02X), %p);\n", __func__, flash,
		 device, writecnt, readcnt, writearr, writearr[0], readarr);

	if (!device) {
		msg_perr("%s: no device specified!\n", __func__);
		return -1;
	}

	command = writearr[0];
	if (writecnt > 1)
		command |= writearr[1] << 24;
	if (writecnt > 2)
		command |= writearr[2] << 16;
	if (writecnt > 3)
		command |= writearr[3] << 8;

	if (writecnt < 4)
		command_size = writecnt;
	else
		command_size = 4;

	mmio_write(R600_ROM_SW_COMMAND, command);

	/*
	 * For some reason, we have an endianness difference between reading
	 * and writing. Also, ati hw only does 32bit register write accesses.
	 * If you write 8bits, the upper bytes will be nulled. Reading is fine.
	 * Furthermore, due to flashrom infrastructure, we need to skip the
	 * command in the writearr.
	 */
	for (i = 4; i < writecnt; i += 4) {
		uint32_t value = 0;
		int remainder = writecnt - i;

		if (remainder > 4)
			remainder = 4;

		if (remainder > 0)
			value |= writearr[i + 0] << 24;
		if (remainder > 1)
			value |= writearr[i + 1] << 16;
		if (remainder > 2)
			value |= writearr[i + 2] << 8;
		if (remainder > 3)
			value |= writearr[i + 3] << 0;

		mmio_write(R600_ROM_SW_DATA(i - 4), value);
	}

	control = (command_size - 1) << 0x10;
	if (readcnt)
		control |= 0x40000 | readcnt;
	else if (writecnt > 4)
		control |= writecnt - 4;
	mmio_write(R600_ROM_SW_CNTL, control);

	for (i = 0; i < R600_ROM_SW_STATUS_LOOP_COUNT; i++) {
		if (mmio_read(R600_ROM_SW_STATUS))
			break;
		programmer_delay(flash, 1000);
	}

	if (i == R600_ROM_SW_STATUS_LOOP_COUNT) {
		msg_perr("%s: still waiting for R600_ROM_SW_STATUS\n",
			 __func__);
		return -1;
	}
	mmio_write(R600_ROM_SW_STATUS, 0);

	for (i = 0; i < readcnt; i++)
		readarr[i] = mmio_read_byte(R600_ROM_SW_DATA(i));

	return 0;
}

static struct spi_master r600_spi_master = {
	.features = 0,
	.max_data_read = R600_SPI_TRANSFER_SIZE,
	.max_data_write = R600_SPI_TRANSFER_SIZE + 1,
	.command = r600_spi_command,
	.read = default_spi_read,
	.write_256 = default_spi_write_256,
	.write_aai = default_spi_write_aai,
	.data = NULL, /* make this our flashrom_pci_device... */
};

/*
 * Used by all Rx6xx and RV710/RV770/RV710. RV730/RV740 use a slightly
 * different enable.
 */
static const struct ati_spi_pci_private r600_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.type = ATI_SPI_TYPE_R600,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

/*
 * Used by RV730/RV740.
 */
static const struct ati_spi_pci_private rv730_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.type = ATI_SPI_TYPE_RV730,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

/*
 * Used by Cypress, Juniper, Redwood and Cedar.
 */
static const struct ati_spi_pci_private evergreen_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.type = ATI_SPI_TYPE_EVERGREEN,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

/*
 * Used by Cayman, Barts, Turks, and Caicos.
 */
static const struct ati_spi_pci_private northern_island_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.type = ATI_SPI_TYPE_NORTHERN_ISLAND,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

/*
 * Used by Lombok.
 * Verde, Pitcairn, Hainan and Oland are pending.
 */
static const struct ati_spi_pci_private southern_island_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
	.type = ATI_SPI_TYPE_SOUTHERN_ISLAND,
	.save = r600_spi_save,
	.restore = r600_spi_restore,
	.enable = r600_spi_enable,
	.master = &r600_spi_master,
};

/*
 * Sea Island support.
 */
#define CI_MMIO_BAR			PCI_BASE_ADDRESS_5

#define CI_SMC1_INDEX			0x0208
#define CI_SMC1_DATA			0x020C

#define CI_GPIOPAD_MASK			0x0608
#define CI_GPIOPAD_A			0x060C
#define CI_GPIOPAD_EN			0x0610

#define CI_DRM_ID_STRAPS		0x5564

#define CI_GENERAL_PWRMGT		0xC0200000

#define CI_ROM_CNTL			0xC0600000
#define CI_PAGE_MIRROR_CNTL		0xC0600004
#define CI_ROM_SW_CNTL			0xC060001C
#define CI_ROM_SW_STATUS		0xC0600020
#define CI_ROM_SW_COMMAND		0xC0600024
#define CI_ROM_SW_DATA_0x00		0xC0600028
/* ... */
#define CI_ROM_SW_DATA_0xFC		0xC0600124
#define CI_ROM_SW_DATA(off)		(CI_ROM_SW_DATA_0x00 + (off))

#define CI_ROM_SW_STATUS_LOOP_COUNT 1000

#define CI_SPI_TRANSFER_SIZE 0x100

static uint32_t
_ci_smc_read(struct ati_spi_data *device, off_t address)
{
	mmio_write(CI_SMC1_INDEX, address);
	return mmio_read(CI_SMC1_DATA);
}
#define smc_read(reg) _ci_smc_read(device, (reg))

static void
_ci_smc_write(struct ati_spi_data *device, off_t address, uint32_t value)
{
	mmio_write(CI_SMC1_INDEX, address);
	mmio_write(CI_SMC1_DATA, value);
}
#define smc_write(reg, val) _ci_smc_write(device, (reg), (val))

static void
_ci_smc_mask(struct ati_spi_data *device, off_t address,
	    uint32_t value, uint32_t mask)
{
	mmio_write(CI_SMC1_INDEX, address);
	mmio_mask(CI_SMC1_DATA, value, mask);
}
#define smc_mask(reg, val, mask) _ci_smc_mask(device, (reg), (val), (mask))

struct ci_spi_data {
	uint32_t reg_gpiopad_mask;
	uint32_t reg_gpiopad_a;
	uint32_t reg_gpiopad_en;

	uint32_t reg_general_pwrmgt;

	uint32_t reg_rom_cntl;
	uint32_t reg_page_mirror_cntl;
};

/*
 * Save for later restore.
 */
static int ci_spi_save(struct ati_spi_data *device)
{
	struct ci_spi_data *data;

	msg_pdbg("%s();\n", __func__);

	if (device->private_data) {
		msg_perr("%s: device->private_data is already assigned.\n",
			 __func__);
		return -1;
	}

	data = calloc(1, sizeof(struct ci_spi_data));

	data->reg_general_pwrmgt = smc_read(CI_GENERAL_PWRMGT);

	data->reg_rom_cntl = smc_read(CI_ROM_CNTL);
	data->reg_page_mirror_cntl = smc_read(CI_PAGE_MIRROR_CNTL);

	data->reg_gpiopad_mask = mmio_read(CI_GPIOPAD_MASK);
	data->reg_gpiopad_a = mmio_read(CI_GPIOPAD_A);
	data->reg_gpiopad_en = mmio_read(CI_GPIOPAD_EN);

	device->private_data = data;

	return 0;
}

/*
 * Restore saved registers, in the order of enable writes.
 */
static int ci_spi_restore(struct ati_spi_data *device)
{
	struct ci_spi_data *data = device->private_data;

	msg_pdbg("%s();\n", __func__);

	if (!data) {
		msg_perr("%s: device->private_data is not assigned.\n",
			 __func__);
		return -1;
	}

	smc_write(CI_ROM_CNTL, data->reg_rom_cntl);

	mmio_write(CI_GPIOPAD_A, data->reg_gpiopad_a);
	mmio_write(CI_GPIOPAD_EN, data->reg_gpiopad_en);
	mmio_write(CI_GPIOPAD_MASK, data->reg_gpiopad_mask);

	smc_write(CI_GENERAL_PWRMGT, data->reg_general_pwrmgt);

	smc_write(CI_PAGE_MIRROR_CNTL, data->reg_page_mirror_cntl);

	free(data);
	device->private_data = NULL;

	return 0;
}

/*
 * Enable SPI Access.
 */
static int ci_spi_enable(struct ati_spi_data *device)
{
	const struct ati_spi_pci_private *private = device->private;
	int i;

	msg_pdbg("%s();\n", __func__);

	/* set sck divider to 1 */
	smc_mask(CI_ROM_CNTL, 0x10000000, 0xF0000000);
	/* software enable clock gating */
	if (private->type == ATI_SPI_TYPE_BONAIRE) {
		uint32_t drm = mmio_read(CI_DRM_ID_STRAPS);

		if (drm & 0xF0000000)
			smc_mask(CI_ROM_CNTL, 0, 0x0000002);
		else
			smc_mask(CI_ROM_CNTL, 0x0000002, 0x0000002);
	} else
		smc_mask(CI_ROM_CNTL, 0x00000002, 0x00000002);

	/* set gpio7,8,9 low */
	mmio_mask(CI_GPIOPAD_A, 0, 0x0700);
	/* gpio7 is input, gpio8/9 are output */
	mmio_mask(CI_GPIOPAD_EN, 0x0600, 0x0700);
	/* only allow software control on gpio7,8,9 */
	mmio_mask(CI_GPIOPAD_MASK, 0x0700, 0x0700);

	if (private->type != ATI_SPI_TYPE_BONAIRE) {
		mmio_mask(CI_GPIOPAD_MASK, 0x40000000, 0x40000000);
		mmio_mask(CI_GPIOPAD_EN, 0x40000000, 0x40000000);
		mmio_mask(CI_GPIOPAD_A, 0x40000000, 0x40000000);
	}

	/* disable open drain pads */
	smc_mask(CI_GENERAL_PWRMGT, 0, 0x0800);

	default_delay(1000);

	mmio_mask(CI_GPIOPAD_MASK, 0, 0x700);
	mmio_mask(CI_GPIOPAD_EN, 0, 0x700);
	mmio_mask(CI_GPIOPAD_A, 0, 0x700);

	/* this feels like a remnant of generations past */
	mmio_mask(CI_GPIOPAD_MASK, 0, 0x80000);
	mmio_mask(CI_GPIOPAD_EN, 0, 0x80000);
	mmio_mask(CI_GPIOPAD_A, 0, 0x80000);

	/* page mirror usage */
	smc_mask(CI_PAGE_MIRROR_CNTL, 0x04000000, 0x0C000000);

	if (smc_read(CI_ROM_SW_STATUS)) {
		for (i = 0; i < CI_ROM_SW_STATUS_LOOP_COUNT; i++) {
			smc_write(CI_ROM_SW_STATUS, 0);
			default_delay(1000);
			if (!smc_read(CI_ROM_SW_STATUS))
				break;
		}

		if (i == CI_ROM_SW_STATUS_LOOP_COUNT) {
			msg_perr("%s: failed to clear CI_ROM_SW_STATUS\n",
				 __func__);
			return -1;
		}
	}

	return 0;
}

/*
 *
 */
static int ci_spi_command(const struct flashctx *flash,
	       unsigned int writecnt, unsigned int readcnt,
	       const unsigned char *writearr, unsigned char *readarr)
{
	const struct spi_master spi_master = flash->mst->spi;
	struct ati_spi_data *device =
		(struct ati_spi_data *) spi_master.data;
	const struct ati_spi_pci_private *private = device->private;
	uint32_t command, control;
	unsigned int i, command_size;

	msg_pdbg("%s(%p(%p), %d, %d, %p (0x%02X), %p);\n", __func__, flash,
		 device, writecnt, readcnt, writearr, writearr[0], readarr);

	if (!device) {
		msg_perr("%s: no device specified!\n", __func__);
		return -1;
	}

	command = writearr[0];
	if (writecnt > 1)
		command |= writearr[1] << 24;
	if (writecnt > 2)
		command |= writearr[2] << 16;
	if (writecnt > 3)
		command |= writearr[3] << 8;

	if (writecnt < 4)
		command_size = writecnt;
	else
		command_size = 4;

	smc_write(CI_ROM_SW_COMMAND, command);

	/*
	 * For some reason, we have an endianness difference between reading
	 * and writing. Also, ati hw only does 32bit register write accesses.
	 * If you write 8bits, the upper bytes will be nulled. Reading is fine.
	 * Furthermore, due to flashrom infrastructure, we need to skip the
	 * command in the writearr.
	 */
	for (i = 4; i < writecnt; i += 4) {
		uint32_t value = 0;
		int remainder = writecnt - i;

		if (remainder > 4)
			remainder = 4;

		if (remainder > 0)
			value |= writearr[i + 0] << 24;
		if (remainder > 1)
			value |= writearr[i + 1] << 16;
		if (remainder > 2)
			value |= writearr[i + 2] << 8;
		if (remainder > 3)
			value |= writearr[i + 3] << 0;

		/* Bonaire has a gap between 0xD8 and 0xE8 */
		if ((private->type == ATI_SPI_TYPE_BONAIRE) && (i >= 0xdc))
			smc_write(CI_ROM_SW_DATA(i + 0x0C), value);
		else
			smc_write(CI_ROM_SW_DATA(i - 4), value);
	}

	control = (command_size - 1) << 0x10;
	if (readcnt)
		control |= 0x40000 | readcnt;
	else if (writecnt > 4)
		control |= writecnt - 4;
	smc_write(CI_ROM_SW_CNTL, control);

	for (i = 0; i < CI_ROM_SW_STATUS_LOOP_COUNT; i++) {
		if (smc_read(CI_ROM_SW_STATUS))
			break;
		programmer_delay(flash, 1000);
	}

	if (i == CI_ROM_SW_STATUS_LOOP_COUNT) {
		msg_perr("%s: still waiting for CI_ROM_SW_STATUS\n",
			 __func__);
		return -1;
	}
	smc_write(CI_ROM_SW_STATUS, 0);

	for (i = 0; i < readcnt; i += 4) {
		uint32_t value;
		int remainder = readcnt - i;

		/* Bonaire has a gap between 0xD8 and 0xE8 */
		if ((private->type == ATI_SPI_TYPE_BONAIRE) && (i >= 0xd8))
			value = smc_read(CI_ROM_SW_DATA(i + 0x10));
		else
			value = smc_read(CI_ROM_SW_DATA(i));

		if (remainder > 0)
			readarr[i] = value;
		if (remainder > 1)
			readarr[i + 1] = value >> 8;
		if (remainder > 2)
			readarr[i + 2] = value >> 16;
		if (remainder > 3)
			readarr[i + 3] = value >> 24;
	}

	return 0;
}

static struct spi_master ci_spi_master = {
	.features = 0,
	.max_data_read = CI_SPI_TRANSFER_SIZE,
	.max_data_write = CI_SPI_TRANSFER_SIZE + 1,
	.command = ci_spi_command,
	.read = default_spi_read,
	.write_256 = default_spi_write_256,
	.write_aai = default_spi_write_aai,
	.data = NULL, /* make this our flashrom_pci_device... */
};

/*
 * Used by Bonaire
 */
static const struct ati_spi_pci_private bonaire_spi_pci_private = {
	.io_bar = CI_MMIO_BAR,
	.type = ATI_SPI_TYPE_BONAIRE,
	.save = ci_spi_save,
	.restore = ci_spi_restore,
	.enable = ci_spi_enable,
	.master = &ci_spi_master,
};

/*
 * Used by Hawaii
 */
static const struct ati_spi_pci_private hawaii_spi_pci_private = {
	.io_bar = CI_MMIO_BAR,
	.type = ATI_SPI_TYPE_HAWAII,
	.save = ci_spi_save,
	.restore = ci_spi_restore,
	.enable = ci_spi_enable,
	.master = &ci_spi_master,
};

struct ati_spi_pci_match {
	uint16_t vendor_id;
	uint16_t device_id;
	const struct ati_spi_pci_private *private;
};

const struct ati_spi_pci_match ati_spi_pci_devices[] = {
	{0x1002, 0x6640, &bonaire_spi_pci_private},
	{0x1002, 0x6641, &bonaire_spi_pci_private},
	{0x1002, 0x6646, &bonaire_spi_pci_private},
	{0x1002, 0x6647, &bonaire_spi_pci_private},
	{0x1002, 0x6649, &bonaire_spi_pci_private},
	{0x1002, 0x6650, &bonaire_spi_pci_private},
	{0x1002, 0x6651, &bonaire_spi_pci_private},
	{0x1002, 0x6658, &bonaire_spi_pci_private},
	{0x1002, 0x665C, &bonaire_spi_pci_private},
	{0x1002, 0x665D, &bonaire_spi_pci_private},
	{0x1002, 0x665F, &bonaire_spi_pci_private},
	{0x1002, 0x6704, &northern_island_spi_pci_private},
	{0x1002, 0x6707, &northern_island_spi_pci_private},
	{0x1002, 0x6718, &northern_island_spi_pci_private},
	{0x1002, 0x6719, &northern_island_spi_pci_private},
	{0x1002, 0x671C, &northern_island_spi_pci_private},
	{0x1002, 0x671D, &northern_island_spi_pci_private},
	{0x1002, 0x671F, &northern_island_spi_pci_private},
	{0x1002, 0x6720, &northern_island_spi_pci_private},
	{0x1002, 0x6738, &northern_island_spi_pci_private},
	{0x1002, 0x6739, &northern_island_spi_pci_private},
	{0x1002, 0x673E, &northern_island_spi_pci_private},
	{0x1002, 0x6740, &northern_island_spi_pci_private},
	{0x1002, 0x6741, &northern_island_spi_pci_private},
	{0x1002, 0x6742, &northern_island_spi_pci_private},
	{0x1002, 0x6743, &northern_island_spi_pci_private},
	{0x1002, 0x6749, &northern_island_spi_pci_private},
	{0x1002, 0x674A, &northern_island_spi_pci_private},
	{0x1002, 0x6750, &northern_island_spi_pci_private},
	{0x1002, 0x6751, &northern_island_spi_pci_private},
	{0x1002, 0x6758, &northern_island_spi_pci_private},
	{0x1002, 0x6759, &northern_island_spi_pci_private},
	{0x1002, 0x675b, &northern_island_spi_pci_private},
	{0x1002, 0x675d, &northern_island_spi_pci_private},
	{0x1002, 0x675f, &northern_island_spi_pci_private},
	{0x1002, 0x6760, &northern_island_spi_pci_private},
	{0x1002, 0x6761, &northern_island_spi_pci_private},
	{0x1002, 0x6763, &northern_island_spi_pci_private},
	{0x1002, 0x6764, &northern_island_spi_pci_private},
	{0x1002, 0x6765, &northern_island_spi_pci_private},
	{0x1002, 0x6766, &northern_island_spi_pci_private},
	{0x1002, 0x6767, &northern_island_spi_pci_private},
	{0x1002, 0x6768, &northern_island_spi_pci_private},
	{0x1002, 0x6770, &northern_island_spi_pci_private},
	{0x1002, 0x6771, &northern_island_spi_pci_private},
	{0x1002, 0x6772, &northern_island_spi_pci_private},
	{0x1002, 0x6778, &northern_island_spi_pci_private},
	{0x1002, 0x6779, &northern_island_spi_pci_private},
	{0x1002, 0x677B, &northern_island_spi_pci_private},
	{0x1002, 0x67A0, &hawaii_spi_pci_private},
	{0x1002, 0x67A1, &hawaii_spi_pci_private},
	{0x1002, 0x67A2, &hawaii_spi_pci_private},
	{0x1002, 0x67A8, &hawaii_spi_pci_private},
	{0x1002, 0x67A9, &hawaii_spi_pci_private},
	{0x1002, 0x67AA, &hawaii_spi_pci_private},
	{0x1002, 0x67B0, &hawaii_spi_pci_private},
	{0x1002, 0x67B1, &hawaii_spi_pci_private},
	{0x1002, 0x67B9, &hawaii_spi_pci_private},
	{0x1002, 0x67BE, &hawaii_spi_pci_private},
	{0x1002, 0x6840, &southern_island_spi_pci_private},
	{0x1002, 0x6841, &southern_island_spi_pci_private},
	{0x1002, 0x6842, &southern_island_spi_pci_private},
	{0x1002, 0x6843, &southern_island_spi_pci_private},
	{0x1002, 0x6880, &evergreen_spi_pci_private},
	{0x1002, 0x6888, &evergreen_spi_pci_private},
	{0x1002, 0x6889, &evergreen_spi_pci_private},
	{0x1002, 0x688A, &evergreen_spi_pci_private},
	{0x1002, 0x688C, &evergreen_spi_pci_private},
	{0x1002, 0x688D, &evergreen_spi_pci_private},
	{0x1002, 0x6898, &evergreen_spi_pci_private},
	{0x1002, 0x6899, &evergreen_spi_pci_private},
	{0x1002, 0x689B, &evergreen_spi_pci_private},
	{0x1002, 0x689C, &evergreen_spi_pci_private},
	{0x1002, 0x689D, &evergreen_spi_pci_private},
	{0x1002, 0x689E, &evergreen_spi_pci_private},
	{0x1002, 0x68A0, &evergreen_spi_pci_private},
	{0x1002, 0x68A1, &evergreen_spi_pci_private},
	{0x1002, 0x68A8, &evergreen_spi_pci_private},
	{0x1002, 0x68A9, &evergreen_spi_pci_private},
	{0x1002, 0x68B8, &evergreen_spi_pci_private},
	{0x1002, 0x68B9, &evergreen_spi_pci_private},
	{0x1002, 0x68BA, &evergreen_spi_pci_private},
	{0x1002, 0x68BE, &evergreen_spi_pci_private},
	{0x1002, 0x68BF, &evergreen_spi_pci_private},
	{0x1002, 0x68C0, &evergreen_spi_pci_private},
	{0x1002, 0x68C1, &evergreen_spi_pci_private},
	{0x1002, 0x68C7, &evergreen_spi_pci_private},
	{0x1002, 0x68C8, &evergreen_spi_pci_private},
	{0x1002, 0x68C9, &evergreen_spi_pci_private},
	{0x1002, 0x68D8, &evergreen_spi_pci_private},
	{0x1002, 0x68D9, &evergreen_spi_pci_private},
	{0x1002, 0x68DA, &evergreen_spi_pci_private},
	{0x1002, 0x68DE, &evergreen_spi_pci_private},
	{0x1002, 0x68E0, &evergreen_spi_pci_private},
	{0x1002, 0x68E1, &evergreen_spi_pci_private},
	{0x1002, 0x68E4, &evergreen_spi_pci_private},
	{0x1002, 0x68E5, &evergreen_spi_pci_private},
	{0x1002, 0x68E8, &evergreen_spi_pci_private},
	{0x1002, 0x68E9, &evergreen_spi_pci_private},
	{0x1002, 0x68F1, &evergreen_spi_pci_private},
	{0x1002, 0x68F2, &evergreen_spi_pci_private},
	{0x1002, 0x68F8, &evergreen_spi_pci_private},
	{0x1002, 0x68F9, &evergreen_spi_pci_private},
	{0x1002, 0x68FA, &evergreen_spi_pci_private},
	{0x1002, 0x68FE, &evergreen_spi_pci_private},
	{0x1002, 0x9400, &r600_spi_pci_private},
	{0x1002, 0x9401, &r600_spi_pci_private},
	{0x1002, 0x9402, &r600_spi_pci_private},
	{0x1002, 0x9403, &r600_spi_pci_private},
	{0x1002, 0x9405, &r600_spi_pci_private},
	{0x1002, 0x940A, &r600_spi_pci_private},
	{0x1002, 0x940B, &r600_spi_pci_private},
	{0x1002, 0x940F, &r600_spi_pci_private},
	{0x1002, 0x9440, &r600_spi_pci_private},
	{0x1002, 0x9441, &r600_spi_pci_private},
	{0x1002, 0x9442, &r600_spi_pci_private},
	{0x1002, 0x9443, &r600_spi_pci_private},
	{0x1002, 0x9444, &r600_spi_pci_private},
	{0x1002, 0x9446, &r600_spi_pci_private},
	{0x1002, 0x944A, &r600_spi_pci_private},
	{0x1002, 0x944B, &r600_spi_pci_private},
	{0x1002, 0x944C, &r600_spi_pci_private},
	{0x1002, 0x944e, &r600_spi_pci_private},
	{0x1002, 0x9450, &r600_spi_pci_private},
	{0x1002, 0x9452, &r600_spi_pci_private},
	{0x1002, 0x9456, &r600_spi_pci_private},
	{0x1002, 0x945A, &r600_spi_pci_private},
	{0x1002, 0x9460, &r600_spi_pci_private},
	{0x1002, 0x9462, &r600_spi_pci_private},
	{0x1002, 0x946A, &r600_spi_pci_private},
	{0x1002, 0x9480, &rv730_spi_pci_private},
	{0x1002, 0x9488, &rv730_spi_pci_private},
	{0x1002, 0x9489, &rv730_spi_pci_private},
	{0x1002, 0x9490, &rv730_spi_pci_private},
	{0x1002, 0x9491, &rv730_spi_pci_private},
	{0x1002, 0x9495, &rv730_spi_pci_private},
	{0x1002, 0x9498, &rv730_spi_pci_private},
	{0x1002, 0x949C, &rv730_spi_pci_private},
	{0x1002, 0x949E, &rv730_spi_pci_private},
	{0x1002, 0x949F, &rv730_spi_pci_private},
	{0x1002, 0x94A0, &rv730_spi_pci_private},
	{0x1002, 0x94A1, &rv730_spi_pci_private},
	{0x1002, 0x94A3, &rv730_spi_pci_private},
	{0x1002, 0x94B3, &rv730_spi_pci_private},
	{0x1002, 0x94B4, &rv730_spi_pci_private},
	{0x1002, 0x94C1, &r600_spi_pci_private},
	{0x1002, 0x94C3, &r600_spi_pci_private},
	{0x1002, 0x94C4, &r600_spi_pci_private},
	{0x1002, 0x94C5, &r600_spi_pci_private},
	{0x1002, 0x94C6, &r600_spi_pci_private},
	{0x1002, 0x94C7, &r600_spi_pci_private},
	{0x1002, 0x94C8, &r600_spi_pci_private},
	{0x1002, 0x94C9, &r600_spi_pci_private},
	{0x1002, 0x94CB, &r600_spi_pci_private},
	{0x1002, 0x94CC, &r600_spi_pci_private},
	{0x1002, 0x9500, &r600_spi_pci_private},
	{0x1002, 0x9501, &r600_spi_pci_private},
	{0x1002, 0x9504, &r600_spi_pci_private},
	{0x1002, 0x9505, &r600_spi_pci_private},
	{0x1002, 0x9506, &r600_spi_pci_private},
	{0x1002, 0x9507, &r600_spi_pci_private},
	{0x1002, 0x9508, &r600_spi_pci_private},
	{0x1002, 0x9509, &r600_spi_pci_private},
	{0x1002, 0x950F, &r600_spi_pci_private},
	{0x1002, 0x9511, &r600_spi_pci_private},
	{0x1002, 0x9513, &r600_spi_pci_private},
	{0x1002, 0x9515, &r600_spi_pci_private},
	{0x1002, 0x9519, &r600_spi_pci_private},
	{0x1002, 0x9540, &r600_spi_pci_private},
	{0x1002, 0x954F, &r600_spi_pci_private},
	{0x1002, 0x9552, &r600_spi_pci_private},
	{0x1002, 0x9553, &r600_spi_pci_private},
	{0x1002, 0x9555, &r600_spi_pci_private},
	{0x1002, 0x9557, &r600_spi_pci_private},
	{0x1002, 0x955F, &r600_spi_pci_private},
	{0x1002, 0x9580, &r600_spi_pci_private},
	{0x1002, 0x9581, &r600_spi_pci_private},
	{0x1002, 0x9583, &r600_spi_pci_private},
	{0x1002, 0x9586, &r600_spi_pci_private},
	{0x1002, 0x9587, &r600_spi_pci_private},
	{0x1002, 0x9588, &r600_spi_pci_private},
	{0x1002, 0x9589, &r600_spi_pci_private},
	{0x1002, 0x958A, &r600_spi_pci_private},
	{0x1002, 0x958B, &r600_spi_pci_private},
	{0x1002, 0x958C, &r600_spi_pci_private},
	{0x1002, 0x958D, &r600_spi_pci_private},
	{0x1002, 0x958E, &r600_spi_pci_private},
	{0x1002, 0x9591, &r600_spi_pci_private},
	{0x1002, 0x9593, &r600_spi_pci_private},
	{0x1002, 0x9595, &r600_spi_pci_private},
	{0x1002, 0x9596, &r600_spi_pci_private},
	{0x1002, 0x9597, &r600_spi_pci_private},
	{0x1002, 0x9598, &r600_spi_pci_private},
	{0x1002, 0x9599, &r600_spi_pci_private},
	{0x1002, 0x95C0, &r600_spi_pci_private},
	{0x1002, 0x95C2, &r600_spi_pci_private},
	{0x1002, 0x95C4, &r600_spi_pci_private},
	{0x1002, 0x95C5, &r600_spi_pci_private},
	{0x1002, 0x95C6, &r600_spi_pci_private},
	{0x1002, 0x95C9, &r600_spi_pci_private},
	{0x1002, 0x95CC, &r600_spi_pci_private},
	{0x1002, 0x95CD, &r600_spi_pci_private},
	{0x1002, 0x95CF, &r600_spi_pci_private},
	{},
};

static const struct dev_entry devs_ati_spi[] = {
	{0x1002, 0x6640, NT, "AMD", "Saturn XT [FirePro M6100]" },
	{0x1002, 0x6641, NT, "AMD", "Saturn PRO [Radeon HD 8930M]" },
	{0x1002, 0x6646, NT, "AMD", "Bonaire XT [Radeon R9 M280X / FirePro W6150M]" },
	{0x1002, 0x6647, NT, "AMD", "Saturn PRO/XT [Radeon R9 M270X/M280X]" },
	{0x1002, 0x6649, NT, "AMD", "Bonaire [FirePro W5100]" },
	{0x1002, 0x6650, NT, "AMD", "Bonaire" },
	{0x1002, 0x6651, NT, "AMD", "Bonaire" },
	{0x1002, 0x6658, NT, "AMD", "Bonaire XTX [Radeon R7 260X/360]" },
	{0x1002, 0x665C, NT, "AMD", "Bonaire XT [Radeon HD 7790/8770 / R7 360 / R9 260/360 OEM]" },
	{0x1002, 0x665D, NT, "AMD", "Bonaire [Radeon R7 200 Series]" },
	{0x1002, 0x665F, OK, "AMD", "Tobago PRO [Radeon R7 360 / R9 360 OEM]" },
	{0x1002, 0x6704, NT, "AMD", "Cayman PRO GL [FirePro V7900]" },
	{0x1002, 0x6707, NT, "AMD", "Cayman LE GL [FirePro V5900]" },
	{0x1002, 0x6718, NT, "AMD", "Cayman XT [Radeon HD 6970]" },
	{0x1002, 0x6719, NT, "AMD", "Cayman PRO [Radeon HD 6950]" },
	{0x1002, 0x671C, NT, "AMD", "Antilles [Radeon HD 6990]" },
	{0x1002, 0x671D, NT, "AMD", "Antilles [Radeon HD 6990]" },
	{0x1002, 0x671F, NT, "AMD", "Cayman CE [Radeon HD 6930]" },
	{0x1002, 0x6720, NT, "AMD", "Blackcomb [Radeon HD 6970M/6990M]" },
	{0x1002, 0x6738, NT, "AMD", "Barts XT [Radeon HD 6870]" },
	{0x1002, 0x6739, NT, "AMD", "Barts PRO [Radeon HD 6850]" },
	{0x1002, 0x673E, NT, "AMD", "Barts LE [Radeon HD 6790]" },
	{0x1002, 0x6740, NT, "AMD", "Whistler [Radeon HD 6730M/6770M/7690M XT]" },
	{0x1002, 0x6741, NT, "AMD", "Whistler [Radeon HD 6630M/6650M/6750M/7670M/7690M]" },
	{0x1002, 0x6742, NT, "AMD", "Whistler LE [Radeon HD 6610M/7610M]" },
	{0x1002, 0x6743, NT, "AMD", "Whistler [Radeon E6760]" },
	{0x1002, 0x6749, NT, "AMD", "Turks GL [FirePro V4900]" },
	{0x1002, 0x674A, NT, "AMD", "Turks GL [FirePro V3900]" },
	{0x1002, 0x6750, NT, "AMD", "Onega [Radeon HD 6650A/7650A]" },
	{0x1002, 0x6751, NT, "AMD", "Turks [Radeon HD 7650A/7670A]" },
	{0x1002, 0x6758, NT, "AMD", "Turks XT [Radeon HD 6670/7670]" },
	{0x1002, 0x6759, NT, "AMD", "Turks PRO [Radeon HD 6570/7570/8550 / R5 230]" },
	{0x1002, 0x675b, NT, "AMD", "Turks [Radeon HD 7600 Series]" },
	{0x1002, 0x675d, NT, "AMD", "Turks PRO [Radeon HD 7570]" },
	{0x1002, 0x675f, NT, "AMD", "Turks LE [Radeon HD 5570/6510/7510/8510]" },
	{0x1002, 0x6760, NT, "AMD", "Seymour [Radeon HD 6400M/7400M Series]" },
	{0x1002, 0x6761, NT, "AMD", "Seymour LP [Radeon HD 6430M]" },
	{0x1002, 0x6763, NT, "AMD", "Seymour [Radeon E6460]" },
	{0x1002, 0x6764, NT, "AMD", "Seymour [Radeon HD 6400M Series]" },
	{0x1002, 0x6765, NT, "AMD", "Seymour [Radeon HD 6400M Series]" },
	{0x1002, 0x6766, NT, "AMD", "Caicos" },
	{0x1002, 0x6767, NT, "AMD", "Caicos" },
	{0x1002, 0x6768, NT, "AMD", "Caicos" },
	{0x1002, 0x6770, NT, "AMD", "Caicos [Radeon HD 6450A/7450A]" },
	{0x1002, 0x6771, NT, "AMD", "Caicos XTX [Radeon HD 8490 / R5 235X OEM]" },
	{0x1002, 0x6772, NT, "AMD", "Caicos [Radeon HD 7450A]" },
	{0x1002, 0x6778, NT, "AMD", "Caicos XT [Radeon HD 7470/8470 / R5 235/310 OEM]" },
	{0x1002, 0x6779, NT, "AMD", "Caicos [Radeon HD 6450/7450/8450 / R5 230 OEM]" },
	{0x1002, 0x677B, NT, "AMD", "Caicos PRO [Radeon HD 7450]" },
	{0x1002, 0x67A0, NT, "AMD", "Hawaii XT GL [FirePro W9100]" },
	{0x1002, 0x67A1, NT, "AMD", "Hawaii PRO GL [FirePro W8100]" },
	{0x1002, 0x67A2, NT, "AMD", "Hawaii GL" },
	{0x1002, 0x67A8, NT, "AMD", "Hawaii" },
	{0x1002, 0x67A9, NT, "AMD", "Hawaii" },
	{0x1002, 0x67AA, NT, "AMD", "Hawaii" },
	{0x1002, 0x67B0, NT, "AMD", "Hawaii XT / Grenada XT [Radeon R9 290X/390X]" },
	{0x1002, 0x67B1, NT, "AMD", "Hawaii PRO [Radeon R9 290/390]" },
	{0x1002, 0x67B9, NT, "AMD", "Vesuvius [Radeon R9 295X2]" },
	{0x1002, 0x67BE, NT, "AMD", "Hawaii LE" },
	{0x1002, 0x6840, NT, "AMD", "Thames [Radeon HD 7500M/7600M Series]" },
	{0x1002, 0x6841, NT, "AMD", "Thames [Radeon HD 7550M/7570M/7650M]" },
	{0x1002, 0x6842, NT, "AMD", "Thames LE [Radeon HD 7000M Series]" },
	{0x1002, 0x6843, NT, "AMD", "Thames [Radeon HD 7670M]" },
	{0x1002, 0x6880, NT, "AMD", "Lexington [Radeon HD 6550M]" },
	{0x1002, 0x6888, NT, "AMD", "Cypress XT [FirePro V8800]" },
	{0x1002, 0x6889, NT, "AMD", "Cypress PRO [FirePro V7800]" },
	{0x1002, 0x688A, NT, "AMD", "Cypress XT [FirePro V9800]" },
	{0x1002, 0x688C, NT, "AMD", "Cypress XT GL [FireStream 9370]" },
	{0x1002, 0x688D, NT, "AMD", "Cypress PRO GL [FireStream 9350]" },
	{0x1002, 0x6898, NT, "AMD", "Cypress XT [Radeon HD 5870]" },
	{0x1002, 0x6899, NT, "AMD", "Cypress PRO [Radeon HD 5850]" },
	{0x1002, 0x689B, NT, "AMD", "Cypress PRO [Radeon HD 6800 Series]" },
	{0x1002, 0x689C, NT, "AMD", "Hemlock [Radeon HD 5970]" },
	{0x1002, 0x689D, NT, "AMD", "Hemlock [Radeon HD 5970]" },
	{0x1002, 0x689E, NT, "AMD", "Cypress LE [Radeon HD 5830]" },
	{0x1002, 0x68A0, NT, "AMD", "Broadway XT [Mobility Radeon HD 5870]" },
	{0x1002, 0x68A1, NT, "AMD", "Broadway PRO [Mobility Radeon HD 5850]" },
	{0x1002, 0x68A8, NT, "AMD", "Granville [Radeon HD 6850M/6870M]" },
	{0x1002, 0x68A9, NT, "AMD", "Juniper XT [FirePro V5800]" },
	{0x1002, 0x68B8, NT, "AMD", "Juniper XT [Radeon HD 5770]" },
	{0x1002, 0x68B9, NT, "AMD", "Juniper LE [Radeon HD 5670 640SP Edition]" },
	{0x1002, 0x68BA, NT, "AMD", "Juniper XT [Radeon HD 6770]" },
	{0x1002, 0x68BE, NT, "AMD", "Juniper PRO [Radeon HD 5750]" },
	{0x1002, 0x68BF, NT, "AMD", "Juniper PRO [Radeon HD 6750]" },
	{0x1002, 0x68C0, NT, "AMD", "Madison [Mobility Radeon HD 5730 / 6570M]" },
	{0x1002, 0x68C1, NT, "AMD", "Madison [Mobility Radeon HD 5650/5750 / 6530M/6550M]" },
	{0x1002, 0x68C7, NT, "AMD", "Pinewood [Mobility Radeon HD 5570/6550A]" },
	{0x1002, 0x68C8, NT, "AMD", "Redwood XT GL [FirePro V4800]" },
	{0x1002, 0x68C9, NT, "AMD", "Redwood PRO GL [FirePro V3800]" },
	{0x1002, 0x68D8, NT, "AMD", "Redwood XT [Radeon HD 5670/5690/5730]" },
	{0x1002, 0x68D9, NT, "AMD", "Redwood PRO [Radeon HD 5550/5570/5630/6510/6610/7570]" },
	{0x1002, 0x68DA, NT, "AMD", "Redwood LE [Radeon HD 5550/5570/5630/6390/6490/7570]" },
	{0x1002, 0x68DE, NT, "AMD", "Redwood" },
	{0x1002, 0x68E0, NT, "AMD", "Park [Mobility Radeon HD 5430/5450/5470]" },
	{0x1002, 0x68E1, NT, "AMD", "Park [Mobility Radeon HD 5430]" },
	{0x1002, 0x68E4, NT, "AMD", "Robson CE [Radeon HD 6370M/7370M]" },
	{0x1002, 0x68E5, NT, "AMD", "Robson LE [Radeon HD 6330M]" },
	{0x1002, 0x68E8, NT, "AMD", "Cedar" },
	{0x1002, 0x68E9, NT, "AMD", "Cedar [ATI FirePro (FireGL) Graphics Adapter]" },
	{0x1002, 0x68F1, NT, "AMD", "Cedar GL [FirePro 2460]" },
	{0x1002, 0x68F2, NT, "AMD", "Cedar GL [FirePro 2270]" },
	{0x1002, 0x68F8, NT, "AMD", "Cedar [Radeon HD 7300 Series]" },
	{0x1002, 0x68F9, NT, "AMD", "Cedar [Radeon HD 5000/6000/7350/8350 Series]" },
	{0x1002, 0x68FA, NT, "AMD", "Cedar [Radeon HD 7350/8350 / R5 220]" },
	{0x1002, 0x68FE, NT, "AMD", "Cedar LE" },
	{0x1002, 0x9400, NT, "AMD", "R600 [Radeon HD 2900 PRO/XT]" },
	{0x1002, 0x9401, NT, "AMD", "R600 [Radeon HD 2900 XT]" },
	{0x1002, 0x9402, NT, "AMD", "R600" },
	{0x1002, 0x9403, NT, "AMD", "R600 [Radeon HD 2900 PRO]" },
	{0x1002, 0x9405, NT, "AMD", "R600 [Radeon HD 2900 GT]" },
	{0x1002, 0x940A, NT, "AMD", "R600 GL [FireGL V8650]" },
	{0x1002, 0x940B, NT, "AMD", "R600 GL [FireGL V8600]" },
	{0x1002, 0x940F, NT, "AMD", "R600 GL [FireGL V7600]" },
	{0x1002, 0x9440, NT, "AMD", "RV770 [Radeon HD 4870]" },
	{0x1002, 0x9441, NT, "AMD", "R700 [Radeon HD 4870 X2]" },
	{0x1002, 0x9442, NT, "AMD", "RV770 [Radeon HD 4850]" },
	{0x1002, 0x9443, NT, "AMD", "R700 [Radeon HD 4850 X2]" },
	{0x1002, 0x9444, NT, "AMD", "RV770 GL [FirePro V8750]" },
	{0x1002, 0x9446, NT, "AMD", "RV770 GL [FirePro V7760]" },
	{0x1002, 0x944A, NT, "AMD", "RV770/M98L [Mobility Radeon HD 4850]" },
	{0x1002, 0x944B, NT, "AMD", "RV770/M98 [Mobility Radeon HD 4850 X2]" },
	{0x1002, 0x944C, NT, "AMD", "RV770 LE [Radeon HD 4830]" },
	{0x1002, 0x944E, NT, "AMD", "RV770 CE [Radeon HD 4710]" },
	{0x1002, 0x9450, NT, "AMD", "RV770 GL [FireStream 9270]" },
	{0x1002, 0x9452, NT, "AMD", "RV770 GL [FireStream 9250]" },
	{0x1002, 0x9456, NT, "AMD", "RV770 GL [FirePro V8700]" },
	{0x1002, 0x945A, NT, "AMD", "RV770/M98-XT [Mobility Radeon HD 4870]" },
	{0x1002, 0x9460, NT, "AMD", "RV790 [Radeon HD 4890]" },
	{0x1002, 0x9462, NT, "AMD", "RV790 [Radeon HD 4860]" },
	{0x1002, 0x946A, NT, "AMD", "RV770 GL [FirePro M7750]" },
	{0x1002, 0x9480, NT, "AMD", "RV730/M96 [Mobility Radeon HD 4650/5165]" },
	{0x1002, 0x9488, NT, "AMD", "RV730/M96-XT [Mobility Radeon HD 4670]" },
	{0x1002, 0x9489, NT, "AMD", "RV730/M96 GL [Mobility FireGL V5725]" },
	{0x1002, 0x9490, NT, "AMD", "RV730 XT [Radeon HD 4670]" },
	{0x1002, 0x9491, NT, "AMD", "RV730/M96-CSP [Radeon E4690]" },
	{0x1002, 0x9495, NT, "AMD", "RV730 [Radeon HD 4600 AGP Series]" },
	{0x1002, 0x9498, NT, "AMD", "RV730 PRO [Radeon HD 4650]" },
	{0x1002, 0x949C, NT, "AMD", "RV730 GL [FirePro V7750]" },
	{0x1002, 0x949E, NT, "AMD", "RV730 GL [FirePro V5700]" },
	{0x1002, 0x949F, NT, "AMD", "RV730 GL [FirePro V3750]" },
	{0x1002, 0x94A0, NT, "AMD", "RV740/M97 [Mobility Radeon HD 4830]" },
	{0x1002, 0x94A1, NT, "AMD", "RV740/M97-XT [Mobility Radeon HD 4860]" },
	{0x1002, 0x94A3, NT, "AMD", "RV740/M97 GL [FirePro M7740]" },
	{0x1002, 0x94B3, NT, "AMD", "RV740 PRO [Radeon HD 4770]" },
	{0x1002, 0x94B4, NT, "AMD", "RV740 PRO [Radeon HD 4750]" },
	{0x1002, 0x94C1, NT, "AMD", "RV610 [Radeon HD 2400 PRO/XT]" },
	{0x1002, 0x94C3, OK, "AMD", "RV610 [Radeon HD 2400 PRO]" },
	{0x1002, 0x94C4, NT, "AMD", "RV610 LE [Radeon HD 2400 PRO AGP]" },
	{0x1002, 0x94C5, NT, "AMD", "RV610 [Radeon HD 2400 LE]" },
	{0x1002, 0x94C6, NT, "AMD", "R600" },
	{0x1002, 0x94C7, NT, "AMD", "RV610 [Radeon HD 2350]" },
	{0x1002, 0x94C8, NT, "AMD", "RV610/M74 [Mobility Radeon HD 2400 XT]" },
	{0x1002, 0x94C9, NT, "AMD", "RV610/M72-S [Mobility Radeon HD 2400]" },
	{0x1002, 0x94CB, NT, "AMD", "RV610 [Radeon E2400]" },
	{0x1002, 0x94CC, NT, "AMD", "RV610 LE [Radeon HD 2400 PRO PCI]" },
	{0x1002, 0x9500, NT, "AMD", "RV670 [Radeon HD 3850 X2]" },
	{0x1002, 0x9501, NT, "AMD", "RV670 [Radeon HD 3870]" },
	{0x1002, 0x9504, NT, "AMD", "RV670/M88 [Mobility Radeon HD 3850]" },
	{0x1002, 0x9505, NT, "AMD", "RV670 [Radeon HD 3690/3850]" },
	{0x1002, 0x9506, NT, "AMD", "RV670/M88 [Mobility Radeon HD 3850 X2]" },
	{0x1002, 0x9507, NT, "AMD", "RV670 [Radeon HD 3830]" },
	{0x1002, 0x9508, NT, "AMD", "RV670/M88-XT [Mobility Radeon HD 3870]" },
	{0x1002, 0x9509, NT, "AMD", "RV670/M88 [Mobility Radeon HD 3870 X2]" },
	{0x1002, 0x950F, NT, "AMD", "R680 [Radeon HD 3870 X2]" },
	{0x1002, 0x9511, OK, "AMD", "RV670 GL [FireGL V7700]" },
	{0x1002, 0x9513, NT, "AMD", "RV670 [Radeon HD 3850 X2]" },
	{0x1002, 0x9515, NT, "AMD", "RV670 PRO [Radeon HD 3850 AGP]" },
	{0x1002, 0x9519, NT, "AMD", "RV670 GL [FireStream 9170]" },
	{0x1002, 0x9540, NT, "AMD", "RV710 [Radeon HD 4550]" },
	{0x1002, 0x954F, NT, "AMD", "RV710 [Radeon HD 4350/4550]" },
	{0x1002, 0x9552, NT, "AMD", "RV710/M92 [Mobility Radeon HD 4330/4350/4550]" },
	{0x1002, 0x9553, NT, "AMD", "RV710/M92 [Mobility Radeon HD 4530/4570/5145/530v/540v/545v]" },
	{0x1002, 0x9555, NT, "AMD", "RV711/M93 [Mobility Radeon HD 4350/4550/530v/540v/545v / FirePro RG220]" },
	{0x1002, 0x9557, NT, "AMD", "RV711/M93 GL [FirePro RG220]" },
	{0x1002, 0x955F, NT, "AMD", "RV710/M92 [Mobility Radeon HD 4330]" },
	{0x1002, 0x9580, NT, "AMD", "RV630 [Radeon HD 2600 PRO]" },
	{0x1002, 0x9581, NT, "AMD", "RV630/M76 [Mobility Radeon HD 2600]" },
	{0x1002, 0x9583, NT, "AMD", "RV630/M76 [Mobility Radeon HD 2600 XT/2700]" },
	{0x1002, 0x9586, NT, "AMD", "RV630 XT [Radeon HD 2600 XT AGP]" },
	{0x1002, 0x9587, NT, "AMD", "RV630 PRO [Radeon HD 2600 PRO AGP]" },
	{0x1002, 0x9588, NT, "AMD", "RV630 XT [Radeon HD 2600 XT]" },
	{0x1002, 0x9589, OK, "AMD", "RV630 PRO [Radeon HD 2600 PRO]" },
	{0x1002, 0x958A, NT, "AMD", "RV630 [Radeon HD 2600 X2]" },
	{0x1002, 0x958B, NT, "AMD", "RV630/M76 [Mobility Radeon HD 2600 XT]" },
	{0x1002, 0x958C, NT, "AMD", "RV630 GL [FireGL V5600]" },
	{0x1002, 0x958D, OK, "AMD", "RV630 GL [FireGL V3600]" },
	{0x1002, 0x958E, NT, "AMD", "R600" },
	{0x1002, 0x9591, NT, "AMD", "RV635/M86 [Mobility Radeon HD 3650]" },
	{0x1002, 0x9593, NT, "AMD", "RV635/M86 [Mobility Radeon HD 3670]" },
	{0x1002, 0x9595, NT, "AMD", "RV635/M86 GL [Mobility FireGL V5700]" },
	{0x1002, 0x9596, NT, "AMD", "RV635 PRO [Radeon HD 3650 AGP]" },
	{0x1002, 0x9597, NT, "AMD", "RV635 PRO [Radeon HD 3650 AGP]" },
	{0x1002, 0x9598, NT, "AMD", "RV635 [Radeon HD 3650/3750/4570/4580]" },
	{0x1002, 0x9599, NT, "AMD", "RV635 PRO [Radeon HD 3650 AGP]" },
	{0x1002, 0x95C0, NT, "AMD", "RV620 PRO [Radeon HD 3470]" },
	{0x1002, 0x95C2, NT, "AMD", "RV620/M82 [Mobility Radeon HD 3410/3430]" },
	{0x1002, 0x95C4, NT, "AMD", "RV620/M82 [Mobility Radeon HD 3450/3470]" },
	{0x1002, 0x95C5, NT, "AMD", "RV620 LE [Radeon HD 3450]" },
	{0x1002, 0x95C6, NT, "AMD", "RV620 LE [Radeon HD 3450 AGP]" },
	{0x1002, 0x95C9, NT, "AMD", "RV620 LE [Radeon HD 3450 PCI]" },
	{0x1002, 0x95CC, NT, "AMD", "RV620 GL [FirePro V3700]" },
	{0x1002, 0x95CD, NT, "AMD", "RV620 GL [FirePro 2450]" },
	{0x1002, 0x95CF, NT, "AMD", "RV620 GL [FirePro 2260]" },
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
