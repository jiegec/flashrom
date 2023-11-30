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

struct ati_spi_pci_private;
struct ati_spi_data {
	struct pci_dev *dev;
	uint8_t *bar;
	const struct ati_spi_pci_private *private;
};

struct ati_spi_pci_private {
	int io_bar;
};


static const struct ati_spi_pci_private r600_spi_pci_private = {
	.io_bar = PCI_BASE_ADDRESS_2,
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

	return 0;
}

const struct programmer_entry programmer_ati_spi = {
	.name			= "ati_spi",
	.type			= PCI,
	.devs.dev		= devs_ati_spi,
	.init			= ati_spi_init,
};
