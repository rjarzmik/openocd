/*
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
 *   Inspired from original code from Kolja Waschk's USB-JTAG project
 *   (http://www.ixo.de/info/usb_jtag/), and from openocd project.
 *
 *   Copyright (C) 2012 Robert Jarzmik robert.jarzmik@free.fr
 *   Copyright (C) 2011 Ali Lown ali@lown.me.uk
 *   Copyright (C) 2009 Catalin Patulea cat@vv.carleton.ca
 *   Copyright (C) 2006 Kolja Waschk usbjtag@ixo.de
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <jtag/interface.h>
#include <jtag/commands.h>

#include "ublast_access.h"
#include "ublast_access_common.h"
#include <ftdi.h>

static LIST_HEAD(read_reqs);
static LIST_HEAD(write_reqs);

static struct ftdi_context *ublast_getftdic(struct ublast_lowlevel *low)
{
	return low->priv;
}

static char *hexdump(uint8_t *buf, unsigned int size)
{
	unsigned int i;
	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2*i, "%02x", buf[i]);
	return str;
}

static int ublast_ftdi_queue_read(struct ublast_lowlevel *low, uint8_t *buf,
				  unsigned size, uint32_t *bytes_read)
{
	*bytes_read = size;
	return ublast_common_queue_read(&read_reqs, buf, size);
}

static int ublast_ftdi_queue_write(struct ublast_lowlevel *low, uint8_t *buf,
				   int size, uint32_t *bytes_written)
{
	*bytes_written = size;
	return ublast_common_queue_write(&write_reqs, buf, (unsigned int)size);
}

static int ublast_ftdi_do_read(struct ublast_lowlevel *low, uint8_t *buf,
			       unsigned size)
{
	int retval;
	uint32_t bytes_read;
	int timeout = 100;
	struct ftdi_context *ftdic = ublast_getftdic(low);
	char *str;

	bytes_read = 0;
	while ((bytes_read < size) && timeout--) {
		retval = ftdi_read_data(ftdic, buf + bytes_read,
					size - bytes_read);
		if (retval < 0)	{
			bytes_read = 0;
			LOG_ERROR("ftdi_read_data: %s",
					ftdi_get_error_string(ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		bytes_read += retval;
		str = retval < 0 ? NULL : hexdump(buf + bytes_read, retval);
		DEBUG_JTAG_IO("USB actual read %d bytes : [%s]", retval, str);
		free(str);
	}
	return bytes_read;
}

static int ublast_ftdi_do_write(struct ublast_lowlevel *low, uint8_t *buf, int size)
{
	struct ftdi_context *ftdic = ublast_getftdic(low);
	char *str;

	str = hexdump(buf, size);
	DEBUG_JTAG_IO("USB actual write %d bytes : [%s]", size, str);
	free(str);
	return ftdi_write_data(ftdic, buf, size);
}

static void ublast_ftdi_flush(struct ublast_lowlevel *low)
{
	int retval;
	struct ftdi_context *ftdic = ublast_getftdic(low);

	retval = ublast_common_flush(low, &read_reqs, &write_reqs,
				     ublast_ftdi_do_read, ublast_ftdi_do_write);
	if (retval < 0)
		LOG_ERROR("ftdi flush: %s",
			  ftdi_get_error_string(ftdic));
}

static int ublast_ftdi_init(struct ublast_lowlevel *low)
{
	uint8_t latency_timer;
	struct ftdi_context *ftdic = ublast_getftdic(low);

	LOG_INFO("usb blaster interface using libftdi");
	if (ftdi_init(ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id */
	if (ftdi_usb_open(ftdic, low->ublast_vid, low->ublast_pid) < 0)	{
		LOG_ERROR("unable to open ftdi device: %s", ftdic->error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(ftdic) < 0) {
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(ftdic, 2) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(ftdic, &latency_timer) < 0) {
		LOG_ERROR("unable to get latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("current latency timer: %u", latency_timer);

	ftdi_disable_bitbang(ftdic);
	return ERROR_OK;
}

static int ublast_ftdi_quit(struct ublast_lowlevel *low)
{
	struct ftdi_context *ftdic = ublast_getftdic(low);

	ftdi_usb_close(ftdic);
	ftdi_deinit(ftdic);
	return ERROR_OK;
};

static struct ublast_lowlevel_priv {
	struct ftdi_context ftdic;
} info;

static struct ublast_lowlevel low = {
	.open = ublast_ftdi_init,
	.close = ublast_ftdi_quit,
	.queue_read = ublast_ftdi_queue_read,
	.queue_write = ublast_ftdi_queue_write,
	.flush = ublast_ftdi_flush,
	.priv = &info,
};

struct ublast_lowlevel *ublast_register_ftdi(void)
{
	return &low;
}
