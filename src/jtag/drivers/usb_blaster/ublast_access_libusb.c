/*
 *   Driver for simple USB buffering access layer
 *
 *   Copyright (C) 2012 Robert Jarzmik robert.jarzmik@free.fr
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
 * This simple access layer does :
 *  - buffering : nothing is actually send over USB before an explicit flush()
 *  - streaming : all read/writes are supposed to be a stream, the USB packet
 *    semantics are lost
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <jtag/interface.h>
#include <jtag/commands.h>

#include "ublast_access.h"
#include "helper/log.h"
#include <libusb-1.0/libusb.h>

#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define FTDI_DEVICE_IN_REQTYPE (0x80 | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE
#define SIO_RESET_REQUEST             0x00
#define SIO_RESET_SIO 0
#define SIO_RESET_PURGE_RX 1
#define SIO_RESET_PURGE_TX 2

struct usb_ctx {
	libusb_context *usb_ctx;
	libusb_device_handle *dev;
	int interface;
	uint8_t ep_in;
	uint8_t ep_out;
	int timeout;
	struct list_head read_reqs;
	struct list_head write_reqs;
};

struct rw_req {
	void *buf;
	int nb;
	struct list_head list;
};

struct transfer_result {
	int nb_sumitted;
	int nb_finished;
};

/** Port interface for chips with multiple interfaces */
enum ftdi_interface
{
    INTERFACE_ANY = 0,
    INTERFACE_A   = 1,
    INTERFACE_B   = 2,
    INTERFACE_C   = 3,
    INTERFACE_D   = 4
};

static void usb_reset(struct usb_ctx *ctx)
{
	int rc;

	rc = libusb_control_transfer(ctx->dev, FTDI_DEVICE_OUT_REQTYPE,
				     SIO_RESET_REQUEST, SIO_RESET_SIO,
				     INTERFACE_A, NULL, 0, ctx->timeout);
	if (rc)
		LOG_INFO("control transfer failed : %d\n", rc);
}


static struct usb_ctx* usb_open(const uint16_t vid, const uint16_t pid,
				 const int usb_interface, const uint8_t ep_in,
				 const uint8_t ep_out)
{
	int rc;
	struct usb_ctx *ctx;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		goto err_mem;
	rc = libusb_init(&ctx->usb_ctx);
	if (rc < 0)
		goto err_init;
	ctx->dev = libusb_open_device_with_vid_pid(ctx->usb_ctx, vid, pid);
	if (!ctx->dev)
		goto err_open;
	ctx->interface = usb_interface;
	rc = libusb_claim_interface(ctx->dev, ctx->interface);
	if (rc != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_claim_interface() failed with %d", rc);
		goto err_interface;
	}
	usb_reset(ctx);
	ctx->ep_in = ep_in;
	ctx->ep_out = ep_out;
	INIT_LIST_HEAD(&ctx->read_reqs);
	INIT_LIST_HEAD(&ctx->write_reqs);

	ctx->timeout = 0;
	return ctx;

err_interface:
	libusb_close(ctx->dev);
err_open:
	libusb_exit(ctx->usb_ctx);
err_init:
	free(ctx);
err_mem:
	return NULL;
}

static void usb_close(struct usb_ctx *ctx)
{
	libusb_close(ctx->dev);
	libusb_exit(ctx->usb_ctx);
	free(ctx);
}

static int usb_queue_read(struct usb_ctx *ctx, void *buf, int nb)
{
	struct rw_req *req = malloc(sizeof(*req));

	if (!req)
		return -ENOMEM;
	req->buf = buf;
	req->nb = nb;
	list_add_tail(&req->list, &ctx->read_reqs);
	return nb;
}

static int usb_queue_write(struct usb_ctx *ctx, void *buf, int nb, bool copy)
{
	struct rw_req *req = malloc(sizeof(*req));

	if (!req)
		return -ENOMEM;
	if (copy)
		req->buf = malloc(nb);
	else
		req->buf = buf;
	if (!req->buf) {
		free(req);
		return -ENOMEM;
	}
	req->nb = nb;
	memcpy(req->buf, buf, nb);
	list_add_tail(&req->list, &ctx->write_reqs);
	return nb;
}

static void read_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *result = transfer->user_data;

	DEBUG_JTAG_IO("status=%d, transferred %d+%d=%d",
		      transfer->status,
		      result->nb_finished,
		      transfer->actual_length,
		      result->nb_finished + transfer->actual_length);
	if (transfer->actual_length >= 2) {
		memmove(transfer->buffer, transfer->buffer + 2,
			transfer->actual_length - 2);
		result->nb_finished += transfer->actual_length - 2;

		if (result->nb_finished < result->nb_sumitted) {
			transfer->length -= (transfer->actual_length - 2);
			transfer->buffer += (transfer->actual_length - 2);
			if (libusb_submit_transfer(transfer))
				result->nb_finished = result->nb_sumitted;
		}
	}
}

static void write_cb(struct libusb_transfer *transfer)
{
	struct transfer_result *result = transfer->user_data;

	DEBUG_JTAG_IO("status=%d, transferred %d+%d=%d",
		      transfer->status,
		      result->nb_finished,
		      transfer->actual_length,
		      result->nb_finished + transfer->actual_length);
	result->nb_finished += transfer->actual_length;

	if (result->nb_finished < result->nb_sumitted) {
		transfer->length -= transfer->actual_length;
		transfer->buffer += transfer->actual_length;
		if (libusb_submit_transfer(transfer))
			result->nb_finished = result->nb_sumitted;
	}
}

static int get_bytes_of_reqs(struct list_head *reqs)
{
	struct rw_req *req;
	int nb = 0;

	list_for_each_entry(req, reqs, list)
		nb += req->nb;
	return nb;
}

static void xfer_reqs_to_buf(unsigned char *buf, struct list_head *reqs)
{
	struct rw_req *req;

	list_for_each_entry(req, reqs, list) {
		memmove(buf, req->buf, req->nb);
		buf += req->nb;
	}
}

static char *hexdump(uint8_t *buf, unsigned int size)
{
	unsigned int i;
	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2*i, "%02x", buf[i]);
	return str;
}

static void xfer_buf_to_reqs(unsigned char *buf, struct list_head *reqs)
{
	struct rw_req *req;
	char *str;

	list_for_each_entry(req, reqs, list) {
		str = hexdump(buf, req->nb);
		free(str);

		memmove(req->buf, buf, req->nb);
		buf += req->nb;
	}
}

static int usb_flush(struct usb_ctx *ctx)
{
	int nb_read, nb_read_align, nb_write, rc = 0;
	unsigned char *buf_read = NULL, *buf_write = NULL;
	struct libusb_transfer *read_transfer, *write_transfer;
	struct transfer_result read_result = { .nb_finished = 0 };
	struct transfer_result write_result = { .nb_finished = 0 };

	nb_read = get_bytes_of_reqs(&ctx->read_reqs);
	nb_read_align = (((nb_read + 2)/ 64) + 1) * 64;
	nb_write = get_bytes_of_reqs(&ctx->write_reqs);
	DEBUG_JTAG_IO("write %d, read %d", nb_write, nb_read);

	if (nb_read == 0 && nb_write == 0)
		return 0;
	read_result.nb_sumitted = nb_read;
	write_result.nb_sumitted = nb_write;

	buf_read = malloc(nb_read_align);
	buf_write = malloc(nb_write);
	if (!buf_read || !buf_write) {
		rc = -ENOMEM;
		goto out;
	}

	xfer_reqs_to_buf(buf_read, &ctx->read_reqs);
	xfer_reqs_to_buf(buf_write, &ctx->write_reqs);

	char *str = hexdump(buf_write, nb_write);
	DEBUG_JTAG_IO("actual write %d bytes [%s]", nb_write, str);
	free(str);

	read_transfer = libusb_alloc_transfer(0);
	write_transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(read_transfer, ctx->dev, ctx->ep_in,
				  buf_read, nb_read_align,
				  read_cb, &read_result, ctx->timeout);
	libusb_fill_bulk_transfer(write_transfer, ctx->dev, ctx->ep_out,
				  buf_write, nb_write,
				  write_cb, &write_result, ctx->timeout);
	if (nb_write)
		rc = libusb_submit_transfer(write_transfer);
	if (!rc && nb_read)
		rc = libusb_submit_transfer(read_transfer);

	/* Polling loop, more or less taken from libftdi */
	while ((read_result.nb_finished < nb_read ||
		write_result.nb_finished < nb_write) && !rc) {
		rc = libusb_handle_events(ctx->usb_ctx);

		keep_alive();
		if (!rc)
			continue;
		if (nb_write)
			libusb_cancel_transfer(write_transfer);
		if (nb_read)
			libusb_cancel_transfer(read_transfer);
		rc = 0;
		while (!rc && nb_write &&
		       write_transfer->status != LIBUSB_TRANSFER_CANCELLED)
			rc = libusb_handle_events(ctx->usb_ctx);
		while (!rc && nb_read &&
		       read_transfer->status != LIBUSB_TRANSFER_CANCELLED)
			rc = libusb_handle_events(ctx->usb_ctx);
	}

	if (rc) {
		LOG_ERROR("libusb_handle_events() failed with %d", rc);
		rc = -EIO;
	} else if (nb_write && write_result.nb_finished < nb_write) {
		LOG_ERROR("usb device did not accept all data: %d, tried %d",
			  write_result.nb_finished, nb_write);
		rc = -EIO;
	} else if (nb_read && read_result.nb_finished < nb_read) {
		LOG_ERROR("usb device did not return all data: %d, expected %d",
			  read_result.nb_finished, nb_read);
		rc = -EIO;;
	} else {
		rc = 0;
		xfer_buf_to_reqs(buf_read, &ctx->read_reqs);
		xfer_buf_to_reqs(buf_write, &ctx->write_reqs);
		str = hexdump(buf_read, nb_read);
		DEBUG_JTAG_IO("actual read %d bytes [%s]", nb_read, str);
		free(str);
	}
	INIT_LIST_HEAD(&ctx->read_reqs);
	INIT_LIST_HEAD(&ctx->write_reqs);

	if (nb_write)
		libusb_free_transfer(write_transfer);
	if (nb_read)
		libusb_free_transfer(read_transfer);
	/* if (rc) */
	/* 	usb_purge(ctx); */

out:
	if (buf_read)
		free(buf_read);
	if (buf_write)
		free(buf_write);
	return rc;
}

/*
 * This is the ublast_access specific part
 */
static struct usb_ctx *ublast_getctx(struct ublast_lowlevel *low)
{
	return low->priv;
}

static int ublast_libusb_read(struct ublast_lowlevel *low, uint8_t *buf,
			      unsigned size, uint32_t *bytes_read)
{
	struct usb_ctx *ctx = ublast_getctx(low);
	int rc;

	rc = usb_queue_read(ctx, buf, size);
	*bytes_read = size;
	return rc < 0 ? ERROR_JTAG_DEVICE_ERROR : ERROR_OK;
}

static int ublast_libusb_write(struct ublast_lowlevel *low, uint8_t *buf, int size,
			       uint32_t *bytes_written)
{
	struct usb_ctx *ctx = ublast_getctx(low);
	int rc;

	rc = usb_queue_write(ctx, buf, size, true);
	*bytes_written = size;
	return rc < 0 ? ERROR_JTAG_DEVICE_ERROR : ERROR_OK;
}

static void ublast_libusb_flush(struct ublast_lowlevel *low)
{
	struct usb_ctx *ctx = ublast_getctx(low);

	usb_flush(ctx);
}

static int ublast_libusb_init(struct ublast_lowlevel *low)
{
	struct usb_ctx *ctx;

	LOG_INFO("usb blaster interface using libusb");
	ctx = usb_open(low->ublast_vid, low->ublast_pid, 0, 0x81, 0x02);
	if (!ctx)
		return ERROR_JTAG_INIT_FAILED;
	low->priv = ctx;

	return ERROR_OK;
}

static int ublast_libusb_quit(struct ublast_lowlevel *low)
{
	struct usb_ctx *ctx = ublast_getctx(low);

	usb_close(ctx);
	return ERROR_OK;
};

static struct ublast_lowlevel low = {
	.open = ublast_libusb_init,
	.close = ublast_libusb_quit,
	.queue_read = ublast_libusb_read,
	.queue_write = ublast_libusb_write,
	.flush = ublast_libusb_flush,
	.priv = NULL,
};

struct ublast_lowlevel *ublast_register_libusb(void)
{
	return &low;
}
