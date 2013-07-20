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

#define NB_READ_REQS 5
#define NB_WRITE_REQS 5

#define FTDI_DEVICE_OUT_REQTYPE (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE)
#define FTDI_DEVICE_IN_REQTYPE (0x80 | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE
#define SIO_RESET_REQUEST             0x00
#define SIO_RESET_SIO 0
#define SIO_RESET_PURGE_RX 1
#define SIO_RESET_PURGE_TX 2

struct fifo_req {
	struct list_head list;
	unsigned char *buf;
	int bufsize;
	struct libusb_transfer *xfer;
	struct reqs_progress *progress;
};

struct ep_fifo {
	struct list_head free;
	struct list_head queued;
};

struct reqs_progress {
	struct list_head queued;
	struct list_head done;

	struct ep_fifo fifo;
	int overflowed;
};

struct usb_ctx {
	libusb_context *usb_ctx;
	libusb_device_handle *dev;
	int interface;
	uint8_t ep_in;
	uint8_t ep_out;
	uint16_t ep_in_maxpkt_size;
	uint16_t ep_out_maxpkt_size;
	int timeout;

	struct reqs_progress read_progress;
	struct reqs_progress write_progress;
};

struct rw_req {
	void *buf;
	int nb;
	int nb_filled;
	struct list_head list;
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

static void read_cb(struct libusb_transfer *transfer);
static void write_cb(struct libusb_transfer *transfer);

static char *hexdump(uint8_t *buf, unsigned int size)
{
	unsigned int i;
	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2*i, "%02x", buf[i]);
	return str;
}

static void usb_reset(struct usb_ctx *ctx)
{
	int rc;

	rc = libusb_control_transfer(ctx->dev, FTDI_DEVICE_OUT_REQTYPE,
				     SIO_RESET_REQUEST, SIO_RESET_SIO,
				     INTERFACE_A, NULL, 0, ctx->timeout);
	if (rc)
		LOG_INFO("control transfer failed : %d\n", rc);
}

static int alloc_free_reqs(struct usb_ctx *ctx, int nb_req, struct ep_fifo *fifo,
			   struct reqs_progress *progress,
			   int maxpkt_size)
{
	int i;
	struct fifo_req *req;

	for (i = 0; i < NB_READ_REQS; i++) {
		req = malloc(sizeof(struct fifo_req));
		if (!req)
			return -ENOMEM;
		req->buf = malloc(maxpkt_size);
		if (!req->buf)
			return -ENOMEM;
		req->progress = progress;
		req->bufsize = maxpkt_size;
		list_add(&req->list, &fifo->free);
		req->xfer = libusb_alloc_transfer(0);
	}
	return 0;
}

static void fill_free_reqs(struct usb_ctx *ctx, struct ep_fifo *fifo,
			   uint8_t ep,
			   void (*cb)(struct libusb_transfer *transfer))
{
	struct fifo_req *req;

	list_for_each_entry(req, &fifo->free, list) {
		libusb_fill_bulk_transfer(req->xfer, ctx->dev, ep,
					  req->buf, req->bufsize,
					  cb, req, ctx->timeout);
	}
}

static void release_free_reqs(struct ep_fifo *fifo)
{
	struct rw_req *req, *tmp;

	list_for_each_entry_safe(req, tmp, &fifo->free, list) {
		free(req->buf);
		list_del(&req->list);
		free(req);
	}
}

static int feed_read_fifo(struct fifo_req *req, struct ep_fifo *fifo)
{
	int rc = 0;

	rc = libusb_submit_transfer(req->xfer);
	if (!rc)
		list_move_tail(&req->list, &fifo->queued);
	else
		LOG_ERROR("queue_free_reqs() submit urb error : %d", rc);
	return rc;
}

static int feed_write_fifo(struct fifo_req *dst_req, struct ep_fifo *fifo,
			   struct reqs_progress *progress)
{
	int nb, remain;
	unsigned char *dst = dst_req->buf;
	struct rw_req *src_req;

	if (list_empty(&progress->queued))
		return 0;

	remain = dst_req->bufsize;
	while (remain > 0 && !list_empty(&progress->queued)) {
		src_req = list_first_entry(&progress->queued, struct rw_req,
					   list);
		nb = MIN(remain, src_req->nb - src_req->nb_filled);
		if (nb) {
			memcpy(dst, src_req->buf + src_req->nb_filled, nb);
			src_req->nb_filled += nb;
			remain -= nb;
			dst += nb;
		} else {
			list_move_tail(&src_req->list, &progress->done);
		}
	}
	list_move_tail(&dst_req->list, &fifo->queued);
	dst_req->xfer->length = dst_req->bufsize - remain;
	return libusb_submit_transfer(dst_req->xfer);
}

static int queue_read_free_reqs(struct usb_ctx *ctx)
{
	struct reqs_progress *progress = &ctx->read_progress;
	struct ep_fifo *fifo = &progress->fifo;
	struct fifo_req *req, *tmp;
	int rc = 0;

	list_for_each_entry_safe(req, tmp, &fifo->free, list) {
		rc = feed_read_fifo(req, fifo);
		if (rc)
			break;
	}
	return rc;
}

static int queue_write_free_reqs(struct usb_ctx *ctx)
{
	struct reqs_progress *progress = &ctx->write_progress;
	struct ep_fifo *fifo = &progress->fifo;
	struct fifo_req *req, *tmp;
	int rc = 0;

	list_for_each_entry_safe(req, tmp, &fifo->free, list) {
		rc = feed_write_fifo(req, fifo, progress);
		if (rc)
			break;
	}
	return 0;
}

static int cancel_all_fifo(struct usb_ctx *ctx, struct ep_fifo *fifo)
{
	struct fifo_req *req, *tmp;
	int rc = 0;

	DEBUG_JTAG_IO("cancel and unqueue request");
	list_for_each_entry_safe(req, tmp, &fifo->queued, list)
		libusb_cancel_transfer(req->xfer);
	while (!list_empty(&fifo->queued))
		rc = libusb_handle_events(ctx->usb_ctx);
	return rc;
}

static struct usb_ctx* usb_open(const uint16_t vid, const uint16_t pid,
				 const int usb_interface, const uint8_t ep_in,
				 const uint8_t ep_out)
{
	int rc;
	struct usb_ctx *ctx;
	uint16_t maxpkt_in = 64, maxpkt_out = 64;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		goto err_mem;
	ctx->ep_in = ep_in;
	ctx->ep_out = ep_out;
	INIT_LIST_HEAD(&ctx->read_progress.queued);
	INIT_LIST_HEAD(&ctx->read_progress.done);
	INIT_LIST_HEAD(&ctx->read_progress.fifo.free);
	INIT_LIST_HEAD(&ctx->read_progress.fifo.queued);

	INIT_LIST_HEAD(&ctx->write_progress.queued);
	INIT_LIST_HEAD(&ctx->write_progress.done);
	INIT_LIST_HEAD(&ctx->write_progress.fifo.free);
	INIT_LIST_HEAD(&ctx->write_progress.fifo.queued);

	rc = libusb_init(&ctx->usb_ctx);
	if (rc < 0) {
		LOG_ERROR("libusb_init failed with %d", rc);
		goto err_init;
	}
	ctx->dev = libusb_open_device_with_vid_pid(ctx->usb_ctx, vid, pid);
	if (!ctx->dev) {
		LOG_ERROR("libusb_open_device_with_vid_pid failed");
		goto err_open;
	}
	ctx->interface = usb_interface;
	rc = libusb_claim_interface(ctx->dev, ctx->interface);
	if (rc != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_claim_interface() failed with %d", rc);
		goto err_interface;
	}
	usb_reset(ctx);

	ctx->timeout = 0;
	rc = alloc_free_reqs(ctx, NB_READ_REQS, &ctx->read_progress.fifo,
			     &ctx->read_progress, maxpkt_in);
	if (rc) {
		LOG_ERROR("alloc_free_reqs() failed with %d", rc);
		goto err_interface;
	}
	rc = alloc_free_reqs(ctx, NB_WRITE_REQS, &ctx->write_progress.fifo,
			     &ctx->write_progress, maxpkt_out);
	if (rc) {
		LOG_ERROR("alloc_free_reqs() failed with %d", rc);
		goto err_alloc_write_reqs;
	}
	fill_free_reqs(ctx, &ctx->read_progress.fifo, ctx->ep_in, read_cb);
	fill_free_reqs(ctx, &ctx->write_progress.fifo, ctx->ep_out, write_cb);

	return ctx;

err_alloc_write_reqs:
	release_free_reqs(&ctx->read_progress.fifo);
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
	release_free_reqs(&ctx->read_progress.fifo);
	release_free_reqs(&ctx->write_progress.fifo);
	free(ctx);
}

static int usb_queue_read(struct usb_ctx *ctx, void *buf, int nb)
{
	struct rw_req *req = malloc(sizeof(*req));

	if (!req)
		return -ENOMEM;
	req->buf = buf;
	req->nb = nb;
	req->nb_filled = 0;
	list_add_tail(&req->list, &ctx->read_progress.queued);
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
	INIT_LIST_HEAD(&req->list);
	req->nb = nb;
	req->nb_filled = 0;
	memcpy(req->buf, buf, nb);
	list_add_tail(&req->list, &ctx->write_progress.queued);
	return nb;
}

static void read_cb(struct libusb_transfer *transfer)
{
	struct fifo_req *src_req = transfer->user_data;
	struct rw_req *dst_req;
	unsigned char *src = src_req->buf;
	int nb, rc = 0, remain = transfer->actual_length;
	struct reqs_progress *progress = src_req->progress;
	char *str;

	str = hexdump(src, remain);
	DEBUG_JTAG_IO("status=%d progress=%d transferred %d [%s]",
		      transfer->status, progress->overflowed,
		      remain, str);
	free(str);

	list_move_tail(&src_req->list, &progress->fifo.free);
	if (transfer->status) {
		progress->overflowed = transfer->status;
		return;
	}

	remain -= 2;
	src += 2;
	while (!progress->overflowed && remain > 0 && !rc) {
		if (list_empty(&progress->queued)) {
			progress->overflowed = 1;
			break;
		}
		dst_req = list_first_entry(&progress->queued, struct rw_req,
					   list);
		nb = MIN(remain, dst_req->nb - dst_req->nb_filled);
		if (nb) {
			memcpy(dst_req->buf + dst_req->nb_filled, src, nb);
			dst_req->nb_filled += nb;
			remain -= nb;
			src += nb;
		}
		if (dst_req->nb_filled >= dst_req->nb) {
			list_move_tail(&dst_req->list, &progress->done);
			str = hexdump(dst_req->buf, dst_req->nb);
			DEBUG_JTAG_IO("filled submited read %d : [%s]",
				      dst_req->nb, str);
			free(str);
		}
		DEBUG_JTAG_IO("dst_req: buf=%p, nb_filled=%d, nb=%d : copied %d bytes",
			      dst_req->buf, dst_req->nb_filled, dst_req->nb, nb);
	}
	rc = feed_read_fifo(src_req, &progress->fifo);
	if (rc)
		progress->overflowed = rc;
}

static void write_cb(struct libusb_transfer *transfer)
{
	struct fifo_req *dst_req = transfer->user_data;
	struct reqs_progress *progress = dst_req->progress;
	int rc;

	char *str = hexdump(dst_req->buf, transfer->actual_length);
	DEBUG_JTAG_IO("status=%d, transferred %d [%s]",
		      transfer->status,
		      transfer->actual_length, str);
	free(str);

	list_move_tail(&dst_req->list, &progress->fifo.free);

	if (transfer->status) {
		progress->overflowed = transfer->status;
		return;
	}

	if (transfer->actual_length < transfer->length) {
		transfer->buffer += transfer->actual_length;
		transfer->length -= transfer->actual_length;
		rc = libusb_submit_transfer(dst_req->xfer);
		return;
	}

	rc = feed_write_fifo(dst_req, &progress->fifo, progress);
	if (rc)
		progress->overflowed = rc;
}

static int get_bytes_of_reqs(struct list_head *reqs)
{
	struct rw_req *req;
	int nb = 0;

	list_for_each_entry(req, reqs, list)
		nb += req->nb;
	return nb;
}

static int usb_flush(struct usb_ctx *ctx)
{
	int nb_read, nb_write, rc = 0;
	struct reqs_progress *rprog, *wprog;

	rprog = &ctx->read_progress;
	wprog = &ctx->write_progress;
	nb_read = get_bytes_of_reqs(&rprog->queued);
	nb_write = get_bytes_of_reqs(&wprog->queued);
	DEBUG_JTAG_IO("write %d, read %d", nb_write, nb_read);

	if (nb_read == 0 && nb_write == 0)
		return 0;
	/*
	 * Initialize progresses
	 */
	rprog->overflowed = 0;
	wprog->overflowed = 0;

	if (nb_read)
		rc = queue_read_free_reqs(ctx);
	if (!rc && nb_write)
		rc = queue_write_free_reqs(ctx);

	/* Polling loop, more or less taken from libftdi */
	while (!rc &&
	       (!list_empty(&rprog->queued) || !list_empty(&wprog->fifo.queued))) {
		rc = libusb_handle_events(ctx->usb_ctx);
		keep_alive();
	}
	if (rc) {
		LOG_ERROR("libusb_handle_events() failed with %d", rc);
		rc = -EIO;
	} else if (rprog->overflowed) {
		LOG_ERROR("usb device did not send all data: rc = %d",
			  rprog->overflowed);
		rc = -EIO;
	} else if (wprog->overflowed) {
		LOG_ERROR("usb device did not accept all data: rc = %d",
			  wprog->overflowed);
		rc = -EIO;
	} else {
		rc = 0;
	}

	cancel_all_fifo(ctx, &rprog->fifo);
	cancel_all_fifo(ctx, &wprog->fifo);
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
	/* ctx = usb_open(low->ublast_vid, low->ublast_pid, 1, 0x85, 0x06); */
	ctx = usb_open(low->ublast_vid, low->ublast_pid, 1, 0x81, 0x02);
	if (!ctx) {
		LOG_ERROR("usb_open error");
		return ERROR_JTAG_INIT_FAILED;
	}
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
