#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <jtag/interface.h>

#include "ublast_access.h"
#include "ublast_access_common.h"

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

static void xfer_buf_to_reqs(unsigned char *buf, struct list_head *reqs)
{
	struct rw_req *req;

	list_for_each_entry(req, reqs, list) {
		memmove(req->buf, buf, req->nb);
		buf += req->nb;
	}
}

int ublast_common_flush(struct ublast_lowlevel *low,
			struct list_head *read_reqs,
			struct list_head *write_reqs,
			int(*read)(struct ublast_lowlevel*, uint8_t*, unsigned),
			int(*write)(struct ublast_lowlevel*, uint8_t*, int))
{
	int retval = 0, nb_read, nb_write;
	struct rw_req *req, *tmp;
	unsigned char *buf_read = NULL, *buf_write = NULL;

	nb_read = get_bytes_of_reqs(read_reqs);
	nb_write = get_bytes_of_reqs(write_reqs);
	buf_read = malloc(nb_read);
	buf_write = malloc(nb_write);
	if (!buf_read || !buf_write) {
		retval = -ENOMEM;
		goto out;
	}

	xfer_reqs_to_buf(buf_read, read_reqs);
	xfer_reqs_to_buf(buf_write, write_reqs);

	retval = write(low, buf_write, nb_write);
	if (retval < 0)	{
		LOG_ERROR("error_write_data: %d", retval);
		goto out;
	}
	retval = read(low, buf_read, nb_read);
	if (retval < 0)	{
		LOG_ERROR("error read data: %d", retval);
		goto out;
	}
out:
	xfer_buf_to_reqs(buf_read, read_reqs);
	list_for_each_entry_safe(req, tmp, write_reqs, list) {
		list_del(&req->list);
		free(req->buf);
		free(req);
	}
	list_for_each_entry_safe(req, tmp, read_reqs, list) {
		list_del(&req->list);
		free(req);
	}
	return retval;
}

int ublast_common_queue_read(struct list_head *read_reqs,
			     uint8_t *buf, unsigned int size)
{
	struct rw_req *req = malloc(sizeof(*req));

	if (!req)
		return -ENOMEM;
	req->buf = buf;
	req->nb = size;
	list_add_tail(&req->list, read_reqs);
	return size;
}

int ublast_common_queue_write(struct list_head *write_reqs,
			      uint8_t *buf, unsigned int size)
{
	struct rw_req *req = malloc(sizeof(*req));

	if (!req)
		return -ENOMEM;
	req->buf = malloc(size);
	if (!req->buf) {
		free(req);
		return -ENOMEM;
	}
	memcpy(req->buf, buf, size);
	req->nb = size;
	list_add_tail(&req->list, write_reqs);
	return size;
}
