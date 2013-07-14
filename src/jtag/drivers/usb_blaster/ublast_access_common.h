/*
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
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
 */

struct rw_req {
	void *buf;
	int nb;
	struct list_head list;
};

extern int ublast_common_flush(struct ublast_lowlevel *low,
			       struct list_head *read_reqs,
			       struct list_head *write_reqs,
			       int(*read)(struct ublast_lowlevel*, uint8_t*, unsigned),
			       int(*write)(struct ublast_lowlevel*, uint8_t*, int));
extern int ublast_common_queue_read(struct list_head *read_reqs,
				    uint8_t *buf, unsigned int size);
extern int ublast_common_queue_write(struct list_head *write_reqs,
				     uint8_t *buf, unsigned int size);

