/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <libqrtr.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "gps_proxy.h"
#include "qmi_gps.h"
#include "util.h"

#define QMI_LOC_SERVICE			0x10
#define QMI_LOC_VERSION			0x02
#define QMI_LOC_INSTANCE		0x00

#define QMI_LOC_EVENT_MASK_NMEA_V02 	((uint64_t)0x00000004ull)
#define MAX_BUF_SZ 			512

typedef enum {
	NEED_SEND = 1,
	NEED_RECV
} op_mask_t;

struct qmi_packet {
	uint8_t flags;
	uint16_t txn_id;
	uint16_t msg_id;
	uint16_t msg_len;
	uint8_t data[];
} __attribute__((__packed__));

/* TODO: include from kernel once it lands */
struct sockaddr_qrtr {
	unsigned short sq_family;
	uint32_t sq_node;
	uint32_t sq_port;
};

static unsigned g_txn = 1;
static uint8_t g_session_id = 0;
static uint32_t g_loc_node = -1;
static uint32_t g_loc_port = -1;
static bool dbgprintf_enabled;

static void dbgprintf(const char *fmt, ...)
{
	va_list ap;

	if (!dbgprintf_enabled)
		return;

	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

int qrtr_send_recv(int sock, unsigned node, unsigned port, void *ptr, size_t len, char *buf,
		 	size_t buf_len, unsigned *in_node, unsigned *in_port,
			struct qmi_packet **qmi, int op_mask)
{
	int ret = 0;
	if (NEED_SEND & op_mask) {
		ret = qrtr_sendto(sock, node, port, ptr, len);
		if (ret < 0) {
			fprintf(stderr, "[GPS] failed to send request: %s\n", strerror(-ret));
			ret = -EINVAL;
		}
	}

	if (NEED_RECV & op_mask) {
		/* wait for response */
		ret = qrtr_recvfrom(sock, buf, buf_len, in_node, in_port);
		if (ret < 0) {
			fprintf(stderr, "[GPS] failed to receive response: %s\n", strerror(-ret));
			ret = -EINVAL;
		}
		
		*qmi = (struct qmi_packet*)buf;
		if ((*qmi)->msg_len != ret - sizeof(struct qmi_packet)) {
			fprintf(stderr, "[GPS] Invalid length of incoming qmi request\n");
			ret = -EINVAL;
		}
	}

	return 0;
}

static int check_result(struct gps_qmi_result *result)
{
	if (!result) {
		fprintf(stderr, "[GPS] Can't get response result from message\n");
		return -EINVAL;
	}
	if (result->result != QMI_GPS_RESULT_SUCCESS) {
		fprintf(stderr, "[GPS] Stop Response error: %d, %d\n",
			(int)result->result,(int)result->error);
		return -EINVAL;
	}
	++g_txn;

	return 0;
}

int gps_send_start(int sock, unsigned node, unsigned port)
{
	int ret = 0;
	struct gps_qmi_result *result;
	struct gps_loc_start_req *req;
	struct gps_loc_start_resp *resp;
	size_t len;
	void *ptr;
	unsigned in_node,in_port;
	char buf[MAX_BUF_SZ];
	struct qmi_packet *qmi;
	unsigned txn;


	dbgprintf("[GPS] send LOC_START to (%d:%d)\n", node, port);

	req = gps_loc_start_req_alloc(g_txn);
	gps_loc_start_req_set_session_id(req,g_session_id);
	ptr = gps_loc_start_req_encode(req, &len);
	if (!ptr)
		goto free_resp;

	ret = qrtr_send_recv(sock, node, port, ptr, len, buf, sizeof(buf), 
				&in_node, &in_port, &qmi, NEED_SEND | NEED_RECV);
	if (!ret)
		goto free_resp;

	/* check response status */
	switch (qmi->msg_id) {
	case QMI_GPS_LOC_START:
		resp = gps_loc_start_resp_parse(qmi, qmi->msg_len, &txn);
		if ((!resp) || (txn != g_txn)) {
			fprintf(stderr, "[GPS] Invalid response message\n");
			ret = -EINVAL;
			goto free_resp;
		}
		result = gps_loc_start_resp_get_result(resp);
		ret = check_result(result);
		if (ret)
			goto free_resp;

		break;

	default:
		fprintf(stderr, "[GPS] Unknown message: id=%d size=%d\n", qmi->msg_id, qmi->msg_len);
		ret = -EINVAL;
		break;
	}


free_resp:
	gps_loc_start_req_free(req);

	return ret;
}

int gps_send_stop(int sock, unsigned node, unsigned port)
{
	int ret = 0;
	struct gps_qmi_result *result;
	struct gps_loc_stop_req *req;
	struct gps_loc_stop_resp *resp;
	void *ptr;
	unsigned in_node,in_port;
	char buf[MAX_BUF_SZ];
	struct qmi_packet *qmi;
	unsigned txn;
	int stop = 0;
	size_t len;

	dbgprintf("[GPS] send LOC_STOP to (%d:%d)\n", node, port);

	req = gps_loc_stop_req_alloc(g_txn);
	gps_loc_stop_req_set_session_id(req,g_session_id);
	ptr = gps_loc_stop_req_encode(req, &len);
	if (!ptr)
		goto free_resp;

	ret = qrtr_send_recv(sock, node, port, ptr, len, buf, sizeof(buf),
				&in_node, &in_port, &qmi, NEED_SEND);
	if (!ret)
		goto free_resp;

	while (!stop) {
		ret = qrtr_send_recv(sock, node, port, ptr, len, buf, sizeof(buf),
					&in_node, &in_port, &qmi, NEED_RECV);
		if (!ret)
			goto free_resp;

		/* check response status */
		switch (qmi->msg_id) {
		case QMI_GPS_LOC_STOP:
			resp = gps_loc_stop_resp_parse(qmi, qmi->msg_len, &txn);
			if ((!resp) || (txn != g_txn)) {
				fprintf(stderr, "[GPS] Invalid response message\n");
				ret = -EINVAL;
				goto free_resp;
			}
			result = gps_loc_stop_resp_get_result(resp);
			ret = check_result(result);
			if (ret)
				goto free_resp;
			stop = 1;

			break;
		case QMI_GPS_LOC_NMEA_IND:
			break;

		default:
			fprintf(stderr, "[GPS] Unknown message: id=%d size=%d\n", qmi->msg_id, qmi->msg_len);
			ret = -EINVAL;
			stop = 1;
			break;
		}
	}

free_resp:
	gps_loc_stop_req_free(req);

	return ret;
}

int gps_send_register_nmea_event(int sock, unsigned node, unsigned port)
{
	int ret = 0;
	struct gps_qmi_result *result;
	struct gps_loc_reg_events_req *req;
	struct gps_loc_reg_events_resp *resp;
	size_t len;
	void *ptr;
	unsigned in_node,in_port;
	char buf[MAX_BUF_SZ];
	struct qmi_packet *qmi;
	unsigned txn;


	dbgprintf("[GPS] send LOC_REG_EVENT to (%d:%d)\n", node, port);

	req = gps_loc_reg_events_req_alloc(g_txn);
	gps_loc_reg_events_req_set_event_reg_mask(req,QMI_LOC_EVENT_MASK_NMEA_V02);
	ptr = gps_loc_reg_events_req_encode(req, &len);
	if (!ptr)
		goto free_resp;

	ret = qrtr_send_recv(sock, node, port, ptr, len, buf, sizeof(buf),
				&in_node, &in_port, &qmi, NEED_SEND | NEED_RECV);
	if (!ret)
		goto free_resp;

	/* check response status */
	switch (qmi->msg_id) {
	case QMI_GPS_LOC_REG_EVENTS:
		resp = gps_loc_reg_events_resp_parse(qmi, qmi->msg_len, &txn);
		if ((!resp) || (txn != g_txn)) {
			fprintf(stderr, "[GPS] Invalid response message\n");
			ret = -EINVAL;
			goto free_resp;
		}
		result = gps_loc_reg_events_resp_get_result(resp);
		ret = check_result(result);
		if (ret)
			goto free_resp;

		break;

	default:
		fprintf(stderr, "[GPS] Unknown message: id=%d size=%d\n", qmi->msg_id, qmi->msg_len);
		ret = -EINVAL;
		break;
	}


free_resp:
	gps_loc_reg_events_req_free(req);

	return ret;
}

void loc_service_cb(void *udata,uint32_t service,uint32_t instance,uint32_t node,uint32_t port)
{
	dbgprintf("[GPS] New LOC service found at (%u:%u)\n", node, port);
	g_loc_node = node;
	g_loc_port = port;	
}

int handle_gps_ind(int sock,int dev)
{
	struct sockaddr_qrtr sq;
	socklen_t sl;
	char buf[MAX_BUF_SZ];
	int ret,len;
	unsigned txn;
	struct gps_loc_event_nmea_ind *req;
	struct gps_proxy_data nmea_data;

	sl = sizeof(sq);
	len = ret = recvfrom(sock, buf, sizeof(buf), 0, (void *)&sq, &sl);
	if (ret < 0) {
		fprintf(stderr, "[GPS] recvfrom failed: %d\n", ret);
		return ret;
	}
	req = gps_loc_event_nmea_ind_parse(buf,len,&txn);
	if (req) {
		if (0 < gps_loc_event_nmea_ind_get_nmea(req,nmea_data.nmea_string,sizeof(nmea_data.nmea_string))) {
			nmea_data.nmea_length = strnlen(nmea_data.nmea_string, sizeof(nmea_data.nmea_string)-1) +1;
			ret = ioctl(dev, QGPS_SEND_NMEA, &nmea_data);
			if ( ret < 0)
			{
				fprintf(stderr, "[GPS] can't post nmea string: %d\n", ret);
			}

		}
	}

	dbgprintf("[GPS] packet; from: %d:%d\n", sq.sq_node, sq.sq_port);
	if (dbgprintf_enabled)
		print_hex_dump("[GPS <-]", buf, len);

	return 0;
}

int main(int argc, char **argv)
{
	int gps_fd;
	fd_set rfds;
	int nfds;
	int ret;
	int ch_fd = 0;

	if (argc == 2 && strcmp(argv[1], "-v") == 0)
		dbgprintf_enabled = true;

	ch_fd = open("/dev/gps_proxy_ch", O_RDONLY);	
	if (ch_fd < 0){
		fprintf(stdout,"OPEN: %s\n", strerror(errno));
		return -1;
	}

	gps_fd = qrtr_open(0);
	if (gps_fd < 0) {
		fprintf(stderr, "failed to create qrtr socket");
		goto clean_file;
	}

	ret = qrtr_lookup(gps_fd, QMI_LOC_SERVICE, QMI_LOC_VERSION,
				QMI_LOC_INSTANCE,0xFFFFFFFF,loc_service_cb,NULL);
	if (ret < 0 || g_loc_node == -1 || g_loc_port == -1) {
		fprintf(stderr, "failed to lookup LOC service");
		goto clean;
	}
	
	for (;;) {
		ret = ioctl(ch_fd, QGPS_REGISTER_HANDLE, 0);

		ret = gps_send_register_nmea_event(gps_fd,g_loc_node,g_loc_port);
		if (ret < 0) {
			fprintf(stderr, "failed to send LOC_REG_EVENTS");
			goto clean;
		}

		ret = gps_send_start(gps_fd,g_loc_node,g_loc_port);
		if (ret < 0) {
			fprintf(stderr, "failed to send LOC_START");
			goto clean;
		}

		for (;;) {
			FD_ZERO(&rfds);
			FD_SET(gps_fd, &rfds);

			nfds = gps_fd + 1;
			ret = select(nfds, &rfds, NULL, NULL, NULL);
			if (ret < 0) {
				fprintf(stderr, "select failed: %d\n", ret);
				break;
			} else if (ret == 0) {
				continue;
			}

			if (FD_ISSET(gps_fd, &rfds))
				handle_gps_ind(gps_fd,ch_fd);
			
			ret = ioctl(ch_fd, QGPS_IS_ACTIVE, 0);
			if (0!=ret) {
				dbgprintf("All clients have disconnected\n");
				break;
			}
		}

		ret = gps_send_stop(gps_fd,g_loc_node,g_loc_port);
		if (ret < 0) {
			fprintf(stderr, "failed to send LOC_STOP");
			goto clean;
		}
	}
clean:
	qrtr_close(gps_fd);
clean_file:
	close(ch_fd);
	return 0;
}
