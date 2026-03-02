/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 * Copyright (C) 2012 Joel Holdsworth <joel@airwebreathe.org.uk>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <math.h>
#include <stdbool.h>
#include <glib.h>
#include <glib/gstdio.h>
#include "protocol.h"

#define DS_CMD_GET_FW_VERSION		0xb0
#define DS_CMD_GET_REVID_VERSION	0xb1
#define DS_CMD_START			0xb2
#define DS_CMD_CONFIG			0xb3
#define DS_CMD_SETTING			0xb4
#define DS_CMD_CONTROL			0xb5
#define DS_CMD_STATUS			0xb6
#define DS_CMD_STATUS_INFO		0xb7
#define DS_CMD_WR_REG			0xb8
#define DS_CMD_WR_NVM			0xb9
#define DS_CMD_RD_NVM			0xba
#define DS_CMD_RD_NVM_PRE		0xbb
#define DS_CMD_GET_HW_INFO		0xbc
#define DSL_CMD_CTL_WR			0xb0
#define DSL_CMD_CTL_RD_PRE		0xb1
#define DSL_CMD_CTL_RD			0xb2

#define DS_START_FLAGS_STOP		(1 << 7)
#define DS_START_FLAGS_CLK_48MHZ	(1 << 6)
#define DS_START_FLAGS_SAMPLE_WIDE	(1 << 5)
#define DS_START_FLAGS_MODE_LA		(1 << 4)

#define DS_ADDR_COMB			0x68
#define DS_ADDR_EEWP			0x70
#define DS_ADDR_VTH			0x78

#define DS_MAX_LOGIC_DEPTH		SR_MHZ(16)
#define DS_MAX_LOGIC_SAMPLERATE		SR_MHZ(100)
#define DS_MAX_TRIG_PERCENT		90

#define DS_MODE_TRIG_EN			(1 << 0)
#define DS_MODE_CLK_TYPE		(1 << 1)
#define DS_MODE_CLK_EDGE		(1 << 2)
#define DS_MODE_RLE_MODE		(1 << 3)
#define DS_MODE_DSO_MODE		(1 << 4)
#define DS_MODE_HALF_MODE		(1 << 5)
#define DS_MODE_QUAR_MODE		(1 << 6)
#define DS_MODE_ANALOG_MODE		(1 << 7)
#define DS_MODE_FILTER			(1 << 8)
#define DS_MODE_INSTANT			(1 << 9)
#define DS_MODE_STRIG_MODE		(1 << 11)
#define DS_MODE_STREAM_MODE		(1 << 12)
#define DS_MODE_LPB_TEST		(1 << 13)
#define DS_MODE_EXT_TEST		(1 << 14)
#define DS_MODE_INT_TEST		(1 << 15)

#define DSLOGIC_ATOMIC_SAMPLES		(sizeof(uint64_t) * 8)
#define DSLOGIC_ATOMIC_BYTES		sizeof(uint64_t)

#define DSL_CTL_START			8
#define DSL_CTL_STOP			9
#define DSL_CTL_BULK_WR			10
#define DSL_CTL_I2C_REG			14
#define DSL_CTL_INTRDY			6
#define DSL_CTL_WORDWIDE		7
#define DSL_CTL_FW_VERSION		0
#define DSL_CTL_HW_STATUS		2
#define DSL_CTL_PROG_B			3
#define DSL_CTL_LED			5

#define DSL_HW_STATUS_GPIF_DONE		(1 << 7)
#define DSL_HW_STATUS_FPGA_DONE		(1 << 6)
#define DSL_HW_STATUS_FPGA_INIT_B	(1 << 5)
#define DSL_HW_STATUS_SYS_CLR		(1 << 3)
#define DSL_WR_PROG_B			(1 << 2)
#define DSL_WR_INTRDY_HIGH		(1 << 7)
#define DSL_WR_WORDWIDE			(1 << 0)
#define DSL_LED_GREEN			(1 << 0)
#define DSL_LED_RED			(1 << 1)

#define DSL_SYS_CLR_POLL_INTERVAL_US	(1000)
#define DSL_SYS_CLR_TIMEOUT_US		(100 * 1000)
#define DSL_FPGA_POLL_INTERVAL_US	(1000)
#define DSL_FPGA_INIT_TIMEOUT_US	(500 * 1000)
#define DSL_FPGA_DONE_TIMEOUT_US	(1000 * 1000)

/*
 * The FPGA is configured with TLV tuples. Length is specified as the
 * number of 16-bit words.
 */
#define _DS_CFG(variable, wordcnt) ((variable << 8) | wordcnt)
#define DS_CFG_START			0xf5a5f5a5
#define DS_CFG_MODE			_DS_CFG(0, 1)
#define DS_CFG_DIVIDER			_DS_CFG(1, 2)
#define DS_CFG_COUNT			_DS_CFG(3, 2)
#define DS_CFG_TRIG_POS			_DS_CFG(5, 2)
#define DS_CFG_TRIG_GLB			_DS_CFG(7, 1)
#define DS_CFG_CH_EN			_DS_CFG(8, 1)
#define DS_CFG_DSO_COUNT		_DS_CFG(8, 2)
#define DS_CFG_CH_EN_WIDE		_DS_CFG(10, 2)
#define DS_CFG_FGAIN			_DS_CFG(12, 1)
#define DS_CFG_TRIG			_DS_CFG(64, 160)
#define DS_CFG_END			0xfa5afa5a

#pragma pack(push, 1)

struct version_info {
	uint8_t major;
	uint8_t minor;
};

struct cmd_start_acquisition {
	uint8_t flags;
	uint8_t sample_delay_h;
	uint8_t sample_delay_l;
};

struct fpga_config {
	uint32_t sync;

	uint16_t mode_header;
	uint16_t mode;
	uint16_t divider_header;
	uint32_t divider;
	uint16_t count_header;
	uint32_t count;
	uint16_t trig_pos_header;
	uint32_t trig_pos;
	uint16_t trig_glb_header;
	uint16_t trig_glb;
	uint16_t ch_en_header;
	uint16_t ch_en;

	uint16_t trig_header;
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];
	uint16_t trig_logic0[NUM_TRIGGER_STAGES];
	uint16_t trig_logic1[NUM_TRIGGER_STAGES];
	uint32_t trig_count[NUM_TRIGGER_STAGES];

	uint32_t end_sync;
};

struct fpga_config_v2 {
	uint32_t sync;

	uint16_t mode_header;
	uint16_t mode;
	uint16_t divider_header;
	uint32_t divider;
	uint16_t count_header;
	uint32_t count;
	uint16_t trig_pos_header;
	uint32_t trig_pos;
	uint16_t trig_glb_header;
	uint16_t trig_glb;
	uint16_t dso_count_header;
	uint32_t dso_count;
	uint16_t ch_en_header;
	uint32_t ch_en;
	uint16_t fgain_header;
	uint16_t fgain;

	uint16_t trig_header;
	uint16_t trig_mask0[NUM_TRIGGER_STAGES];
	uint16_t trig_mask1[NUM_TRIGGER_STAGES];
	uint16_t trig_value0[NUM_TRIGGER_STAGES];
	uint16_t trig_value1[NUM_TRIGGER_STAGES];
	uint16_t trig_edge0[NUM_TRIGGER_STAGES];
	uint16_t trig_edge1[NUM_TRIGGER_STAGES];
	uint16_t trig_logic0[NUM_TRIGGER_STAGES];
	uint16_t trig_logic1[NUM_TRIGGER_STAGES];
	uint32_t trig_count[NUM_TRIGGER_STAGES];

	uint32_t end_sync;
};

struct dsl_ctl_header {
	uint8_t dest;
	uint16_t offset;
	uint8_t size;
};

#pragma pack(pop)

/*
 * This should be larger than the FPGA bitstream image so that it'll get
 * uploaded in one big operation. There seem to be issues when uploading
 * it in chunks.
 */
#define FW_BUFSIZE (1024 * 1024)

#define FPGA_UPLOAD_DELAY (10 * 1000)

#define USB_TIMEOUT (3 * 1000)

static int command_ctl_rd_data(libusb_device_handle *devhdl, uint8_t dest,
			       uint8_t *buf, uint8_t size);

static int command_get_fw_version(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_FW_VERSION, 0x0000, 0x0000,
		(unsigned char *)vi, sizeof(struct version_info), USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get version info: %s.",
		       libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_get_fw_version_v2(libusb_device_handle *devhdl,
				  struct version_info *vi)
{
	uint8_t data[2] = { 0, 0 };
	int ret;

	ret = command_ctl_rd_data(devhdl, DSL_CTL_FW_VERSION, data, sizeof(data));
	if (ret != SR_OK)
		return ret;

	vi->major = data[0];
	vi->minor = data[1];
	return SR_OK;
}

static int command_get_revid_version(struct sr_dev_inst *sdi, uint8_t *revid)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	libusb_device_handle *devhdl = usb->devhdl;
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DS_CMD_GET_REVID_VERSION, 0x0000, 0x0000,
		revid, 1, USB_TIMEOUT);

	if (ret < 0) {
		sr_err("Unable to get REVID: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_start_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct dslogic_mode mode;
	int ret;

	mode.flags = DS_START_FLAGS_MODE_LA | DS_START_FLAGS_SAMPLE_WIDE;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send start command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_stop_acquisition(const struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb;
	struct dslogic_mode mode;
	int ret;

	mode.flags = DS_START_FLAGS_STOP;
	mode.sample_delay_h = mode.sample_delay_l = 0;

	usb = sdi->conn;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_START, 0x0000, 0x0000,
			(unsigned char *)&mode, sizeof(struct dslogic_mode), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send stop command: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_ctl_wr_simple(libusb_device_handle *devhdl, uint8_t dest)
{
	struct dsl_ctl_header cmd = { .dest = dest, .offset = 0, .size = 0 };
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, DSL_CMD_CTL_WR, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CMD_CTL_WR command(dest:%u): %s.",
			dest, libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_ctl_wr_reg_byte(libusb_device_handle *devhdl, uint8_t dest,
		uint16_t offset, uint8_t value)
{
	struct {
		struct dsl_ctl_header h;
		uint8_t data[1];
	} cmd = { 0 };
	int ret;

	cmd.h.dest = dest;
	cmd.h.offset = offset;
	cmd.h.size = 1;
	cmd.data[0] = value;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, DSL_CMD_CTL_WR, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CMD_CTL_WR command(dest:%u, off:%u): %s.",
			dest, offset, libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_ctl_wr_bulk_words(libusb_device_handle *devhdl, uint32_t words)
{
	struct {
		struct dsl_ctl_header h;
		uint8_t data[3];
	} cmd = { 0 };
	int ret;

	cmd.h.dest = DSL_CTL_BULK_WR;
	cmd.h.offset = 0;
	cmd.h.size = 3;
	cmd.data[0] = (uint8_t)(words & 0xff);
	cmd.data[1] = (uint8_t)((words >> 8) & 0xff);
	cmd.data[2] = (uint8_t)((words >> 16) & 0xff);

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, DSL_CMD_CTL_WR, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CTL_BULK_WR command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_ctl_wr_bulk_bytes(libusb_device_handle *devhdl, uint32_t bytes)
{
	struct {
		struct dsl_ctl_header h;
		uint8_t data[3];
	} cmd = { 0 };
	int ret;

	cmd.h.dest = DSL_CTL_BULK_WR;
	cmd.h.offset = 0;
	cmd.h.size = 3;
	cmd.data[0] = (uint8_t)(bytes & 0xff);
	cmd.data[1] = (uint8_t)((bytes >> 8) & 0xff);
	cmd.data[2] = (uint8_t)((bytes >> 16) & 0xff);

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, DSL_CMD_CTL_WR, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CTL_BULK_WR(byte mode) command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_ctl_rd_data(libusb_device_handle *devhdl, uint8_t dest,
		uint8_t *data, uint8_t size)
{
	struct dsl_ctl_header cmd = { .dest = dest, .offset = 0, .size = size };
	int ret;

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_OUT, DSL_CMD_CTL_RD_PRE, 0x0000, 0x0000,
		(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CMD_CTL_RD_PRE command(dest:%u): %s.",
			dest, libusb_error_name(ret));
		return SR_ERR;
	}

	g_usleep(10 * 1000);

	ret = libusb_control_transfer(devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_ENDPOINT_IN, DSL_CMD_CTL_RD, 0x0000, 0x0000,
		data, size, USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Unable to send DSL_CMD_CTL_RD command(dest:%u): %s.",
			dest, libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

static int command_start_acquisition_v2(const struct sr_dev_inst *sdi)
{
	const struct sr_usb_dev_inst *usb = sdi->conn;
	return command_ctl_wr_simple(usb->devhdl, DSL_CTL_START);
}

static int command_stop_acquisition_v2(const struct sr_dev_inst *sdi)
{
	const struct sr_usb_dev_inst *usb = sdi->conn;
	return command_ctl_wr_simple(usb->devhdl, DSL_CTL_STOP);
}

SR_PRIV int dslogic_fpga_firmware_upload(const struct sr_dev_inst *sdi)
{
	const char *name = NULL;
	uint64_t sum;
	struct sr_resource bitstream;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	unsigned char *buf;
	ssize_t chunksize;
	int transferred;
	int result, ret;
	const uint8_t cmd[3] = {0, 0, 0};

	drvc = sdi->driver->context;
	devc = sdi->priv;
	usb = sdi->conn;

	if (!strcmp(devc->profile->model, "DSLogic")) {
		if (devc->cur_threshold < 1.40)
			name = DSLOGIC_FPGA_FIRMWARE_3V3;
		else
			name = DSLOGIC_FPGA_FIRMWARE_5V;
	} else if (!strcmp(devc->profile->model, "DSLogic Pro")){
		name = DSLOGIC_PRO_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Plus")){
		name = DSLOGIC_PLUS_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic Basic")){
		name = DSLOGIC_BASIC_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSLogic U2Basic")){
		name = DSLOGIC_U2BASIC_FPGA_FIRMWARE;
	} else if (!strcmp(devc->profile->model, "DSCope")) {
		name = DSCOPE_FPGA_FIRMWARE;
	} else {
		sr_err("Failed to select FPGA firmware.");
		return SR_ERR;
	}

	sr_dbg("Uploading FPGA firmware '%s'.", name);

	result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, name);
	if (result != SR_OK)
		return result;

	/* Tell the device firmware is coming. */
	if ((ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_CONFIG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), USB_TIMEOUT)) < 0) {
		sr_err("Failed to upload FPGA firmware: %s.", libusb_error_name(ret));
		sr_resource_close(drvc->sr_ctx, &bitstream);
		return SR_ERR;
	}

	/* Give the FX2 time to get ready for FPGA firmware upload. */
	g_usleep(FPGA_UPLOAD_DELAY);

	buf = g_malloc(FW_BUFSIZE);
	sum = 0;
	result = SR_OK;
	while (1) {
		chunksize = sr_resource_read(drvc->sr_ctx, &bitstream,
				buf, FW_BUFSIZE);
		if (chunksize < 0)
			result = SR_ERR;
		if (chunksize <= 0)
			break;

		if ((ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
				buf, chunksize, &transferred, USB_TIMEOUT)) < 0) {
			sr_err("Unable to configure FPGA firmware: %s.",
					libusb_error_name(ret));
			result = SR_ERR;
			break;
		}
		sum += transferred;
		sr_spew("Uploaded %" PRIu64 "/%" PRIu64 " bytes.",
			sum, bitstream.size);

		if (transferred != chunksize) {
			sr_err("Short transfer while uploading FPGA firmware.");
			result = SR_ERR;
			break;
		}
	}
	g_free(buf);
	sr_resource_close(drvc->sr_ctx, &bitstream);

	if (result == SR_OK)
		sr_dbg("FPGA firmware upload done.");

	return result;
}

static int poll_hw_status_bit(libusb_device_handle *devhdl, uint8_t mask,
		unsigned int timeout_us, const char *what, uint8_t *status_out)
{
	uint8_t status = 0;
	unsigned int waited_us = 0;
	int ret;

	while (waited_us <= timeout_us) {
		ret = command_ctl_rd_data(devhdl, DSL_CTL_HW_STATUS, &status, sizeof(status));
		if (ret != SR_OK)
			return ret;
		if (status & mask) {
			if (status_out)
				*status_out = status;
			return SR_OK;
		}
		g_usleep(DSL_FPGA_POLL_INTERVAL_US);
		waited_us += DSL_FPGA_POLL_INTERVAL_US;
	}

	if (status_out)
		*status_out = status;
	sr_err("Timeout waiting for %s (HW_STATUS=0x%02x).", what, status);
	return SR_ERR;
}

static int dslogic_u2basic_fpga_firmware_upload_v2(const struct sr_dev_inst *sdi)
{
	struct sr_resource bitstream;
	struct drv_context *drvc;
	struct sr_usb_dev_inst *usb;
	unsigned char *buf = NULL;
	uint8_t hw_status = 0;
	uint64_t sum = 0;
	ssize_t chunksize;
	int transferred;
	int result, ret;

	drvc = sdi->driver->context;
	usb = sdi->conn;

	result = sr_resource_open(drvc->sr_ctx, &bitstream,
			SR_RESOURCE_FIRMWARE, DSLOGIC_U2BASIC_FPGA_FIRMWARE);
	if (result != SR_OK)
		return result;

	if (bitstream.size == 0 || bitstream.size > 0xFFFFFF) {
		sr_err("Invalid U2Basic FPGA bitstream size: %" PRIu64 ".", bitstream.size);
		result = SR_ERR;
		goto out;
	}

	buf = g_malloc(FW_BUFSIZE);
	if (!buf) {
		result = SR_ERR_MALLOC;
		goto out;
	}

	sr_info("Uploading U2Basic FPGA bitstream '%s' (%" PRIu64 " bytes).",
		DSLOGIC_U2BASIC_FPGA_FIRMWARE, bitstream.size);

	/* Step 0: assert PROG_B low. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_PROG_B, 0,
			(uint8_t)~DSL_WR_PROG_B);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 1: turn off RED/GREEN LED. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_LED, 0,
			(uint8_t)~(DSL_LED_GREEN | DSL_LED_RED));
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 2: assert PROG_B high. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_PROG_B, 0, DSL_WR_PROG_B);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 3: wait INIT_B high. */
	ret = poll_hw_status_bit(usb->devhdl, DSL_HW_STATUS_FPGA_INIT_B,
			DSL_FPGA_INIT_TIMEOUT_US, "FPGA_INIT_B", &hw_status);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 4: assert INTRDY low and send BULK_WR byte size. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_INTRDY, 0,
			(uint8_t)~DSL_WR_INTRDY_HIGH);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}
	ret = command_ctl_wr_bulk_bytes(usb->devhdl, (uint32_t)bitstream.size);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 5: send bitstream over EP2 OUT. */
	while (1) {
		chunksize = sr_resource_read(drvc->sr_ctx, &bitstream, buf, FW_BUFSIZE);
		if (chunksize < 0) {
			result = SR_ERR;
			goto out;
		}
		if (chunksize == 0)
			break;

		ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT, buf,
				(int)chunksize, &transferred, USB_TIMEOUT);
		if (ret < 0 || transferred != chunksize) {
			sr_err("U2Basic FPGA upload bulk transfer failed: %s (tx=%d exp=%zd).",
				libusb_error_name(ret), transferred, chunksize);
			result = SR_ERR;
			goto out;
		}
		sum += transferred;
	}
	if (sum != bitstream.size) {
		sr_err("U2Basic FPGA upload short write: %" PRIu64 "/%" PRIu64 ".",
			sum, bitstream.size);
		result = SR_ERR;
		goto out;
	}

	/* Step 6: assert INTRDY high. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_INTRDY, 0, DSL_WR_INTRDY_HIGH);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 7: wait GPIF_DONE. */
	ret = poll_hw_status_bit(usb->devhdl, DSL_HW_STATUS_GPIF_DONE,
			DSL_FPGA_DONE_TIMEOUT_US, "GPIF_DONE", &hw_status);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 8: assert INTRDY low. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_INTRDY, 0,
			(uint8_t)~DSL_WR_INTRDY_HIGH);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 9: wait FPGA_DONE. */
	ret = poll_hw_status_bit(usb->devhdl, DSL_HW_STATUS_FPGA_DONE,
			DSL_FPGA_DONE_TIMEOUT_US, "FPGA_DONE", &hw_status);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	/* Step 10: LED green and recover GPIF word-wide mode. */
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_LED, 0, DSL_LED_GREEN);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}
	ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_WORDWIDE, 0, DSL_WR_WORDWIDE);
	if (ret != SR_OK) {
		result = ret;
		goto out;
	}

	sr_info("U2Basic FPGA configuration complete.");
	result = SR_OK;

out:
	g_free(buf);
	sr_resource_close(drvc->sr_ctx, &bitstream);
	return result;
}

SR_PRIV int dslogic_u2basic_ensure_fpga_configured(const struct sr_dev_inst *sdi,
		gboolean force_upload)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t hw_status = 0;
	int ret;

	ret = command_ctl_rd_data(usb->devhdl, DSL_CTL_HW_STATUS, &hw_status,
			sizeof(hw_status));
	if (ret != SR_OK)
		return ret;

	if (force_upload || !(hw_status & DSL_HW_STATUS_FPGA_DONE)) {
		sr_info("U2Basic FPGA upload required (force=%d, HW_STATUS=0x%02x).",
			force_upload, hw_status);
		return dslogic_u2basic_fpga_firmware_upload_v2(sdi);
	}

	sr_dbg("U2Basic FPGA already configured (HW_STATUS=0x%02x).", hw_status);
	return SR_OK;
}

static unsigned int enabled_channel_count(const struct sr_dev_inst *sdi)
{
	unsigned int count = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			count++;
	}
	return count;
}

static uint16_t enabled_channel_mask(const struct sr_dev_inst *sdi)
{
	unsigned int mask = 0;
	for (const GSList *l = sdi->channels; l; l = l->next) {
		const struct sr_channel *const probe = (struct sr_channel *)l->data;
		if (probe->enabled)
			mask |= 1 << probe->index;
	}
	return mask;
}

/*
 * Get the session trigger and configure the FPGA structure
 * accordingly.
 * @return @c true if any triggers are enabled, @c false otherwise.
 */
static bool set_trigger(const struct sr_dev_inst *sdi, struct fpga_config *cfg,
		bool u2basic_v2)
{
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	struct dev_context *devc;
	const GSList *l, *m;
	const unsigned int num_enabled_channels = enabled_channel_count(sdi);
	int num_trigger_stages = 0;

	int channelbit, i = 0;
	uint32_t trigger_point;

	devc = sdi->priv;

	cfg->ch_en = enabled_channel_mask(sdi);

	for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
		cfg->trig_mask0[i] = 0xffff;
		cfg->trig_mask1[i] = 0xffff;
		cfg->trig_value0[i] = 0;
		cfg->trig_value1[i] = 0;
		cfg->trig_edge0[i] = 0;
		cfg->trig_edge1[i] = 0;
		cfg->trig_logic0[i] = 2;
		cfg->trig_logic1[i] = 2;
		cfg->trig_count[i] = 0;
	}

	trigger_point = (devc->capture_ratio * devc->limit_samples) / 100;
	if (trigger_point < DSLOGIC_ATOMIC_SAMPLES)
		trigger_point = DSLOGIC_ATOMIC_SAMPLES;
	const uint32_t mem_depth = devc->profile->mem_depth;
	const uint32_t max_trigger_point = devc->continuous_mode ? ((mem_depth * 10) / 100) :
		((mem_depth * DS_MAX_TRIG_PERCENT) / 100);
	if (trigger_point > max_trigger_point)
		trigger_point = max_trigger_point;
	cfg->trig_pos = trigger_point & ~(DSLOGIC_ATOMIC_SAMPLES - 1);

	if (!(trigger = sr_session_trigger_get(sdi->session))) {
		sr_dbg("No session trigger found");
		cfg->trig_glb = u2basic_v2
			? ((num_enabled_channels & 0x1f) << 8)
			: (num_enabled_channels << 4);
		return false;
	}

	for (l = trigger->stages; l; l = l->next) {
		stage = l->data;
		num_trigger_stages++;
		for (m = stage->matches; m; m = m->next) {
			match = m->data;
			if (!match->channel->enabled)
				/* Ignore disabled channels with a trigger. */
				continue;
			channelbit = 1 << (match->channel->index);
			/* Simple trigger support (event). */
			if (match->match == SR_TRIGGER_ONE) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_ZERO) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
			} else if (match->match == SR_TRIGGER_FALLING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_RISING) {
				cfg->trig_mask0[0] &= ~channelbit;
				cfg->trig_mask1[0] &= ~channelbit;
				cfg->trig_value0[0] |= channelbit;
				cfg->trig_value1[0] |= channelbit;
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			} else if (match->match == SR_TRIGGER_EDGE) {
				cfg->trig_edge0[0] |= channelbit;
				cfg->trig_edge1[0] |= channelbit;
			}
		}
	}

	if (u2basic_v2) {
		cfg->trig_glb = ((num_enabled_channels & 0x1f) << 8) |
			(num_trigger_stages & 0xff);
	} else {
		cfg->trig_glb = (num_enabled_channels << 4) |
			(num_trigger_stages - 1);
	}

	return num_trigger_stages != 0;
}

static int fpga_configure(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	const bool u2basic_v2 = !strcmp(devc->profile->model, "DSLogic U2Basic");
	uint8_t c[3];
	struct fpga_config cfg;
	struct fpga_config_v2 cfg_v2;
	uint16_t mode = 0;
	uint32_t divider, cfg_words, count_units, actual_samples;
	int transferred, len, ret, i;

	sr_dbg("Configuring FPGA.");
	memset(&cfg, 0, sizeof(cfg));

	if (set_trigger(sdi, &cfg, u2basic_v2))
		mode |= DS_MODE_TRIG_EN;

	if (devc->mode == DS_OP_INTERNAL_TEST)
		mode |= DS_MODE_INT_TEST;
	else if (devc->mode == DS_OP_EXTERNAL_TEST)
		mode |= DS_MODE_EXT_TEST;
	else if (devc->mode == DS_OP_LOOPBACK_TEST)
		mode |= DS_MODE_LPB_TEST;

	if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 2)
		mode |= DS_MODE_HALF_MODE;
	else if (devc->cur_samplerate == DS_MAX_LOGIC_SAMPLERATE * 4)
		mode |= DS_MODE_QUAR_MODE;

	if (devc->continuous_mode)
		mode |= DS_MODE_STREAM_MODE;
	if (devc->external_clock) {
		mode |= DS_MODE_CLK_TYPE;
		if (devc->clock_edge == DS_EDGE_FALLING)
			mode |= DS_MODE_CLK_EDGE;
	}
	if (devc->limit_samples > DS_MAX_LOGIC_DEPTH *
		ceil(devc->cur_samplerate * 1.0 / DS_MAX_LOGIC_SAMPLERATE)
		&& !devc->continuous_mode) {
		/* Enable RLE for long captures.
		 * Without this, captured data present errors.
		 */
			mode |= DS_MODE_RLE_MODE;
	}

	divider = ceil(DS_MAX_LOGIC_SAMPLERATE * 1.0 / devc->cur_samplerate);
	actual_samples = (devc->limit_samples + (DSLOGIC_ATOMIC_SAMPLES - 1)) &
		~(DSLOGIC_ATOMIC_SAMPLES - 1);
	count_units = actual_samples / 16;

	if (u2basic_v2) {
		uint8_t hw_status = 0;
		unsigned int waited_us = 0;

		memset(&cfg_v2, 0, sizeof(cfg_v2));
		WL32(&cfg_v2.sync, DS_CFG_START);
		WL16(&cfg_v2.mode_header, DS_CFG_MODE);
		WL16(&cfg_v2.mode, mode);
		WL16(&cfg_v2.divider_header, DS_CFG_DIVIDER);
		WL32(&cfg_v2.divider, divider);
		WL16(&cfg_v2.count_header, DS_CFG_COUNT);
		WL32(&cfg_v2.count, count_units);
		WL16(&cfg_v2.trig_pos_header, DS_CFG_TRIG_POS);
		WL32(&cfg_v2.trig_pos, cfg.trig_pos);
		WL16(&cfg_v2.trig_glb_header, DS_CFG_TRIG_GLB);
		WL16(&cfg_v2.trig_glb, cfg.trig_glb);
		WL16(&cfg_v2.dso_count_header, DS_CFG_DSO_COUNT);
		/*
		 * U2Basic v2 expects DSO_COUNT in raw sample units (not /16),
		 * matching DSView's DSL_setting encoding.
		 */
		WL32(&cfg_v2.dso_count, actual_samples);
		WL16(&cfg_v2.ch_en_header, DS_CFG_CH_EN_WIDE);
		WL32(&cfg_v2.ch_en, cfg.ch_en);
		WL16(&cfg_v2.fgain_header, DS_CFG_FGAIN);
		WL16(&cfg_v2.fgain, 0);
		WL16(&cfg_v2.trig_header, DS_CFG_TRIG);
		for (i = 0; i < NUM_TRIGGER_STAGES; i++) {
			WL16(&cfg_v2.trig_mask0[i], cfg.trig_mask0[i]);
			WL16(&cfg_v2.trig_mask1[i], cfg.trig_mask1[i]);
			WL16(&cfg_v2.trig_value0[i], cfg.trig_value0[i]);
			WL16(&cfg_v2.trig_value1[i], cfg.trig_value1[i]);
			WL16(&cfg_v2.trig_edge0[i], cfg.trig_edge0[i]);
			WL16(&cfg_v2.trig_edge1[i], cfg.trig_edge1[i]);
			WL16(&cfg_v2.trig_logic0[i], cfg.trig_logic0[i]);
			WL16(&cfg_v2.trig_logic1[i], cfg.trig_logic1[i]);
			WL32(&cfg_v2.trig_count[i], cfg.trig_count[i]);
		}
		WL32(&cfg_v2.end_sync, DS_CFG_END);

		ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_WORDWIDE, 0,
				DSL_WR_WORDWIDE);
		if (ret != SR_OK)
			return ret;

		cfg_words = sizeof(struct fpga_config_v2) / 2;
		ret = command_ctl_wr_bulk_words(usb->devhdl, cfg_words);
		if (ret != SR_OK)
			return ret;

		/*
		 * Match DSView arm sequence: wait until SYS_CLR before pushing
		 * bulk payload. Some devices stay in pattern-like output if this
		 * handshake is skipped.
		 */
		while (waited_us < DSL_SYS_CLR_TIMEOUT_US) {
			ret = command_ctl_rd_data(usb->devhdl, DSL_CTL_HW_STATUS,
					&hw_status, sizeof(hw_status));
			if (ret == SR_OK && (hw_status & DSL_HW_STATUS_SYS_CLR))
				break;
			g_usleep(DSL_SYS_CLR_POLL_INTERVAL_US);
			waited_us += DSL_SYS_CLR_POLL_INTERVAL_US;
		}
		if (!(hw_status & DSL_HW_STATUS_SYS_CLR))
			sr_warn("U2Basic arm: SYS_CLR not observed (status=0x%02x).", hw_status);

		len = sizeof(struct fpga_config_v2);
		ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
				(unsigned char *)&cfg_v2, len, &transferred, USB_TIMEOUT);
		if (ret < 0 || transferred != len) {
			sr_err("Failed to send U2Basic FPGA configuration: %s.",
				libusb_error_name(ret));
			return SR_ERR;
		}
		ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_INTRDY, 0,
				DSL_WR_INTRDY_HIGH);
		if (ret != SR_OK)
			return ret;
		ret = command_ctl_rd_data(usb->devhdl, DSL_CTL_HW_STATUS,
				&hw_status, sizeof(hw_status));
		if (ret == SR_OK && !(hw_status & DSL_HW_STATUS_GPIF_DONE))
			sr_warn("U2Basic arm: GPIF_DONE not set (status=0x%02x).", hw_status);
		return SR_OK;
	}

	WL32(&cfg.sync, DS_CFG_START);
	WL16(&cfg.mode_header, DS_CFG_MODE);
	WL16(&cfg.mode, mode);
	WL16(&cfg.divider_header, DS_CFG_DIVIDER);
	WL32(&cfg.divider, divider);
	WL16(&cfg.count_header, DS_CFG_COUNT);
	WL32(&cfg.count, count_units);
	WL16(&cfg.trig_pos_header, DS_CFG_TRIG_POS);
	WL16(&cfg.trig_glb_header, DS_CFG_TRIG_GLB);
	WL16(&cfg.ch_en_header, DS_CFG_CH_EN);
	WL16(&cfg.trig_header, DS_CFG_TRIG);
	WL32(&cfg.end_sync, DS_CFG_END);

	/* Pass in the length of a fixed-size struct. Really. */
	len = sizeof(struct fpga_config) / 2;
	c[0] = len & 0xff;
	c[1] = (len >> 8) & 0xff;
	c[2] = (len >> 16) & 0xff;
	ret = libusb_control_transfer(usb->devhdl, LIBUSB_REQUEST_TYPE_VENDOR |
			LIBUSB_ENDPOINT_OUT, DS_CMD_SETTING, 0x0000, 0x0000,
			c, sizeof(c), USB_TIMEOUT);
	if (ret < 0) {
		sr_err("Failed to send FPGA configure command: %s.",
			libusb_error_name(ret));
		return SR_ERR;
	}

	len = sizeof(struct fpga_config);
	ret = libusb_bulk_transfer(usb->devhdl, 2 | LIBUSB_ENDPOINT_OUT,
			(unsigned char *)&cfg, len, &transferred, USB_TIMEOUT);
	if (ret < 0 || transferred != len) {
		sr_err("Failed to send FPGA configuration: %s.", libusb_error_name(ret));
		return SR_ERR;
	}

	return SR_OK;
}

SR_PRIV int dslogic_set_voltage_threshold(const struct sr_dev_inst *sdi, double threshold)
{
	int ret;
	struct dev_context *const devc = sdi->priv;
	const struct sr_usb_dev_inst *const usb = sdi->conn;
	const uint8_t value = (threshold / 5.0) * 255;
	const uint16_t cmd = value | (DS_ADDR_VTH << 8);

	if (!strcmp(devc->profile->model, "DSLogic U2Basic")) {
		ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_I2C_REG,
				DS_ADDR_VTH, value);
		if (ret != SR_OK)
			return ret;
		devc->cur_threshold = threshold;
		return SR_OK;
	}

	/* Send the control command. */
	ret = libusb_control_transfer(usb->devhdl,
			LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT,
			DS_CMD_WR_REG, 0x0000, 0x0000,
			(unsigned char *)&cmd, sizeof(cmd), 3000);
	if (ret < 0) {
		sr_err("Unable to set voltage-threshold register: %s.",
		libusb_error_name(ret));
		return SR_ERR;
	}

	devc->cur_threshold = threshold;

	return SR_OK;
}

SR_PRIV int dslogic_dev_open(struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
	libusb_device **devlist;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	struct version_info vi;
	int ret = SR_ERR, i, device_count;
	uint8_t revid;
	char connection_id[64];

	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
		       libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
		    || des.idProduct != devc->profile->pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING) ||
				(sdi->status == SR_ST_INACTIVE)) {
			/* Check device by its physical USB bus/port address. */
			if (usb_get_port_path(devlist[i], connection_id, sizeof(connection_id)) < 0)
				continue;

			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.",
			       libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl, USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
						libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		if (!strcmp(devc->profile->model, "DSLogic U2Basic")) {
			ret = command_get_fw_version_v2(usb->devhdl, &vi);
			if (ret != SR_OK)
				/* Fallback for early/legacy variants. */
				ret = command_get_fw_version(usb->devhdl, &vi);
		} else {
			ret = command_get_fw_version(usb->devhdl, &vi);
		}
		if (ret != SR_OK) {
			sr_err("Failed to get firmware version.");
			break;
		}

		ret = command_get_revid_version(sdi, &revid);
		if (ret != SR_OK) {
			sr_err("Failed to get REVID.");
			break;
		}

		/*
		 * Changes in major version mean incompatible/API changes, so
		 * bail out if we encounter an incompatible version.
		 * Different minor versions are OK, they should be compatible.
		 */
		if (!strcmp(devc->profile->model, "DSLogic U2Basic")) {
			/*
			 * Newer U2Basic devices may report 2.x firmware,
			 * while some revisions return 0.0 on the legacy
			 * version request but still accept capture commands.
			 */
			if (vi.major != 0 && vi.major != 2) {
				sr_err("Expected firmware version 2.x (or legacy 0.x) for %s, "
				       "got %d.%d.", devc->profile->model, vi.major, vi.minor);
				ret = SR_ERR;
				break;
			}
		} else if (vi.major != DSLOGIC_REQUIRED_VERSION_MAJOR) {
			sr_err("Expected firmware version %d.x, "
			       "got %d.%d.", DSLOGIC_REQUIRED_VERSION_MAJOR,
			       vi.major, vi.minor);
			ret = SR_ERR;
			break;
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), "
			"interface %d, firmware %d.%d.",
			usb->bus, usb->address, connection_id,
			USB_INTERFACE, vi.major, vi.minor);

		sr_info("Detected REVID=%d, it's a Cypress CY7C68013%s.",
			revid, (revid != 1) ? " (FX2)" : "A (FX2LP)");

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

SR_PRIV struct dev_context *dslogic_dev_new(void)
{
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_samples = 0;
	devc->capture_ratio = 0;
	devc->continuous_mode = FALSE;
	devc->clock_edge = DS_EDGE_RISING;

	return devc;
}

static void abort_acquisition(struct dev_context *devc)
{
	int i;

	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}
}

static void finish_acquisition(struct sr_dev_inst *sdi)
{
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);
	devc->transfers = NULL;
	g_free(devc->deinterleave_buffer);
	devc->deinterleave_buffer = NULL;
	devc->submitted_transfers = 0;
}

static void free_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;
	gboolean removed = FALSE;

	sdi = transfer->user_data;
	devc = sdi->priv;

	g_free(transfer->buffer);
	transfer->buffer = NULL;
	libusb_free_transfer(transfer);

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			removed = TRUE;
			break;
		}
	}

	if (removed && devc->submitted_transfers > 0)
		devc->submitted_transfers--;
	if (removed && devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer)
{
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);

}

static void deinterleave_buffer(const uint8_t *src, size_t length,
	uint16_t *dst_ptr, size_t channel_count, uint16_t channel_mask)
{
	uint16_t sample;

	for (const uint64_t *src_ptr = (uint64_t*)src;
		src_ptr < (uint64_t*)(src + length);
		src_ptr += channel_count) {
		for (int bit = 0; bit != 64; bit++) {
			const uint64_t *word_ptr = src_ptr;
			sample = 0;
			for (unsigned int channel = 0; channel != 16;
				channel++) {
				const uint16_t m = channel_mask >> channel;
				if (!m)
					break;
				if ((m & 1) && ((*word_ptr++ >> bit) & UINT64_C(1)))
					sample |= 1 << channel;
			}
			*dst_ptr++ = sample;
		}
	}
}

static void send_data(struct sr_dev_inst *sdi,
	uint16_t *data, size_t sample_count)
{
	const struct sr_datafeed_logic logic = {
		.length = sample_count * sizeof(uint16_t),
		.unitsize = sizeof(uint16_t),
		.data = data
	};

	const struct sr_datafeed_packet packet = {
		.type = SR_DF_LOGIC,
		.payload = &logic
	};

	sr_session_send(sdi, &packet);
}

static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
	struct sr_dev_inst *const sdi = transfer->user_data;
	struct dev_context *const devc = sdi->priv;
	const size_t channel_count = enabled_channel_count(sdi);
	const uint16_t channel_mask = enabled_channel_mask(sdi);
	const unsigned int cur_sample_count = DSLOGIC_ATOMIC_SAMPLES *
		transfer->actual_length /
		(DSLOGIC_ATOMIC_BYTES * channel_count);

	gboolean packet_has_error = FALSE;
	unsigned int num_samples;
	int trigger_offset;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {
		free_transfer(transfer);
		return;
	}

	if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT &&
			transfer->actual_length == 0) {
		sr_spew("receive_transfer(): timeout with no data.");
	} else {
		sr_dbg("receive_transfer(): status %s received %d bytes.",
			libusb_error_name(transfer->status), transfer->actual_length);
	}

	/* Save incoming transfer before reusing the transfer struct. */

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		abort_acquisition(devc);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			/*
			 * The FX2 gave up. End the acquisition, the frontend
			 * will work out that the samplecount is short.
			 */
			abort_acquisition(devc);
			free_transfer(transfer);
		} else {
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

	if (!devc->limit_samples || devc->sent_samples < devc->limit_samples) {
		if (devc->limit_samples && devc->sent_samples + cur_sample_count > devc->limit_samples)
			num_samples = devc->limit_samples - devc->sent_samples;
		else
			num_samples = cur_sample_count;

		/**
		 * The DSLogic emits sample data as sequences of 64-bit sample words
		 * in a round-robin i.e. 64-bits from channel 0, 64-bits from channel 1
		 * etc. for each of the enabled channels, then looping back to the
		 * channel.
		 *
		 * Because sigrok's internal representation is bit-interleaved channels
		 * we must recast the data.
		 *
		 * Hopefully in future it will be possible to pass the data on as-is.
		 */
		if (transfer->actual_length % (DSLOGIC_ATOMIC_BYTES * channel_count) != 0)
			sr_err("Invalid transfer length!");
		deinterleave_buffer(transfer->buffer, transfer->actual_length,
			devc->deinterleave_buffer, channel_count, channel_mask);

		/* Send the incoming transfer to the session bus. */
		if (devc->trigger_pos > devc->sent_samples
			&& devc->trigger_pos <= devc->sent_samples + num_samples) {
			/* DSLogic trigger in this block. Send trigger position. */
			trigger_offset = devc->trigger_pos - devc->sent_samples;
			/* Pre-trigger samples. */
			send_data(sdi, devc->deinterleave_buffer, trigger_offset);
			devc->sent_samples += trigger_offset;
			/* Trigger position. */
			devc->trigger_pos = 0;
			std_session_send_df_trigger(sdi);
			/* Post trigger samples. */
			num_samples -= trigger_offset;
			send_data(sdi, devc->deinterleave_buffer
				+ trigger_offset, num_samples);
			devc->sent_samples += num_samples;
		} else {
			send_data(sdi, devc->deinterleave_buffer, num_samples);
			devc->sent_samples += num_samples;
		}
	}

	if (devc->limit_samples && devc->sent_samples >= devc->limit_samples) {
		abort_acquisition(devc);
		free_transfer(transfer);
	} else
		resubmit_transfer(transfer);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;

	(void)fd;
	(void)revents;

	drvc = (struct drv_context *)cb_data;

	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

static size_t to_bytes_per_ms(const struct sr_dev_inst *sdi)
{
	const struct dev_context *const devc = sdi->priv;
	const size_t ch_count = enabled_channel_count(sdi);

	if (devc->continuous_mode)
		return (devc->cur_samplerate * ch_count) / (1000 * 8);


	/* If we're in buffered mode, the transfer rate is not so important,
	 * but we expect to get at least 10% of the high-speed USB bandwidth.
	 */
	return 35000000 / (1000 * 10);
}

static size_t get_buffer_size(const struct sr_dev_inst *sdi)
{
	/*
	 * The buffer should be large enough to hold 10ms of data and
	 * a multiple of the size of a data atom.
	 */
	const size_t block_size = enabled_channel_count(sdi) * 512;
	const size_t s = 10 * to_bytes_per_ms(sdi);
	if (!block_size)
		return s;
	return ((s + block_size - 1) / block_size) * block_size;
}

static unsigned int get_number_of_transfers(const struct sr_dev_inst *sdi)
{
	/* Total buffer size should be able to hold about 100ms of data. */
	const unsigned int s = get_buffer_size(sdi);
	const unsigned int n = (100 * to_bytes_per_ms(sdi) + s - 1) / s;
	return (n > NUM_SIMUL_TRANSFERS) ? NUM_SIMUL_TRANSFERS : n;
}

static unsigned int get_timeout(const struct sr_dev_inst *sdi)
{
	const size_t total_size = get_buffer_size(sdi) *
		get_number_of_transfers(sdi);
	const unsigned int timeout = total_size / to_bytes_per_ms(sdi);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int start_transfers(const struct sr_dev_inst *sdi)
{
	const size_t channel_count = enabled_channel_count(sdi);
	const size_t size = get_buffer_size(sdi);
	const unsigned int num_transfers = get_number_of_transfers(sdi);
	const unsigned int timeout = get_timeout(sdi);

	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_transfer *transfer;
	unsigned int i;
	int ret;
	unsigned char *buf;

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;
	devc->submitted_transfers = 0;

	g_free(devc->transfers);
	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	devc->deinterleave_buffer = g_try_malloc(DSLOGIC_ATOMIC_SAMPLES *
		(size / (channel_count * DSLOGIC_ATOMIC_BYTES)) * sizeof(uint16_t));
	if (!devc->deinterleave_buffer) {
		sr_err("Deinterleave buffer malloc failed.");
		g_free(devc->deinterleave_buffer);
		return SR_ERR_MALLOC;
	}

	devc->num_transfers = num_transfers;
	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
				6 | LIBUSB_ENDPOINT_IN, buf, size,
				receive_transfer, (void *)sdi, timeout);
		sr_info("submitting transfer: %d", i);
		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			sr_err("Failed to submit transfer: %s.",
			       libusb_error_name(ret));
			libusb_free_transfer(transfer);
			g_free(buf);
			abort_acquisition(devc);
			return SR_ERR;
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	std_session_send_df_header(sdi);

	return SR_OK;
}

static void LIBUSB_CALL trigger_receive(struct libusb_transfer *transfer)
{
	const struct sr_dev_inst *sdi;
	struct dslogic_trigger_pos *tpos;
	struct dev_context *devc;

	sdi = transfer->user_data;
	devc = sdi->priv;
	if (transfer->status == LIBUSB_TRANSFER_CANCELLED) {
		sr_dbg("Trigger transfer canceled.");
		/* Terminate session. */
		std_session_send_df_end(sdi);
		usb_source_remove(sdi->session, devc->ctx);
		devc->num_transfers = 0;
		g_free(devc->transfers);
	} else if (transfer->status == LIBUSB_TRANSFER_COMPLETED
			&& transfer->actual_length == sizeof(struct dslogic_trigger_pos)) {
		tpos = (struct dslogic_trigger_pos *)transfer->buffer;
		sr_info("tpos real_pos %d ram_saddr %d cnt_h %d cnt_l %d", tpos->real_pos,
			tpos->ram_saddr, tpos->remain_cnt_h, tpos->remain_cnt_l);
		devc->trigger_pos = tpos->real_pos;
		g_free(tpos);
		start_transfers(sdi);
	}
	libusb_free_transfer(transfer);
}

SR_PRIV int dslogic_acquisition_start(const struct sr_dev_inst *sdi)
{
	const unsigned int timeout = get_timeout(sdi);

	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct dslogic_trigger_pos *tpos;
	struct libusb_transfer *transfer;
	const char *skip_arm_env;
	gboolean skip_u2basic_arm = FALSE;
	int ret;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	skip_arm_env = g_getenv("SIGROK_U2BASIC_SKIP_ARM");
	if (skip_arm_env && strcmp(skip_arm_env, "0") != 0)
		skip_u2basic_arm = TRUE;

	usb_source_add(sdi->session, devc->ctx, timeout, receive_data, drvc);

	if (!strcmp(devc->profile->model, "DSLogic U2Basic")) {
		/* U2Basic v2 firmware uses DSL CTL transport for run control. */
		ret = command_ctl_wr_reg_byte(usb->devhdl, DSL_CTL_I2C_REG,
			DS_ADDR_EEWP, 0);
		if (ret != SR_OK)
			return ret;
		if ((ret = command_stop_acquisition_v2(sdi)) != SR_OK)
			return ret;
		/*
		 * Ensure normal capture mode is explicitly armed for each run.
		 * Without this, some devices can stay in stale/test-like output.
		 */
		if (!skip_u2basic_arm) {
			if ((ret = fpga_configure(sdi)) != SR_OK)
				return ret;
		} else {
			sr_warn("SIGROK_U2BASIC_SKIP_ARM=1: skipping FPGA arm/config.");
		}
		/*
		 * Match DSView order: queue IN transfers before asserting START.
		 * This avoids losing initial sample alignment.
		 */
		if ((ret = start_transfers(sdi)) != SR_OK)
			return ret;
		if ((ret = command_start_acquisition_v2(sdi)) != SR_OK) {
			abort_acquisition(devc);
			return ret;
		}
		return SR_OK;
	}

	if ((ret = command_stop_acquisition(sdi)) != SR_OK)
		return ret;

	if ((ret = fpga_configure(sdi)) != SR_OK)
		return ret;

	if ((ret = command_start_acquisition(sdi)) != SR_OK)
		return ret;

	sr_dbg("Getting trigger.");
	tpos = g_malloc(sizeof(struct dslogic_trigger_pos));
	transfer = libusb_alloc_transfer(0);
	libusb_fill_bulk_transfer(transfer, usb->devhdl, 6 | LIBUSB_ENDPOINT_IN,
			(unsigned char *)tpos, sizeof(struct dslogic_trigger_pos),
			trigger_receive, (void *)sdi, 0);
	if ((ret = libusb_submit_transfer(transfer)) < 0) {
		sr_err("Failed to request trigger: %s.", libusb_error_name(ret));
		libusb_free_transfer(transfer);
		g_free(tpos);
		return SR_ERR;
	}

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers));
	if (!devc->transfers) {
		sr_err("USB trigger_pos transfer malloc failed.");
		return SR_ERR_MALLOC;
	}
	devc->num_transfers = 1;
	devc->submitted_transfers++;
	devc->transfers[0] = transfer;

	return ret;
}

SR_PRIV int dslogic_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (devc && devc->profile && !strcmp(devc->profile->model, "DSLogic U2Basic"))
		command_stop_acquisition_v2(sdi);
	else
		command_stop_acquisition(sdi);
	abort_acquisition(sdi->priv);
	return SR_OK;
}
