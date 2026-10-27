// SPDX-License-Identifier: GPL-2.0-or-later
/* HPM Prober RISC-V debug and raw JTAG tunnels over USB. */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <limits.h>
#include <string.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include <helper/binarybuffer.h>
#include <target/riscv/debug_defines.h>
#include "bitbang.h"
#include "libusb_helper.h"

enum hpm_prober_feature {
	HPM_PROBER_FEATURE_SBA,
	HPM_PROBER_FEATURE_PROGBUF,
	HPM_PROBER_FEATURE_ABSTRACT,
	HPM_PROBER_FEATURE_COUNT,
};

#define HPM_MAX_PAYLOAD 8192
#define HPM_EP_OUT 0x02
#define HPM_EP_IN 0x81
#define HPM_RAW_LEGACY_BITS 2048
#define HPM_RAW_BITS 3584
#define HPM_RAW_SAMPLES HPM_RAW_LEGACY_BITS

static struct libusb_device_handle *device;
static unsigned int packet_size;
static unsigned int max_payload = HPM_MAX_PAYLOAD;
static unsigned int raw_max_bits = HPM_RAW_LEGACY_BITS;
static int failure;
static int actual_khz = 1000;
static uint32_t configured_hz;
static uint8_t transport_type = 1;
static bool configured;
static bool raw_mode;
static bool raw_sequence_supported;
static bool dmi_batch_enabled;
static bool feature_enabled[HPM_PROBER_FEATURE_COUNT] = {true, true, true};
static const char *const feature_names[HPM_PROBER_FEATURE_COUNT] = {
	"sba", "progbuf", "abstract"
};
static uint32_t firmware_capabilities[2];
static bool capabilities_known[2];

static int hpm_prober_exchange(uint8_t tunnel, const uint8_t *payload, unsigned int size,
		uint8_t *reply, unsigned int *reply_size)
{
	uint8_t frame[HPM_MAX_PAYLOAD + 4];
	int transferred = 0;
	if (size > max_payload || !size || failure || !device) {
		return ERROR_FAIL;
	}
	frame[0] = tunnel;
	frame[1] = 0;
	frame[2] = size & 0xff;
	frame[3] = size >> 8;
	memcpy(frame + 4, payload, size);
	int length = size + 4;
	int ret = jtag_libusb_bulk_write(device, HPM_EP_OUT, (char *)frame, length,
		LIBUSB_TIMEOUT_MS, &transferred);
	if (ret != ERROR_OK || transferred != length) {
		goto io_error;
	}
	/* The firmware terminates each request on a short packet, including ZLP. */
	if (packet_size && length % packet_size == 0) {
		ret = jtag_libusb_bulk_write(device, HPM_EP_OUT, (char *)frame, 0,
			LIBUSB_TIMEOUT_MS, &transferred);
		if (ret != ERROR_OK) {
			goto io_error;
		}
	}
	ret = jtag_libusb_bulk_read(device, HPM_EP_IN, (char *)frame, sizeof(frame),
		LIBUSB_TIMEOUT_MS, &transferred);
	if (ret != ERROR_OK || transferred < 6) {
		goto io_error;
	}
	unsigned int n = frame[2] | (frame[3] << 8);
	if (frame[0] != tunnel || (frame[1] & ~0x04) != 1 || n + 4 != (unsigned int)transferred ||
			n < 2 || frame[4] != payload[0] || n > *reply_size) {
		LOG_ERROR("hpm_probex: malformed response; reconnect the probe");
		failure = 1;
		return ERROR_FAIL;
	}
	if (frame[5] != 0) {
		LOG_ERROR("hpm_probex: tunnel 0x%02x opcode 0x%02x returned status 0x%02x",
			tunnel, payload[0], frame[5]);
		return ERROR_FAIL;
	}
	/* Missing extension words in older firmware describe no capabilities. */
	if (tunnel == 1 && payload[0] == 1 && n >= 27) {
		firmware_capabilities[0] = frame[13] | ((uint32_t)frame[14] << 8) |
			((uint32_t)frame[15] << 16) | ((uint32_t)frame[16] << 24);
		capabilities_known[0] = true;
		capabilities_known[1] = true;
		firmware_capabilities[1] = 0;
		if (n >= 31) {
			firmware_capabilities[1] = frame[31] | ((uint32_t)frame[32] << 8) |
				((uint32_t)frame[33] << 16) | ((uint32_t)frame[34] << 24);
		}
	}
	memcpy(reply, frame + 4, n);
	*reply_size = n;
	return ERROR_OK;
io_error:
	LOG_ERROR("hpm_probex: USB transfer failed or was short; reconnect the probe");
	failure = 1;
	return ERROR_FAIL;
}

static int hpm_read_idcode(uint32_t *id)
{
	uint8_t req = 1, rep[64];
	unsigned int n = sizeof(rep);
	if (hpm_prober_exchange(1, &req, 1, rep, &n) != ERROR_OK || n < 27) {
		return ERROR_FAIL;
	}
	*id = buf_get_u32(rep + 13, 0, 32);
	return ERROR_OK;
}

static int hpm_dtmcs(uint32_t out, uint32_t *in)
{
	uint8_t req = out ? 2 : 1, rep[64];
	unsigned int n = sizeof(rep), offset = out ? 2 : 17;
	if (out && out != DTM_DTMCS_DMIRESET) {
		return ERROR_NOT_IMPLEMENTED;
	}
	if (hpm_prober_exchange(1, &req, 1, rep, &n) != ERROR_OK || n < offset + 4) {
		return ERROR_FAIL;
	}
	if (in) {
		*in = buf_get_u32(rep + offset, 0, 32);
	}
	return ERROR_OK;
}

static int hpm_dmi(bool write, uint32_t address, uint32_t *data)
{
	uint8_t req[9] = {write ? 0x04 : 0x03}, rep[8];
	unsigned int n = sizeof(rep);
	buf_set_u32(req + 1, 0, 32, address);
	buf_set_u32(req + 5, 0, 32, *data);
	if (hpm_prober_exchange(1, req, write ? 9 : 5, rep, &n) != ERROR_OK || (!write && n < 6)) {
		return ERROR_FAIL;
	}
	if (!write) {
		*data = buf_get_u32(rep + 2, 0, 32);
	}
	return ERROR_OK;
}

static int hpm_select_hart(uint32_t hartid)
{
	uint8_t req[5] = {0x06}, rep[16];
	unsigned int n = sizeof(rep);
	buf_set_u32(req + 1, 0, 32, hartid);
	if (hpm_prober_exchange(1, req, sizeof(req), rep, &n) != ERROR_OK || n != 11 ||
			buf_get_u32(rep + 2, 0, 32) != hartid) {
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int hpm_query_capabilities(uint32_t *supported, uint32_t *enabled)
{
	uint8_t req = 1, rep[64];
	unsigned int n = sizeof(rep);
	*supported = 0;
	*enabled = 0;
	if (feature_enabled[HPM_PROBER_FEATURE_SBA]) {
		*enabled |= ADAPTER_CAP_RISCV_MEMORY_SBA;
	}
	if (feature_enabled[HPM_PROBER_FEATURE_PROGBUF]) {
		*enabled |= ADAPTER_CAP_RISCV_MEMORY_PROGBUF;
	}
	if (feature_enabled[HPM_PROBER_FEATURE_ABSTRACT]) {
		*enabled |= ADAPTER_CAP_RISCV_MEMORY_ABSTRACT;
	}
	if (hpm_prober_exchange(1, &req, 1, rep, &n) != ERROR_OK) {
		return ERROR_FAIL;
	}
	if (n >= 13) {
		uint32_t word = buf_get_u32(rep + 9, 0, 32);
		if (word & (1U << 30)) {
			*supported |= ADAPTER_CAP_RISCV_MEMORY_SBA;
		}
		if (word & (1U << 19)) {
			*supported |= ADAPTER_CAP_RISCV_MEMORY_PROGBUF;
		}
	}
	if (n >= 31 &&
			(buf_get_u32(rep + 27, 0, 32) & (1U << 8))) {
		*supported |= ADAPTER_CAP_RISCV_MEMORY_ABSTRACT;
	}
	return ERROR_OK;
}

static const struct adapter_capability_ops hpm_capabilities = {
	.query = hpm_query_capabilities,
};

static int hpm_query_raw_capabilities(uint32_t *supported, uint32_t *enabled)
{
	*supported = ADAPTER_CAP_RISCV_DMI_READ_BATCH;
	*enabled = dmi_batch_enabled ? ADAPTER_CAP_RISCV_DMI_READ_BATCH : 0;
	return ERROR_OK;
}

static const struct adapter_capability_ops hpm_raw_capabilities = {
	.query = hpm_query_raw_capabilities,
};

static int hpm_memory(enum riscv_direct_memory_method method, uint64_t address,
		uint32_t size, uint32_t count, uint8_t *read_buffer, const uint8_t *write_buffer)
{
	uint8_t req[HPM_MAX_PAYLOAD], rep[HPM_MAX_PAYLOAD];
	bool read = read_buffer != NULL;
	if (!size || max_payload <= 12 || (max_payload - 12) / size == 0) {
		return ERROR_FAIL;
	}
	while (count) {
		uint32_t n = MIN(count, MIN(8176U, max_payload - 12) / size);
		unsigned int length = n * size, reply_size = sizeof(rep);
		memset(req, 0, 12);
		switch (method) {
		case RISCV_DIRECT_SBA:
			req[0] = read ? 0x27 : 0x28;
			break;
		case RISCV_DIRECT_PROGBUF:
			req[0] = read ? 0x1e : 0x1f;
			break;
		case RISCV_DIRECT_ABSTRACT:
			req[0] = read ? 0x2e : 0x2f;
			break;
		default:
			return ERROR_FAIL;
		}
		req[1] = size;
		buf_set_u32(req + 2, 0, 16, n);
		buf_set_u64(req + 4, 0, 64, address);
		if (!read) {
			memcpy(req + 12, write_buffer, length);
		}
		if (hpm_prober_exchange(1, req, read ? 12 : 12 + length, rep, &reply_size) != ERROR_OK ||
				(read ? reply_size != length + 2 : reply_size != 2)) {
			return ERROR_FAIL;
		}
		if (read) {
			memcpy(read_buffer, rep + 2, length);
			read_buffer += length;
		} else {
			write_buffer += length;
		}
		address += length;
		count -= n;
		keep_alive();
	}
	return ERROR_OK;
}

static const struct riscv_debug_ops hpm_riscv_debug_ops = {
	.dtmcs = hpm_dtmcs, .dmi = hpm_dmi, .select_hart = hpm_select_hart,
	.memory = hpm_memory,
};

static uint8_t raw_cycles[HPM_RAW_BITS / 4];
static uint8_t raw_capture[HPM_RAW_BITS];
static uint8_t raw_samples[HPM_RAW_SAMPLES];
static unsigned int raw_bits, raw_samples_head, raw_samples_count;
static bool raw_pending_sample, raw_has_capture, raw_failed;

static int hpm_raw_flush_packed(void)
{
	uint8_t req[4 + sizeof(raw_cycles)] = {0x02, 0};
	uint8_t rep[4 + HPM_RAW_BITS / 8];
	unsigned int n = sizeof(rep);
	req[2] = raw_bits;
	req[3] = raw_bits >> 8;
	memcpy(req + 4, raw_cycles, (raw_bits + 3) / 4);
	if (hpm_prober_exchange(2, req, 4 + (raw_bits + 3) / 4, rep, &n) != ERROR_OK ||
			n != 4 + (raw_bits + 7) / 8 || buf_get_u32(rep + 2, 0, 16) != raw_bits) {
		goto fail;
	}
	for (unsigned int i = 0; i < raw_bits; i++) {
		if (raw_capture[i]) {
			if (raw_samples_count == HPM_RAW_SAMPLES) {
				goto fail;
			}
			raw_samples[raw_samples_count++] = (rep[4 + i / 8] >> (i % 8)) & 1;
		}
	}
	return ERROR_OK;
fail:
	return ERROR_FAIL;
}

static int hpm_raw_flush_sequence(void)
{
	unsigned int bit = 0;
	while (bit < raw_bits) {
		uint8_t req[2 + HPM_RAW_BITS * 2] = {0x07, 0};
		uint8_t rep[2 + HPM_RAW_BITS];
		unsigned int starts[255];
		uint8_t lengths[255];
		unsigned int req_size = 2, rep_size = 2;
		unsigned int start_count = 0;
		while (bit < raw_bits && start_count < 255) {
			unsigned int start = bit;
			bool tms = (raw_cycles[bit / 4] >> ((bit % 4) * 2 + 1)) & 1;
			bool capture = raw_capture[bit];
			unsigned int count = 1;
			while (count < 64 && start + count < raw_bits) {
				unsigned int next = start + count;
				bool next_tms = (raw_cycles[next / 4] >> ((next % 4) * 2 + 1)) & 1;
				if (next_tms != tms || (bool)raw_capture[next] != capture) {
					break;
				}
				count++;
			}
			unsigned int bytes = (count + 7) / 8;
			if (req_size + 1 + bytes > max_payload ||
					rep_size + (capture ? bytes : 0) > max_payload) {
				break;
			}
			starts[start_count] = start;
			lengths[start_count++] = count;
			req[req_size++] = (count == 64 ? 0 : count) |
					(tms ? 0x40 : 0) | (capture ? 0x80 : 0);
			memset(req + req_size, 0, bytes);
			for (unsigned int i = 0; i < count; i++) {
				unsigned int source = start + i;
				unsigned int tdi = (raw_cycles[source / 4] >>
						((source % 4) * 2)) & 1;
				req[req_size + i / 8] |= tdi << (i % 8);
			}
			req_size += bytes;
			if (capture) {
				rep_size += bytes;
			}
			bit += count;
		}
		if (!start_count) {
			return ERROR_FAIL;
		}
		req[1] = start_count;
		unsigned int n = sizeof(rep);
		if (hpm_prober_exchange(2, req, req_size, rep, &n) != ERROR_OK ||
				n != rep_size) {
			return ERROR_FAIL;
		}
		unsigned int response = 2;
		for (unsigned int sequence = 0; sequence < start_count; sequence++) {
			unsigned int start = starts[sequence];
			if (!raw_capture[start]) {
				continue;
			}
			unsigned int count = lengths[sequence];
			if (raw_samples_count + count > HPM_RAW_SAMPLES) {
				return ERROR_FAIL;
			}
			for (unsigned int i = 0; i < count; i++) {
				raw_samples[raw_samples_count++] =
						(rep[response + i / 8] >> (i % 8)) & 1;
			}
			response += (count + 7) / 8;
		}
	}
	return ERROR_OK;
}

static int hpm_raw_flush(void)
{
	if (raw_failed) {
		return ERROR_FAIL;
	}
	if (!raw_bits) {
		return ERROR_OK;
	}
	int result = raw_sequence_supported ? hpm_raw_flush_sequence() : hpm_raw_flush_packed();
	if (result != ERROR_OK) {
		raw_failed = true;
	} else {
		raw_bits = 0;
		raw_has_capture = false;
	}
	return result;
}

static int hpm_raw_write(int tck, int tms, int tdi)
{
	if (raw_failed) {
		return ERROR_FAIL;
	}
	if (!tck) {
		return ERROR_OK;
	}
	/* Keep common write batches below the 255-record sequence limit. */
	unsigned int bits = raw_sequence_supported && !raw_has_capture ?
		HPM_RAW_BITS : HPM_RAW_LEGACY_BITS;
	unsigned int limit = MIN(bits, MIN(raw_max_bits, (max_payload - 4) * 4));
	if (raw_bits >= limit && hpm_raw_flush() != ERROR_OK) {
		return ERROR_FAIL;
	}
	unsigned int bit = raw_bits++;
	unsigned int shift = (bit % 4) * 2;
	if (bit % 4 == 0) {
		raw_cycles[bit / 4] = 0;
	}
	raw_cycles[bit / 4] |= ((tdi & 1) | ((tms & 1) << 1)) << shift;
	raw_capture[bit] = raw_pending_sample;
	raw_has_capture |= raw_pending_sample;
	raw_pending_sample = false;
	return ERROR_OK;
}

static int hpm_raw_sample(void)
{
	if (raw_failed) {
		return ERROR_FAIL;
	}
	raw_pending_sample = true;
	return ERROR_OK;
}

static enum bb_value hpm_raw_read_sample(void)
{
	if (hpm_raw_flush() != ERROR_OK || raw_samples_head == raw_samples_count) {
		return BB_ERROR;
	}
	enum bb_value value = raw_samples[raw_samples_head++] ? BB_HIGH : BB_LOW;
	if (raw_samples_head == raw_samples_count) {
		raw_samples_head = 0;
		raw_samples_count = 0;
	}
	return value;
}

static int hpm_raw_sleep(unsigned int microseconds)
{
	if (hpm_raw_flush() != ERROR_OK) {
		return ERROR_FAIL;
	}
	jtag_sleep(microseconds);
	return ERROR_OK;
}

static const struct bitbang_interface hpm_raw_bitbang = {
	.buf_size = HPM_RAW_SAMPLES,
	.defer_scan_readback = true,
	.sample = hpm_raw_sample,
	.read_sample = hpm_raw_read_sample,
	.write = hpm_raw_write,
	.sleep = hpm_raw_sleep,
	.flush = hpm_raw_flush,
};

static int hpm_tap_reset(void)
{
	/* Reacquiring either firmware transport performs a physical TAP reset. */
	if (!configured || !configured_hz) {
		return ERROR_FAIL;
	}
	uint32_t hz = configured_hz;
	uint8_t req[] = { 0x21, transport_type, hz, hz >> 8, hz >> 16, hz >> 24 };
	uint8_t rep[11];
	unsigned int n = sizeof(rep);
	if (hpm_prober_exchange(0, req, sizeof(req), rep, &n) != ERROR_OK ||
			n != sizeof(rep) || rep[2] != transport_type ||
			buf_get_u32(rep + 7, 0, 32) != hz) {
		return ERROR_FAIL;
	}
	memset(capabilities_known, 0, sizeof(capabilities_known));
	return ERROR_OK;
}

static int hpm_execute_queue(struct jtag_command *queue)
{
	if (raw_mode) {
		for (struct jtag_command *cmd = queue; cmd; cmd = cmd->next) {
			if (cmd->type == JTAG_RESET) {
				if (cmd->cmd.reset->trst > 0 || cmd->cmd.reset->srst > 0) {
					LOG_ERROR("hpm_probex: TRST/SRST pin control is unsupported");
					return ERROR_JTAG_QUEUE_FAILED;
				}
			}
		}
		for (struct jtag_command *cmd = queue; cmd;) {
			if (cmd->type == JTAG_RESET) {
				cmd = cmd->next;
				continue;
			}
			if (transport_type == 2 && cmd->type == JTAG_TLR_RESET) {
				if (hpm_raw_flush() != ERROR_OK || hpm_tap_reset() != ERROR_OK) {
					return ERROR_JTAG_QUEUE_FAILED;
				}
				/* Re-entry leaves Idle. Execute the queued TLR below as well,
				 * so physical clocks reach its requested end state. */
				tap_set_state(TAP_IDLE);
				tap_set_end_state(TAP_IDLE);
			}
			struct jtag_command *last = cmd;
			while (last->next && last->next->type != JTAG_RESET &&
					!(transport_type == 2 && last->next->type == JTAG_TLR_RESET)) {
				last = last->next;
			}
			struct jtag_command *next = last->next;
			last->next = NULL;
			int result = bitbang_execute_queue(cmd);
			last->next = next;
			if (result != ERROR_OK || raw_failed) {
				return ERROR_JTAG_QUEUE_FAILED;
			}
			cmd = next;
		}
		return hpm_raw_flush() == ERROR_OK ? ERROR_OK : ERROR_JTAG_QUEUE_FAILED;
	}
	for (struct jtag_command *cmd = queue; cmd; cmd = cmd->next) {
		if (cmd->type == JTAG_TLR_RESET) {
			if (hpm_tap_reset() != ERROR_OK) {
				return ERROR_JTAG_QUEUE_FAILED;
			}
			continue;
		}
		if (cmd->type == JTAG_RESET && cmd->cmd.reset->trst <= 0 &&
				cmd->cmd.reset->srst <= 0) {
			continue;
		}
		if (cmd->type == JTAG_SLEEP) {
			alive_sleep((cmd->cmd.sleep->us + 999) / 1000);
			continue;
		}
		LOG_ERROR("hpm_probex: raw JTAG commands and TRST/SRST pin control are unsupported");
		return ERROR_JTAG_QUEUE_FAILED;
	}
	return ERROR_OK;
}
static int hpm_speed(int khz)
{
	if (khz <= 0 || khz > INT_MAX / 1000) {
		return ERROR_FAIL;
	}
	if (configured) {
		LOG_ERROR("hpm_probex: runtime speed changes are unsupported; restart OpenOCD");
		return ERROR_FAIL;
	}
	uint32_t hz = (uint32_t)khz * 1000;
	uint8_t req[] = { 0x21, transport_type, hz, hz >> 8, hz >> 16, hz >> 24 };
	uint8_t rep[11];
	unsigned int n = sizeof(rep);
	if (hpm_prober_exchange(0, req, sizeof(req), rep, &n) != ERROR_OK ||
			n != sizeof(rep) || rep[2] != transport_type) {
		return ERROR_FAIL;
	}
	uint32_t got = rep[7] | ((uint32_t)rep[8] << 8) | ((uint32_t)rep[9] << 16) | ((uint32_t)rep[10] << 24);
	actual_khz = got / 1000;
	configured_hz = got;
	configured = true;
	LOG_INFO("hpm_probex: configured %s clock %u Hz", transport_type == 1 ? "JTAG" : "cJTAG", got);
	return ERROR_OK;
}
static int hpm_khz(int khz, int *speed)
{
	if (khz <= 0 || khz > INT_MAX / 1000) {
		return ERROR_FAIL;
	}
	*speed = khz;
	return ERROR_OK;
}
static int hpm_speed_div(int speed, int *khz)
{
	(void)speed;
	*khz = actual_khz;
	return ERROR_OK;
}
static int hpm_init(void)
{
	bitbang_interface = raw_mode ? &hpm_raw_bitbang : NULL;
	raw_bits = 0;
	raw_samples_head = 0;
	raw_samples_count = 0;
	raw_pending_sample = false;
	raw_has_capture = false;
	raw_failed = false;
	raw_sequence_supported = false;
	raw_max_bits = HPM_RAW_LEGACY_BITS;
	static const uint16_t vids[] = { 0x34b7, 0x34b7, 0 };
	static const uint16_t pids[] = { 0x8003, 0xe001, 0 };
	failure = 0;
	configured = false;
	configured_hz = 0;
	memset(capabilities_known, 0, sizeof(capabilities_known));
	max_payload = HPM_MAX_PAYLOAD;

	if (jtag_libusb_open(vids, pids, NULL, &device, NULL) != ERROR_OK) {
		LOG_ERROR("hpm_probex: cannot open probe");
		return ERROR_FAIL;
	}
	libusb_set_auto_detach_kernel_driver(device, 1);
	if (libusb_claim_interface(device, 0) != LIBUSB_SUCCESS) {
		LOG_ERROR("hpm_probex: cannot claim USB interface 0");
		goto fail;
	}
	int ps = libusb_get_max_packet_size(libusb_get_device(device), HPM_EP_OUT);
	if (ps <= 0) {
		LOG_ERROR("hpm_probex: cannot determine endpoint packet size");
		goto release;
	}
	packet_size = ps;
	/* Discard replies left by a previous host before establishing the session. */
	uint8_t discard[HPM_MAX_PAYLOAD + 4];
	int transferred;
	while (libusb_bulk_transfer(device, HPM_EP_IN, discard, sizeof(discard),
			&transferred, 10) == 0 && transferred > 0) {
		;
	}
	uint8_t hello[] = { 1, 1, 1, 0, 0, 0x20 };
	uint8_t rep[32];
	unsigned int n = sizeof(rep);
	if (hpm_prober_exchange(0, hello, sizeof(hello), rep, &n) != ERROR_OK || n != 11 || rep[2] != 1 ||
			(rep[5] | (rep[6] << 8)) < 64) {
		LOG_ERROR("hpm_probex: incompatible protocol or payload limit");
		goto release;
	}
	max_payload = rep[5] | (rep[6] << 8);
	if (max_payload > HPM_MAX_PAYLOAD) {
		max_payload = HPM_MAX_PAYLOAD;
	}
	if (raw_mode) {
		uint8_t req = 1, info[16];
		bool raw_cjtag_supported = false;
		n = sizeof(info);
		if (hpm_prober_exchange(2, &req, 1, info, &n) == ERROR_OK &&
				n >= 10 && info[2] == 1) {
			unsigned int max_cycles = buf_get_u32(info + 4, 0, 16);
			uint32_t features = buf_get_u32(info + 6, 0, 32);
			raw_cjtag_supported = features & (1U << 3);
			if (max_cycles) {
				raw_max_bits = MIN(max_cycles, HPM_RAW_BITS);
			}
			raw_sequence_supported = features & (1U << 2);
		}
		if (transport_type == 2 && !raw_cjtag_supported) {
			LOG_ERROR("hpm_probex: firmware does not support raw cJTAG cycles");
			goto release;
		}
		if (failure) {
			goto release;
		}
		LOG_INFO("hpm_probex: raw JTAG %s", raw_sequence_supported ?
			"sequences" : "packed cycles");
	}
	return ERROR_OK;
release:
	libusb_release_interface(device, 0);
fail:
	jtag_libusb_close(device);
	device = NULL;
	return ERROR_FAIL;
}
static int hpm_quit(void)
{
	if (device) {
		if (!failure) {
			uint8_t req[] = { 0x23 }, rep[16];
			unsigned int n = sizeof(rep);
			hpm_prober_exchange(0, req, sizeof(req), rep, &n);
		}
		libusb_release_interface(device, 0);
		jtag_libusb_close(device);
		device = NULL;
	}
	memset(capabilities_known, 0, sizeof(capabilities_known));
	return ERROR_OK;
}
/* The adapter's DMI bridge is required for operation; these switches control
 * only the optional probe-side memory offloads. */
COMMAND_HANDLER(hpm_feature_command)
{
	if (CMD_ARGC > 2) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	int first = 0, last = HPM_PROBER_FEATURE_COUNT;
	if (CMD_ARGC > 0) {
		for (first = 0; first < HPM_PROBER_FEATURE_COUNT; first++) {
			if (!strcmp(CMD_ARGV[0], feature_names[first])) {
				break;
			}
		}
		if (first == HPM_PROBER_FEATURE_COUNT) {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		last = first + 1;
	}
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_ON_OFF(CMD_ARGV[1], feature_enabled[first]);
	}

	for (int i = first; i < last; i++) {
		unsigned int word = i == HPM_PROBER_FEATURE_ABSTRACT ? 1 : 0;
		unsigned int bit = i == HPM_PROBER_FEATURE_SBA ? 30 :
			(i == HPM_PROBER_FEATURE_PROGBUF ? 19 : 8);
		const char *firmware = !capabilities_known[word] ? "unknown" :
			(firmware_capabilities[word] & (1UL << bit)) ? "supported" : "unsupported";
		command_print(CMD, "%s: %s (firmware: %s)", feature_names[i],
			feature_enabled[i] ? "on" : "off", firmware);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(hpm_transport_command)
{
	if (CMD_ARGC != 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (!strcmp(CMD_ARGV[0], "jtag")) {
		transport_type = 1;
	} else if (!strcmp(CMD_ARGV[0], "cjtag")) {
		transport_type = 2;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}
COMMAND_HANDLER(hpm_mode_command)
{
	if (CMD_ARGC != 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (!strcmp(CMD_ARGV[0], "raw")) {
		raw_mode = true;
	} else if (!strcmp(CMD_ARGV[0], "direct")) {
		raw_mode = false;
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	hpm_prober_adapter_driver.riscv_debug_ops = raw_mode ? NULL : &hpm_riscv_debug_ops;
	hpm_prober_adapter_driver.capabilities = raw_mode ?
		&hpm_raw_capabilities : &hpm_capabilities;
	hpm_prober_adapter_driver.jtag_ops->read_idcode = raw_mode ? NULL : hpm_read_idcode;
	return ERROR_OK;
}
COMMAND_HANDLER(hpm_dmi_batch_command)
{
	if (CMD_ARGC != 1) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], dmi_batch_enabled);
	return ERROR_OK;
}
static const struct command_registration hpm_subcommands[] = {
	{ .name = "feature", .handler = hpm_feature_command, .mode = COMMAND_ANY,
	  .help = "Query or enable probe-side memory offloads", .usage = "[sba|progbuf|abstract [on|off]]" },
	{ .name = "transport", .handler = hpm_transport_command, .mode = COMMAND_CONFIG,
	  .help = "Select physical four-wire JTAG or two-wire cJTAG", .usage = "jtag|cjtag" },
	{ .name = "mode", .handler = hpm_mode_command, .mode = COMMAND_CONFIG,
	  .help = "Force raw JTAG scans or use direct probe operations", .usage = "raw|direct" },
	{ .name = "dmi_batch", .handler = hpm_dmi_batch_command, .mode = COMMAND_ANY,
	  .help = "Enable queued raw JTAG DMI reads when supported by the DM", .usage = "on|off" },
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration hpm_commands[] = {
	{ .name = "hpm_probex", .mode = COMMAND_ANY, .chain = hpm_subcommands,
	  .help = "HPM Probe X configuration", .usage = "" },
	COMMAND_REGISTRATION_DONE
};
static struct jtag_interface hpm_jtag = { .execute_queue = hpm_execute_queue, .read_idcode = hpm_read_idcode };
struct adapter_driver hpm_prober_adapter_driver = {
	.name = "hpm_probex", .transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG, .commands = hpm_commands,
	.init = hpm_init, .quit = hpm_quit, .speed = hpm_speed,
	.khz = hpm_khz, .speed_div = hpm_speed_div, .jtag_ops = &hpm_jtag,
	.capabilities = &hpm_capabilities, .riscv_debug_ops = &hpm_riscv_debug_ops,
};
