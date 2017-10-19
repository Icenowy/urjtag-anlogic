#include <stdlib.h>
#include <string.h>
#include <sysdep.h>
#include <urjtag/usbconn.h>
#include <urjtag/cable.h>
#include <urjtag/chain.h>

#include "usbconn/libusb.h"
#include "generic.h"
#include "generic_usbconn.h"

/* DirtyJTAG USB interface */
#define DIRTYJTAG_USB_TIMEOUT 1000 /* 100ms */
#define DIRTYJTAG_WRITE_ENDPOINT 0x01
#define DIRTYJTAG_READ_ENDPOINT 0x82
#define DIRTYJTAG_BUFFER_SIZE 64

/* DirtyJTAG commands */
#define CMD_STOP 0x00
#define CMD_INFO 0x01
#define CMD_FREQ 0x02
#define CMD_XFER 0x03
#define CMD_SETSIG 0x04
#define CMD_GETSIG 0x05
#define CMD_CLK 0x06

/* DirtyJTAG signal definitions */
#define SIG_TCK (1 << 1)
#define SIG_TDI (1 << 2)
#define SIG_TDO (1 << 3)
#define SIG_TMS (1 << 4)
#define SIG_TRST (1 << 5)
#define SIG_SRST (1 << 6)

static uint8_t current_signals = 0;

/**
 * @brief Initialize JTAG adapter
 *
 * This function initialize the USB link and the logic level of
 * the adapter's output.
 *
 * @param cable Cable structure pointer
 */
static int dirtyjtag_init(urj_cable_t *cable);

/**
 * @brief Set JTAG probe frequency
 *
 * @param cable Cable structure pointer
 * @param frequency Clock frequency in kHz
 */
static void dirtyjtag_set_frequency(urj_cable_t *cable, uint32_t frequency);

/**
 * @brief Send multiple clock impulses with specified TMS & TDI states
 *
 * @param cable Cable structure pointer
 * @param tms TMS state
 * @param tdi TDI state
 * @param n Number of clock pulses
 */
static void dirtyjtag_clock(urj_cable_t *cable, int tms, int tdi, int n);

/**
 * @brief Get TDO state
 *
 * @param cable Cable structure pointer
 */
static int dirtyjtag_get_tdo(urj_cable_t *cable);

/**
 * @brief Set JTAG signals state
 *
 * @param cable Cable structure pointer
 * @param mask Signal mask
 * @param mask Signal state
 */
static int dirtyjtag_set_signal(urj_cable_t *cable, int mask, int val);

/**
 * @brief Read and write on TDI/TDO
 *
 * @param cable Cable structure pointer
 * @param len Number of bits exchanges
 * @param in Data which is outputted on TDI pin
 * @param out Data which is read from TDO pin
 */
static int dirtyjtag_transfer(urj_cable_t *cable, int len,
			      const char *in, char *out);

/**
 * @brief Send data using USB bulk transfer
 *
 * @param cable Cable structure pointer
 * @param data Data being sent
 * @param length Number of bytes sent
 */
static int dirtyjtag_send(urj_cable_t *cable, uint8_t *data, int length);

/**
 * @brief Read data from USB bulk transfer
 *
 * @param cable Cable structure pointer
 * @param data Received data buffer (this function does not allocate memory for it !)
 * @param length Buffer size
 */
static int dirtyjtag_read(urj_cable_t *cable, uint8_t *data, int length);

/**
 * @brief Determine the lowest value
 */
static int min(int a, int b) {
  if (a < b) {
    return a;
  } else {
    return b;
  }
}

static void dirtyjtag_set_frequency(urj_cable_t *cable, uint32_t frequency) {
  uint8_t command[3];

  /* Convert the frequency from MHz to KHz */
  frequency /= 1E3;

  command[0] = CMD_FREQ;
  command[1] = (uint8_t)(frequency >> 8) & 0xFF;
  command[2] = (uint8_t)frequency & 0xFF;

  dirtyjtag_send(cable, command, 3);
}

static int dirtyjtag_init(urj_cable_t *cable) {
  uint8_t commands[6];

  if (urj_tap_usbconn_open(cable->link.usb) != URJ_STATUS_OK) {
    return URJ_STATUS_FAIL;
  }

  /* Set frequency to 100kHz */
  commands[0] = CMD_FREQ;
  commands[1] = 0;
  commands[2] = 100;

  /* Set TDI, TMS, TCK to LOW */
  commands[3] = CMD_SETSIG;
  commands[4] = SIG_TDI | SIG_TMS | SIG_TCK;
  commands[5] = 0;

  dirtyjtag_send(cable, commands, 6);

  return URJ_STATUS_OK;
}

static void dirtyjtag_clock(urj_cable_t *cable, int tms, int tdi, int clock_pulses) {
  uint8_t signals = 0, clock_packets;
  uint8_t *command_buffer;
  size_t i;

  /* This is equivalent to CEIL(clock_pulses/255) */
  clock_packets = (clock_pulses+254)/255;

  /* Each clock command is 3 bytes long (1 byte for cmd identifier,
     1 byte for TDI/TMS state, 1 bytes for the number of clocks) */
  command_buffer = malloc(3 * clock_packets);

  signals |= tms ? SIG_TMS : 0;
  signals |= tdi ? SIG_TDI : 0;

  /* We can only do 255 clock pulses in one command, so we need
     to send multiple clock commands */
  for (i = 0; i < clock_packets; i++) {
    command_buffer[i*3] = CMD_CLK;
    command_buffer[i*3 + 1] = signals;
    command_buffer[i*3 + 2] = min(255,clock_pulses);

    clock_pulses -= min(255,clock_pulses);
  }

  dirtyjtag_send(cable, command_buffer, 3*clock_packets);
}

static int dirtyjtag_get_tdo(urj_cable_t *cable) {
  uint8_t command_byte, response;

  command_byte = CMD_GETSIG;

  dirtyjtag_send(cable, &command_byte, 1);

  dirtyjtag_read(cable, &response, 1);

  return (response & SIG_TDO) ? 1 : 0;
}

static int dirtyjtag_set_signal(urj_cable_t *cable, int mask, int val) {
  uint8_t commands[3];
  uint8_t signal_value = 0, signal_mask = 0;

  /* Applying mask */
  mask &= URJ_POD_CS_TMS | URJ_POD_CS_TCK | URJ_POD_CS_TDI | \
    URJ_POD_CS_TRST | URJ_POD_CS_RESET;
  val &= mask;

  signal_mask |= (mask & URJ_POD_CS_TCK) ? SIG_TCK : 0;
  signal_mask |= (mask & URJ_POD_CS_TDI) ? SIG_TDI : 0;
  signal_mask |= (mask & URJ_POD_CS_TMS) ? SIG_TMS : 0;
  signal_mask |= (mask & URJ_POD_CS_TRST) ? SIG_TRST : 0;
  signal_mask |= (mask & URJ_POD_CS_RESET) ? SIG_SRST : 0;

  signal_value |= (val & URJ_POD_CS_TCK) ? SIG_TCK : 0;
  signal_value |= (val & URJ_POD_CS_TDI) ? SIG_TDI : 0;
  signal_value |= (val & URJ_POD_CS_TMS) ? SIG_TMS : 0;
  signal_value |= (val & URJ_POD_CS_TRST) ? SIG_TRST : 0;
  signal_value |= (val & URJ_POD_CS_RESET) ? SIG_SRST : 0;
		
  commands[0] = CMD_SETSIG;
  commands[1] = signal_mask;
  commands[2] = signal_value;

  /* Send commands */
  dirtyjtag_send(cable, commands, 3);

  /* Updating signal status */
  current_signals &= ~mask;
  current_signals |= val;

  return val;
}

static int dirtyjtag_get_signal(urj_cable_t *cable, urj_pod_sigsel_t sig) {
  return sig & current_signals;
}

static int dirtyjtag_transfer(urj_cable_t *cable, int len,
			      const char *in, char *out) {
  uint8_t packet[32], response[32];
  uint8_t bits_in_packet;
  int num_bytes, num_packets, packet_id, ret;
  size_t sent_bits, i;

  sent_bits = 0;

  /* Each packet holds 30 bytes (240 bits) of data */
  num_bytes = (len+7)/8;
  num_packets = (num_bytes+29)/30;

  for (packet_id = 0; packet_id < num_packets; packet_id++) {
    memset(packet, 0, 32);

    bits_in_packet = (uint8_t)min(240, len);

    packet[0] = CMD_XFER;
    packet[1] = bits_in_packet;

    /* Pack bits into send packet */
    for (i = 0; i < bits_in_packet; i++) {
      packet[2 + i/8] |= in[sent_bits + i] ? (0x80 >> (i%8)) : 0;
    }

    /* Send packet */
    dirtyjtag_send(cable, packet, 32);

    /* Receive response */
    ret = dirtyjtag_read(cable, response, 32);
    if (ret) {
      printf("USB read failed (timeout expired ?)\n");
      return 0;
    }

    /* Unpack response */
    if (out) {
      for (i = 0; i < bits_in_packet; i++) {
	out[sent_bits + i] = (response[i/8] & (0x80 >> (i%8))) ? 1:0;
      }
    }

    len -= bits_in_packet;
    sent_bits += bits_in_packet;
  }

  /* TODO : update this accordingly to firmware */
  current_signals &= ~(URJ_POD_CS_TDI | URJ_POD_CS_TCK | URJ_POD_CS_TMS);

  return len;
}

static int dirtyjtag_send(urj_cable_t *cable, uint8_t *data, int length) {
  urj_usbconn_libusb_param_t *params;
  int result, unused;
  uint8_t *commands_buffer;

  params = cable->link.usb->params;

  commands_buffer = malloc(length+1);
  memcpy(commands_buffer, data, length);
  commands_buffer[length] = 0x00;

  result = libusb_bulk_transfer(params->handle,
				DIRTYJTAG_WRITE_ENDPOINT,
				commands_buffer, length+1, &unused,
				DIRTYJTAG_USB_TIMEOUT);

  free(commands_buffer);

  return result;
}

static int dirtyjtag_read(urj_cable_t *cable, uint8_t *data, int length) {
  urj_usbconn_libusb_param_t *params;
  int result, read_bytes;

  params = cable->link.usb->params;

  result = libusb_bulk_transfer(params->handle,
				DIRTYJTAG_READ_ENDPOINT,
				data, length, &read_bytes,
				DIRTYJTAG_USB_TIMEOUT);

  return result;
}

const urj_cable_driver_t urj_tap_cable_dirtyjtag_driver = {
  "DirtyJTAG",
  "DirtyJTAG STM32-based cable",
  URJ_CABLE_DEVICE_USB,
  { .usb = urj_tap_cable_generic_usbconn_connect },
  urj_tap_cable_generic_disconnect,
  urj_tap_cable_generic_usbconn_free,
  dirtyjtag_init,
  urj_tap_cable_generic_usbconn_done,
  dirtyjtag_set_frequency,
  dirtyjtag_clock,
  dirtyjtag_get_tdo,
  dirtyjtag_transfer,
  dirtyjtag_set_signal,
  dirtyjtag_get_signal,
  urj_tap_cable_generic_flush_using_transfer,
  urj_tap_cable_generic_usbconn_help
};
URJ_DECLARE_USBCONN_CABLE(0x1209, 0xC0CA, "libusb", "dirtyjtag", dirtyjtag)
