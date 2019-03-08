#include <stdlib.h>
#include <string.h>
#include <sysdep.h>
#include <urjtag/usbconn.h>
#include <urjtag/cable.h>
#include <urjtag/chain.h>

#include "usbconn/libusb.h"
#include "generic.h"
#include "generic_usbconn.h"

/* Anlogic JTAG cable USB interface */
#define ANLOGIC_JTAG_USB_TIMEOUT 1000 /* 100ms */
#define ANLOGIC_JTAG_WRITE_ENDPOINT 0x06
#define ANLOGIC_JTAG_READ_ENDPOINT 0x82
#define ANLOGIC_JTAG_MODE_ENDPOINT 0x08

/* Anlogic JTAG cable modes */
#define ANLOGIC_JTAG_JTAG_MODE 1
#define ANLOGIC_JTAG_TEST_MODE 2

/* Anlogic JTAG cable speeds */
#define ANLOGIC_JTAG_SPEED_6M 0
#define ANLOGIC_JTAG_SPEED_3M 0x4
#define ANLOGIC_JTAG_SPEED_2M 0x8
#define ANLOGIC_JTAG_SPEED_1M 0x14
#define ANLOGIC_JTAG_SPEED_600K 0x24
#define ANLOGIC_JTAG_SPEED_400K 0x38
#define ANLOGIC_JTAG_SPEED_200K 0x70
#define ANLOGIC_JTAG_SPEED_100K 0xe8
#define ANLOGIC_JTAG_SPEED_90K 0xff

#define ANLOGIC_JTAG_TCK (1 << 2)
#define ANLOGIC_JTAG_TDI (1 << 1)
#define ANLOGIC_JTAG_TMS (1 << 0)
#define ANLOGIC_JTAG_OUT_MASK 0x7

#define ANLOGIC_JTAG_MAX_XFER_SIZE 1024

static uint8_t last_status;
static uint8_t last_tdo;

/**
 * @brief Initialize JTAG adapter
 *
 * This function initialize the USB link and the logic level of
 * the adapter's output.
 *
 * @param cable Cable structure pointer
 */
static int anlogic_init(urj_cable_t *cable);

/**
 * @brief Set JTAG probe frequency
 *
 * @param cable Cable structure pointer
 * @param frequency Clock frequency in kHz
 */
static void anlogic_set_frequency(urj_cable_t *cable, uint32_t frequency);

/**
 * @brief Send multiple clock impulses with specified TMS & TDI states
 *
 * @param cable Cable structure pointer
 * @param tms TMS state
 * @param tdi TDI state
 * @param n Number of clock pulses
 */
static void anlogic_clock(urj_cable_t *cable, int tms, int tdi, int n);

/**
 * @brief Get TDO state
 *
 * @param cable Cable structure pointer
 */
static int anlogic_get_tdo(urj_cable_t *cable);

/**
 * @brief Set JTAG signals state
 *
 * @param cable Cable structure pointer
 * @param mask Signal mask
 * @param mask Signal state
 */
static int anlogic_set_signal(urj_cable_t *cable, int mask, int val);

/**
 * @brief Read and write on TDI/TDO
 *
 * @param cable Cable structure pointer
 * @param len Number of bits exchanges
 * @param in Data which is outputted on TDI pin
 * @param out Data which is read from TDO pin
 */
static int anlogic_transfer(urj_cable_t *cable, int len,
			    const char *in, char *out);

/**
 * @brief Send then receive pin status using USB bulk transfer
 *
 * @param cable Cable structure pointer
 * @param out_data Data being sent
 * @param in_data Received data buffer (this function does not allocate memory for it !)
 * @param length Number of bytes sent and read
 */
static int anlogic_usb_xfer(urj_cable_t *cable, const uint8_t *out_data, uint8_t *in_data, int length);

static void anlogic_set_frequency(urj_cable_t *cable, uint32_t frequency) {
  urj_usbconn_libusb_param_t *params;
  int unused;
  uint8_t command[2];

  command[0] = ANLOGIC_JTAG_JTAG_MODE;

  if (frequency >= 6000000)
    command[1] = ANLOGIC_JTAG_SPEED_6M;
  else if (frequency >= 3000000)
    command[1] = ANLOGIC_JTAG_SPEED_3M;
  else if (frequency >= 2000000)
    command[1] = ANLOGIC_JTAG_SPEED_2M;
  else if (frequency >= 1000000)
    command[1] = ANLOGIC_JTAG_SPEED_1M;
  else if (frequency >= 600000)
    command[1] = ANLOGIC_JTAG_SPEED_600K;
  else if (frequency >= 400000)
    command[1] = ANLOGIC_JTAG_SPEED_400K;
  else if (frequency >= 200000)
    command[1] = ANLOGIC_JTAG_SPEED_200K;
  else if (frequency >= 100000)
    command[1] = ANLOGIC_JTAG_SPEED_100K;
  else
    command[1] = ANLOGIC_JTAG_SPEED_90K;

  params = cable->link.usb->params;

  libusb_bulk_transfer(params->handle,
		       ANLOGIC_JTAG_MODE_ENDPOINT,
		       command, sizeof(command), &unused,
		       ANLOGIC_JTAG_USB_TIMEOUT);

  return;
}

static int anlogic_init(urj_cable_t *cable) {
  int result;
  uint8_t out_val;

  if (urj_tap_usbconn_open(cable->link.usb) != URJ_STATUS_OK)
    return URJ_STATUS_FAIL;

  anlogic_set_frequency(cable, 90000);

  out_val = 0;
  result = anlogic_usb_xfer(cable, &out_val, NULL, 1);
  
  return result;
}

static void anlogic_clock(urj_cable_t *cable, int tms, int tdi, int clock_pulses) {
  uint8_t *buf, *curr_buf;
  int buf_size, curr_buf_size, xfer_size;

  buf_size = clock_pulses * 2 + 1;
  buf = malloc(buf_size);
  for (int i = 0; i < buf_size; i++) {
    buf[i] = 0;
    if (tms)
      buf[i] |= ANLOGIC_JTAG_TMS;
    if (tdi)
      buf[i] |= ANLOGIC_JTAG_TDI;
    if (i % 2)
      buf[i] |= ANLOGIC_JTAG_TCK;
  }

  curr_buf = buf;
  curr_buf_size = buf_size;
  while (curr_buf_size > 0) {
    if (curr_buf_size > ANLOGIC_JTAG_MAX_XFER_SIZE)
      xfer_size = ANLOGIC_JTAG_MAX_XFER_SIZE;
    else
      xfer_size = curr_buf_size;

    anlogic_usb_xfer(cable, curr_buf, NULL, xfer_size);

    curr_buf_size -= xfer_size;
    curr_buf += xfer_size;
  }
}

static int anlogic_get_tdo(urj_cable_t *cable) {
  return last_tdo;
}

static int anlogic_set_signal(urj_cable_t *cable, int mask, int val) {
  uint8_t status;

  status = last_status;

  if (mask & URJ_POD_CS_TCK) {
    if (val & URJ_POD_CS_TCK)
      status |= ANLOGIC_JTAG_TCK;
    else
      status &= ~ANLOGIC_JTAG_TCK;
  }

  if (mask & URJ_POD_CS_TMS) {
    if (val & URJ_POD_CS_TMS)
      status |= ANLOGIC_JTAG_TMS;
    else
      status &= ~ANLOGIC_JTAG_TMS;
  }

  if (mask & URJ_POD_CS_TDI) {
    if (val & URJ_POD_CS_TDI)
      status |= ANLOGIC_JTAG_TDI;
    else
      status &= ~ANLOGIC_JTAG_TDI;
  }

  anlogic_usb_xfer(cable, &status, NULL, 1);

  return val & mask;
}

static int anlogic_get_signal(urj_cable_t *cable, urj_pod_sigsel_t sig) {
  int signals = 0;

  if ((sig & URJ_POD_CS_TCK) && (last_status & ANLOGIC_JTAG_TCK))
    signals |= URJ_POD_CS_TCK;

  if ((sig & URJ_POD_CS_TMS) && (last_status & ANLOGIC_JTAG_TMS))
    signals |= URJ_POD_CS_TMS;

  if ((sig & URJ_POD_CS_TDI) && (last_status & ANLOGIC_JTAG_TDI))
    signals |= URJ_POD_CS_TDI;

  return signals;
}

static int anlogic_transfer(urj_cable_t *cable, int len,
			    const char *in, char *out)
{
  uint8_t *buf, *curr_buf;
  uint8_t *res_buf, *curr_res_buf;
  int buf_size, curr_buf_size, xfer_size;
  int result;

  buf_size = len * 3;
  buf = malloc(buf_size);
  res_buf = malloc(buf_size);

  for (int i = 0; i < buf_size; i++) {
    buf[i] = 0;
    if (in[i / 3])
      buf[i] |= ANLOGIC_JTAG_TDI;
    if (i % 3 == 1)
      buf[i] |= ANLOGIC_JTAG_TCK;
  }

  buf[buf_size - 1] = in[len - 1] ? ANLOGIC_JTAG_TDI : 0;

  curr_buf = buf;
  curr_res_buf = res_buf;
  curr_buf_size = buf_size;
  while (curr_buf_size > 0) {
    if (curr_buf_size > ANLOGIC_JTAG_MAX_XFER_SIZE)
      xfer_size = ANLOGIC_JTAG_MAX_XFER_SIZE;
    else
      xfer_size = curr_buf_size;

    result = anlogic_usb_xfer(cable, curr_buf, curr_res_buf, xfer_size);
    if (result) {
      free(buf);
      free(res_buf);
      return result;
    }

    curr_buf_size -= xfer_size;
    curr_buf += xfer_size;
    curr_res_buf += xfer_size;
  }

  if (out) {
    for (int i = 0; i < len; i++)
      out[i] = res_buf[i * 3];
  }

  return URJ_STATUS_OK;
}

static int anlogic_usb_xfer(urj_cable_t *cable, const uint8_t *out_data, uint8_t *in_data, int length)
{
  urj_usbconn_libusb_param_t *params;
  int result, unused;
  uint8_t *out_raw_buffer, *in_raw_buffer;
  int data_index, raw_length;

  params = cable->link.usb->params;

  raw_length = ANLOGIC_JTAG_MAX_XFER_SIZE / 2;
  out_raw_buffer = malloc(raw_length);
  in_raw_buffer = malloc(raw_length);

  for (int i = 0; i < ANLOGIC_JTAG_MAX_XFER_SIZE; i++) {
    data_index = i >= length ? length - 1 : i;
    if (i % 2)
      out_raw_buffer[i / 2] |= (out_data[data_index] & ANLOGIC_JTAG_OUT_MASK) << 4;
    else
      out_raw_buffer[i / 2] = out_data[data_index] & ANLOGIC_JTAG_OUT_MASK;
  }

  last_status = out_raw_buffer[raw_length - 1] >> 4;

  result = libusb_bulk_transfer(params->handle,
				ANLOGIC_JTAG_WRITE_ENDPOINT,
				out_raw_buffer, raw_length, &unused,
				ANLOGIC_JTAG_USB_TIMEOUT);

  if (result)
    goto end;

  result = libusb_bulk_transfer(params->handle,
				ANLOGIC_JTAG_READ_ENDPOINT,
				in_raw_buffer, raw_length, &unused,
				ANLOGIC_JTAG_USB_TIMEOUT);

  if (result)
    goto end;

  if (in_data) {
    for (int i = 0; i < length; i++) {
      if (i % 2)
        in_data[i] = (in_raw_buffer[i/2] >> 4) & 1;
      else
        in_data[i] = in_raw_buffer[i/2] & 1;
    }
  }

  last_tdo = (in_raw_buffer[raw_length - 1] >> 4) & 1;

end:
  free(out_raw_buffer);
  free(in_raw_buffer);

  return result;
}

const urj_cable_driver_t urj_tap_cable_anlogic_driver = {
  "Anlogic",
  "Anlogic JTAG cable",
  URJ_CABLE_DEVICE_USB,
  { .usb = urj_tap_cable_generic_usbconn_connect },
  urj_tap_cable_generic_disconnect,
  urj_tap_cable_generic_usbconn_free,
  anlogic_init,
  urj_tap_cable_generic_usbconn_done,
  anlogic_set_frequency,
  anlogic_clock,
  anlogic_get_tdo,
  anlogic_transfer,
  anlogic_set_signal,
  anlogic_get_signal,
  urj_tap_cable_generic_flush_using_transfer,
  urj_tap_cable_generic_usbconn_help
};
URJ_DECLARE_USBCONN_CABLE(0x0547, 0x1002, "libusb", "anlogic", anlogic)
