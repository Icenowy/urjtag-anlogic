/*
 * Copyright (C) 2017 Hiroki Mori
 * BSD GPIO JTAG Cable Driver
 *
 * Based on GPIO JTAG Cable Driver
 * (C) Copyright 2010
 * Stefano Babic, DENX Software Engineering, sbabic@denx.de.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.     See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 */

#include <sysdep.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgpio.h>

#include <stdlib.h>
#include <string.h>

#include <urjtag/error.h>
#include <urjtag/log.h>
#include <urjtag/parport.h>
#include <urjtag/cable.h>

#include "generic.h"

/* pin mapping */
enum {
    GPIO_TDI = 0,
    GPIO_TCK,
    GPIO_TMS,
    GPIO_TDO,
    GPIO_REQUIRED
};

typedef struct {
    unsigned int jtag_bsdgpios[4];
    int          signals;
    uint32_t     lastout;
    int          fd_bsdgpios;
} bsdgpio_params_t;

static int bsdgpio_direction (int fd, unsigned int bsdgpio, int flag)
{
    gpio_config_t conf;

    conf.g_pin = bsdgpio;
    conf.g_flags = flag;
    gpio_pin_set_flags(fd, &conf);

    return URJ_STATUS_OK;
}

static int bsdgpio_get_value (int fd, unsigned int bsdgpio)
{
    int value;

    value = gpio_pin_get (fd, bsdgpio);

    return value;
}

static int
bsdgpio_open (urj_cable_t *cable)
{
    bsdgpio_params_t *p = cable->params;
    char fname[50];
    int i;

    snprintf (fname, sizeof (fname) - 1, "/dev/gpioc%d", 0);

    p->fd_bsdgpios = open (fname, O_RDWR);
    if (p->fd_bsdgpios < 0)
    {
        urj_warning (_("%s: cannot open bsdgpio\n"), fname);
        return URJ_STATUS_FAIL;
    }

    /* Export all bsdgpios */
    for (i = 0; i < GPIO_REQUIRED; i++)
    {
        unsigned int bsdgpio = p->jtag_bsdgpios[i];

        bsdgpio_direction (p->fd_bsdgpios, bsdgpio, (i == GPIO_TDO) ? GPIO_PIN_INPUT : GPIO_PIN_OUTPUT);

   }

    return URJ_STATUS_OK;
}

static int
bsdgpio_close (urj_cable_t *cable)
{
    int i;
    bsdgpio_params_t *p = cable->params;

    for (i = 0; i < GPIO_REQUIRED; i++)
    {
        if (p->fd_bsdgpios)
            close (p->fd_bsdgpios);
    }

    return URJ_STATUS_OK;
}

static void
bsdgpio_help (urj_log_level_t ll, const char *cablename)
{
    urj_log (ll,
        _("Usage: cable %s tdi=<gpio_tdi> tdo=<gpio_tdo> "
        "tck=<gpio_tck> tms=<gpio_tms>\n"
        "\n"), cablename);
}

static int
bsdgpio_connect (urj_cable_t *cable, const urj_param_t *params[])
{
    bsdgpio_params_t *cable_params;
    int i;

    cable_params = calloc (1, sizeof (*cable_params));
    if (!cable_params)
    {
        urj_error_set (URJ_ERROR_OUT_OF_MEMORY, _("calloc(%zd) fails"),
                       sizeof (*cable_params));
        free (cable);
        return URJ_STATUS_FAIL;
    }

    cable_params->jtag_bsdgpios[GPIO_TDI] = GPIO_REQUIRED;
    cable_params->jtag_bsdgpios[GPIO_TDO] = GPIO_REQUIRED;
    cable_params->jtag_bsdgpios[GPIO_TMS] = GPIO_REQUIRED;
    cable_params->jtag_bsdgpios[GPIO_TCK] = GPIO_REQUIRED;
    if (params != NULL)
        /* parse arguments beyond the cable name */
        for (i = 0; params[i] != NULL; i++)
        {
            switch (params[i]->key)
            {
            case URJ_CABLE_PARAM_KEY_TDI:
                cable_params->jtag_bsdgpios[GPIO_TDI] = params[i]->value.lu;
                break;
            case URJ_CABLE_PARAM_KEY_TDO:
                cable_params->jtag_bsdgpios[GPIO_TDO] = params[i]->value.lu;
                break;
            case URJ_CABLE_PARAM_KEY_TMS:
                cable_params->jtag_bsdgpios[GPIO_TMS] = params[i]->value.lu;
                break;
            case URJ_CABLE_PARAM_KEY_TCK:
                cable_params->jtag_bsdgpios[GPIO_TCK] = params[i]->value.lu;
                break;
            default:
                break;
            }
        }

    urj_log (URJ_LOG_LEVEL_NORMAL,
        _("Initializing BSD GPIO JTAG Chain\n"));

    cable->params = cable_params;
    cable->chain = NULL;
    cable->delay = 10;

    return URJ_STATUS_OK;
}

static void
bsdgpio_disconnect (urj_cable_t *cable)
{
    urj_tap_chain_disconnect (cable->chain);
    bsdgpio_close (cable);
}

static void
bsdgpio_cable_free (urj_cable_t *cable)
{
    free (cable->params);
    free (cable);
}

static int
bsdgpio_init (urj_cable_t *cable)
{
    bsdgpio_params_t *p = cable->params;

    if (bsdgpio_open (cable) != URJ_STATUS_OK)
        return URJ_STATUS_FAIL;

    p->signals = URJ_POD_CS_TRST;

    return URJ_STATUS_OK;
}

static void
bsdgpio_done (urj_cable_t *cable)
{
    bsdgpio_close (cable);
}

static void
bsdgpio_clock (urj_cable_t *cable, int tms, int tdi, int n)
{
    bsdgpio_params_t *p = cable->params;
    int i;

    tms = tms ? 1 : 0;
    tdi = tdi ? 1 : 0;

    gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TMS], tms);
    gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TDI], tdi);

    for (i = 0; i < n; i++)
    {
        gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TCK], 0);
        gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TCK], 1);
        gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TCK], 0);
    }
}

static int
bsdgpio_get_tdo ( urj_cable_t *cable )
{
    bsdgpio_params_t *p = cable->params;

    gpio_pin_set(p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TCK], 0);
    gpio_pin_set(p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TDI], 0);
    gpio_pin_set(p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TMS], 0);
    p->lastout &= ~(URJ_POD_CS_TMS | URJ_POD_CS_TDI | URJ_POD_CS_TCK);

    urj_tap_cable_wait (cable);

    return bsdgpio_get_value (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TDO]);
}

static int
bsdgpio_current_signals (urj_cable_t *cable)
{
    bsdgpio_params_t *p = cable->params;

    int sigs = p->signals & ~(URJ_POD_CS_TMS | URJ_POD_CS_TDI | URJ_POD_CS_TCK);

    if (p->lastout & URJ_POD_CS_TCK) sigs |= URJ_POD_CS_TCK;
    if (p->lastout & URJ_POD_CS_TDI) sigs |= URJ_POD_CS_TDI;
    if (p->lastout & URJ_POD_CS_TMS) sigs |= URJ_POD_CS_TMS;

    return sigs;
}

static int
bsdgpio_set_signal (urj_cable_t *cable, int mask, int val)
{
    int prev_sigs = bsdgpio_current_signals (cable);
    bsdgpio_params_t *p = cable->params;

    mask &= (URJ_POD_CS_TDI | URJ_POD_CS_TCK | URJ_POD_CS_TMS); // only these can be modified

    if (mask != 0)
    {
        if (mask & URJ_POD_CS_TMS)
            gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TMS], val & URJ_POD_CS_TMS);
        if (mask & URJ_POD_CS_TDI)
            gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TDI], val & URJ_POD_CS_TDI);
        if (mask & URJ_POD_CS_TCK)
            gpio_pin_set (p->fd_bsdgpios, p->jtag_bsdgpios[GPIO_TCK], val & URJ_POD_CS_TCK);
    }

    p->lastout = val & mask;

    return prev_sigs;
}

static int
bsdgpio_get_signal (urj_cable_t *cable, urj_pod_sigsel_t sig)
{
    return (bsdgpio_current_signals (cable) & sig) ? 1 : 0;
}

const urj_cable_driver_t urj_tap_cable_bsdgpio_driver = {
    "bsdgpio",
    N_("BSD GPIO JTAG Chain"),
    URJ_CABLE_DEVICE_OTHER,
    { .other = bsdgpio_connect, },
    bsdgpio_disconnect,
    bsdgpio_cable_free,
    bsdgpio_init,
    bsdgpio_done,
    urj_tap_cable_generic_set_frequency,
    bsdgpio_clock,
    bsdgpio_get_tdo,
    urj_tap_cable_generic_transfer,
    bsdgpio_set_signal,
    bsdgpio_get_signal,
    urj_tap_cable_generic_flush_one_by_one,
    bsdgpio_help
};
