/*
 * $Id$
 *
 * Copyright (C) 2011 Steve Tell
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * Python bindings for urjtag intially written by Steve Tell.
 * Additional methods by Jonathan Stroud.
 *
 */
#include <Python.h>
#include "structmember.h"
#include "pycompat23.h"

#include <sysdep.h>

#include <urjtag/urjtag.h>
#include <urjtag/chain.h>
#include <urjtag/cmd.h>

#include "py_urjtag.h"

extern PyObject *UrjtagError;

static void
urj_pyr_dealloc (urj_pyregister_t *self)
{
    Py_TYPE (self)->tp_free ((PyObject *) self);
}

static PyObject *
urj_pyr_str (PyObject *s)
{
    urj_pyregister_t *self = (urj_pyregister_t *) s;
    char buf[200];
    urj_chain_t *urc = self->urc;

    if (self->urreg == NULL || self->inst == NULL)
        snprintf (buf, 200, "<urjtag.register chain=%p invalid>", urc);
    else
        snprintf (buf, 200, "<urjtag.register chain=%p reg=%s inst=%s>", urc, 
             self->urreg->name, self->inst->name);
    return PyString_FromString (buf);
}


static PyObject *
urj_pyr_get_dr (urj_pyregister_t *self, int in, int string, PyObject *args)
{
    urj_data_register_t *dr = self->urreg;
    urj_chain_t *urc = self->urc;
    urj_tap_register_t *r;
    int lsb = -1;
    int msb = -1;
    const char *value_string;

    if (!PyArg_ParseTuple (args, "|ii", &msb, &lsb))
        return NULL;
    if (lsb == -1)
        lsb = msb;
    if (!urj_pyc_precheck (urc, UPRC_CBL))
        return NULL;

    if (dr == NULL)
    {
        PyErr_SetString (UrjtagError,
                         _("invalid data register object"));
        return NULL;
    }

    if (in)
        r = dr->in;             /* input buffer for next shift_dr */
    else
        r = dr->out;            /* recently captured+scanned-out values */

    if (msb == -1)
        value_string = urj_tap_register_get_string (r);
    else
        value_string = urj_tap_register_get_string_bit_range (r, msb, lsb);
    if (value_string == NULL)
    {
        PyErr_SetString (UrjtagError,
                         _("error obtaining tap register value"));
        return NULL;
    }

    if (string)
        return Py_BuildValue ("s", value_string);
    else
        return PyLong_FromString ((char *)value_string, NULL, 2);
}

static PyObject *
urj_pyr_get_str_dr_out (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_get_dr (self, 0, 1, args);
}

static PyObject *
urj_pyr_get_str_dr_in (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_get_dr (self, 1, 1, args);
}

static PyObject *
urj_pyr_get_int_dr_out (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_get_dr (self, 0, 0, args);
}

static PyObject *
urj_pyr_get_int_dr_in (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_get_dr (self, 1, 0, args);
}

static PyObject *
urj_pyr_set_dr (urj_pyregister_t *self, int in, PyObject *args)
{
    urj_data_register_t *dr = self->urreg;
    urj_tap_register_t *r;
    char *newstr = NULL;
    uint64_t newval;
    int lsb = -1;
    int msb = -1;


    if (!PyArg_ParseTuple (args, "s|ii", &newstr, &msb, &lsb))
    {
        PyErr_Clear ();
        if (!PyArg_ParseTuple (args, "L|ii", &newval, &msb, &lsb))
            return NULL;
    }

    if (dr == NULL)
    {
        PyErr_SetString (UrjtagError,
                         _("invalid register object"));
        return NULL;
    }

    if (in)
        r = dr->in;
    else
        r = dr->out;

    if (msb == -1)
    {
        if (newstr)
            return urj_py_chkret (urj_tap_register_set_string (r, newstr));
        else
            return urj_py_chkret (urj_tap_register_set_value (r, newval));
    }
    else
    {
        if (lsb == -1)
            lsb = msb;

        if (newstr)
            return urj_py_chkret (urj_tap_register_set_string_bit_range (r, newstr, msb, lsb));
        else
            return urj_py_chkret (urj_tap_register_set_value_bit_range (r, newval, msb, lsb));
    }
}

static PyObject *
urj_pyr_set_dr_out (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_set_dr (self, 0, args);
}

static PyObject *
urj_pyr_set_dr_in (urj_pyregister_t *self, PyObject *args)
{
    return urj_pyr_set_dr (self, 1, args);
}

static PyObject *
urj_pyr_shift_dr (urj_pyregister_t *self, PyObject *args)
{

    urj_chain_t *urc = self->urc;
    int partn = self->part;
    char *instname = NULL;
    urj_part_t *part;

    if (!PyArg_ParseTuple (args, "|s", &instname))
        return NULL;


    if (!urj_pyc_precheck (urc, UPRC_CBL))
        return NULL;

    if (self->urreg == NULL)
    {
        PyErr_SetString (UrjtagError,
                         _("invalid register object"));
        return NULL;
    }

    
    urc->active_part = partn;
    part = urj_tap_chain_active_part (urc);
    if (part == NULL)
    {
        PyErr_SetString (UrjtagError, _("No active part on chain"));
        return NULL;
    }
    if (instname) 
    {
        urj_part_set_instruction (part, instname);
    }
    else 
    {
        if (!self->inst) { 
            PyErr_SetString (UrjtagError, _("no instruction for data register"));
            return NULL;
        }
        part->active_instruction = self->inst;
    }

    return urj_py_chkret (urj_tap_chain_shift_data_registers (urc, 1));
}

static PyObject *
urj_pyr_shift_ir (urj_pyregister_t *self,  PyObject *args)
{
    urj_chain_t *urc = self->urc;
    char *instname = NULL;
    urj_part_t *part;

    if (!PyArg_ParseTuple (args, "|s", &instname)) 
        return NULL;

    if (!urj_pyc_precheck (urc, UPRC_CBL))
        return NULL;

    urc->active_part = self->part;
    part = urj_tap_chain_active_part (urc);
    if (part == NULL)
    {
        PyErr_SetString (UrjtagError, _("No active part on chain"));
        return NULL;
    }
    if (instname) {
        urj_part_set_instruction (part, instname);
    } 
    else 
    {
        if (!self->inst) 
        { 
            PyErr_SetString (UrjtagError, _("no instruction for data register"));
            return NULL;
        }
        part->active_instruction = self->inst;
    }

    return urj_py_chkret (urj_tap_chain_shift_instructions (urc));
}



static PyMethodDef urj_pyr_methods[] =
{
    {"get_dr_in", (PyCFunction) urj_pyr_get_int_dr_in, METH_VARARGS,
     "get bits that will be scanned in on next shift_dr, as integer"},
    {"get_dr_in_string", (PyCFunction) urj_pyr_get_str_dr_in, METH_VARARGS,
     "get bits that will be scanned in on next shift_dr, as string"},
    {"get_dr_out", (PyCFunction) urj_pyr_get_int_dr_out, METH_VARARGS,
     "retrieve values scanned out from the data registers on the last shift_dr, as integer"},
    {"get_dr_out_string", (PyCFunction) urj_pyr_get_str_dr_out, METH_VARARGS,
     "retrieve values scanned out from the data registers on the last shift_dr, as string"},

    {"set_dr_in", (PyCFunction) urj_pyr_set_dr_in, METH_VARARGS,
     "set bits that will be scanned in on next shiftdr"},
    {"set_dr_out", (PyCFunction) urj_pyr_set_dr_out, METH_VARARGS,
     "set the holding register for values scanned out from the data register"},
    {"shift_dr", (PyCFunction) urj_pyr_shift_dr, METH_VARARGS,
     "scan values through the data register"},
    {"shift_ir", (PyCFunction) urj_pyr_shift_ir, METH_VARARGS,
     "scan values through the instruction register to select this data regsiter"},
    {NULL},
};


PyTypeObject urj_pyregister_Type =
{
    PyVarObject_HEAD_INIT (NULL, 0) "urjtag.register", /* tp_name */
    sizeof (urj_pyregister_t),             /* tp_basicsize */
    0,                          /* tp_itemsize */
    (destructor) urj_pyr_dealloc, /* tp_dealloc */
    0,                          /* tp_print */
    0,                          /* tp_getattr */
    0,                          /* tp_setattr */
    0,                          /* tp_compare */
    urj_pyr_str,                /* tp_repr */
    0,                          /* tp_as_number */
    0,                          /* tp_as_sequence */
    0,                          /* tp_as_mapping */
    0,                          /* tp_hash */
    0,                          /* tp_call */
    urj_pyr_str,                /* tp_str */
    0,                          /* tp_getattro */
    0,                          /* tp_setattro */
    0,                          /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,   /* tp_flags */
    "JTAG register object",     /* tp_doc */
    0,                          /* tp_traverse */
    0,                          /* tp_clear */
    0,                          /* tp_richcompare */
    0,                          /* tp_weaklistoffset */
    0,                          /* tp_iter */
    0,                          /* tp_iternext */
    urj_pyr_methods,            /* tp_methods */
    0,                          /* tp_members */
    0,                          /* tp_getset */
    0,                          /* tp_base */
    0,                          /* tp_dict */
    0,                          /* tp_descr_get */
    0,                          /* tp_descr_set */
    0,                          /* tp_dictoffset */
    0,                          /* tp_init */
    0,                          /* tp_alloc */
    0,                          /* tp_new */
};


/* Local Variables: */
/* mode:c */
/* comment-column:0 */
/* c-basic-offset:4 */
/* space-before-funcall:t */
/* indent-tabs-mode:nil */
/* End: */
