/*
 * common definitions for python bindings
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
 */

typedef struct urj_pyregister urj_pyregister_t;
struct urj_pyregister
{
    PyObject_HEAD urj_data_register_t *urreg;
    int part;
    urj_chain_t *urc;
    urj_part_instruction_t *inst;
    urj_pyregister_t *next;
};

extern PyTypeObject urj_pyregister_Type;

extern PyObject *urj_py_chkret (int rc);
PyObject *UrjtagError;

extern int urj_pyc_precheck (urj_chain_t *urc, int checks_needed);
#define UPRC_CBL 1
#define UPRC_DET 2
#define UPRC_BUS 4
