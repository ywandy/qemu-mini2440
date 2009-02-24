/*
 * QEMU EEPROM 24c0x emulation
 *
 * Copyright (c) 2009 Michel Pollet <buserror@gmail.com>
 * Copyright (c) 2006 Aurelien Jarno
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef EEPROM24C0X_H
#define EEPROM24C0X_H

typedef struct _eeprom24c0x_t eeprom24c0x_t;

enum {
	EE_24C01 = 0,
	EE_24C02,
	EE_24C04,
	EE_24C08,
};

/* Create a new EEPROM of the siae specified. */
eeprom24c0x_t * eeprom24c0x_new(int eeprom_kind);

/* Destroy an existing EEPROM. */
void eeprom24c0x_free(eeprom24c0x_t *eeprom);

/* Read from the EEPROM. */
uint8_t eeprom24c0x_read(eeprom24c0x_t * eeprom);

/* Write to the EEPROM. */
void eeprom24c0x_write(eeprom24c0x_t * eeprom, int scl, int sda);

/* Get EEPROM data array. */
uint8_t *eeprom24c0x_data(eeprom24c0x_t *eeprom);

#endif /* EEPROM24C0X_H */
