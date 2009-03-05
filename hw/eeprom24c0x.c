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

#include <assert.h>
#include "hw.h"
#include "eeprom24c0x.h"

/*
 * EEPROM 24C01 / 24C02 emulation.
 *
 * Emulation for serial EEPROMs:
 * 24C01 - 1024 bit (128 x 8)
 * 24C02 - 2048 bit (256 x 8)
 * 24C04 - 4096 bit (512 x 8)
 * 24C08 - 8192 bit (1024 x 8)
 *
 * Typical device names include Microchip 24C02SC or SGS Thomson ST24C02.
 */

#define DEBUG

#if defined(DEBUG)
#  define logout(fmt, args...) fprintf(stderr, "24C\t%-24s" fmt, __func__, ##args)
#else
#  define logout(fmt, args...) ((void)0)
#endif

struct _eeprom24c0x_t {
	uint8_t kind;
	uint8_t tick;
	uint8_t address;
	uint8_t command;
	uint8_t ack;
	uint8_t scl;
	uint8_t sda;
	uint8_t data;
	//~ uint16_t size;
	uint8_t contents[1024];
};

//typedef struct _eeprom24c0x_t eeprom24c0x_t;


eeprom24c0x_t * eeprom24c0x_new(int eeprom_kind)
{
    eeprom24c0x_t *eeprom = (eeprom24c0x_t *)qemu_mallocz(sizeof(eeprom24c0x_t));

	eeprom->kind = eeprom_kind;
    memset(eeprom->contents, 0xff, sizeof(eeprom->contents));
    return eeprom;
}

void eeprom24c0x_free(eeprom24c0x_t *eeprom)
{
    logout("eeprom = 0x%p\n", eeprom);
    qemu_free(eeprom);
}

uint8_t *eeprom24c0x_data(eeprom24c0x_t *eeprom)
{
	return eeprom->contents;
}

uint8_t eeprom24c0x_read(eeprom24c0x_t * eeprom)
{
    logout("%u: scl = %u, sda = %u, data = 0x%02x\n",
        eeprom->tick, eeprom->scl, eeprom->sda, eeprom->data);
    return eeprom->sda;
}

void eeprom24c0x_write(eeprom24c0x_t * eeprom, int scl, int sda)
{
    if (eeprom->scl && scl && (eeprom->sda != sda)) {
        logout("%u: scl = %u->%u, sda = %u->%u i2c %s\n",
                eeprom->tick, eeprom->scl, scl, eeprom->sda, sda, sda ? "stop" : "start");
        if (!sda) {
            eeprom->tick = 1;
            eeprom->command = 0;
        }
    } else if (eeprom->tick == 0 && !eeprom->ack) {
        /* Waiting for start. */
        logout("%u: scl = %u->%u, sda = %u->%u wait for i2c start\n",
                eeprom->tick, eeprom->scl, scl, eeprom->sda, sda);
    } else if (!eeprom->scl && scl) {
        logout("%u: scl = %u->%u, sda = %u->%u trigger bit\n",
                eeprom->tick, eeprom->scl, scl, eeprom->sda, sda);
        if (eeprom->ack) {
            logout("\ti2c ack bit = 0\n");
            sda = 0;
            eeprom->ack = 0;
        } else if (eeprom->sda == sda) {
            uint8_t bit = (sda != 0);
            logout("\ti2c bit = %d\n", bit);
            if (eeprom->tick < 9) {
                eeprom->command <<= 1;
                eeprom->command += bit;
                eeprom->tick++;
                if (eeprom->tick == 9) {
                    logout("\tcommand 0x%04x, %s\n", eeprom->command, bit ? "read" : "write");
                    eeprom->ack = 1;
                }
            } else if (eeprom->tick < 17) {
                if (eeprom->command & 1) {
                    sda = ((eeprom->data & 0x80) != 0);
                }
                eeprom->address <<= 1;
                eeprom->address += bit;
                eeprom->tick++;
                eeprom->data <<= 1;
                if (eeprom->tick == 17) {
                    eeprom->data = eeprom->contents[eeprom->address];
                    logout("\taddress 0x%04x, data 0x%02x\n", eeprom->address, eeprom->data);
                    eeprom->ack = 1;
                    eeprom->tick = 0;
                }
            } else if (eeprom->tick >= 17) {
                sda = 0;
            }
        } else {
            logout("\tsda changed with raising scl\n");
        }
    } else {
        logout("%u: scl = %u->%u, sda = %u->%u\n", eeprom->tick, eeprom->scl, scl, eeprom->sda, sda);
    }
    eeprom->scl = scl;
    eeprom->sda = sda;
}
