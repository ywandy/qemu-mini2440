/* hw/dm9000.h
 *
 * DM9000 Ethernet interface
 *
 * Copyright Daniel Silverstone and Vincent Sanders
 * Copyright Michel Pollet <buserror@gmail.com>
 *
 * This file is under the terms of the GNU General Public
 * License Version 2
 */

#ifndef QEMU_HW_DM9000_H
#define QEMU_HW_DM9000_H

void dm9000_init(NICInfo *nd, target_phys_addr_t base_addr, uint32_t addr_offset,
                 uint32_t data_offset, qemu_irq irq);

#endif
