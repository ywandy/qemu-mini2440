/*
 * Samsung S3C2410A RISC Microprocessor support (ARM920T based SoC).
 *
 * Copyright (c) 2007 OpenMoko, Inc.
 * Author: Andrzej Zaborowski <andrew@openedhand.com>
 * With:	Michel Pollet <buserror@gmail.com>
 *
 * This code is licenced under the GNU GPL v2.
 */

#include "s3c.h"
#include "hw.h"

struct s3c2410_nand_s {
	struct s3c_nand_driver_s driver;

    /* NAND Flash controller */
    target_phys_addr_t nand_base;
    NANDFlashState *nand;
    uint16_t nfconf;
    uint8_t nfcmd;
    uint8_t nfaddr;
    ECCState nfecc;
    int nfwp;
};

/* NAND Flash controller */
#define S3C_NFCONF	0x00	/* NAND Flash Configuration register */
#define S3C_NFCMD	0x04	/* NAND Flash Command Set register */
#define S3C_NFADDR	0x08	/* NAND Flash Address Set register */
#define S3C_NFDATA	0x0c	/* NAND Flash Data register */
#define S3C_NFSTAT	0x10	/* NAND Flash Operation Status register */
#define S3C_NFECC	0x14	/* NAND Flash ECC register */

static void s3c2410_nand_reset(void * opaque)
{
	struct s3c2410_nand_s *s = (struct s3c2410_nand_s *)opaque;
    s->nfconf = 0;
    s->nfcmd = 0;
    s->nfaddr = 0;
    ecc_reset(&s->nfecc);
}

static uint32_t s3c2410_nand_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    int rb, shr = 0;
    if (!s->nand)
        return 0;

    switch (addr) {
    case S3C_NFCONF:
        return s->nfconf;
    case S3C_NFCMD:
        return s->nfcmd;
    case S3C_NFADDR:
        return s->nfaddr;
    case S3C_NFDATA:
        if (s->nfconf & (1 << 15))
            return ecc_digest(&s->nfecc, nand_getio(s->nand));
        break;
    case S3C_NFSTAT:
        nand_getpins(s->nand, &rb);
        return rb;
    case S3C_NFECC + 2: shr += 8;
    case S3C_NFECC + 1: shr += 8;
    case S3C_NFECC:
#define ECC(shr, b, shl)	((s->nfecc.lp[b] << (shl - shr)) & (1 << shl))
        return (~(
            ECC(0, 1, 0) | ECC(0, 0, 1) | ECC(1, 1, 2) | ECC(1, 0, 3) |
            ECC(2, 1, 4) | ECC(2, 0, 5) | ECC(3, 1, 6) | ECC(3, 0, 7) |
            ECC(4, 1, 8) | ECC(4, 0, 9) | ECC(5, 1, 10) | ECC(5, 0, 11) |
            ECC(6, 1, 12) | ECC(6, 0, 13) | ECC(7, 1, 14) | ECC(7, 0, 15) |
            ECC(8, 1, 16) | ECC(8, 0, 17) | (s->nfecc.cp << 18)) >> shr) &
            0xff;
#undef ECC
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

static void s3c2410_nand_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    if (!s->nand)
        return;

    switch (addr) {
    case S3C_NFCONF:
        s->nfconf = value & 0x9fff;
        if (value & (1 << 12))
            ecc_reset(&s->nfecc);
        break;
    case S3C_NFCMD:
        s->nfcmd = value & 0xff;
        if (s->nfconf & (1 << 15)) {
            nand_setpins(s->nand, 1, 0, (s->nfconf >> 11) & 1, s->nfwp, 0);
            nand_setio(s->nand, s->nfcmd);
            nand_setpins(s->nand, 0, 0, (s->nfconf >> 11) & 1, s->nfwp, 0);
        }
        break;
    case S3C_NFADDR:
        s->nfaddr = value & 0xff;
        if (s->nfconf & (1 << 15)) {
            nand_setpins(s->nand, 0, 1, (s->nfconf >> 11) & 1, s->nfwp, 0);
            nand_setio(s->nand, s->nfaddr);
            nand_setpins(s->nand, 0, 0, (s->nfconf >> 11) & 1, s->nfwp, 0);
        }
        break;
    case S3C_NFDATA:
        if (s->nfconf & (1 << 15))
            nand_setio(s->nand, ecc_digest(&s->nfecc, value & 0xff));
        break;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
    }
}

static void s3c2410_nand_register(void * opaque, struct nand_flash_s *chip)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    s->nand = chip;
}

static void s3c2410_nand_setwp(void * opaque, int wp)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    s->nfwp = wp;
}

static CPUReadMemoryFunc *s3c2410_nand_readfn[] = {
    s3c2410_nand_read,
    s3c2410_nand_read,
    s3c2410_nand_read,
};

static CPUWriteMemoryFunc *s3c2410_nand_writefn[] = {
    s3c2410_nand_write,
    s3c2410_nand_write,
    s3c2410_nand_write,
};

static void s3c2410_nand_save(QEMUFile *f, void *opaque)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    qemu_put_be16s(f, &s->nfconf);
    qemu_put_8s(f, &s->nfcmd);
    qemu_put_8s(f, &s->nfaddr);
    qemu_put_be32(f, s->nfwp);
    ecc_put(f, &s->nfecc);
}

static int s3c2410_nand_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c2410_nand_s *s = (struct s3c2410_nand_s *) opaque;
    qemu_get_be16s(f, &s->nfconf);
    qemu_get_8s(f, &s->nfcmd);
    qemu_get_8s(f, &s->nfaddr);
    s->nfwp = qemu_get_be32(f);
    ecc_get(f, &s->nfecc);
    return 0;
}

static const struct s3c_nand_driver_s s3c2410_nand_driver = {
	.reset = s3c2410_nand_reset,
	.setwp = s3c2410_nand_setwp,
	.reg = s3c2410_nand_register
};

struct s3c_nand_driver_s * s3c2410_nand_init(void)
{
	int iomemtype;
	struct s3c2410_nand_s *nand = (struct s3c2410_nand_s *)
	            qemu_mallocz(sizeof(struct s3c2410_nand_s));
	nand->driver = s3c2410_nand_driver;
	nand->nand_base = 0x4e000000;
	nand->driver.reset(nand);
	iomemtype = cpu_register_io_memory(0, s3c2410_nand_readfn, s3c2410_nand_writefn, nand);
	cpu_register_physical_memory(nand->nand_base, 0xffffff, iomemtype);
	register_savevm("s3c2410_nand", 0, 0, s3c2410_nand_save, s3c2410_nand_load, nand);
	return &nand->driver;
}
