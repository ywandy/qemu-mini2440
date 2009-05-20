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

struct s3c2440_nand_s {
	struct s3c_nand_driver_s driver;

    /* NAND Flash controller */
    target_phys_addr_t nand_base;
    NANDFlashState *nand;

    uint16_t nfconf;
    uint16_t nfcont;
    uint8_t nfcmd;
    uint32_t nfaddr;
    ECCState nfecc;
    ECCState nfsecc;	/* spare area */
    int nfwp;

    uint32_t nfaddr_cur;
    uint32_t nfsblk;
    uint32_t nfeblk;

};

extern struct s3c_state_s *g_s3c;

/* NAND Flash controller */
#define S3C_NFCONF		0x00	/* NAND Flash Configuration register */

#define S3C_NFCONT		0x04	/* NAND Flash Configuration register */
#define S3C_NFCONT_MODE		(1 << 0)	/* NAND flash controller operating mode */
#define S3C_NFCONT_CE		(1 << 1)	/* NAND Flash Memory nFCE signal control 0: active) */
#define S3C_NFCONT_INITECC	(1 << 4)	/* Initialize ECC decoder/encoder(Write-only) */
#define S3C_NFCONT_MECCL	(1 << 5)	/* Lock Main data area ECC generation */
#define S3C_NFCONT_SECCL	(1 << 6)	/* Lock spare area ECC generation */
#define S3C_NFCONT_SLOCK	(1 << 12)	/* Soft Lock configuration */
#define S3C_NFCONT_TLOCK	(1 << 13)	/* Lock-tight configuration */
#define S3C_NFCONT_LOCK (S3C_NFCONT_SLOCK | S3C_NFCONT_TLOCK)

#define S3C_NFCMD		0x08	/* NAND Flash Command Set register */
#define S3C_NFADDR		0x0c	/* NAND Flash Address Set register */
#define S3C_NFDATA		0x10	/* NAND Flash Data register */

#define S3C_NFMECCD0	0x14	/* NAND Flash ECC register : 1st and 2d ecc*/
#define S3C_NFMECCD1	0x18	/* NAND Flash ECC register : 3rd and 4th (unimplemented) */
#define S3C_NFSECCD		0x1c	/* NAND Flash ECC register Spare Area */

#define S3C_NFSTAT		0x20	/* NAND Flash Operation Status register */

#define S3C_NFESTAT0	0x24	/* NAND flash ECC Status register for I/O [7:0] (unimplemented)*/
#define S3C_NFESTAT1	0x28	/* NAND flash ECC Status register for I/O [15:8] (unimplemented)*/

#define S3C_NFMECC0		0x2c	/* NAND flash ECC register for data[7:0] (unimplemented)*/
#define S3C_NFMECC1		0x30	/* NAND flash ECC register for data[15:8] (unimplemented)*/
#define S3C_NFSECC		0x34	/* NAND flash ECC register for I/O [15:0] (spare area) (unimplemented)*/

#define S3C_NFSBLK		0x38	/* NAND flash programmable start block address (unimplemented)*/
#define S3C_NFEBLK		0x3c	/* NAND flash programmable end block address (unimplemented)*/

static void s3c2440_nand_reset(void * opaque)
{
	struct s3c2440_nand_s *s = (struct s3c2440_nand_s *)opaque;
    s->nfconf = 0x1000 |
		(0 << 3) |		/* NCON0: not an advanced flash */
		(1 << 2) | 		/* GPG13: 0: 256 Word/page, 1:   512 Bytes/page */
		(1 << 1) |		/* GPG14: 0: 3 address cycle 1: 4 address cycle */
		(0 << 2) 		/* GPG15: 0: 8-bit bus 1: 16-bit bus */
		;
    s->nfcont = 0x0384;
    s->nfcmd = 0;
    s->nfaddr = 0;
    s->nfsblk = 0;
    s->nfeblk = 0;
    ecc_reset(&s->nfecc);
    ecc_reset(&s->nfsecc);
}

static uint8_t dbu[16],cmd;

static uint32_t s3c2440_nand_read(void *opaque, target_phys_addr_t addr)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    int rb, shr = 0;
    if (!s->nand)
        return 0;

    switch (addr) {
    case S3C_NFCONF:
        return s->nfconf;
    case S3C_NFCONT:
        return s->nfcont;
    case S3C_NFCMD:
        return s->nfcmd;
    case S3C_NFADDR:
        return s->nfaddr >> 24; // last 8 bits poked
    case S3C_NFDATA:
        if (s->nfcont & S3C_NFCONT_MODE) {
            uint32_t value = nand_getio(s->nand);

        	if (s->nfaddr_cur < 512) {
        		if (!(s->nfcont & S3C_NFCONT_MECCL)) {
        			value = ecc_digest(&s->nfecc, value & 0xff);
        		/*	printf("ecc %02x -> %08x %08x cp %08x cnt %d\n", value, s->nfecc.lp[0], s->nfecc.lp[1], s->nfecc.cp, s->nfecc.count); */
        		}
        	} else {
        		if (!(s->nfcont & S3C_NFCONT_SECCL))
        			value = ecc_digest(&s->nfsecc, value & 0xff);
        	}
        	if (s->nfaddr_cur < 16) dbu[s->nfaddr_cur] = value;
        	s->nfaddr_cur++;
            return value;
        }
        break;
    case S3C_NFSTAT:
        nand_getpins(s->nand, &rb);
        return rb;
    case S3C_NFMECCD0 + 3: shr += 8;
    case S3C_NFMECCD0 + 2: shr += 8;
    case S3C_NFMECCD0 + 1: shr += 8;
    case S3C_NFMECCD0: {
#define ECC(shr, b, shl)	((s->nfecc.lp[b] << (shl - shr)) & (1 << shl))
        uint32 ecc = ~(
            ECC(0, 1, 0)	| ECC(0, 0, 1)	| ECC(1, 1, 2)	| ECC(1, 0, 3)	| ECC(2, 1, 4)	| ECC(2, 0, 5)	| ECC(3, 1, 6)	| ECC(3, 0, 7)	|
            ECC(4, 1, 8)	| ECC(4, 0, 9)	| ECC(5, 1, 10)	| ECC(5, 0, 11)	| ECC(6, 1, 12)	| ECC(6, 0, 13)	| ECC(7, 1, 14)	| ECC(7, 0, 15)	|
            ECC(8, 1, 16)	| ECC(8, 0, 17)	| ((s->nfecc.cp & 0x3f) << 18) |
            ECC(9, 1, 28)	| ECC(9, 0, 29)	| ECC(10, 1, 30)	| ECC(10, 0, 31)
            );
      /*  printf("Read ECC %02x = %08x >> %d [ 0: %08x 1: %08x cnt %d ]\n", addr, ecc, shr, s->nfecc.lp[0], s->nfecc.lp[1], s->nfecc.count); */
		ecc = (ecc >> shr) & 0xff;
        return ecc;
    }	break;
    case S3C_NFMECCD1 + 3:
    case S3C_NFMECCD1 + 2:
    case S3C_NFMECCD1 + 1:
    case S3C_NFMECCD1:
    	/* the ecc digester is limited to 2, the s3c2440 can do 4... */
        printf("%s: Bad register S3C_NFECCD1 NOT HANDLED\n", __FUNCTION__);
    	break;
#undef ECC

    case S3C_NFSECCD + 3: shr += 8;
    case S3C_NFSECCD + 2: shr += 8;
    case S3C_NFSECCD + 1: shr += 8;
    case S3C_NFSECCD:
#define ECC(shr, b, shl)	((s->nfsecc.lp[b] << (shl - shr)) & (1 << shl))
        return (~(
            ECC(0, 1, 0)	| ECC(0, 0, 1)	| ECC(1, 1, 2)	| ECC(1, 0, 3)	| ECC(2, 1, 4)	| ECC(2, 0, 5)	| ECC(3, 1, 6)	| ECC(3, 0, 7)	|
            ECC(4, 1, 8)	| ECC(4, 0, 9)	| ECC(5, 1, 10)	| ECC(5, 0, 11)	| ECC(6, 1, 12)	| ECC(6, 0, 13)	| ECC(7, 1, 14)	| ECC(7, 0, 15)	|
            ECC(8, 1, 16)	| ECC(8, 0, 17)	| ((s->nfecc.cp & 0x3f) << 18) |
            ECC(9, 1, 28)	| ECC(9, 0, 29)	| ECC(10, 1, 30)	| ECC(10, 0, 31)
            ) >> shr) &
            0xff;
#undef ECC

    case S3C_NFSBLK:
    	return s->nfsblk;
    case S3C_NFEBLK:
    	return s->nfeblk;

    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)addr);
        break;
    }
    return 0;
}

/*
 * 16 and 32 bits access to NFDATA. u-boot uses the 16 bits acess, and the kernel uses
 * the 32 bits one extensively.
 */
static uint32_t s3c2440_nand_read16(void *opaque, target_phys_addr_t addr)
{
	uint32_t res = s3c2440_nand_read(opaque, addr);
	if (addr == S3C_NFDATA)
		res = (s3c2440_nand_read(opaque, addr) << 8) | (res & 0xff);
	return res;
}

static uint32_t s3c2440_nand_read32(void *opaque, target_phys_addr_t addr)
{
	uint32_t res = s3c2440_nand_read16(opaque, addr);

    if (addr == S3C_NFDATA) {
		res = (s3c2440_nand_read16(opaque, addr) << 16) | (res & 0xffff);
    }
	return res;
}

static void s3c2440_nand_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    if (!s->nand)
        return;

    static int did_write =0;

    switch (addr) {
    case S3C_NFCONF:
        s->nfconf = (s->nfconf & 0xe) | (value & 0xfff1);
        break;
    case S3C_NFCONT:
        if (value & S3C_NFCONT_INITECC) {
            ecc_reset(&s->nfecc);
            ecc_reset(&s->nfsecc);
        }
        /* tight lock is sticky */
        s->nfcont = (value & (0xffff ^ S3C_NFCONT_INITECC)) | (s->nfcont & S3C_NFCONT_TLOCK);
		break;
    case S3C_NFCMD:
        s->nfcmd = value & 0xff;
        if (s->nfcont & S3C_NFCONT_MODE) {
            nand_setpins(s->nand, 1, 0, s->nfcont & S3C_NFCONT_CE, s->nfwp, 0);
            nand_setio(s->nand, s->nfcmd);
            nand_setpins(s->nand, 0, 0, s->nfcont & S3C_NFCONT_CE, s->nfwp, 0);
        }
        break;
    case S3C_NFSTAT:	// it's OK to write to this on the 2440
		break;
    case S3C_NFADDR:
#if 0
    	if (s->nfaddr_cur) {
    		int i;
    		printf("%08x cmd %02x : %s %d bytes : ", s->nfaddr<<1, cmd, did_write ? "wrote" : "read", s->nfaddr_cur);
    		for (i=0; i < 16 && i < s->nfaddr_cur; i++) printf("%02x ", dbu[i]);
    		printf("\n");
    	}
    	did_write = 0;
    	cmd = s->nfcmd;
#endif

        s->nfaddr = (s->nfaddr >> 8) | (value << 24);
        s->nfaddr_cur = 0;
        if (s->nfcont & S3C_NFCONT_MODE) {
            nand_setpins(s->nand, 0, 1, s->nfcont & S3C_NFCONT_CE, s->nfwp, 0);
            nand_setio(s->nand, value & 0xff);
            nand_setpins(s->nand, 0, 0, s->nfcont & S3C_NFCONT_CE, s->nfwp, 0);
        }
        break;
    case S3C_NFDATA:
        if (s->nfcont & S3C_NFCONT_MODE) {
        	if (s->nfcont & S3C_NFCONT_LOCK) {
        		if (s->nfaddr_cur < (s->nfsblk << 6) ||
        				s->nfaddr_cur > (s->nfeblk << 6)) {
        			/* TODO: ADD IRQ */
        			break;
        		}
        	}
        	if (s->nfaddr_cur < 512) {
        		if (!(s->nfcont & S3C_NFCONT_MECCL)) {
        			value = ecc_digest(&s->nfecc, value & 0xff);
        		/*	printf("ecc %02x -> %08x %08x cp %08x cnt %d\n", value, s->nfecc.lp[0], s->nfecc.lp[1], s->nfecc.cp, s->nfecc.count); */
        		}
        	} else {
        		if (!(s->nfcont & S3C_NFCONT_SECCL))
        			value = ecc_digest(&s->nfsecc, value & 0xff);
        	}
        	did_write=1;
        	if (s->nfaddr_cur < 16) dbu[s->nfaddr_cur] = value;
        	s->nfaddr_cur++;
            nand_setio(s->nand, value & 0xff);
        }
        break;
    case S3C_NFSBLK:
    	s->nfsblk = value & 0xffffff;
      /*  printf("%s: S3C_NFSBLK set to 0x%x\n", __FUNCTION__, value); */
    	break;
    case S3C_NFEBLK:
    	s->nfeblk = value & 0xffffff;
      /*  printf("%s: S3C_NFEBLK set to 0x%x\n", __FUNCTION__, value); */
    	break;

    default:
        printf("%s: Bad register 0x%lx=%x\n", __FUNCTION__, (unsigned long)addr, value);
    }
}

/*
 * 16 and 32 bits access to NFDATA. u-boot uses the 16 bits acess, and the kernel uses
 * the 32 bits one extensively.
 */
static void s3c2440_nand_write16(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
	s3c2440_nand_write(opaque, addr, value);
	if (addr == S3C_NFDATA) {
		s3c2440_nand_write(opaque, addr, value >> 8);
	}
}

static void s3c2440_nand_write32(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
	s3c2440_nand_write16(opaque, addr, value);
	if (addr == S3C_NFDATA) {
		s3c2440_nand_write16(opaque, addr, value >> 16);
	}
}

static void s3c2440_nand_register(void * opaque, NANDFlashState *chip)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    s->nand = chip;
}

static void s3c2440_nand_setwp(void * opaque, int wp)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    s->nfwp = wp;
}

static CPUReadMemoryFunc *s3c2440_nand_readfn[] = {
    s3c2440_nand_read,
    s3c2440_nand_read16,
    s3c2440_nand_read32,
};

static CPUWriteMemoryFunc *s3c2440_nand_writefn[] = {
    s3c2440_nand_write,
    s3c2440_nand_write16,
    s3c2440_nand_write32,
};

static void s3c2440_nand_save(QEMUFile *f, void *opaque)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    qemu_put_be16s(f, &s->nfconf);
    qemu_put_be16s(f, &s->nfcont);
    qemu_put_8s(f, &s->nfcmd);
    qemu_put_be32s(f, &s->nfaddr);
    qemu_put_be32(f, s->nfwp);
    ecc_put(f, &s->nfecc);
}

static int s3c2440_nand_load(QEMUFile *f, void *opaque, int version_id)
{
    struct s3c2440_nand_s *s = (struct s3c2440_nand_s *) opaque;
    qemu_get_be16s(f, &s->nfconf);
    qemu_get_be16s(f, &s->nfcont);
    qemu_get_8s(f, &s->nfcmd);
    qemu_get_be32s(f, &s->nfaddr);
    s->nfwp = qemu_get_be32(f);
    ecc_get(f, &s->nfecc);
    return 0;
}

static const struct s3c_nand_driver_s s3c2440_nand_driver = {
	.reset = s3c2440_nand_reset,
	.setwp = s3c2440_nand_setwp,
	.reg = s3c2440_nand_register
};

struct s3c_nand_driver_s * s3c2440_nand_init(void)
{
	int iomemtype;
	struct s3c2440_nand_s *nand = (struct s3c2440_nand_s *)
	            qemu_mallocz(sizeof(struct s3c2440_nand_s));
	nand->driver = s3c2440_nand_driver;
	nand->nand_base = 0x4e000000;
	nand->driver.reset(nand);
	iomemtype = cpu_register_io_memory(0, s3c2440_nand_readfn, s3c2440_nand_writefn, nand);
	cpu_register_physical_memory(nand->nand_base, 0xffffff, iomemtype);
	register_savevm("s3c2440_nand", 0, 0, s3c2440_nand_save, s3c2440_nand_load, nand);
	return &nand->driver;
}
