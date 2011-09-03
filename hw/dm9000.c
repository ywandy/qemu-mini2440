/* hw/dm9000.c
 *
 * DM9000 Ethernet interface
 *
 * Copyright Daniel Silverstone and Vincent Sanders
 * Copyright Michel Pollet <buserror@gmail.com>
 *
 * This file is under the terms of the GNU General Public
 * License Version 2
 */

#include <string.h>
#include "qemu-common.h"
#include "hw/irq.h"
#include "hw.h"
#include "net.h"
#include "dm9000.h"

/* Comment this out if you don't want register debug on stderr */
//#define DM9000_DEBUG

/* Comment this out if you don't want a packet dump */
//#define DM9000_DUMP_FILENAME "/tmp/dm9k_dump"

#ifdef DM9000_DEBUG
#define DM9000_DBF(X...) fprintf(stderr, X)
#else
#define DM9000_DBF(X...) if(0) fprintf(stderr, X)
#endif

#define DM9000_REG_NCR 			0x00
#define DM9000_NCR_RESET 			(1 << 0)

#define DM9000_NCR_EXT_PHY         (1<<7)
#define DM9000_NCR_WAKEEN          (1<<6)
#define DM9000_NCR_FCOL            (1<<4)
#define DM9000_NCR_FDX             (1<<3)
#define DM9000_NCR_LBK             (3<<1)

#define DM9000_NCR_RESETPROTECT		(DM9000_NCR_EXT_PHY | DM9000_NCR_WAKEEN)

#define DM9000_REG_NSR 			0x01

#define DM9000_NSR_SPEED           (1<<7)
#define DM9000_NSR_LINKST          (1<<6)
#define DM9000_NSR_WAKEST          (1<<5)
#define DM9000_NSR_TX2END          (1<<3)
#define DM9000_NSR_TX1END          (1<<2)
#define DM9000_NSR_RXOV            (1<<1)

#define DM9000_NSR_RESETPROTECT		(DM9000_NSR_WAKEST)
#define DM9000_NSR_READONLY			(DM9000_NSR_SPEED|DM9000_NSR_LINKST|(1<<4)|DM9000_NSR_RXOV|(1<<0))

#define DM9000_REG_TCR 			0x02
#define DM9000_TCR_TXREQ 			(1 << 0)

#define DM9000_REG_TSR1 		0x03
#define DM9000_REG_TSR2 		0x04
#define DM9000_REG_RCR 			0x05
#define DM9000_RCR_DIS_LONG 		(1 << 5)	/* RX Discard long frames (>1522) */
#define DM9000_RCR_DIS_CRC 			(1 << 4)	/* RX Discard bad CRC */
#define DM9000_RCR_ALL	 			(1 << 3)	/* RX Pass All Multicast */
#define DM9000_RCR_RUNT 			(1 << 2)	/* RX Pass Runt Frames (frame < 64 bytes) */
#define DM9000_RCR_PRMSC 			(1 << 1)	/* RX Promiscuous Mode */
#define DM9000_RCR_RXEN 			(1 << 0)	/* RX Enabled */

#define DM9000_REG_RSR 			0x06
#define DM9000_RSR_RF 				(1 << 7)	/* RX Runt Frame (frame < 64 bytes) */
#define DM9000_RSR_MF 				(1 << 6)	/* RX Multicast Frame */
#define DM9000_RSR_FOE 				(1 << 0)	/* RX FIFO overflow */

#define DM9000_REG_ROCR 		0x07
#define DM9000_REG_BPTR 		0x08
#define DM9000_REG_FCTR 		0x09
#define DM9000_REG_FCR 			0x0A
#define DM9000_FCR_TXP0				(1 << 7)	/* TX Pause Packet (when empty) */
#define DM9000_FCR_TXPF				(1 << 6)	/* TX Pause Packet (when full) */
#define DM9000_FCR_TXPEN			(1 << 5)	/* Force pause/unpause packets */
#define DM9000_FCR_BKPA				(1 << 4)
#define DM9000_FCR_BKPM				(1 << 3)
#define DM9000_FCR_RXPS				(1 << 2)	/* RX Pause Packet Status, latch and read to clear */
#define DM9000_FCR_RXPCS			(1 << 1)	/* RX Pause Packet Current Status */
#define DM9000_FCR_FLCE				(1 << 0)	/* Flow Control Enable */

#define DM9000_REG_EPCR 		0x0B
#define DM9000_REG_EPAR 		0x0C
#define DM9000_REG_EPDRL 		0x0D
#define DM9000_REG_EPDRH 		0x0E
#define DM9000_REG_WCR 			0x0F
#define DM9000_REG_PAR0 		0x10
#define DM9000_REG_PAR1 		0x11
#define DM9000_REG_PAR2 		0x12
#define DM9000_REG_PAR3 		0x13
#define DM9000_REG_PAR4 		0x14
#define DM9000_REG_PAR5 		0x15
#define DM9000_REG_MAR0 		0x16
#define DM9000_REG_MAR1 		0x17
#define DM9000_REG_MAR2 		0x18
#define DM9000_REG_MAR3 		0x19
#define DM9000_REG_MAR4 		0x1A
#define DM9000_REG_MAR5 		0x1B
#define DM9000_REG_MAR6 		0x1C
#define DM9000_REG_MAR7 		0x1D
#define DM9000_REG_GPCR 		0x1E
#define DM9000_REG_GPR 			0x1F
#define DM9000_REG_TRPAL 		0x22
#define DM9000_REG_TRPAH 		0x23
#define DM9000_REG_RWPAL 		0x24
#define DM9000_REG_RWPAH 		0x25
#define DM9000_REG_VIDL 		0x28
#define DM9000_REG_VIDH 		0x29
#define DM9000_REG_PIDL 		0x2A
#define DM9000_REG_PIDH 		0x2B
#define DM9000_REG_CHIPR 		0x2C
#define DM9000_REG_SMCR 		0x2F
#define DM9000_REG_MRCMDX 		0xF0
#define DM9000_REG_MRCMD 		0xF2
#define DM9000_REG_MRRL 		0xF4
#define DM9000_REG_MRRH 		0xF5
#define DM9000_REG_MWCMDX 		0xF6
#define DM9000_REG_MWCMD 		0xF8
#define DM9000_REG_MWRL 		0xFA
#define DM9000_REG_MWRH 		0xFB
#define DM9000_REG_TXPLL 		0xFC
#define DM9000_REG_TXPLH 		0xFD
#define DM9000_REG_ISR 			0xFE
#define DM9000_ISR_ROOS            (1<<3)
#define DM9000_ISR_ROS             (1<<2)
#define DM9000_ISR_PTS             (1<<1)
#define DM9000_ISR_PRS             (1<<0)
#define DM9000_ISR_CLR_STATUS      (ISR_ROOS | ISR_ROS | ISR_PTS | ISR_PRS)

#define DM9000_REG_IMR 			0xFF
#define DM9000_IMR_AUTOWRAP 		0x80


#define DM9000_MII_READ 		0x0C
#define DM9000_MII_WRITE 		0x0A

#define DM9000_MII_REG_BMCR 0x00
#define DM9000_MII_REG_STATUS 0x01
#define DM9000_MII_REG_PHYID1 0x02
#define DM9000_MII_REG_PHYID2 0x03
#define DM9000_MII_REG_ANAR 0x04
#define DM9000_MII_REG_ANLPAR 0x05
#define DM9000_MII_REG_ANER 0x06
#define DM9000_MII_REG_DSCR 0x10
#define DM9000_MII_REG_DSCSR 0x11
#define DM9000_MII_REG_10BTCSR 0x12

enum {
	DM9K_TX_FIFO_START	= 0,
	DM9K_TX_FIFO_SIZE	= (3 * 1024),

	DM9K_RX_FIFO_START	= DM9K_TX_FIFO_SIZE,
	DM9K_RX_FIFO_SIZE	= (13 * 1024),

	DM9K_FIFO_SIZE = (DM9K_TX_FIFO_SIZE + DM9K_RX_FIFO_SIZE)
};

#define DM9K_WRAP_TX_INDEX(_v) 		((_v >= DM9K_TX_FIFO_SIZE) ? (_v) - DM9K_TX_FIFO_SIZE : (_v))
#define DM9K_WRAP_RX_INDEX(_v) 		((_v >= DM9K_FIFO_SIZE) ? (_v) - DM9K_RX_FIFO_SIZE : (_v))
    /* DM9KNOTE: Assumes 16bit wiring */
#define DM9K_CLIP_TX_INDEX(_v) 		((_v) & 1 ? DM9K_WRAP_TX_INDEX((_v)+1) : (_v))
#define DM9K_CLIP_RX_INDEX(_v) 		((_v) & 1 ? DM9K_WRAP_RX_INDEX((_v)+1) : (_v))

typedef struct {
    uint32_t addr; /* address port */
    uint32_t data; /* data port */
    VLANClientState *vc;
    qemu_irq irq;
    uint8_t macaddr[6];		/* MAC address -- default to qemu, can/will be overridden by guest driver */
    uint8_t mult[8];		/* multicast filtering fields */

    uint8_t address; /* The internal magical register address */

    /*
     * Transmit buffer is the first 3KB,
     * followed by the receive buffer
     */
    uint8_t packet_buffer[DM9K_FIFO_SIZE];

    uint16_t dm9k_trpa;
    uint16_t dm9k_rwpa; /* TX Read ptr address, RX write ptr address */

	uint8_t fctr, fcr;
	uint16_t fc_high_mark;
	uint16_t fc_low_mark;

    uint16_t dm9k_mrr;
    uint16_t dm9k_mwr; /* Read and write address registers */
    uint16_t dm9k_txpl; /* TX packet length */

    uint8_t dm9k_imr, dm9k_isr; /* Interrupt mask register and status register*/
    uint8_t dm9k_ncr, dm9k_nsr; /* Network control register, network status register */
    uint8_t dm9k_rcr; /* RX Control Register */
    uint8_t dm9k_rsr; /* RX Status Register */
    uint8_t dm9k_wcr; /* Wakeup control */
    uint8_t dm9k_tcr; /* Transmission control register */
    uint8_t packet_copy_buffer[DM9K_TX_FIFO_SIZE]; /* realigned packet copy buffer */
    unsigned int packet_index:1; /* 0 == packet I, 1 == packet II */

    /* Internal MII PHY state */
    uint8_t dm9k_epcr; /* EEPROM/PHY control register */
    uint8_t dm9k_epar; /* EEPROM/PHY address register */
    uint16_t dm9k_epdr; /* EEPROM/PHY data register */
    /* MII Regs */
    uint16_t dm9k_mii_bmcr;
    uint16_t dm9k_mii_anar;
    uint16_t dm9k_mii_dscr;

} dm9000_state;

static void dm9000_save(QEMUFile *f, void *opaque)
{
    dm9000_state *s = (dm9000_state *)opaque;

    qemu_put_be32s(f, &s->addr);
    qemu_put_be32s(f, &s->data);
    qemu_put_8s(f, &s->address);
    qemu_put_buffer(f, s->macaddr, sizeof(s->macaddr));
    qemu_put_buffer(f, s->mult, sizeof(s->mult));
    qemu_put_buffer(f, s->packet_buffer, sizeof(s->packet_buffer));
    qemu_put_buffer(f, s->packet_copy_buffer, sizeof(s->packet_copy_buffer));
    qemu_put_be16s(f, &s->dm9k_trpa);
    qemu_put_be16s(f, &s->dm9k_rwpa);
    qemu_put_8s(f, &s->fctr);
    qemu_put_8s(f, &s->fcr);
    qemu_put_be16s(f, &s->fc_high_mark);
    qemu_put_be16s(f, &s->fc_low_mark);
    qemu_put_be16s(f, &s->dm9k_mrr);
    qemu_put_be16s(f, &s->dm9k_mwr);
    qemu_put_be16s(f, &s->dm9k_txpl);
    qemu_put_8s(f, &s->dm9k_imr);
    qemu_put_8s(f, &s->dm9k_isr);
    qemu_put_8s(f, &s->dm9k_ncr);
    qemu_put_8s(f, &s->dm9k_nsr);
    qemu_put_8s(f, &s->dm9k_rcr);
    qemu_put_8s(f, &s->dm9k_rsr);
    qemu_put_8s(f, &s->dm9k_wcr);
    qemu_put_8s(f, &s->dm9k_tcr);
    qemu_put_8s(f, &s->dm9k_epcr);
    qemu_put_8s(f, &s->dm9k_epar);
    qemu_put_be16s(f, &s->dm9k_epdr);
    qemu_put_be16s(f, &s->dm9k_mii_bmcr);
    qemu_put_be16s(f, &s->dm9k_mii_anar);
    qemu_put_be16s(f, &s->dm9k_mii_dscr);
}

static int dm9000_load(QEMUFile *f, void *opaque, int version_id)
{
    dm9000_state *s = (dm9000_state *)opaque;
    qemu_get_be32s(f, &s->addr);
    qemu_get_be32s(f, &s->data);
    qemu_get_8s(f, &s->address);
    qemu_get_buffer(f, s->macaddr, sizeof(s->macaddr));
    qemu_get_buffer(f, s->mult, sizeof(s->mult));
    qemu_get_buffer(f, s->packet_buffer, sizeof(s->packet_buffer));
    qemu_get_buffer(f, s->packet_copy_buffer, sizeof(s->packet_copy_buffer));
    qemu_get_be16s(f, &s->dm9k_trpa);
    qemu_get_be16s(f, &s->dm9k_rwpa);
    qemu_get_8s(f, &s->fctr);
    qemu_get_8s(f, &s->fcr);
    qemu_get_be16s(f, &s->fc_high_mark);
    qemu_get_be16s(f, &s->fc_low_mark);
    qemu_get_be16s(f, &s->dm9k_mrr);
    qemu_get_be16s(f, &s->dm9k_mwr);
    qemu_get_be16s(f, &s->dm9k_txpl);
    qemu_get_8s(f, &s->dm9k_imr);
    qemu_get_8s(f, &s->dm9k_isr);
    qemu_get_8s(f, &s->dm9k_ncr);
    qemu_get_8s(f, &s->dm9k_nsr);
    qemu_get_8s(f, &s->dm9k_rcr);
    qemu_get_8s(f, &s->dm9k_rsr);
    qemu_get_8s(f, &s->dm9k_wcr);
    qemu_get_8s(f, &s->dm9k_tcr);
    qemu_get_8s(f, &s->dm9k_epcr);
    qemu_get_8s(f, &s->dm9k_epar);
    qemu_get_be16s(f, &s->dm9k_epdr);
    qemu_get_be16s(f, &s->dm9k_mii_bmcr);
    qemu_get_be16s(f, &s->dm9k_mii_anar);
    qemu_get_be16s(f, &s->dm9k_mii_dscr);
    return 0;
}


#ifdef DM9000_DUMP_FILENAME
#include <arpa/inet.h>
static uint8_t pcap_header[24] = {
  0xA1, 0xB2, 0xC3, 0xD4, /* TCPDUMP Magic */
  0x00, 0x02, 0x00, 0x04, /* Major 2, Minor 4 */
  0x00, 0x00, 0x00, 0x00, /* Timezone offset */
  0x00, 0x00, 0x00, 0x01, /* Accuracy of timestamps */
  0x00, 0x00, 0x0C, 0x00, /* Snaplen 3KiB */
  0x00, 0x00, 0x00, 0x01, /* Ethernet frames */
};
static uint8_t nulls[8] = {0, 0, 0, 0, 0, 0, 0, 0};
static void dm9k_dump_packet(uint8_t *buf, uint32_t size)
{
  FILE* dm9k_fileh = fopen(DM9000_DUMP_FILENAME, "ab+");
  unsigned long bsize = htonl(size);
  DM9000_DBF("Dumping packet at %08x (%d bytes)\n", buf, size);
  fseek(dm9k_fileh, 0, SEEK_END);
  if(ftell(dm9k_fileh)==0) fwrite(pcap_header, 1, 24, dm9k_fileh);
  fwrite(nulls, 1, 8, dm9k_fileh);
  fwrite(&bsize, 1, 4, dm9k_fileh);
  fwrite(&bsize, 1, 4, dm9k_fileh);
  fwrite(buf, 1, size, dm9k_fileh);
  fclose(dm9k_fileh);
}
#else
#define dm9k_dump_packet(X...) do { } while(0)
#endif


static void dm9000_raise_irq(dm9000_state *s)
{
    int level = ((s->dm9k_isr & s->dm9k_imr) & 0x0f) != 0;
    //DM9000_DBF("DM9000: Set IRQ level %d (isr = %02x imr %02x\n", level, s->dm9k_isr, s->dm9k_imr);
    qemu_set_irq(s->irq, level);
}

static void dm9000_soft_reset_mii(dm9000_state *s)
{
    s->dm9k_mii_bmcr = 0x3100; /* 100Mbps, AUTONEG, FULL DUPLEX */
    s->dm9k_mii_anar = 0x01E1;
    s->dm9k_mii_dscr = 0x0410;
}

static void dm9000_soft_reset(dm9000_state *s)
{
    DM9000_DBF("DM9000: Soft Reset\n");
    s->dm9k_txpl = 0;
    s->dm9k_mrr = s->dm9k_mwr = 0;
    s->dm9k_trpa = 0;
    s->dm9k_rwpa = DM9K_RX_FIFO_START;
    s->dm9k_imr = 0;
    s->dm9k_isr = 0; /* 16 bit mode, no interrupts asserted */
    s->dm9k_tcr = 0;
    s->dm9k_rcr = 0;
    s->dm9k_rsr = 0;
    s->fcr = 0x38;
    s->fctr = (3 << 4) | (7 << 0);	// flow control high/low marks
    s->fc_high_mark = 3 * 1024;
    s->fc_low_mark = 8 * 1024;

    s->packet_index = 0;
    memset(s->packet_buffer, 0, sizeof(s->packet_buffer));
    memset(s->packet_copy_buffer, 0, sizeof(s->packet_copy_buffer));
    /* These registers have some bits "unaffected by software reset" */
    /* Clear the reset bits */
    s->dm9k_ncr &= DM9000_NCR_RESETPROTECT;
    s->dm9k_nsr &= DM9000_NSR_RESETPROTECT;
    /* Claim full duplex */
    s->dm9k_ncr |= DM9000_NCR_FDX;
    /* Set link status to 1 */
    s->dm9k_nsr |= DM9000_NSR_LINKST;
    /* dm9k_wcr is unaffected or reserved, never reset */
    /* MII control regs */
    s->dm9k_epcr = 0x00;
    s->dm9k_epar = 0x40;
    /* reset the MII */
    dm9000_soft_reset_mii(s);
    dm9000_raise_irq(s); /* Clear any potentially pending IRQ */
}

static void dm9000_hard_reset(dm9000_state *s)
{
    s->dm9k_ncr = 0x00;
    s->dm9k_nsr = 0x00;
    s->dm9k_wcr = 0x00;
    dm9000_soft_reset(s);
}

static uint16_t dm9000_get_rx_fifo_fill_state(dm9000_state *s)
{
	uint16_t res;
	if (s->dm9k_mrr >= s->dm9k_rwpa)
		res = (DM9K_FIFO_SIZE-s->dm9k_rwpa) + (s->dm9k_mrr-DM9K_RX_FIFO_START);
	else
		res = s->dm9k_rwpa - s->dm9k_mrr;
	return res;
}

static void dm9000_do_transmit(dm9000_state *s) {
	uint16_t idx, cnt, tptr;
	idx = s->dm9k_trpa;

	cnt = s->dm9k_txpl;
	if (cnt > DM9K_TX_FIFO_SIZE)
		cnt = DM9K_TX_FIFO_SIZE; /* HARD CAP AT 3KiB */

	tptr = 0;
	while (cnt--) {
		s->packet_copy_buffer[tptr++] = s->packet_buffer[idx];
		idx = DM9K_WRAP_TX_INDEX(idx+1);
	}

#ifdef DM9000_DEBUG
	{
		uint8_t *buf = &s->packet_copy_buffer[6];
		DM9000_DBF("TX_Packet: %02x:%02x:%02x:%02x:%02x:%02x %d bytes from %04x\n",
				buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],
				tptr, idx);
//		hexdump(s->packet_copy_buffer, tptr);
	}
#endif

	s->dm9k_trpa = DM9K_CLIP_TX_INDEX(idx);
	dm9k_dump_packet(s->packet_copy_buffer, s->dm9k_txpl);
	/* We have the copy buffer, now we do the transmit */
	qemu_send_packet(s->vc, s->packet_copy_buffer, s->dm9k_txpl);

	/* Clear the "please xmit" bit */
	s->dm9k_tcr &= ~DM9000_TCR_TXREQ;
	/* Set the TXEND bit */
	s->dm9k_nsr |= 1 << (2 + s->packet_index);
	DM9000_DBF("TX: NSR=%02x PI=%d\n", s->dm9k_nsr, s->packet_index);
	/* Claim a TX complete IRQ */
	s->dm9k_isr |= DM9000_ISR_PTS; /* Packet transmitted latch */
	/* And flip the next-packet bit */
	s->packet_index++;

	dm9000_raise_irq(s);
}

static void dm9000_mii_read(dm9000_state *s)
{
    int mii_reg = (s->dm9k_epar) & 0x3f;
    uint16_t ret = 0;
    switch(mii_reg) {
    case DM9000_MII_REG_BMCR:
        ret = s->dm9k_mii_bmcr;
        break;
    case DM9000_MII_REG_STATUS:
        ret = 0x782D; /* No 100/T4, Can 100/FD, Can 100/HD, Can 10/FD, Can 10/HD,
                       * No Preamble suppression, Autoneg complete, No remote fault,
                       * Can autoneg, link up, no jabber, extended capability */
        break;
    case DM9000_MII_REG_PHYID1:
        ret = 0x0181;
        break;
    case DM9000_MII_REG_PHYID2:
        ret = 0xB8C0;
        break;
    case DM9000_MII_REG_ANAR:
        ret = s->dm9k_mii_anar;
        break;
    case DM9000_MII_REG_ANLPAR:
        ret = 0x0400;
        break;
    case DM9000_MII_REG_ANER:
        ret = 0x0001;
        break;
    case DM9000_MII_REG_DSCR:
        ret = s->dm9k_mii_dscr;
        break;
    case DM9000_MII_REG_DSCSR:
        ret = 0xF008;
        break;
    case DM9000_MII_REG_10BTCSR:
        ret = 0x7800;
    default:
        printf("%s: Bad register 0x%lx\n", __FUNCTION__, (unsigned long)mii_reg);
    }
    s->dm9k_epdr = ret;
 //   DM9000_DBF("DM9000:MIIPHY: Read of MII reg %d gives %04x\n", mii_reg, s->dm9k_epdr);
}

static void dm9000_mii_write(dm9000_state *s)
{
    int mii_reg = (s->dm9k_epar) & 0x3f;
    DM9000_DBF("DM9000:MIIPHY: Write of MII reg %d value %04x\n", mii_reg, s->dm9k_epdr);
    switch(mii_reg) {
    case DM9000_MII_REG_BMCR:
        s->dm9k_mii_bmcr = (s->dm9k_epdr &~0x8000);
        if( s->dm9k_epdr & 0x8000 ) dm9000_soft_reset_mii(s);
        break;
    case DM9000_MII_REG_ANAR:
        s->dm9k_mii_anar = s->dm9k_epdr;
        break;
    case DM9000_MII_REG_DSCR:
        s->dm9k_mii_dscr = s->dm9k_epdr & ~0x0008;
        break;
    default:
        printf("%s: Bad register 0x%lx=%4x\n", __FUNCTION__, (unsigned long)mii_reg, s->dm9k_epdr);
    }
}

static void dm9000_write(void *opaque, target_phys_addr_t address,
                             uint32_t value)
{
    dm9000_state *s = (dm9000_state *)opaque;
#ifdef DM9000_DEBUG
    int suppress_debug = 0;
#endif

    if (address == s->addr) {
    //    if( (value != DM9000_REG_MRCMD) && (value != DM9000_REG_MWCMD) )
    //        DM9000_DBF("DM9000: Address set to 0x%02x\n", value);
        s->address = value;
        return;
    }

    switch(s->address) {
    case DM9000_REG_NCR:
        DM9000_DBF("DM9000_REG_NCR: %02x=%04x\n", s->address, value);
        s->dm9k_ncr = value | DM9000_NCR_FDX;
        if (s->dm9k_ncr & DM9000_NCR_RESET)
            dm9000_soft_reset(s);
        break;
    case DM9000_REG_NSR:
        s->dm9k_nsr &= ~(value & ~DM9000_NSR_READONLY);
        break;
    case DM9000_REG_TCR:
        s->dm9k_tcr = value & 0xFF;
        if ( value & DM9000_TCR_TXREQ )
        	dm9000_do_transmit(s);
        break;
    case DM9000_REG_EPCR:
        s->dm9k_epcr = value & 0xFF;
        if( value & DM9000_MII_READ )
            dm9000_mii_read(s);
        else if( value & DM9000_MII_WRITE )
            dm9000_mii_write(s);
        break;
    case DM9000_REG_RCR:
    	s->dm9k_rcr = value & 0xFF;
        break;

    case DM9000_REG_BPTR: /* can be ignored */
    	break;

    case DM9000_REG_FCTR: {/* 0x09 Flow Control Threshold Register */
    	s->fc_high_mark = ((value >> 4) & 0xf) * 1024;	// emit a pause packet time=0xffff
    	s->fc_low_mark = ((value) & 0xf) * 1024;		// emit an unpause packet time=0x0000
    	s->fctr = value;
    }	break;

    case DM9000_REG_FCR:	/* 0x0a Flow Control Register */
    	s->fcr = value;
    	break;

    case DM9000_REG_EPAR:
        s->dm9k_epar = value & 0xFF;
        break;
    case DM9000_REG_EPDRL:
        s->dm9k_epdr &= 0xFF00;
        s->dm9k_epdr |= value & 0xFF;
        break;
    case DM9000_REG_EPDRH:
        s->dm9k_epdr &= 0xFF;
        s->dm9k_epdr |= (value & 0xFF) << 8;
        break;
    case DM9000_REG_PAR0 ... DM9000_REG_PAR5:
		/* MAC address is set by the QEMU Nic */
        s->macaddr[s->address - DM9000_REG_PAR0] = value;
        break;
    case DM9000_REG_MAR0 ... DM9000_REG_MAR7:
		/* Multicast address is ignored */
		s->mult[s->address - DM9000_REG_MAR0] = value;
		break;
    case DM9000_REG_GPCR:
    case DM9000_REG_GPR:	/* General purpose reg (GPIOs, LED?) */
    	break;
    case DM9000_REG_SMCR:
    	if (value)
    		printf("%s: something playing with special mode ? 0x%lx=%x\n", __FUNCTION__, (unsigned long)s->address, value);
    	break;
    case DM9000_REG_MRRL:
        s->dm9k_mrr &= 0xFF00;
        s->dm9k_mrr |= value & 0xFF;
        break;
    case DM9000_REG_MRRH:
        s->dm9k_mrr &= 0xFF;
        s->dm9k_mrr |= (value & 0xFF) << 8;
        break;
    case DM9000_REG_MWCMDX:
    case DM9000_REG_MWCMD:
        /* DM9KNOTE: This assumes a 16bit wide wiring */
        s->packet_buffer[s->dm9k_mwr] = value & 0xFF;
		s->packet_buffer[s->dm9k_mwr + 1] = (value >> 8) & 0xFF;
		if (s->address == DM9000_REG_MWCMD) {
			if (s->dm9k_imr & DM9000_IMR_AUTOWRAP)
				s->dm9k_mwr = DM9K_WRAP_TX_INDEX(s->dm9k_mwr + 2);
			else if (s->dm9k_mwr + 2 < DM9K_TX_FIFO_SIZE) // clip it
				s->dm9k_mwr += 2;
		}
#ifdef DM9000_DEBUG
        suppress_debug = 1;
#endif
        break;
    case DM9000_REG_MWRL:
        s->dm9k_mwr &= 0xFF00;
        s->dm9k_mwr |= value & 0xFF;
        break;
    case DM9000_REG_MWRH:
        s->dm9k_mwr &= 0xFF;
        s->dm9k_mwr |= (value & 0xFF) << 8;
        break;
    case DM9000_REG_TXPLL:
        s->dm9k_txpl &= 0xFF00;
        s->dm9k_txpl |= value & 0xFF;
        break;
    case DM9000_REG_TXPLH:
        s->dm9k_txpl &= 0xFF;
        s->dm9k_txpl |= (value & 0xFF) << 8;
        break;
    case DM9000_REG_ISR:
        s->dm9k_isr &= ~(value & 0x0F);
        dm9000_raise_irq(s);
        break;
    case DM9000_REG_IMR:
        if( !(s->dm9k_imr & DM9000_IMR_AUTOWRAP) && (value & DM9000_IMR_AUTOWRAP) )
            s->dm9k_mrr = 0x0C00 | (s->dm9k_mrr & 0xFF);
        s->dm9k_imr = value & 0xFF;
        dm9000_raise_irq(s);
        break;
    default:
        printf("%s: Bad register 0x%lx=%x\n", __FUNCTION__, (unsigned long)s->address, value);
    }
#if 0 // def DM9000_DEBUG
    if(!suppress_debug) DM9000_DBF("DM9000: Write value %02x=%04x\n", s->address, value);
#endif
}

static uint32_t dm9000_read(void *opaque, target_phys_addr_t address)
{
    dm9000_state *s = (dm9000_state *)opaque;
    uint32_t ret = 0;
#ifdef DM9000_DEBUG
    int suppress_debug = 0;
#endif

    if (address == s->addr)
        return s->address;
    switch(s->address) {
    case DM9000_REG_NCR:
        ret = s->dm9k_ncr;
        break;
    case DM9000_REG_NSR:
        ret = s->dm9k_nsr;
        /* Note, TX1END and TX2END are *CLEAR ON READ* */
        s->dm9k_nsr &= ~(DM9000_NSR_TX1END | DM9000_NSR_TX2END);
        break;
    case DM9000_REG_TCR:
        ret = s->dm9k_tcr;
        break;
    case DM9000_REG_TSR1:
    case DM9000_REG_TSR2:
        ret = 0x00; /* No error, yay! */
        break;
    case DM9000_REG_EPCR:
        ret = s->dm9k_epcr;
        break;
    case DM9000_REG_RCR:
        ret = s->dm9k_rcr;
        break;
    case DM9000_REG_RSR:
        ret = s->dm9k_rsr;
        break;
    case DM9000_REG_EPAR:
        ret = s->dm9k_epar;
        break;
    case DM9000_REG_EPDRL:
        ret = s->dm9k_epdr & 0xFF;
        break;
    case DM9000_REG_EPDRH:
        ret = (s->dm9k_epdr >> 8) & 0xFF;
        break;
    case DM9000_REG_PAR0...DM9000_REG_PAR5:
        ret = s->macaddr[s->address - DM9000_REG_PAR0];
        break;
    case DM9000_REG_MAR0...DM9000_REG_MAR7:
		ret = s->mult[s->address - DM9000_REG_MAR0];
		break;
    case DM9000_REG_TRPAL:
        ret = s->dm9k_trpa & 0xFF;
        break;
    case DM9000_REG_TRPAH:
        ret = s->dm9k_trpa >> 8;
        break;
    case DM9000_REG_RWPAL:
        ret = s->dm9k_rwpa & 0xFF;
        break;
    case DM9000_REG_RWPAH:
        ret = s->dm9k_rwpa >> 8;
        break;
    case DM9000_REG_VIDL:
        ret = 0x46;
        break;
    case DM9000_REG_VIDH:
        ret = 0x0A;
        break;
    case DM9000_REG_PIDL:
        ret = 0x00;
        break;
    case DM9000_REG_PIDH:
        ret = 0x90;
        break;
    case DM9000_REG_CHIPR:
        ret = 0x00;
        break;
    case DM9000_REG_MRCMDX:
    case DM9000_REG_MRCMD:
		// drivers read the fifo looking for a 0x01 to indicate a packet is there,
		// so we just return it a zero if there is nothing to read
		if (s->dm9k_mrr == s->dm9k_rwpa)
			ret = 0;
		else {
	        /* DM9KNOTE: This assumes a 16bit wide wiring */
			ret = s->packet_buffer[s->dm9k_mrr];
			ret |= s->packet_buffer[s->dm9k_mrr + 1] << 8;
			if( s->address == DM9000_REG_MRCMD ) {
				if( s->dm9k_imr & DM9000_IMR_AUTOWRAP )
					s->dm9k_mrr = DM9K_WRAP_RX_INDEX(s->dm9k_mrr + 2);
				else if (s->dm9k_mrr + 2 < DM9K_FIFO_SIZE) // clip it
					s->dm9k_mrr += 2;
			}
		}
#ifdef DM9000_DEBUG
		if (s->address==DM9000_REG_MRCMD)
			suppress_debug = 1;
#endif
		break;
    case DM9000_REG_MRRL:
        ret = s->dm9k_mrr & 0xFF;
        break;
    case DM9000_REG_MRRH:
        ret = s->dm9k_mrr >> 8;
        break;
    case DM9000_REG_MWRL:
        ret = s->dm9k_mwr & 0xFF;
        break;
    case DM9000_REG_MWRH:
        ret = s->dm9k_mwr >> 8;
        break;
    case DM9000_REG_TXPLL:
        ret = s->dm9k_txpl & 0xFF;
        break;
    case DM9000_REG_TXPLH:
        ret = s->dm9k_txpl >> 8;
        break;
    case DM9000_REG_ISR:
        ret = s->dm9k_isr;
        break;
    case DM9000_REG_IMR:
        ret = s->dm9k_imr;
        break;
    default:
        ret = 0;
    }

#if 0 // def DM9000_DEBUG
    if(!suppress_debug) DM9000_DBF("DM9000: Read gives: %04x\n", ret);
#endif
    return ret;
}


static int dm9000_can_receive(void *opaque)
{
    dm9000_state *s = (dm9000_state *)opaque;
    uint16_t rx_space = dm9000_get_rx_fifo_fill_state(s);
    DM9000_DBF("DM9000:RX_Packet: Asked about RX, rwpa=%d mrr=%d => space is %d bytes\n",
                 s->dm9k_rwpa, s->dm9k_mrr, rx_space);
    return rx_space > 1522;
}

#define POLYNOMINAL 0x04c11db6

/* From FreeBSD */
/* XXX: optimize */
static int compute_mcast_idx(const uint8_t *ep)
{
    uint32_t crc;
    int carry, i, j;
    uint8_t b;

    crc = 0xffffffff;
    for (i = 0; i < 6; i++) {
        b = *ep++;
        for (j = 0; j < 8; j++) {
            carry = ((crc & 0x80000000L) ? 1 : 0) ^ (b & 0x01);
            crc <<= 1;
            b >>= 1;
            if (carry)
                crc = ((crc ^ POLYNOMINAL) | carry);
        }
    }
    return (crc >> 26);
}

static void dm9000_receive(void *opaque, const uint8_t *buf, int size)
{
    dm9000_state *s = (dm9000_state *)opaque;
    uint16_t rxptr = s->dm9k_rwpa;
    unsigned int mcast_idx = 0;
    int pad = 4;

    if (!(s->dm9k_rcr & DM9000_RCR_RXEN))
    	return;
    s->dm9k_rsr = 0;

    if (!(s->dm9k_rcr & DM9000_RCR_PRMSC)) {
    	if (buf[0] & 0x01) {
            /* multi/broadcast */
            if (!(s->dm9k_rcr & DM9000_RCR_ALL)) {
            	mcast_idx = compute_mcast_idx(buf);
            	if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7))))
            		return;
                s->dm9k_rsr |= DM9000_RSR_MF;
            }
        } else if (!memcmp(buf, s->macaddr, 6)) {
            /* match */
        } else {
            return;
        }
        if (size < 64 && !(s->dm9k_rcr & DM9000_RCR_RUNT)) {
        //    printf("rcr %02x RUNT %d\n", s->dm9k_rcr, size);
            s->dm9k_rsr |= DM9000_RSR_RF;
        //	return;
        }
        if (size > 1522 && (s->dm9k_rcr & DM9000_RCR_DIS_LONG))
        	return;
    }

    DM9000_DBF("DM9000:RX_Packet: %02x:%02x:%02x:%02x:%02x:%02x -> %02x:%02x:%02x:%02x:%02x:%02x : %d bytes into buffer at %04x [RCR %02x]\n",
    		buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],
    		buf[6],buf[7],buf[8],buf[9],buf[10],buf[11],
    		size, rxptr, s->dm9k_rcr);
    dm9k_dump_packet(buf, size);

    /*
     * apparently even runt frames are padded to 64
     */
    if (size < 64)
    	pad = 64 - size;

    rxptr = DM9K_CLIP_RX_INDEX(rxptr);
    /* store header */
    s->packet_buffer[rxptr] = 0x01; /* Packet read */
    rxptr = DM9K_WRAP_RX_INDEX(rxptr+1);
    s->packet_buffer[rxptr] = 0x00; /* Status OK */
    rxptr = DM9K_WRAP_RX_INDEX(rxptr+1);
    s->packet_buffer[rxptr] = (size+pad) & 0xFF; /* Size LOW */
    rxptr = DM9K_WRAP_RX_INDEX(rxptr+1);
    s->packet_buffer[rxptr] = ((size+pad) >> 8) & 0xff; /* Size HIGH */
    rxptr = DM9K_WRAP_RX_INDEX(rxptr+1);

    if (DM9K_FIFO_SIZE - rxptr > size) {
    	memcpy(s->packet_buffer + rxptr, buf, size);
		rxptr += size;
    } else {
    	int p1 = DM9K_FIFO_SIZE - rxptr;
    	memcpy(s->packet_buffer + rxptr, buf, p1);
    	buf += p1;
		rxptr = DM9K_RX_FIFO_START;	/* wrap */
		p1 = size - p1;	/* remaining */
    	memcpy(s->packet_buffer + rxptr, buf, p1);
		rxptr += p1;
    }
    /* obligatory padding */
    while (pad--) {
		s->packet_buffer[rxptr] = 0;
		rxptr = DM9K_WRAP_RX_INDEX(rxptr+1);
    }

    s->dm9k_rwpa = DM9K_CLIP_RX_INDEX(rxptr);
    s->dm9k_isr |= DM9000_ISR_PRS; /* RX interrupt, yay */
    dm9000_raise_irq(s);
}


static CPUReadMemoryFunc *dm9000_readfn[] = {
    dm9000_read,
    dm9000_read,
    dm9000_read
};

static CPUWriteMemoryFunc *dm9000_writefn[] = {
    dm9000_write,
    dm9000_write,
    dm9000_write
};

static void dm9000_cleanup(VLANClientState *vc)
{
    /* dm9000_state *s = (dm9000_state *)vc->opaque; */
}

/* initialises a dm9000 ethernet controller
 * The dm9k has a single 16bit wide address and data port through which all
 *  operations are multiplexed, there is a single IRQ
 */
void dm9000_init(NICInfo *nd, target_phys_addr_t base_addr,
                 uint32_t addr_offset, uint32_t data_offset,
                 qemu_irq irq)
{
    dm9000_state *s;
    int iomemtype;

    s = (dm9000_state *)qemu_mallocz(sizeof(dm9000_state));
    iomemtype = cpu_register_io_memory(0, dm9000_readfn,
                                       dm9000_writefn, s);
    cpu_register_physical_memory(base_addr, MAX(addr_offset, data_offset) + 4, iomemtype);
    s->addr = addr_offset;
    s->data = data_offset;
    s->irq = irq;
    memcpy(s->macaddr, nd->macaddr, 6);
    memset(s->mult, 0xff, 8);

    {
		uint8_t * buf = s->macaddr;
    	printf("DM9000: INIT QEMU MAC : %02x:%02x:%02x:%02x:%02x:%02x\n",
    		buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
    }

    register_savevm("dm9000", 0, 0, dm9000_save, dm9000_load, s);

    dm9000_hard_reset(s);

    s->vc = qemu_new_vlan_client(nd->vlan, nd->model, nd->name,
			dm9000_receive, dm9000_can_receive,
			dm9000_cleanup, s);
    qemu_format_nic_info_str(s->vc, s->macaddr);

}
