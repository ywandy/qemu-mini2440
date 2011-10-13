/*
 * mini2440 development board support
 *
 * Copyright Michel Pollet <buserror@gmail.com>
 *
 * This code is licensed under the GNU GPL v2.
 */

#include "hw.h"
#include "s3c.h"
#include "arm-misc.h"
#include "sysemu.h"
#include "i2c.h"
#include "qemu-timer.h"
#include "devices.h"
#include "audio/audio.h"
#include "boards.h"
#include "console.h"
#include "usb.h"
#include "net.h"
#include "sd.h"
#include "dm9000.h"
#include "eeprom24c0x.h"

#define mini2440_printf(format, ...)	\
    fprintf(stderr, "QEMU %s: " format, __FUNCTION__, ##__VA_ARGS__)

#define MINI2440_GPIO_BACKLIGHT		S3C_GPG(4)
#define MINI2440_GPIO_LCD_RESET		S3C_GPC(6)
#define MINI2440_GPIO_nSD_DETECT	S3C_GPG(8)
#define MINI2440_GPIO_WP_SD			S3C_GPH(8)
#define MINI2440_GPIO_DM9000		S3C_GPF(7)
#define MINI2440_GPIO_USB_PULLUP	S3C_GPC(5)

#define MINI2440_IRQ_nSD_DETECT		S3C_EINT(16)
#define MINI2440_IRQ_DM9000			S3C_EINT(7)

#define FLASH_NOR_SIZE (2*1024*1024)

#define BOOT_NONE	0
#define BOOT_NOR	1
#define BOOT_NAND	2

struct mini2440_board_s {
    struct s3c_state_s *cpu;
    unsigned int ram;
    struct ee24c08_s * eeprom;
    const char * kernel;
    SDState * mmc;
    NANDFlashState *nand;
    pflash_t * nor;
    int bl_level;
    int boot_mode;
};

/*
 * the 24c08 sits on 4 addresses on the bus, and uses the lower address bits
 * to address the 256 byte "page" of the eeprom. We thus need to use 4 i2c_slaves
 * and keep track of which one was used to set the read/write pointer into the data
 */
struct ee24c08_s;
typedef struct ee24cxx_page_s {
	i2c_slave i2c;
	struct ee24c08_s * eeprom;
	uint8_t page;
} ee24cxx_page_s;
typedef struct ee24c08_s {
	/* that memory takes 4 addresses */
	i2c_slave * slave[4];
	uint16_t ptr;
	uint16_t count;
	uint8_t data[1024];
} ee24c08;

static void ee24c08_event(i2c_slave *i2c, enum i2c_event event)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);

    if (!s->eeprom)
    	return;

    s->eeprom->ptr = s->page * 256;
    s->eeprom->count = 0;
}

static int ee24c08_tx(i2c_slave *i2c, uint8_t data)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);

    if (!s->eeprom)
    	return 0;
    if (s->eeprom->count++ == 0) {
    	/* first byte is address offset */
        s->eeprom->ptr = (s->page * 256) + data;
    } else {
    	mini2440_printf("write %04x=%02x\n", s->eeprom->ptr, data);
    	s->eeprom->data[s->eeprom->ptr] = data;
        s->eeprom->ptr = (s->eeprom->ptr & ~0xff) | ((s->eeprom->ptr + 1) & 0xff);
        s->eeprom->count++;
    }
    return 0;
}

static int ee24c08_rx(i2c_slave *i2c)
{
    ee24cxx_page_s *s = FROM_I2C_SLAVE(ee24cxx_page_s, i2c);
    uint8_t res;
    if (!s->eeprom)
    	return 0;

    res =  s->eeprom->data[s->eeprom->ptr];

    s->eeprom->ptr = (s->eeprom->ptr & ~0xff) | ((s->eeprom->ptr + 1) & 0xff);
    s->eeprom->count++;
    return res;
}

static void ee24c08_save(QEMUFile *f, void *opaque)
{
	struct ee24c08_s *s = (struct ee24c08_s *) opaque;
	int i;

    qemu_put_be16s(f, &s->ptr);
    qemu_put_be16s(f, &s->count);
    qemu_put_buffer(f, s->data, sizeof(s->data));

	for (i = 0; i < 4; i++)
		i2c_slave_save(f, s->slave[i]);
}

static int ee24c08_load(QEMUFile *f, void *opaque, int version_id)
{
	struct ee24c08_s *s = (struct ee24c08_s *) opaque;
	int i;

    qemu_get_be16s(f, &s->ptr);
    qemu_get_be16s(f, &s->count);
    qemu_get_buffer(f, s->data, sizeof(s->data));

	for (i = 0; i < 4; i++)
		i2c_slave_load(f, s->slave[i]);
    return 0;
}

static void ee24c08_page_init(i2c_slave *i2c)
{
	/* nothing to do here */
}

static I2CSlaveInfo ee24c08_info = {
    .init = ee24c08_page_init,
    .event = ee24c08_event,
    .recv = ee24c08_rx,
    .send = ee24c08_tx
};

static void ee24c08_register_devices(void)
{
    i2c_register_slave("24C08", sizeof(ee24cxx_page_s), &ee24c08_info);
}

device_init(ee24c08_register_devices);

static ee24c08 * ee24c08_init(i2c_bus * bus)
{
	ee24c08 *s = qemu_malloc(sizeof(ee24c08));
	int i = 0;

	printf("QEMU: %s\n", __FUNCTION__);

	memset(s->data, 0xff, sizeof(s->data));

	for (i = 0; i < 4; i++) {
		DeviceState *dev = i2c_create_slave(bus, "24C08", 0x50 + i);
		if (!dev) {
			mini2440_printf("failed address %02x\n", 0x50+i);
		}
		s->slave[i] = I2C_SLAVE_FROM_QDEV(dev);
		ee24cxx_page_s *ss = FROM_I2C_SLAVE(ee24cxx_page_s, s->slave[i]);
		ss->page = i;
		ss->eeprom = s;
	}
    register_savevm("ee24c08", -1, 0, ee24c08_save, ee24c08_load, s);
    return s;
}

/* Handlers for output ports */
static void mini2440_bl_switch(void *opaque, int line, int level)
{
	printf("QEMU: %s: LCD Backlight now %s (%d).\n", __FUNCTION__, level ? "on" : "off", level);
}

static void mini2440_bl_intensity(int line, int level, void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;

    if ((level >> 8) != s->bl_level) {
        s->bl_level = level >> 8;
        printf("%s: LCD Backlight now at %04x\n", __FUNCTION__, level);
    }
}

static void mini2440_gpio_setup(struct mini2440_board_s *s)
{
	/* set the "input" pin values */
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(13), 1);
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(14), 1);
	s3c_gpio_set_dat(s->cpu->io, S3C_GPG(15), 0);

    s3c_gpio_out_set(s->cpu->io, MINI2440_GPIO_BACKLIGHT,
                    *qemu_allocate_irqs(mini2440_bl_switch, s, 1));

    s3c_timers_cmp_handler_set(s->cpu->timers, 1, mini2440_bl_intensity, s);

    /* Register the SD card pins to the lower SD driver */
 	sd_set_cb(s->mmc,
 			s3c_gpio_in_get(s->cpu->io)[MINI2440_GPIO_WP_SD],
			qemu_irq_invert(s3c_gpio_in_get(s->cpu->io)[MINI2440_IRQ_nSD_DETECT]));

}

#if 0
static void hexdump(const void* address, uint32_t len)
{
    const unsigned char* p = address;
    int i, j;

    for (i = 0; i < len; i += 16) {
	for (j = 0; j < 16 && i + j < len; j++)
	    fprintf(stderr, "%02x ", p[i + j]);
	for (; j < 16; j++)
	    fprintf(stderr, "   ");
	fprintf(stderr, " ");
	for (j = 0; j < 16 && i + j < len; j++)
	    fprintf(stderr, "%c", (p[i + j] < ' ' || p[i + j] > 0x7f) ? '.' : p[i + j]);
	fprintf(stderr, "\n");
    }
}
#endif

static int mini2440_load_from_nand(NANDFlashState *nand,
		uint32_t nand_offset, uint32_t s3c_base_offset, uint32_t size)
{
	uint8_t buffer[512];
	uint32_t src = 0;
	int page = 0;
	uint32_t dst = 0;

	if (!nand)
		return 0;

	for (page = 0; page < (size / 512); page++, src += 512 + 16, dst += 512) {
		if (nand_readraw(nand, nand_offset + src, buffer, 512)) {
			cpu_physical_memory_write(s3c_base_offset + dst, buffer, 512);
		} else {
			mini2440_printf("failed to load nand %d:%d\n",
			        nand_offset + src, 512 + 16);
			return 0;
		}
	}
	return (int) size;
}

static void mini2440_reset(void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;
    int32_t image_size;

    s->cpu->env->regs[15] = 0;

	if (s->boot_mode == BOOT_NAND) {
		/*
		 * Normally we would load 4 KB of nand to SRAM and jump there, but
		 * it is not working perfectly as expected, so we cheat and load
		 * it from nand directly relocated to 0x33f80000 and jump there
		 */
		if (mini2440_load_from_nand(s->nand, 0, S3C_RAM_BASE | 0x03f80000, 256*1024)> 0) {
			mini2440_printf("loaded default u-boot from NAND\n");
			s->cpu->env->regs[15] = S3C_RAM_BASE | 0x03f80000; /* start address, u-boot already relocated */
		}
#if 0 && defined(LATER)
		if (mini2440_load_from_nand(s->nand, 0, S3C_SRAM_BASE_NANDBOOT, S3C_SRAM_SIZE) > 0) {
			s->cpu->env->regs[15] = S3C_SRAM_BASE_NANDBOOT;	/* start address, u-boot relocating code */
			mini2440_printf("4KB SteppingStone loaded from NAND\n");
		}
#endif

		/*
		 * if a u--boot is available as a file, we always use it
		 */
		{
			image_size = load_image("mini2440/u-boot.bin", qemu_get_ram_ptr(0x03f80000));
			if (image_size < 0)
				image_size = load_image("u-boot.bin", qemu_get_ram_ptr(0x03f80000));
			if (image_size > 0) {
				if (image_size & (512 -1))	/* round size to a NAND block size */
					image_size = (image_size + 512) & ~(512-1);
				mini2440_printf("loaded override u-boot (size %x)\n", image_size);
				s->cpu->env->regs[15] = S3C_RAM_BASE | 0x03f80000;	/* start address, u-boot already relocated */
			}
		}
	}
	/*
	 * if a kernel was explicitly specified, we load it too
	 */
	if (s->kernel) {
	   	image_size = load_image(s->kernel, qemu_get_ram_ptr(0x02000000));
	   	if (image_size > 0) {
	   		if (image_size & (512 -1))	/* round size to a NAND block size */
	   			image_size = (image_size + 512) & ~(512-1);
	   		mini2440_printf("loaded %s (size %x)\n", s->kernel, image_size);
	    }
	}

}

/* Typical touchscreen calibration values */
static const int mini2440_ts_scale[6] = {
    0, (90 - 960) * 256 / 1021, -90 * 256 * 32,
    (940 - 75) * 256 / 1021, 0, 75 * 256 * 32,
};

static void
mini2440_init(
		ram_addr_t ram_size,
		const char *boot_device,
		const char *kernel_filename,
		const char *kernel_cmdline,
		const char *initrd_filename,
		const char *cpu_model)
{
    struct mini2440_board_s *mini =
    		(struct mini2440_board_s *)qemu_mallocz(sizeof(struct mini2440_board_s));
    int sd_idx = drive_get_index(IF_SD, 0, 0);
    int nor_idx = drive_get_index(IF_PFLASH, 0, 0);
    int nand_idx = drive_get_index(IF_MTD, 0, 0);
    int nand_cid = 0x76;		// 128MB flash == 0xf1

    mini->ram = 0x04000000;
    mini->kernel = kernel_filename;
    mini->mmc = sd_idx >= 0 ? sd_init(drives_table[sd_idx].bdrv, 0) : NULL;
    mini->boot_mode = BOOT_NAND;

    if (cpu_model && strcmp(cpu_model, "arm920t")) {
    	mini2440_printf("This platform requires an ARM920T core\n");
        exit(2);
    }

    uint32_t sram_base = 0;
    /*
     * Use an environment variable to set the boot mode "switch"
     */
    const char * boot_mode = getenv("MINI2440_BOOT");

    if (boot_mode) {
    	if (!strcasecmp(boot_mode, "nor")) {
    		if (nor_idx < 0) {
    			printf("%s MINI2440_BOOT(nor) error, no flash file specified", __func__);
    			abort();
    		} else
    		    mini->boot_mode = BOOT_NOR;
    	} else if (!strcasecmp(boot_mode, "nand")) {
    		if (nor_idx < 0) {
    			printf("%s MINI2440_BOOT(nand) error, no flash file specified", __func__);
    			abort();
    		} else
    		    mini->boot_mode = BOOT_NOR;
    	} else
			printf("%s MINI2440_BOOT(%s) ignored, invalid value", __func__, boot_mode);
    }
    printf("%s: Boot mode: %s\n", __func__, mini->boot_mode == BOOT_NOR ? "NOR": "NAND");
    /* Check the boot mode */
    switch (mini->boot_mode) {
    	case BOOT_NAND:
    	    sram_base = S3C_SRAM_BASE_NANDBOOT;
    	    int size = bdrv_getlength(drives_table[nand_idx].bdrv);
    	    switch (size) {
    	    	case 2 * 65536 * (512 + 16):
					nand_cid = 0x76;
    	    		break;
    	    	case 4 * 65536 * (512 + 16):
    	    		nand_cid = 0xf1;
					break;
    	    	default:
    	    		printf("%s: Unknown NAND size/id %d (%dMB) defaulting to old 64MB\n",
    	    				__func__, size, ((size / (512 + 16)) * 512) / 1024 / 1024);
    	    		break;
    	    }
    	    break;
    	case BOOT_NOR: {
    	    sram_base = S3C_SRAM_BASE_NORBOOT;
    	    int nor_size = bdrv_getlength(drives_table[nor_idx].bdrv);
    	    if (nor_size > FLASH_NOR_SIZE)
    		    printf("%s: file too big (2MBytes)\n", __func__);
    	    printf("%s: Register parallel flash %d size 0x%x '%s'\n", __func__,
    			    nor_idx, nor_size,
    			    bdrv_get_device_name(drives_table[nor_idx].bdrv));
    	}	break;
    }

    mini->cpu = s3c24xx_init(S3C_CPU_2440, 12000000 /* 12 mhz */, mini->ram, sram_base, mini->mmc);

    /* Setup peripherals */
    mini2440_gpio_setup(mini);

	mini->eeprom = ee24c08_init(s3c_i2c_bus(mini->cpu->i2c));

	{
		NICInfo* nd;
		nd = &nd_table[0];
		if (!nd->model)
		    nd->model = "dm9000";
		if (strcmp(nd->model, "dm9000") == 0) {
			dm9000_init(nd, 0x20000000, 0x300, 0x304, s3c_gpio_in_get(mini->cpu->io)[MINI2440_IRQ_DM9000]);
		}
	}

    s3c_adc_setscale(mini->cpu->adc, mini2440_ts_scale);

    /* Setup initial (reset) machine state */
    qemu_register_reset(mini2440_reset, mini);

	// Original 64MB NAND (obsolete part)
 //   mini->nand = nand_init(NAND_MFR_SAMSUNG, 0x76);
	// 128MB nand -- the hardware can also have 256, 1GB parts
    mini->nand = nand_init(NAND_MFR_SAMSUNG, nand_cid);
    mini->cpu->nand->reg(mini->cpu->nand, mini->nand);

	mini->nor = pflash_cfi02_register(0, 
		qemu_ram_alloc(FLASH_NOR_SIZE),
		nor_idx != -1 ? drives_table[nor_idx].bdrv : NULL, (64 * 1024),
		FLASH_NOR_SIZE >> 16,
		1, 2, 0x0000, 0x0000, 0x0000, 0x0000,
		0x555, 0x2aa);

    mini2440_reset(mini);
}

QEMUMachine mini2440_machine = {
    "mini2440",
    "MINI2440 Chinese Samsung SoC dev board (S3C2440A)",
    .init = mini2440_init,
};

