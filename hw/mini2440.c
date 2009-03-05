/*
 * Neo1973 mobile telephone platforms emulation.
 * Detailed information at openmoko.org.
 *
 * Copyright (c) 2007 OpenMoko, Inc.
 * Author: Andrzej Zaborowski <andrew@openedhand.com>
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
    fprintf(stderr, "%s: " format, __FUNCTION__, ##__VA_ARGS__)

/* Wiring common to all revisions */
#define MINI2440_GPIO_BACKLIGHT	S3C_GPB(1)
#define MINI2440_GPIO_LCD_RESET	S3C_GPC(6)
#define MINI2440_GPIO_nSD_DETECT	S3C_GPG(8)
#define MINI2440_IRQ_nSD_DETECT	S3C_EINT(16)
#define MINI2440_IRQ_DM9000		S3C_EINT(7)
#define MINI2440_GPIO_DM9000		S3C_GPF(7)

#define MINI2440_GPIO_SDMMC_ON	S3C_GPB(2)
#define MINI2440_GPIO_USB_PULLUP	S3C_GPB(9)
#define MINI2440_GPIO_USB_ATTACH	S3C_GPB(10)

struct mini2440_board_s {
    struct s3c_state_s *cpu;
    unsigned int ram;
    i2c_slave * eeprom;
    const char * kernel;
    SDState * mmc;
    struct nand_flash_s *nand;
    int bl_level;
};


/* Handlers for output ports */
static void mini2440_bl_switch(void *opaque, int line, int level)
{
	printf("%s: LCD Backlight now %s.\n", __FUNCTION__, level ? "on" : "off");
}

static void mini2440_bl_intensity(int line, int level, void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;

    if ((level >> 8) != s->bl_level) {
        s->bl_level = level >> 8;
     //   printf("%s: LCD Backlight now at %i/64.\n", __FUNCTION__, s->bl_level);
    }
}

static void mini2440_gpio_setup(struct mini2440_board_s *s)
{
    s3c_gpio_out_set(s->cpu->io, MINI2440_GPIO_BACKLIGHT,
                    *qemu_allocate_irqs(mini2440_bl_switch, s, 1));

    s3c_timers_cmp_handler_set(s->cpu->timers, 1, mini2440_bl_intensity, s);

    // this confuses the kernel, we will need a way to bridge this IRQ to the SD system
    // right now without this, qemu will not know how to pass the SD card insert/remove
    // properly to the kernel
//	sd_set_cb(s->mmc, 0, s3c_gpio_in_get(s->cpu->io)[MINI2440_IRQ_nSD_DETECT]);
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

static void mini2440_reset(void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;
    uint32_t image_size;

	if (1) {
	    image_size = load_image("mini2440/u-boot.bin", phys_ram_base + 0x03f80000);
	    if (!image_size)
		    image_size = load_image("u-boot.bin", phys_ram_base + 0x03f80000);
	   	if (image_size) {
	   		if (image_size & (512 -1))	/* round size to a NAND block size */
	   			image_size = (image_size + 512) & ~(512-1);
	        fprintf(stderr, "%s: loaded default u-boot (size %x)\n", __FUNCTION__, image_size);
		    s->cpu->env->regs[15] = S3C_RAM_BASE | 0x03f80000;	// start address, u-boot already relocated
	   	}
	}
#if 0
    /*
     * Performs Samsung S3C2440 bootup, but loading 4KB of the nand at the base of the RAM
     * and jumping there
     */
    if (s->nand) {
    	uint32_t src = 0;
	    int page = 0;
	    uint8_t stone[S3C_SRAM_SIZE];
	    uint8_t * dst = stone;

	    fprintf(stderr, "%s: attempting boot from NAND\n", __FUNCTION__);

	    for (page = 0; page < (S3C_SRAM_SIZE / 512); page++, src += 512+16, dst += 512)
	    	if (nand_readraw(s->nand, src, dst, 512) == 0) {
	      		fprintf(stderr, "%s: failed to load nand %d:%d\n", __FUNCTION__, src, 512+16);
	    	}
		cpu_physical_memory_write(S3C_SRAM_BASE_NANDBOOT, stone, S3C_SRAM_SIZE);
	    s->cpu->env->regs[15] = S3C_SRAM_BASE_NANDBOOT;	// start address, u-boot relocating code
	    fprintf(stderr, "%s: 4KB SteppingStone loaded from NAND\n", __FUNCTION__);
    }
#endif
	if (1) {
	   	image_size = load_image(s->kernel, phys_ram_base + 0x02000000);
	   	if (image_size) {
	   		if (image_size & (512 -1))	/* round size to a NAND block size */
	   			image_size = (image_size + 512) & ~(512-1);
	        fprintf(stderr, "%s: loaded %s (size %x)\n", __FUNCTION__, s->kernel, image_size);
	    }
	}
	if (0) {
	   	image_size = load_image("/tftpboot/minimalist-image-mini2440.jffs2", phys_ram_base);
	   	if (image_size) {
	   		if (image_size & (512 -1))	/* round size to a NAND block size */
	   			image_size = (image_size + 512) & ~(512-1);
	        fprintf(stderr, "%s: loaded jffs2 (size %x)\n", __FUNCTION__, image_size);
	    }
	}

}

/* Typical touchscreen calibration values */
static const int mini2440_ts_scale[6] = {
    0, (90 - 960) * 256 / 1021, -90 * 256 * 32,
    (940 - 75) * 256 / 1021, 0, 75 * 256 * 32,
};

/* Board init.  */
static struct mini2440_board_s *mini2440_init_common(int ram_size,
                const char *kernel_filename, const char *cpu_model,
                SDState *mmc)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *)
            qemu_mallocz(sizeof(struct mini2440_board_s));

    s->ram = 0x04000000;
    s->kernel = kernel_filename;
    s->mmc = mmc;

    /* Setup CPU & memory */
    if (ram_size < s->ram + S3C_SRAM_SIZE) {
        fprintf(stderr, "This platform requires %i bytes of memory (not %d)\n",
                        s->ram + S3C_SRAM_SIZE, ram_size);
        exit(1);
    }
    if (cpu_model && strcmp(cpu_model, "arm920t")) {
        fprintf(stderr, "This platform requires an ARM920T core\n");
        exit(2);
    }
    s->cpu = s3c24xx_init(S3C_CPU_2440, s->ram, S3C_SRAM_BASE_NANDBOOT, s->mmc);

    /* Setup peripherals */
    mini2440_gpio_setup(s);

  //  s->eeprom = eeprom24c0x_new(EE_24C08);

//    if (usb_enabled)
//        usb_device_attach(usb_bt_init(local_piconet));

	{
		NICInfo* nd;
		nd = &nd_table[0];
		if (!nd->model)
		    nd->model = "dm9000";
		if (strcmp(nd->model, "dm9000") == 0) {
			dm9000_init(nd, 0x20000000, 0x300, 0x304, s3c_gpio_in_get(s->cpu->io)[MINI2440_IRQ_DM9000]);
		}
	}

    s3c_adc_setscale(s->cpu->adc, mini2440_ts_scale);

    /* Setup initial (reset) machine state */
    qemu_register_reset(mini2440_reset, s);
#if 0
    arm_load_kernel(s->ram, kernel_filename, kernel_cmdline,
                    initrd_filename, 0x49e, S3C_RAM_BASE);
#endif

    return s;
}

static void mini2440_init(ram_addr_t ram_size, int vga_ram_size,
                const char *boot_device,
                const char *kernel_filename, const char *kernel_cmdline,
                const char *initrd_filename, const char *cpu_model)
{
    struct mini2440_board_s *mini;
    int sd_idx = drive_get_index(IF_SD, 0, 0);
    SDState *sd = 0;

    if (sd_idx >= 0)
        sd = sd_init(drives_table[sd_idx].bdrv, 0);

    mini = mini2440_init_common(ram_size,
                    kernel_filename, cpu_model, sd);

	mini->nand = nand_init(NAND_MFR_SAMSUNG, 0x36);
    mini->cpu->nand->reg(mini->cpu->nand, mini->nand);

    mini2440_reset(mini);
}

QEMUMachine mini2440_machine = {
    "mini2440",
    "MINI2440 Chinese Samsung SoC dev board (S3C2440A)",
    .init = mini2440_init,
    .ram_require = (0x04000000 + S3C_SRAM_SIZE) | RAMSIZE_FIXED
};

