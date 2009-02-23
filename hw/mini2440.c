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

#define mini2440_printf(format, ...)	\
    fprintf(stderr, "%s: " format, __FUNCTION__, ##__VA_ARGS__)

#define GTA01Bv4		1

enum {
    MACH_MINI2440	= 1999
};

/* Wiring common to all revisions */
#define GTA01_GPIO_BACKLIGHT	S3C_GPB(1)
#define GTA01_GPIO_LCD_RESET	S3C_GPC(6)
#define GTA01_GPIO_nSD_DETECT	S3C_GPG(8)
#define GTA01_IRQ_nSD_DETECT	S3C_EINT(16)
#define GTA01_IRQ_DM9000		S3C_EINT(7)
#define GTA01_GPIO_DM9000		S3C_GPF(7)

#define GTA01_GPIO_SDMMC_ON	S3C_GPB(2)
#define GTA01_GPIO_USB_PULLUP	S3C_GPB(9)
#define GTA01_GPIO_USB_ATTACH	S3C_GPB(10)

struct mini2440_board_s {
    struct s3c_state_s *cpu;
    unsigned int ram;
    i2c_slave *eeprom;
    const char *kernel;
    SDState *mmc;
    uint32_t id;

    int bl_level;
};


static void mini2440_gpio_setup(struct mini2440_board_s *s)
{
//    s3c_gpio_out_set(s->cpu->io, GTA01_GPIO_BACKLIGHT,
//                    *qemu_allocate_irqs(mini2440_bl_switch, s, 1));

//    s3c_timers_cmp_handler_set(s->cpu->timers, 0, mini2440_bl_intensity, s);

        sd_set_cb(s->mmc, 0,
                        s3c_gpio_in_get(s->cpu->io)[GTA01_IRQ_nSD_DETECT]);
}

static void mini2440_reset(void *opaque)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *) opaque;
#if 0
    s->cpu->env->regs[15] = S3C_SRAM_BASE;
#else
    load_image("u-boot.bin", phys_ram_base + 0x03f80000);
    load_image(s->kernel, phys_ram_base + 0x02000000);

    s->cpu->env->regs[15] = S3C_RAM_BASE | 0x03f80000;
#if 0
    if (strstr(s->kernel, "u-boot")) {	/* FIXME */
        /* Exploit preboot-override to set up an initial environment */
        stl_raw(phys_ram_base + 0x03f80040, S3C_RAM_BASE | 0x000fff00);
        strcpy(phys_ram_base + 0x000fff00,
                        "setenv stdin serial; "
                        "setenv stdout serial; "
                        "setenv stderr serial; ");
        /* Disable ENV pre-load */
        stl_raw(phys_ram_base + 0x03f80044, 0x00000000);
    }
#endif
#endif

}

/* Typical touchscreen calibration values */
static const int mini2440_ts_scale[6] = {
    0, (90 - 960) * 256 / 1021, -90 * 256 * 32,
    (940 - 75) * 256 / 1021, 0, 75 * 256 * 32,
};

/* Board init.  */
static struct mini2440_board_s *mini2440_init_common(int ram_size, DisplayState *ds,
                const char *kernel_filename, const char *cpu_model,
                SDState *mmc, uint32_t id)
{
    struct mini2440_board_s *s = (struct mini2440_board_s *)
            qemu_mallocz(sizeof(struct mini2440_board_s));

    s->ram = 0x08000000;
    s->kernel = kernel_filename;
    s->mmc = mmc;
    s->id = id;

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
    s->cpu = s3c24xx_init(S3C_CPU_2440, s->ram, ds, s->mmc);

    /* Setup peripherals */
    mini2440_gpio_setup(s);

//    if (usb_enabled)
//        usb_device_attach(usb_bt_init(local_piconet));

	{
		NICInfo* nd;
		nd = &nd_table[0];
		if (!nd->model)
		    nd->model = "dm9000";
		if (strcmp(nd->model, "dm9000") == 0) {
//    		s3c_gpio_out_set(s->cpu->io, GTA01_GPIO_DM9000,
//	                    *qemu_allocate_irqs(mini2440_bl_switch, s, 1));

			dm9000_init(nd, 0x20000300, 0x00, 0x04, s3c_gpio_in_get(s->cpu->io)[7]);
		}
	}

    s3c_adc_setscale(s->cpu->adc, mini2440_ts_scale);

    /* Setup initial (reset) machine state */
    qemu_register_reset(mini2440_reset, s);
#if 0
    arm_load_kernel(s->ram, kernel_filename, kernel_cmdline,
                    initrd_filename, 0x49e, S3C_RAM_BASE);
#endif
    mini2440_reset(s);

//    dpy_resize(ds, 240, 320);

    return s;
}
//QEMUMachineInitFunc init;

static void mini2440_init(ram_addr_t ram_size, int vga_ram_size,
                const char *boot_device,
                const char *kernel_filename, const char *kernel_cmdline,
                const char *initrd_filename, const char *cpu_model)
{
    struct mini2440_board_s *neo;
    int sd_idx = drive_get_index(IF_SD, 0, 0);
    SDState *sd = 0;

    DisplayState *ds = get_displaystate();

    if (sd_idx >= 0)
        sd = sd_init(drives_table[sd_idx].bdrv, 0);

    neo = mini2440_init_common(ram_size, ds,
                    kernel_filename, cpu_model, sd, MACH_MINI2440);

    s3c_nand_register(neo->cpu, nand_init(NAND_MFR_SAMSUNG, 0xa2));


}

QEMUMachine mini2440_machine = {
    "mini2440",
    "MINI2440 Chinese Samsung SoC dev board (S3C2440A)",
    .init = mini2440_init,
};

