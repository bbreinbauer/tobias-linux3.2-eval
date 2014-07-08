/*
 * Code for AM335X EVM.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/mfd/tps65217.h>
#include <linux/pwm_backlight.h>
#include <linux/rtc/rtc-omap.h>

/* LCD controller is similar to DA850 */
#include <video/da8xx-fb.h>

#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>

#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"

/* Header for code common to all OMAP2+ machines. */
#include "common.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

unsigned int gigabit_enable = 1;

static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
        {
                .mmc            = 1,
                .caps           = MMC_CAP_4_BIT_DATA,
                .gpio_cd        = GPIO_TO_PIN(0, 6),
                .gpio_wp        = GPIO_TO_PIN(3, 18),
                .ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
        },
        {
                .mmc            = 0,    /* will be set at runtime */
        },
        {
                .mmc            = 0,    /* will be set at runtime */
        },
        {}      /* Terminator */
};

//***************** EVM ID is necessary in some supporting SW for AM335x - 
//* To eliminate dependency, board port developers should search for 
//  functions that call am335x_evm_get_id()
//
//* For Board Port the evm id is defaulted to Beagle Bone
static int am33xx_evmid = BEAGLE_BONE_A3;

/*
* am335x_evm_get_id - returns Board Type (EVM/BB/EVM-SK ...)
*
* Note:
*       returns BEAGLE_BONE_A3 as the board port example is built 
*       around that board only.
*/
int am335x_evm_get_id(void)
{
        return am33xx_evmid;
}
EXPORT_SYMBOL(am335x_evm_get_id);


/* module pin mux structure */
struct pinmux_config {
        const char *string_name; /* signal name format */
        int val; /* Options for the mux register value */
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
        {"mmc0_dat3.mmc0_dat3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat2.mmc0_dat2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat1.mmc0_dat1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_dat0.mmc0_dat0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_clk.mmc0_clk",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mmc0_cmd.mmc0_cmd",   OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mcasp0_aclkr.mmc0_sdwp", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
        {"spi0_cs1.mmc0_sdcd",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
        {NULL, 0},
};

/* Module pin mux for mii1 */
static struct pinmux_config mii1_pin_mux[] = {
        {"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_txen.mii1_txen", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"mii1_rxdv.mii1_rxdv", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_txd3.mii1_txd3", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"mii1_txd2.mii1_txd2", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"mii1_txd1.mii1_txd1", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"mii1_txd0.mii1_txd0", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
        {"mii1_txclk.mii1_txclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_rxclk.mii1_rxclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_rxd3.mii1_rxd3", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_rxd2.mii1_rxd2", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
        {"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
        {"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
        {NULL, 0},
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*                       details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
        int i;

        for (i = 0; pin_mux->string_name != NULL; pin_mux++)
                omap_mux_init_signal(pin_mux->string_name, pin_mux->val);

}

/* Enable clkout2 */
static struct pinmux_config clkout2_pin_mux[] = {
        {"xdma_event_intr1.clkout2", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
        {NULL, 0},
};

static void mmc0_init(void)
{
        setup_pin_mux(mmc0_pin_mux);

        omap2_hsmmc_init(am335x_mmc);
        return;
}

static void mii1_init(void)
{
        setup_pin_mux(mii1_pin_mux);
        return;
}

static void __init clkout2_enable(void)
{
        struct clk *ck_32;

        ck_32 = clk_get(NULL, "clkout2_ck");
        if (IS_ERR(ck_32)) {
                pr_err("Cannot clk_get ck_32\n");
                return;
        }

        clk_enable(ck_32);

        setup_pin_mux(clkout2_pin_mux);
}

void __iomem *am33xx_emif_base;

void __iomem * __init am33xx_get_mem_ctlr(void)
{

        am33xx_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

        if (!am33xx_emif_base)
                pr_warning("%s: Unable to map DDR2 controller", __func__);

        return am33xx_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
        return am33xx_emif_base;
}

void __iomem *am33xx_gpio0_base;

/* am33xx_get_gpio0_base is needed in arch/arm/mach-omap2/sleep33xx.S */ 
void __iomem *am33xx_get_gpio0_base(void)
{
        am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);
 
        return am33xx_gpio0_base;
}

static struct resource am33xx_cpuidle_resources[] = {
        {
                .start          = AM33XX_EMIF0_BASE,
                .end            = AM33XX_EMIF0_BASE + SZ_32K - 1,
                .flags          = IORESOURCE_MEM,
        },
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
        .ddr2_pdown     = 1,
};

static struct platform_device am33xx_cpuidle_device = {
        .name                   = "cpuidle-am33xx",
        .num_resources          = ARRAY_SIZE(am33xx_cpuidle_resources),
        .resource               = am33xx_cpuidle_resources,
        .dev = {
                .platform_data  = &am33xx_cpuidle_pdata,
        },
};

static void __init am33xx_cpuidle_init(void)
{
        int ret;

        am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

        ret = platform_device_register(&am33xx_cpuidle_device);

        if (ret)
                pr_warning("AM33XX cpuidle registration failed\n");

}

/* initialize system-relevant gpios (VTT, PHY, ...) */
static void __init tobias_display_gpio_init(void)
{
    int err;
    unsigned gpio_phy = GPIO_TO_PIN(1, 0);
    unsigned gpio_vtt = GPIO_TO_PIN(1, 3);
    err = gpio_request(gpio_phy, "nReset_PHY");
    if (err == 0) {
        gpio_direction_output(gpio_phy, 1);
    }
    else {
        pr_err("Failed to set PHY nReset\n");
    }
    err = gpio_request(gpio_vtt, "nReset_PHY");
    if (err == 0) {
        gpio_direction_output(gpio_vtt, 1);
    }
    else {
        pr_err("Failed to set VTT EN\n");
    }
}

/* Called as part of board initialization, defined in MACHINE_START */
static void __init am335x_evm_init(void)
{
    am33xx_cpuidle_init();
	am33xx_mux_init(NULL);
    omap_serial_init();
    clkout2_enable();
    omap_sdrc_init(NULL, NULL);
    tobias_display_gpio_init();

   /* Beagle Bone has Micro-SD slot which doesn't have Write Protect pin */
    am335x_mmc[0].gpio_wp = -EINVAL;
	mmc0_init();

    mii1_init();
    am33xx_cpsw_init_generic(MII_MODE_ENABLE,gigabit_enable);
}

static void __init am335x_evm_map_io(void)
{
        omap2_set_globals_am33xx();
        omapam33xx_map_common_io();
}

MACHINE_START(AM335XEVM, "am335xevm")
        /* Maintainer: Texas Instruments */
        .atag_offset    = 0x100,
        .map_io         = am335x_evm_map_io,
        .init_early     = am33xx_init_early,
        .init_irq       = ti81xx_init_irq,
        .handle_irq     = omap3_intc_handle_irq,
        .timer          = &omap3_am33xx_timer,
        .init_machine   = am335x_evm_init,
MACHINE_END
