/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mfd/tps65023.h>
#include <linux/power_supply.h>
#include <linux/clk.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/io.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/sirc.h>
#include <mach/dma.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_hsusb_hw.h>
#include <mach/msm_serial_hs.h>
#include <mach/pmic.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_spi.h>
#include <mach/msm_tsif.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#ifdef CONFIG_BATTERY_X10_ZEUS
#include <linux/max17040.h>
#include <mach/semc_battery_data.h>
#endif
#ifdef CONFIG_SENSORS_AKM8973
#include <linux/akm8973.h>
#endif

#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#include "msm-keypad-devices.h"
#include "pm.h"
#include "smd_private.h"
#include "proc_comm.h"
#include <linux/msm_kgsl.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include "board-es209ra.h"
#include "board-es209ra-keypad.h"
#ifdef CONFIG_ES209RA_HEADSET
#include "es209ra_headset.h"
#endif
#include <linux/spi/es209ra_touch.h>
#include <asm/setup.h>
#include "qdsp6/q6audio.h"
#include <../../../drivers/video/msm/mddi_tmd_nt35580.h>
#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
#include <mach/semc_low_batt_shutdown.h>
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */

#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
#include  <linux/semc/msm_pmic_vibrator.h>
#endif

#include <linux/bma150_ng.h>

/* MDDI includes */
#include "../../../drivers/video/msm/msm_fb_panel.h"
#include "../../../drivers/video/msm/mddihost.h"


#define TOUCHPAD_SUSPEND 	34
#define TOUCHPAD_IRQ 		38

#define MSM_PMEM_MDP_SIZE	0x1C91000

#define SMEM_SPINLOCK_I2C	"S:6"

#define MSM_PMEM_ADSP_SIZE	0x2196000
#define MSM_FB_SIZE			0x500000
#define MSM_AUDIO_SIZE		0x80000
#define MSM_GPU_PHYS_SIZE 	SZ_2M

#ifdef CONFIG_MSM_SOC_REV_A
#define MSM_SMI_BASE		0xE0000000
#else
#define MSM_SMI_BASE		0x00000000
#endif

#define MSM_SHARED_RAM_PHYS	(MSM_SMI_BASE + 0x00100000)

#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + 0x02B00000)
#define MSM_PMEM_SMI_SIZE	0x01500000

#define MSM_FB_BASE		MSM_PMEM_SMI_BASE
#define MSM_GPU_PHYS_BASE 	(MSM_FB_BASE + MSM_FB_SIZE)
#define MSM_PMEM_SMIPOOL_BASE	(MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE)
#define MSM_PMEM_SMIPOOL_SIZE	(MSM_PMEM_SMI_SIZE - MSM_FB_SIZE \
					- MSM_GPU_PHYS_SIZE)

#define PMEM_KERNEL_EBI1_SIZE	0x28000

#define PMIC_VREG_WLAN_LEVEL	2600
#define PMIC_VREG_GP6_LEVEL		2850

#define FPGA_SDCC_STATUS	0x70000280

#ifdef CONFIG_ANDROID_RAM_CONSOLE 
#define MSM_RAM_CONSOLE_START   0x38000000 - MSM_RAM_CONSOLE_SIZE
#define MSM_RAM_CONSOLE_SIZE    128 * SZ_1K
#endif

#ifdef CONFIG_CAPTURE_KERNEL
#define AMSSCORE_RAM_START 0x00000000
#define AMSSCORE_RAM_END   0x03FFFFFF
#define SMEMCORE_RAM_START 0x00100000
#define SMEMCORE_RAM_END   0x001FFFFF
#define ADSPCORE_RAM_START 0x2E000000
#define ADSPCORE_RAM_END   0x2FFFFFFF
#endif

#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
static int msm7227_platform_set_vib_voltage(u16 volt_mv)
{
	int rc = pmic_vib_mot_set_volt(volt_mv);

	if (rc)
		printk(KERN_ERR "%s: Failed to set motor voltage\n", __func__);
	return rc;
}

static int msm7227_platform_init_vib_hw(void)
{
	int rc = pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);

	if (rc) {
		printk(KERN_ERR "%s: Failed to set pin mode\n", __func__);
		return rc;
	}
	return pmic_vib_mot_set_volt(0);
}

static struct msm_pmic_vibrator_platform_data vibrator_platform_data = {
	.min_voltage = 1200,
	.max_voltage = 2500,
	.off_voltage = 0,
	.default_voltage = 2500,
	.mimimal_on_time = 10,
	.platform_set_vib_voltage = msm7227_platform_set_vib_voltage,
	.platform_init_vib_hw = msm7227_platform_init_vib_hw,
};
static struct platform_device vibrator_device = {
	.name = "msm_pmic_vibrator",
	.id = -1,
	.dev = {
		.platform_data = &vibrator_platform_data,
	},
};
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ram_console_resources[] = {
        [0] = {
                .start  = MSM_RAM_CONSOLE_START,
                .end    = MSM_RAM_CONSOLE_START+MSM_RAM_CONSOLE_SIZE-1,
                .flags  = IORESOURCE_MEM,
        },
};

static struct platform_device ram_console_device = {
        .name           = "ram_console",
        .id             = -1,
        .num_resources  = ARRAY_SIZE(ram_console_resources),
        .resource       = ram_console_resources,
};
#endif

#ifdef CONFIG_SMC91X
static struct resource smc91x_resources[] = {
	[0] = {
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.flags  = IORESOURCE_IRQ,
	},
};
#endif

#ifdef CONFIG_USB_ANDROID
/* dynamic composition */
static char *usb_func_msc[] = {
	"usb_mass_storage",
};
static char *usb_func_msc_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_func_msc_adb_eng[] = {
	"usb_mass_storage",
	"adb",
	"modem",
	"nmea",
	"diag",
};

#if defined(CONFIG_USB_ANDROID_MTP_ARICENT)
static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};

static char *usb_functions_mtp_msc[] = {
	"mtp",
	"usb_mass_storage",
};

static char *usb_functions_mtp_adb_eng[] = {
	"mtp",
	"adb",
	"modem",
	"nmea",
	"diag",
};
#endif

static char *usb_func_rndis[] = {
	"rndis",
};

static char *usb_func_adb_rndis[] = {
	"rndis",
	"adb",
};

static char *usb_functions_diag[] = {
	"adb",
	"modem",
	"diag",
};

static char *usb_functions_all[] = {
	"rndis",
	"usb_mass_storage",
#if defined(CONFIG_USB_ANDROID_MTP_ARICENT)
	"mtp",
#endif
	"adb",
	"modem",
	"nmea",
	"diag",
};
static struct android_usb_product android_usb_products[] = {
	{
		.product_id = 0xE12E,
		.functions = usb_func_msc,
		.num_functions = ARRAY_SIZE(usb_func_msc),
	},
	{
		.product_id = 0x612E,
		.functions = usb_func_msc_adb,
		.num_functions = ARRAY_SIZE(usb_func_msc_adb),
	},
#if defined(CONFIG_USB_ANDROID_MTP_ARICENT)
	{
		.product_id	= 0x0000 | CONFIG_USB_PRODUCT_SUFFIX,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp),
		.functions	= usb_functions_mtp,
	},
	{
		.product_id	= 0x512E | CONFIG_USB_PRODUCT_SUFFIX,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb),
		.functions	= usb_functions_mtp_adb,
	},
	{
		.product_id	= 0x412E | CONFIG_USB_PRODUCT_SUFFIX,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_msc),
		.functions	= usb_functions_mtp_msc,
	},
#endif
	{
		.product_id = 0x712E,
		.functions = usb_func_rndis,
		.num_functions = ARRAY_SIZE(usb_func_rndis),
	},
	{
		.product_id = 0x812E,
		.functions = usb_func_adb_rndis,
		.num_functions = ARRAY_SIZE(usb_func_adb_rndis),
	},
	{
		.product_id	= 0x912E,
		.functions	= usb_functions_diag,
		.num_functions	= ARRAY_SIZE(usb_functions_diag),
	},
#if defined(CONFIG_USB_ANDROID_MTP_ARICENT)
	{
		.product_id	= 0x5146,
		.num_functions	= ARRAY_SIZE(usb_functions_mtp_adb_eng),
		.functions	= usb_functions_mtp_adb_eng,
	},
#endif
	{
		.product_id = 0x6146,
		.functions = usb_func_msc_adb_eng,
		.num_functions = ARRAY_SIZE(usb_func_msc_adb_eng),
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns = 1,
	.vendor	= "SEMC",
	.product = "Mass Storage",
	.release = 0x0100,

	.cdrom_nluns = 1,
	.cdrom_vendor = "SEMC",
	.cdrom_product = "CD-ROM",
	.cdrom_release = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x0FCE,
	.vendorDescr	= "SEMC",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0FCE,
	.product_id		= 0xE12E,
	.version		= 0x0100,
	.product_name		= "SEMC HSUSB Device",
	.manufacturer_name	= "SEMC",
	.serial_number		= "1234567890ABCDEF",
	.num_products		= ARRAY_SIZE(android_usb_products),
	.products		= android_usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

#ifdef CONFIG_SMC91X
static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "8k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_gpio fsusb_config[] = {
	{ GPIO_CFG(139, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_dat" },
	{ GPIO_CFG(140, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_se0" },
	{ GPIO_CFG(141, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "fs_oe_n" },
};

static int fsusb_gpio_init(void)
{
	return msm_gpios_request(fsusb_config, ARRAY_SIZE(fsusb_config));
}

static void msm_fsusb_setup_gpio(unsigned int enable)
{
	if (enable)
		msm_gpios_enable(fsusb_config, ARRAY_SIZE(fsusb_config));
	else
		msm_gpios_disable(fsusb_config, ARRAY_SIZE(fsusb_config));

}
#endif

#define MSM_USB_BASE              ((unsigned)addr)

static struct vreg *vreg_usb;
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	switch (PHY_TYPE(phy_info)) {
	case USB_PHY_INTEGRATED:
		if (on)
			msm_hsusb_vbus_powerup();
		else
			msm_hsusb_vbus_shutdown();
		break;
	case USB_PHY_SERIAL_PMIC:
		if (on)
			vreg_enable(vreg_usb);
		else
			vreg_disable(vreg_usb);
		break;
	default:
		pr_err("%s: undefined phy type ( %X ) \n", __func__,
						phy_info);
	}
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_180NM),
};

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
};

#ifdef CONFIG_USB_FS_HOST
static struct msm_usb_host_platform_data msm_usb_host2_pdata = {
	.phy_info	= USB_PHY_SERIAL_PMIC,
	.config_gpio = msm_fsusb_setup_gpio,
	.vbus_power = msm_hsusb_vbus_power,
};
#endif

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION

static struct android_pmem_platform_data android_pmem_kernel_smi_pdata = {
	.name = PMEM_KERNEL_SMI_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_smipool_pdata = {
	.name = "pmem_smipool",
	.start = MSM_PMEM_SMIPOOL_BASE,
	.size = MSM_PMEM_SMIPOOL_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_smipool_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_smipool_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static struct platform_device android_pmem_kernel_smi_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_smi_pdata },
};
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
			ret = 0;
		else
			ret = -ENODEV;
	} else if ((machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf())
			&& !strcmp(name, "lcdc_external"))
		ret = 0;

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA1200000,
		.end	= 0xA1200000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

static struct platform_device qsd_device_spi = {
	.name	        = "spi_qsd",
	.id	        = 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static struct es209ra_touch_platform_data es209ra_touch_data = {
	.gpio_irq_pin		= 37,
	.gpio_reset_pin		= 30,
	.x_min			= 0,
	.x_max			= 480,
	.y_min			= 0,
	.y_max			= 854,
};

static struct spi_board_info msm_spi_board_info[] __initdata = {
	{
		.modalias	= "es209ra_touch",
		.mode		= SPI_MODE_0,
		.irq		= INT_ES209RA_GPIO_TOUCHPAD,
		.platform_data  = &es209ra_touch_data,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 1000000,
	},
};

#define CT_CSR_PHYS		0xA8700000
#define TCSR_SPI_MUX		(ct_csr_base + 0x54)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_csr_base = 0;
	u32 spi_mux;
	int ret = 0;

	ct_csr_base = ioremap(CT_CSR_PHYS, PAGE_SIZE);
	if (!ct_csr_base) {
		pr_err("%s: Could not remap %x\n", __func__, CT_CSR_PHYS);
		return -1;
	}

	spi_mux = readl(TCSR_SPI_MUX);
	switch (spi_mux) {
	case (1):
		qsd_spi_resources[4].start  = DMOV_HSUART1_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART1_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[4].start  = DMOV_HSUART2_RX_CHAN;
		qsd_spi_resources[4].end    = DMOV_HSUART2_TX_CHAN;
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[4].start  = DMOV_CE_OUT_CHAN;
		qsd_spi_resources[4].end    = DMOV_CE_IN_CHAN;
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -1;
	}

	iounmap(ct_csr_base);
	return ret;
}

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(17, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_clk" },
	{ GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(19, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "spi_miso" },
	{ GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
//	{ GPIO_CFG(21, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "spi_pwr" },
};

static int msm_qsd_spi_gpio_config(void)
{
	int rc;

	rc = msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
	if (rc)
		return rc;

	/* Set direction for SPI_PWR */
//	gpio_direction_output(21, 1);

	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 19200000,
	.clk_name = "spi_clk",
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

/*
static void msm_fb_vreg_config(const char *name, int on)
{
	struct vreg *vreg;
	int ret = 0;

	vreg = vreg_get(NULL, name);
	if (IS_ERR(vreg)) {
		printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
		__func__, name, PTR_ERR(vreg));
		return;
	}

	ret = (on) ? vreg_enable(vreg) : vreg_disable(vreg);
	if (ret)
		printk(KERN_ERR "%s: %s(%s) failed!\n",
			__func__, (on) ? "vreg_enable" : "vreg_disable", name);
}
*/

#define MDDI_RST_OUT_GPIO 100

static int mddi_power_save_on;
static int msm_fb_mddi_power_save(int on)
{
	int flag_on = !!on;
	int ret = 0;


	if (mddi_power_save_on == flag_on)
		return ret;

	mddi_power_save_on = flag_on;

	if (!flag_on && (machine_is_qsd8x50_ffa()
				|| machine_is_qsd8x50a_ffa())) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
	}

	ret = pmic_lp_mode_control(flag_on ? OFF_CMD : ON_CMD,
		PM_VREG_LP_MSME2_ID);
	if (ret)
		printk(KERN_ERR "%s: pmic_lp_mode_control failed!\n", __func__);

/*
	msm_fb_vreg_config("gp5", flag_on);
	msm_fb_vreg_config("boost", flag_on);

	if (flag_on && (machine_is_qsd8x50_ffa()
			|| machine_is_qsd8x50a_ffa())) {
		gpio_set_value(MDDI_RST_OUT_GPIO, 0);
		mdelay(1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		gpio_set_value(MDDI_RST_OUT_GPIO, 1);
		mdelay(1);
	}
*/
	return ret;
}

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 98,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	msm_fb_register_device("emdh", &mddi_pdata);
	msm_fb_register_device("tvenc", 0);
	msm_fb_register_device("lcdc", 0);
}

#define NT35580_GPIO_XRST 100
static struct vreg *vreg_mmc;
static struct vreg *vreg_gp2;

static void tmd_wvga_lcd_power_on(void)
{
	int rc = 0;

	local_irq_disable();

	rc = vreg_enable(vreg_gp2);
	if (rc) {
		local_irq_enable();
		printk(KERN_ERR"%s:vreg_enable(gp2)err. rc=%d\n", __func__, rc);
		return;
	}
	rc = vreg_enable(vreg_mmc);
	if (rc) {
		local_irq_enable();
		printk(KERN_ERR"%s:vreg_enable(mmc)err. rc=%d\n", __func__, rc);
		return;
	}
	local_irq_enable();

	msleep(50);
	gpio_set_value(NT35580_GPIO_XRST, 1);
	msleep(10);
	gpio_set_value(NT35580_GPIO_XRST, 0);
	msleep(1);
	gpio_set_value(NT35580_GPIO_XRST, 1);
	msleep(210);
}

static void tmd_wvga_lcd_power_off(void)
{
	gpio_set_value(NT35580_GPIO_XRST, 0);
	msleep(10);

	local_irq_disable();
	vreg_disable(vreg_mmc);
	vreg_disable(vreg_gp2);
	local_irq_enable();
}

static struct panel_data_ext tmd_wvga_panel_ext = {
	.power_on = tmd_wvga_lcd_power_on,
	.power_off = tmd_wvga_lcd_power_off,
};

static struct msm_fb_panel_data tmd_wvga_panel_data;

static struct platform_device mddi_tmd_wvga_display_device = {
	.name = "mddi_tmd_wvga",
	.id = -1,
};

static void __init msm_mddi_tmd_fwvga_display_device_init(void)
{
	struct msm_fb_panel_data *panel_data = &tmd_wvga_panel_data;

	printk(KERN_DEBUG "%s \n", __func__);

	panel_data->panel_info.xres = 480;
	panel_data->panel_info.yres = 854;
	panel_data->panel_info.type = MDDI_PANEL;
	panel_data->panel_info.pdest = DISPLAY_1;
	panel_data->panel_info.wait_cycle = 0;
	panel_data->panel_info.bpp = 16;
	panel_data->panel_info.clk_rate = 192000000;
	panel_data->panel_info.clk_min =  190000000;
	panel_data->panel_info.clk_max = 200000000;
	panel_data->panel_info.fb_num = 2;

	panel_data->panel_info.mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;

	panel_data->panel_info.lcd.vsync_enable = TRUE;
	panel_data->panel_info.lcd.v_back_porch = 12;
	panel_data->panel_info.lcd.v_front_porch = 2;
	panel_data->panel_info.lcd.v_pulse_width = 0;
	panel_data->panel_info.lcd.hw_vsync_mode = TRUE;
	panel_data->panel_info.lcd.vsync_notifier_period = 0;

	panel_data->panel_info.lcd.refx100 = 100000000 / 16766;

	panel_data->panel_ext = &tmd_wvga_panel_ext;

	mddi_tmd_wvga_display_device.dev.platform_data =
						&tmd_wvga_panel_data;

	vreg_gp2 = vreg_get(NULL, "gp2");
	if (IS_ERR(vreg_gp2)) {
		printk(KERN_ERR "%s: vreg_get(gp2) err.\n", __func__);
		return;
	}
	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg_get(mmc) err.\n", __func__);
		return;
	}

	platform_device_register(&mddi_tmd_wvga_display_device);
}

static struct resource msm_audio_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 68,
		.end    = 68,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 69,
		.end    = 69,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 70,
		.end    = 70,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 71,
		.end    = 71,
		.flags  = IORESOURCE_IO,
	},
/* Do anyone know who use these GPIO? */
#if 0
	{
		.name   = "sdac_din",
		.start  = 144,
		.end    = 144,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_dout",
		.start  = 145,
		.end    = 145,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "sdac_wsout",
		.start  = 143,
		.end    = 143,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "cc_i2s_clk",
		.start  = 142,
		.end    = 142,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "audio_master_clkout",
		.start  = 146,
		.end    = 146,
		.flags  = IORESOURCE_IO,
	},
#endif
	{
		.name	= "audio_base_addr",
		.start	= 0xa0700000,
		.end	= 0xa0700000 + 4,
		.flags	= IORESOURCE_MEM,
	},

};

static unsigned audio_gpio_on[] = {
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_CLK */
#if 0
	GPIO_CFG(142, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CC_I2S_CLK */
	GPIO_CFG(143, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* SADC_WSOUT */
	GPIO_CFG(144, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* SADC_DIN */
	GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* SDAC_DOUT */
	GPIO_CFG(146, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* MA_CLK_OUT */
#endif 
};

static void __init audio_gpio_init(void)
{
	int pin, rc;

	for (pin = 0; pin < ARRAY_SIZE(audio_gpio_on); pin++) {
		rc = gpio_tlmm_config(audio_gpio_on[pin],
			GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_gpio_on[pin], rc);
			return;
		}
	}
#ifdef CONFIG_MSM_QDSP6
	set_audio_gpios(msm_audio_resources[1].start);
#endif
}

static struct platform_device msm_audio_device = {
	.name   = "msm_audio",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_audio_resources),
	.resource       = msm_audio_resources,
};

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 29,
		.end	= 29,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= INT_ES209RA_GPIO_BT_HOSTWAKE,
		.end	= INT_ES209RA_GPIO_BT_HOSTWAKE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_SYSRST,
	BT_WAKE,
	BT_HOST_WAKE,
	BT_VDD_IO,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_VDD_FREG
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(29, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	"BT_WAKE" },
	{ GPIO_CFG(21, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	"HOST_WAKE" },
	{ GPIO_CFG(77, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "BT_VDD_IO" },
	{ GPIO_CFG(157, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "UART1DM_RFR" },
	{ GPIO_CFG(141, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "UART1DM_CTS" },
	{ GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "UART1DM_RX" },
	{ GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "UART1DM_TX" }
};
/*
static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(29, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// WAKE 
	{ GPIO_CFG(21, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// HOST_WAKE 
	{ GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// PWR_EN 
	{ GPIO_CFG(157, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// RFR 
	{ GPIO_CFG(141, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// CTS 
	{ GPIO_CFG(139, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// Rx 
	{ GPIO_CFG(140, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)},	// Tx 
};
*/
static int bluetooth_power(int on)
{
	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	gpio_set_value(77, on); /* PWR_EN */

	return 0;
}

static void __init bt_power_init(void)
{
	int pin, rc;

	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
		rc = gpio_tlmm_config(bt_config_power_on[pin].gpio_cfg,
				      GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
			       "%s: gpio_tlmm_config(%#x)=%d\n",
			       __func__, bt_config_power_on[pin].gpio_cfg, rc);
			return;
		}
	}
	gpio_set_value(77, 0); /* PWR_EN */

	msm_bt_power_device.dev.platform_data = &bluetooth_power;
	printk(KERN_DEBUG "Bluetooth power switch initialized\n");
}
/*
static int bluetooth_power(int on)
{
	int rc;
	struct vreg *vreg_wlan;

	vreg_wlan = vreg_get(NULL, "wlan");

	if (IS_ERR(vreg_wlan)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_wlan));
		return PTR_ERR(vreg_wlan);
	}

	if (on) {
		// units of mV, steps of 50 mV 
		rc = vreg_set_level(vreg_wlan, PMIC_VREG_WLAN_LEVEL);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}

		rc = msm_gpios_enable(bt_config_power_on,
					ARRAY_SIZE(bt_config_power_on));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: bt power on gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}

		gpio_set_value(22, on); // VDD_IO 
		gpio_set_value(18, on); // SYSRST 

		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			gpio_set_value(138, 0); // WLAN: CHIP_PWD 
			gpio_set_value(113, on); // WLAN 
		}
	} else {
		if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
			gpio_set_value(138, on); // WLAN: CHIP_PWD 
			gpio_set_value(113, on); // WLAN 
		}

		gpio_set_value(18, on); // SYSRST 
		gpio_set_value(22, on); // VDD_IO 

		rc = vreg_disable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: vreg wlan disable failed (%d)\n",
					__func__, rc);
			return -EIO;
		}

		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: bt power off gpio config failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static void __init bt_power_init(void)
{
	struct vreg *vreg_bt;
	int rc;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		gpio_set_value(138, 0); // WLAN: CHIP_PWD 
		gpio_set_value(113, 0); // WLAN 
	}

	gpio_set_value(18, 0); // SYSRST 
	gpio_set_value(22, 0); // VDD_IO 

	// do not have vreg bt defined, gp6 is the same 
	// vreg_get parameter 1 (struct device *) is ignored 
	vreg_bt = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_bt)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_bt));
		goto exit;
	}

	// units of mV, steps of 50 mV
	rc = vreg_set_level(vreg_bt, PMIC_VREG_GP6_LEVEL);
	if (rc) {
		printk(KERN_ERR "%s: vreg bt set level failed (%d)\n",
		       __func__, rc);
		goto exit;
	}
	rc = vreg_enable(vreg_bt);
	if (rc) {
		printk(KERN_ERR "%s: vreg bt enable failed (%d)\n",
		       __func__, rc);
		goto exit;
	}

	if (bluetooth_power(0))
		goto exit;

	msm_bt_power_device.dev.platform_data = &bluetooth_power;

	printk(KERN_DEBUG "Bluetooth power switch: initialized\n");

exit:
	return;
}
*/
#else
#define bt_power_init(x) do {} while (0)
#endif

static struct resource kgsl_resources[] = {
       {
		.name  = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
       },
       {
		.name   = "kgsl_phys_memory",
		.start = MSM_GPU_PHYS_BASE,
		.end = MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE - 1,
		.flags = IORESOURCE_MEM,
       },
       {
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
       },
};

static struct kgsl_platform_data kgsl_pdata = {
	.high_axi_3d = 128000, // Max for 8K 
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL,
	.max_grp3d_freq = 0,
	.min_grp3d_freq = 0,
	.set_grp3d_async = NULL,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp2d0_clk_name = NULL,
};

static struct platform_device msm_device_kgsl = {
       .name = "kgsl",
       .id = -1,
       .num_resources = ARRAY_SIZE(kgsl_resources),
       .resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

#ifdef CONFIG_ES209RA_HEADSET
struct es209ra_headset_platform_data es209ra_headset_data = {
	.keypad_name = "es209ra_keypad",
	.gpio_detout = 114,
	.gpio_detin = 132,
	.wait_time = 800,
};

static struct platform_device es209ra_audio_jack_device = {
	.name		= "es209ra_audio_jack",
	.dev = {
        .platform_data = &es209ra_headset_data,
    },
};
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_A_SYNC      GPIO_CFG(106, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_DATA      GPIO_CFG(107, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_EN        GPIO_CFG(108, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_A_CLK       GPIO_CFG(109, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_A_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_A_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_A_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_A_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "tsif_clk",
	.tsif_ref_clk = "tsif_ref_clk",
};

#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

#ifdef CONFIG_QSD_SVS
#define TPS65023_MAX_DCDC1	1600
#else
#define TPS65023_MAX_DCDC1	CONFIG_QSD_PMIC_DEFAULT_DCDC1
#endif

static int qsd8x50_tps65023_set_dcdc1(int mVolts)
{
	int rc = 0;
#ifdef CONFIG_QSD_SVS
	rc = tps65023_set_dcdc1_level(mVolts);
	/* By default the TPS65023 will be initialized to 1.225V.
	 * So we can safely switch to any frequency within this
	 * voltage even if the device is not probed/ready.
	 */
	if (rc == -ENODEV && mVolts <= CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = 0;
#else
	/* Disallow frequencies not supported in the default PMIC
	 * output voltage.
	 */
	if (mVolts > CONFIG_QSD_PMIC_DEFAULT_DCDC1)
		rc = -EFAULT;
#endif
	return rc;
}

static struct msm_acpu_clock_platform_data qsd8x50_clock_data = {
	.acpu_switch_time_us = 20,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.max_vdd = TPS65023_MAX_DCDC1,
	.acpu_set_vdd = qsd8x50_tps65023_set_dcdc1,
};

/* Driver(s) to be notified upon change in bdata */
static char *bdata_supplied_to[] = {
	MAX17040_NAME,
};

#ifdef CONFIG_BATTERY_X10_ZEUS
static struct semc_battery_platform_data semc_battery_platform_data = {
	.supplied_to = bdata_supplied_to,
	.num_supplicants = ARRAY_SIZE(bdata_supplied_to),
};

static struct platform_device bdata_driver = {
	.name = SEMC_BDATA_NAME,
	.id = -1,
	.dev = {
		.platform_data = &semc_battery_platform_data,
	},
};

static struct max17040_platform_data max17040_platform_data = {
	.model_desc = {
		.ocv_test = { 0xD9, 0x80 },
		.soc_low = 0xF4,
		.soc_high = 0xF6,
		.model_data = {
			{
				0xA6, 0xA0, 0xB7, 0x50, 0xB8, 0xB0, 0xB8, 0xE0,
				0xB9, 0x30, 0xBB, 0x60, 0xBB, 0xF0, 0xBC, 0x40
			},
			{
				0xBC, 0xA0, 0xBD, 0x50, 0xBE, 0x20, 0xC0, 0x20,
				0xC3, 0xF0, 0xC6, 0xE0, 0xCB, 0x40, 0xCF, 0x80
			},
			{
				0x03, 0xA0, 0x1A, 0x80, 0xAD, 0x60, 0x43, 0x60,
				0x00, 0x40, 0x7E, 0x40, 0x0E, 0x80, 0x72, 0x00
			},
			{
				0x4C, 0x20, 0x3B, 0x40, 0x29, 0xE0, 0x1B, 0x00,
				0x1B, 0x20, 0x13, 0x60, 0x12, 0x40, 0x12, 0x40
			}
		},
		.exp = 1
	},
	.rcomp_data = {
		.rcomp0 = 0x55,
		.temp_co_hot = -1400,
		.temp_co_cold = -9725,
		.temp_div = 1000,
	},
	.chg_max_temp = 550,
	.chg_min_temp = 50,
};
#endif

#ifdef CONFIG_SENSORS_AKM8973
#define AKM8973_GPIO_RESET_PIN 2
#define AKM8973_GPIO_RESET_ON 0
#define AKM8973_GPIO_RESET_OFF 1

static int ak8973_gpio_config(int enable)
{
	if (enable) {
		if (gpio_request(AKM8973_GPIO_RESET_PIN, "akm8973_xres")) {
			printk(KERN_ERR "%s: gpio_req xres"
				" - Fail!", __func__);
			return -EIO;
		}
		if (gpio_tlmm_config(GPIO_CFG(AKM8973_GPIO_RESET_PIN, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),GPIO_CFG_ENABLE)) {
			printk(KERN_ERR "%s: gpio_tlmm_conf xres"
				" - Fail!", __func__);
			goto ak8973_gpio_fail_0;
		}
		/* Reset is active low, so just a precaution setting. */
		gpio_set_value(AKM8973_GPIO_RESET_PIN, 1);

	} else {
		gpio_free(AKM8973_GPIO_RESET_PIN);
	}
	return 0;

ak8973_gpio_fail_0:
	gpio_free(AKM8973_GPIO_RESET_PIN);
	return -EIO;
}

static int ak8973_xres(void)
{
	gpio_set_value(AKM8973_GPIO_RESET_PIN, 0);
	msleep(10);
	gpio_set_value(AKM8973_GPIO_RESET_PIN, 1);
	msleep(20);
	return 0;
}

static struct akm8973_i2c_platform_data akm8973_platform_data = {
	.gpio_config = ak8973_gpio_config,
	.xres = ak8973_xres
};
#endif /* CONFIG_SENSORS_AKM8973 */

static int bma150_gpio_setup(bool request)
{
	if (request)
		return gpio_request(ES209RA_GPIO_ACCEL, "bma150_irq");
	else
		gpio_free(ES209RA_GPIO_ACCEL);
	return 0;
}

struct bma150_platform_data bma150_ng_platform_data = {
	.gpio_setup = &bma150_gpio_setup,
};

static struct i2c_board_info msm_i2c_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
	},
#ifdef CONFIG_BATTERY_X10_ZEUS
	{
		I2C_BOARD_INFO(MAX17040_NAME, 0x6C >> 1),
		.platform_data = &max17040_platform_data,
	},
#endif
	{
		I2C_BOARD_INFO("lv5219lg", 0x74),
	},
#ifdef CONFIG_SENSORS_AKM8973
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.platform_data = &akm8973_platform_data,
		.irq = INT_ES209RA_GPIO_ECOMPASS,
	},
#endif
	{
		I2C_BOARD_INFO("bma150", 0x38), 
		.irq		   =  INT_ES209RA_GPIO_ACCEL,
		.platform_data = &bma150_ng_platform_data,
	},
	{
		I2C_BOARD_INFO("semc_imx046_camera", 0x1F),
		.irq		   =  INT_ES209RA_GPIO_CAM_ISP,
	},
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
//	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
//	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* MCLK */
	GPIO_CFG(27, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCAML2_EN */
	GPIO_CFG(43, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCAML_EN */
	GPIO_CFG(142,0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCAMSD_EN */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
//	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
//	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* MCLK */
	GPIO_CFG(27, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* VCAML2_EN */
	GPIO_CFG(43, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* VCAML_EN */
	GPIO_CFG(142, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* VCAMSD_EN */
};

static uint32_t camera_on_gpio_ffa_table[] = {
	// parallel CAMERA interfaces 
	GPIO_CFG(95,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), // I2C_SCL 
	GPIO_CFG(96,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), // I2C_SDA 
	// FFA front Sensor Reset 
	GPIO_CFG(137,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
};

static uint32_t camera_off_gpio_ffa_table[] = {
	// FFA front Sensor Reset 
	GPIO_CFG(137,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
};
	
static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 1800);
		if (rc) {
			printk(KERN_ERR "%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "gp3");
		if (IS_ERR(vreg_gp3)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 2800);
		if (rc) {
			printk(KERN_ERR "%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 enable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 disable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}

static void config_camera_on_gpios(void)
{
	int vreg_en = 1;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		config_gpio_table(camera_on_gpio_ffa_table,
		ARRAY_SIZE(camera_on_gpio_ffa_table));

		msm_camera_vreg_config(vreg_en);
		gpio_set_value(137, 0);
	}
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
	int vreg_en = 0;

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa()) {
		config_gpio_table(camera_off_gpio_ffa_table,
		ARRAY_SIZE(camera_off_gpio_ffa_table));

		msm_camera_vreg_config(vreg_en);
	}
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA0F00000,
		.end	= 0xA0F00000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 30,
	._fsrc.pmic_src.high_current = 100,
	//._fsrc.pmic_src.led_src_1 = 0,
	//._fsrc.pmic_src.led_src_2 = 0,
	//._fsrc.pmic_src.pmic_set_current = pmic_set_flash_led_current,
};

#ifdef CONFIG_SEMC_IMX046_CAMERA
static struct msm_camera_sensor_flash_data flash_semc_imx046_camera = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_semc_imx046_camera_data = {
	.sensor_name    = "semc_imx046_camera",
	.sensor_reset   = 0,
	.sensor_pwd     = 0,
	.vcm_pwd        = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_semc_imx046_camera,
	.mclk           = 15,
	.sensor_int     = INT_ES209RA_GPIO_CAM_ISP,
	.sensor_vsync   = INT_ES209RA_GPIO_CAM_VSYNC,
	.vcam_l1        = {.type = MSM_CAMERA_SENSOR_PWR_GPIO, .resource.number = 27,       },
	.vcam_l2        = {.type = MSM_CAMERA_SENSOR_PWR_GPIO, .resource.number = 43,       },
	.vcam_sd        = {.type = MSM_CAMERA_SENSOR_PWR_GPIO, .resource.number = 142,      },
	.vcam_io        = {.type = MSM_CAMERA_SENSOR_PWR_VREG, .resource.name   = "rfrx2",  },
	.vcam_af        = {.type = MSM_CAMERA_SENSOR_PWR_VREG, .resource.name   = "rftx",   },
	.vcam_sa        = {.type = MSM_CAMERA_SENSOR_PWR_VREG, .resource.name   = "gp1",    },
};

static struct platform_device msm_camera_sensor_semc_imx046_camera = {
	.name      = "msm_camera_semc_imx046_camera",
	.dev       = {
		.platform_data = &msm_camera_sensor_semc_imx046_camera_data,
	},
};
#endif
#endif /*CONFIG_MSM_CAMERA*/

static struct platform_device msm_wlan_ar6000_pm_device = {
        .name           = "wlan_ar6000_pm_dev",
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
};

/*
static u32 msm_calculate_batt_capacity(u32 current_voltage);

static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;

	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
}

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};
*/

#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
static struct lbs_platform_data lbs_data = {
	.threshold_vol = 3500,
};

static struct platform_device lbs_device = {
	.name	= "Low-Battery Shutdown",
	.id	= -1,
	.dev	= {
		.platform_data = &lbs_data,
	},
};
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */

#ifdef CONFIG_PMIC_TIME
static struct platform_device pmic_time_device = {
	.name = "pmic_time",
};
#endif

static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		ret = msm_pm_app_rpc_init(callback);
	} else {
		msm_pm_app_rpc_deinit(callback);
		ret = 0;
	}
	return ret;
}
static int msm_hsusb_ldo_init(int init);
static int msm_hsusb_ldo_enable(int enable);

/* Driver(s) to be notified upon change in USB */
static char *hsusb_chg_supplied_to[] = {
	MAX17040_NAME,
};

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.pmic_notif_init         = msm_hsusb_pmic_notif_init,
	.pemp_level              = PRE_EMPHASIS_WITH_10_PERCENT,
	.cdr_autoreset           = CDR_AUTO_RESET_DEFAULT,
	.drv_ampl                = HS_DRV_AMPLITUDE_5_PERCENT,
	.vbus_power		 = msm_hsusb_vbus_power,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.phy_can_powercollapse	 = 1,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;

static struct platform_device *devices[] __initdata = {
	&msm_wlan_ar6000_pm_device,
	&msm_fb_device,
#ifdef CONFIG_SMC91X
	&smc91x_device,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&android_pmem_kernel_ebi1_device,
#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	&android_pmem_kernel_smi_device,
#endif
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_smipool_device,
	&msm_device_nand,
	&msm_device_i2c,
	&qsd_device_spi,
/*
#ifdef CONFIG_USB_FUNCTION
	&mass_storage_device,
#endif
*/
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
	&android_usb_device,
#endif
	&msm_device_tssc,
	&msm_audio_device,
	&msm_device_uart1,
	&msm_bluesleep_device,
#ifdef CONFIG_BT
	&msm_bt_power_device,
	&msm_device_uart_dm2,
#endif
#if !defined(CONFIG_MSM_SERIAL_DEBUGGER)
	//&msm_device_uart3,
#endif
	&msm_device_kgsl,
	&hs_device,
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_SEMC_IMX046_CAMERA
	&msm_camera_sensor_semc_imx046_camera,
#endif
#ifdef CONFIG_SEMC_MSM_PMIC_VIBRATOR
	&vibrator_device,
#endif
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ram_console_device,
#endif
#ifdef CONFIG_ES209RA_HEADSET
	&es209ra_audio_jack_device,
#endif
#ifdef CONFIG_SEMC_LOW_BATT_SHUTDOWN
	&lbs_device,
#endif /* CONFIG_SEMC_LOW_BATT_SHUTDOWN */
#ifdef CONFIG_CAPTURE_KERNEL
	&kdump_amsscoredump_device,
#endif
#ifdef CONFIG_PMIC_TIME
	&pmic_time_device,
#endif
	&bdata_driver,
	//&msm_batt_device,
};

static void __init es209ra_init_irq(void)
{
	msm_init_irq();
	msm_init_sirc();
}

static void kgsl_phys_memory_init(void)
{
	request_mem_region(kgsl_resources[1].start,
		resource_size(&kgsl_resources[1]), "kgsl");
}

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 20;

	if (machine_is_qsd8x50_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
			MPP_CFG(MPP_DLOGIC_LVL_VDD,
				MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
				"to enable 3.3V LDO failed\n", __func__);
	}
}

/* TBD: 8x50 FFAs have internal 3p3 voltage regulator as opposed to
 * external 3p3 voltage regulator on Surf platform. There is no way
 * s/w can detect fi concerned regulator is internal or external to
 * to MSM. Internal 3p3 regulator is powered through boost voltage
 * regulator where as external 3p3 regulator is powered through VPH.
 * So for internal voltage regulator it is required to power on
 * boost voltage regulator first. Unfortunately some of the FFAs are
 * re-worked to install external 3p3 regulator. For now, assuming all
 * FFAs have 3p3 internal regulators and all SURFs have external 3p3
 * regulator as there is no way s/w can determine if theregulator is
 * internal or external. May be, we can implement this flag as kernel
 * boot parameters so that we can change code behaviour dynamically
 */
static int regulator_3p3_is_internal;
static struct vreg *vreg_5v;
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		if (regulator_3p3_is_internal) {
			vreg_5v = vreg_get(NULL, "boost");
			if (IS_ERR(vreg_5v))
				return PTR_ERR(vreg_5v);
			vreg_set_level(vreg_5v, 5000);
		}

		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, 3300);
	} else {
		if (regulator_3p3_is_internal)
			vreg_put(vreg_5v);
		vreg_put(vreg_3p3);
	}

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;
	int ret;

	if (ldo_status == enable)
		return 0;

	if (regulator_3p3_is_internal && (!vreg_5v || IS_ERR(vreg_5v)))
		return -ENODEV;
	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	ldo_status = enable;

	if (enable) {
		if (regulator_3p3_is_internal) {
			ret = vreg_enable(vreg_5v);
			if (ret)
				return ret;

			/* power supply to 3p3 regulator can vary from
			 * USB VBUS or VREG 5V. If the power supply is
			 * USB VBUS cable disconnection cannot be
			 * deteted. Select power supply to VREG 5V
			 */
			/* TBD: comeup with a better name */
			ret = pmic_vote_3p3_pwr_sel_switch(1);
			if (ret)
				return ret;
		}
		ret = vreg_enable(vreg_3p3);

		return ret;
	} else {
		if (regulator_3p3_is_internal) {
			ret = vreg_disable(vreg_5v);
			if (ret)
				return ret;
			ret = pmic_vote_3p3_pwr_sel_switch(0);
			if (ret)
				return ret;
		}
			ret = vreg_disable(vreg_3p3);

			return ret;
	}
}

static void __init qsd8x50_init_usb(void)
{
	usb_mpp_init();

	if (machine_is_qsd8x50_ffa())
		regulator_3p3_is_internal = 1;

#ifdef CONFIG_USB_MSM_OTG_72K
	platform_device_register(&msm_device_otg);
#endif

#ifdef CONFIG_USB_FUNCTION_MSM_HSUSB
	platform_device_register(&msm_device_hsusb_peripheral);
#endif

#ifdef CONFIG_USB_MSM_72K
	platform_device_register(&msm_device_gadget_peripheral);
#endif

	if (machine_is_qsd8x50_ffa() || machine_is_qsd8x50a_ffa())
		return;

	vreg_usb = vreg_get(NULL, "boost");

	if (IS_ERR(vreg_usb)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_usb));
		return;
	}

	platform_device_register(&msm_device_hsusb_otg);
	msm_add_host(0, &msm_usb_host_pdata);
#ifdef CONFIG_USB_FS_HOST
	if (fsusb_gpio_init())
		return;
	msm_add_host(1, &msm_usb_host2_pdata);
#endif
}

static struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_clk"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
	},
};

static unsigned long vreg_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		rc = vreg_set_level(vreg_mmc, PMIC_VREG_GP6_LEVEL);
		if (!rc)
			rc = vreg_enable(vreg_mmc);
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#endif
#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT))

static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_qsd8x50_surf() || machine_is_qsd8x50a_surf()))
		return -1;

	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (readl(wp_addr) >> ((pdev->id - 1) << 1)) & (0x03);
	pr_info("%s: WP/CD Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);
	return ((ret == 0x02) ? 1 : 0);

}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data qsd8x50_sdc1_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data qsd8x50_sdc2_data = {
	.ocr_mask       = MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.wpswitch	= msm_sdcc_get_wpswitch,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 25000000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

static void __init qsd8x50_init_mmc(void)
{
		vreg_mmc = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	msm_add_sdcc(1, &qsd8x50_sdc1_data);
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
		msm_add_sdcc(2, &qsd8x50_sdc2_data);
#endif
}

#ifdef CONFIG_SMC91X
static void __init qsd8x50_cfg_smc91x(void)
{
	int rc = 0;

	smc91x_resources[0].start = 0x70000300;
	smc91x_resources[0].end = 0x700003ff;
	smc91x_resources[1].start = INT_ES209RA_GPIO_ETHER;
	smc91x_resources[1].end = INT_ES209RA_GPIO_ETHER;

	rc = gpio_tlmm_config(GPIO_CFG(107, 0, GPIO_INPUT,
					       GPIO_PULL_DOWN, GPIO_2MA),
					       GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",
				__func__, rc);
	}
}
#endif

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;

	switch (iface)
	{
		case 0: /* primary */
			gpio_scl = 95;
			gpio_sda = 96;
			break;
		case 1: /* secondary */
		default:
			printk(KERN_INFO "%s: es209ra has only primary I2C.\n", __func__);
			return;
	}

	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 384000,
	.rsl_id = SMEM_SPINLOCK_I2C,
	.pri_clk = 95,
	.pri_dat = 96,
#if 0
	.aux_clk = 60,
	.aux_dat = 61,
#endif
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(95, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(96, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

	msm_i2c_pdata.rmutex = (uint32_t)smem_alloc(SMEM_I2C_MUTEX, 8);
	msm_i2c_pdata.pm_lat =
		msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static void __init pmem_kernel_ebi1_size_setup(char **p)
{
	pmem_kernel_ebi1_size = memparse(*p, p);
}
__early_param("pmem_kernel_ebi1_size=", pmem_kernel_ebi1_size_setup);

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
static unsigned pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
static void __init pmem_kernel_smi_size_setup(char **p)
{
	pmem_kernel_smi_size = memparse(*p, p);

	/* Make sure that we don't allow more SMI memory then is
	   available - the kernel mapping code has no way of knowing
	   if it has gone over the edge */

	if (pmem_kernel_smi_size > MSM_PMEM_SMIPOOL_SIZE)
		pmem_kernel_smi_size = MSM_PMEM_SMIPOOL_SIZE;
}
__early_param("pmem_kernel_smi_size=", pmem_kernel_smi_size_setup);
#endif

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static void __init pmem_mdp_size_setup(char **p)
{
	pmem_mdp_size = memparse(*p, p);
}
__early_param("pmem_mdp_size=", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static void __init pmem_adsp_size_setup(char **p)
{
	pmem_adsp_size = memparse(*p, p);
}
__early_param("pmem_adsp_size=", pmem_adsp_size_setup);


static unsigned audio_size = MSM_AUDIO_SIZE;
static void __init audio_size_setup(char **p)
{
	audio_size = memparse(*p, p);
}
__early_param("audio_size=", audio_size_setup);

/* SEMC:SYS: Get startup reason - start */
unsigned int es209ra_startup_reason = 0;

static int __init es209ra_startup_reason_setup(char *str)
{
	es209ra_startup_reason = simple_strtoul(str, NULL, 16);
	return 1;
}
__setup_param("startup=", es209ra_startup_reason_setup_1, es209ra_startup_reason_setup, 0);
__setup_param("semcandroidboot.startup=", es209ra_startup_reason_setup_2, es209ra_startup_reason_setup, 0);

static int es209ra_hw_version = 0;

static int __init es209ra_hw_version_setup(char *str)
{
	es209ra_hw_version = simple_strtoul(str, NULL, 0);

	return 1;
}
__setup_param("hwversion=", es209ra_hw_version_setup_1, es209ra_hw_version_setup, 0);
__setup_param("semcandroidboot.hwversion=", es209ra_hw_version_setup_2, es209ra_hw_version_setup, 0);
int get_predecode_repair_cache(void);
int set_predecode_repair_cache(void);
static void __init es209ra_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n", __func__);
	printk(KERN_INFO "%s: startup_reason: 0x%08x\n",
					__func__, es209ra_startup_reason);
	printk(KERN_ERR "PVR0F2: %x\n", get_predecode_repair_cache());
	set_predecode_repair_cache();
	printk(KERN_ERR "PVR0F2: %x\n", get_predecode_repair_cache());

	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
#ifdef CONFIG_SMC91X
	qsd8x50_cfg_smc91x();
#endif
	msm_acpu_clock_init(&qsd8x50_clock_data);

	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;

	msm_otg_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_gadget_pdata.is_phy_status_timer_on = 1;

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	platform_add_devices(devices, ARRAY_SIZE(devices));
	msm_fb_add_devices();
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	qsd8x50_init_usb();
	hsusb_chg_set_supplicants(hsusb_chg_supplied_to,
				  ARRAY_SIZE(hsusb_chg_supplied_to));
	qsd8x50_init_mmc();
	bt_power_init();
	audio_gpio_init();
	msm_device_i2c_init();
	msm_qsd_spi_init();
	i2c_register_board_info(0, msm_i2c_board_info,
				ARRAY_SIZE(msm_i2c_board_info));
	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	kgsl_phys_memory_init();
	platform_device_register(&es209ra_keypad_device);
	msm_mddi_tmd_fwvga_display_device_init();
}

#ifndef CONFIG_CAPTURE_KERNEL
static void __init es209ra_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_KERNEL_PMEM_SMI_REGION
	size = pmem_kernel_smi_size;
	if (size > MSM_PMEM_SMIPOOL_SIZE) {
		printk(KERN_ERR "pmem kernel smi arena size %lu is too big\n",
			size);

		size = MSM_PMEM_SMIPOOL_SIZE;
	}

	android_pmem_kernel_smi_pdata.start = MSM_PMEM_SMIPOOL_BASE;
	android_pmem_kernel_smi_pdata.size = size;

	pr_info("allocating %lu bytes at %lx (%lx physical)"
		"for pmem kernel smi arena\n", size,
		(long unsigned int) MSM_PMEM_SMIPOOL_BASE,
		__pa(MSM_PMEM_SMIPOOL_BASE));
#endif

	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}


	size = MSM_FB_SIZE;
	addr = (void *)MSM_FB_BASE;
	msm_fb_resources[0].start = (unsigned long)addr;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("using %lu bytes of SMI at %lx physical for fb\n",
	       size, (unsigned long)addr);

	size = audio_size ? : MSM_AUDIO_SIZE;
	addr = alloc_bootmem(size);
	msm_audio_resources[0].start = __pa(addr);
	msm_audio_resources[0].end = msm_audio_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for audio\n",
		size, addr, __pa(addr));
}

static void __init es209ra_fixup(struct machine_desc *desc, struct tag *tags,
                               char **cmdline, struct meminfo *mi)
{
	mi->nr_banks=2;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(mi->bank[0].start);
	mi->bank[0].size = (232*1024*1024);

	mi->bank[1].start = 0x30000000;
	mi->bank[1].size = (127*1024*1024);
	mi->bank[1].node = PHYS_TO_NID(mi->bank[1].start);
}
#endif

static void __init es209ra_map_io(void)
{
	msm_shared_ram_phys = MSM_SHARED_RAM_PHYS;
	msm_map_qsd8x50_io();
#ifndef CONFIG_CAPTURE_KERNEL
	es209ra_allocate_memory_regions();
#endif
	msm_clock_init(msm_clocks_8x50, msm_num_clocks_8x50);
}

static int __init board_serialno_setup(char *serialno)
{
#ifdef CONFIG_USB_ANDROID
	int i;
	char *src = serialno;
	android_usb_pdata.serial_number = serialno;
	printk(KERN_INFO "USB serial number: %s\n", android_usb_pdata.serial_number);

	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++)
		rndis_pdata.ethaddr[i % (ETH_ALEN -1)+1] ^= *src++;
#endif
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = serialno;
	printk(KERN_INFO "USB serial number: %s\n", msm_hsusb_pdata.serial_number);
#endif
	return 1;
}

__setup_param("serialno=", board_serialno_setup_1, board_serialno_setup, 0);
__setup_param("semcandroidboot.serialno=", board_serialno_setup_2, board_serialno_setup, 0);

MACHINE_START(ES209RA, "ES209RA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
#ifdef CONFIG_CAPTURE_KERNEL
	.boot_params    = PHYS_OFFSET + 0x1000,
#else
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = es209ra_fixup,
#endif
	.map_io		= es209ra_map_io,
	.init_irq	= es209ra_init_irq,
	.init_machine	= es209ra_init,
	.timer = &msm_timer,
MACHINE_END
