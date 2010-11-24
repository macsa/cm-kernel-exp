/*
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/dma-mapping.h>
#include <mach/irqs.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>
#include <mach/board.h>
#include <linux/gpio.h>

#include "devices.h"
#include "proc_comm.h"
#include "smd_private.h"

#include <asm/mach/flash.h>

#include "clock.h"
#include <linux/msm_kgsl.h>
#include <linux/android_pmem.h>
#include <mach/mmc.h>
#include <mach/board_htc.h>
#include <mach/dal_axi.h>

static struct resource resources_uart1[] = {
	{
		.start	= INT_UART1,
		.end	= INT_UART1,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART1_PHYS,
		.end	= MSM_UART1_PHYS + MSM_UART1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart2[] = {
	{
		.start	= INT_UART2,
		.end	= INT_UART2,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART2_PHYS,
		.end	= MSM_UART2_PHYS + MSM_UART2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource resources_uart3[] = {
	{
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= MSM_UART3_PHYS,
		.end	= MSM_UART3_PHYS + MSM_UART3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_uart1 = {
	.name	= "msm_serial",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(resources_uart1),
	.resource	= resources_uart1,
};

struct platform_device msm_device_uart2 = {
	.name	= "msm_serial",
	.id	= 1,
	.num_resources	= ARRAY_SIZE(resources_uart2),
	.resource	= resources_uart2,
};

struct platform_device msm_device_uart3 = {
	.name	= "msm_serial",
	.id	= 2,
	.num_resources	= ARRAY_SIZE(resources_uart3),
	.resource	= resources_uart3,
};

static struct resource msm_uart1_dm_resources[] = {
	{
		.start = MSM_UART1DM_PHYS,
		.end   = MSM_UART1DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART1DM_IRQ,
		.end   = INT_UART1DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART1DM_RX,
		.end   = INT_UART1DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART1_TX_CHAN,
		.end   = DMOV_HSUART1_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART1_TX_CRCI,
		.end   = DMOV_HSUART1_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm1_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm1 = {
	.name = "msm_serial_hs",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_uart1_dm_resources),
	.resource = msm_uart1_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm1_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource msm_uart2_dm_resources[] = {
	{
		.start = MSM_UART2DM_PHYS,
		.end   = MSM_UART2DM_PHYS + PAGE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_UART2DM_IRQ,
		.end   = INT_UART2DM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = INT_UART2DM_RX,
		.end   = INT_UART2DM_RX,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = DMOV_HSUART2_TX_CHAN,
		.end   = DMOV_HSUART2_RX_CHAN,
		.name  = "uartdm_channels",
		.flags = IORESOURCE_DMA,
	},
	{
		.start = DMOV_HSUART2_TX_CRCI,
		.end   = DMOV_HSUART2_RX_CRCI,
		.name  = "uartdm_crci",
		.flags = IORESOURCE_DMA,
	},
};

static u64 msm_uart_dm2_dma_mask = DMA_BIT_MASK(32);

struct platform_device msm_device_uart_dm2 = {
	.name = "msm_serial_hs",
	.id = 1,
	.num_resources = ARRAY_SIZE(msm_uart2_dm_resources),
	.resource = msm_uart2_dm_resources,
	.dev		= {
		.dma_mask = &msm_uart_dm2_dma_mask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},
};

static struct resource resources_i2c[] = {
	{
		.start	= MSM_I2C_PHYS,
		.end	= MSM_I2C_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C,
		.end	= INT_PWB_I2C,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c = {
	.name		= "msm_i2c",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_i2c),
	.resource	= resources_i2c,
};

#define GPIO_I2C_CLK 70
#define GPIO_I2C_DAT 71
void msm_set_i2c_mux(bool gpio, int *gpio_clk, int *gpio_dat)
{
	unsigned id;
	if (gpio) {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT, 0, GPIO_OUTPUT,
				   GPIO_NO_PULL, GPIO_2MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		*gpio_clk = GPIO_I2C_CLK;
		*gpio_dat = GPIO_I2C_DAT;
	} else {
		id = PCOM_GPIO_CFG(GPIO_I2C_CLK, 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
		id = PCOM_GPIO_CFG(GPIO_I2C_DAT , 1, GPIO_INPUT,
				   GPIO_NO_PULL, GPIO_8MA);
		msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	}
}

static struct resource resources_i2c_2[] = {
	{
		.start	= MSM_I2C_2_PHYS,
		.end	= MSM_I2C_2_PHYS + MSM_I2C_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_PWB_I2C_2,
		.end	= INT_PWB_I2C_2,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_i2c_2 = {
	.name		= "msm_i2c",
	.id		= 22,
	.num_resources	= ARRAY_SIZE(resources_i2c_2),
	.resource	= resources_i2c_2,
};

static struct resource resources_qup[] = {
	{
		.name   = "qup_phys_addr",
		.start	= MSM_QUP_PHYS,
		.end	= MSM_QUP_PHYS + MSM_QUP_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "gsbi_qup_i2c_addr",
		.start	= MSM_GSBI_QUP_I2C_PHYS,
		.end	= MSM_GSBI_QUP_I2C_PHYS + 4 - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "qup_in_intr",
		.start	= INT_PWB_QUP_IN,
		.end	= INT_PWB_QUP_IN,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_out_intr",
		.start	= INT_PWB_QUP_OUT,
		.end	= INT_PWB_QUP_OUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "qup_err_intr",
		.start	= INT_PWB_QUP_ERR,
		.end	= INT_PWB_QUP_ERR,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_qup_i2c = {
	.name		= "qup_i2c",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_qup),
	.resource	= resources_qup,
};

struct platform_device qup_device_i2c = {
        .name           = "qup_i2c",
        .id             = 4,
        .num_resources  = ARRAY_SIZE(resources_qup),
        .resource       = resources_qup,
};

static struct resource resources_hsusb[] = {
	{
		.start	= MSM_HSUSB_PHYS,
		.end	= MSM_HSUSB_PHYS + MSM_HSUSB_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_USB_HS,
		.end	= INT_USB_HS,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_hsusb = {
	.name		= "msm_hsusb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_hsusb),
	.resource	= resources_hsusb,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct flash_platform_data msm_nand_data = {
	.parts		= NULL,
	.nr_parts	= 0,
};

static struct resource resources_nand[] = {
	[0] = {
		.start	= 7,
		.end	= 7,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_nand = {
	.name		= "msm_nand",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resources_nand),
	.resource	= resources_nand,
	.dev		= {
		.platform_data	= &msm_nand_data,
	},
};

struct platform_device msm_device_smd = {
	.name	= "msm_smd",
	.id	= -1,
};

static struct resource resources_sdc1[] = {
	{
		.start	= MSM_SDC1_PHYS,
		.end	= MSM_SDC1_PHYS + MSM_SDC1_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC1_0,
		.end	= INT_SDC1_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
	{
		.start	= INT_SDC1_1,
		.end	= INT_SDC1_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc2[] = {
	{
		.start	= MSM_SDC2_PHYS,
		.end	= MSM_SDC2_PHYS + MSM_SDC2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC2_0,
		.end	= INT_SDC2_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC2_1,
		.end	= INT_SDC2_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc3[] = {
	{
		.start	= MSM_SDC3_PHYS,
		.end	= MSM_SDC3_PHYS + MSM_SDC3_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC3_0,
		.end	= INT_SDC3_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC3_1,
		.end	= INT_SDC3_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

static struct resource resources_sdc4[] = {
	{
		.start	= MSM_SDC4_PHYS,
		.end	= MSM_SDC4_PHYS + MSM_SDC4_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SDC4_0,
		.end	= INT_SDC4_0,
		.flags	= IORESOURCE_IRQ,
		.name	= "cmd_irq",
	},
		{
		.start	= INT_SDC4_1,
		.end	= INT_SDC4_1,
		.flags	= IORESOURCE_IRQ,
		.name	= "pio_irq",
	},
	{
		.flags	= IORESOURCE_IRQ | IORESOURCE_DISABLED,
		.name	= "status_irq"
	},
	{
		.start	= 8,
		.end	= 8,
		.flags	= IORESOURCE_DMA,
	},
};

struct platform_device msm_device_sdc1 = {
	.name		= "msm_sdcc",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(resources_sdc1),
	.resource	= resources_sdc1,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc2 = {
	.name		= "msm_sdcc",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(resources_sdc2),
	.resource	= resources_sdc2,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc3 = {
	.name		= "msm_sdcc",
	.id		= 3,
	.num_resources	= ARRAY_SIZE(resources_sdc3),
	.resource	= resources_sdc3,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

struct platform_device msm_device_sdc4 = {
	.name		= "msm_sdcc",
	.id		= 4,
	.num_resources	= ARRAY_SIZE(resources_sdc4),
	.resource	= resources_sdc4,
	.dev		= {
		.coherent_dma_mask	= 0xffffffff,
	},
};

static struct platform_device *msm_sdcc_devices[] __initdata = {
	&msm_device_sdc1,
	&msm_device_sdc2,
	&msm_device_sdc3,
	&msm_device_sdc4,
};

int __init msm_add_sdcc(unsigned int controller, struct mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags)
{
	struct platform_device	*pdev;
	struct resource *res;

	if (controller < 1 || controller > 4)
		return -EINVAL;

	pdev = msm_sdcc_devices[controller-1];
	pdev->dev.platform_data = plat;

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "status_irq");
	if (!res)
		return -EINVAL;
	else if (stat_irq) {
		res->start = res->end = stat_irq;
		res->flags &= ~IORESOURCE_DISABLED;
		res->flags |= stat_irq_flags;
	}

	return platform_device_register(pdev);
}

static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
	.dev = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};

#ifdef CONFIG_MSM_ROTATOR
static struct resource resources_msm_rotator[] = {
        {
                .start  = MSM_ROTATOR_PHYS,
                .end    = MSM_ROTATOR_PHYS + MSM_ROTATOR_SIZE - 1,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_ROTATOR,
                .end    = INT_ROTATOR,
                .flags  = IORESOURCE_IRQ,
        },
};

struct platform_device msm_rotator_device = {
        .name           = "msm_rotator",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(resources_msm_rotator),
        .resource       = resources_msm_rotator,
};
#endif

static struct resource resources_ssbi_pmic[] = {
	{
		.start	= MSM_PMIC_SSBI_PHYS,
		.end	= MSM_PMIC_SSBI_PHYS + MSM_PMIC_SSBI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

struct platform_device msm_device_ssbi_pmic = {
	.name		= "msm_ssbi",
	.id		= -1,
	.resource	= resources_ssbi_pmic,
	.num_resources	= ARRAY_SIZE(resources_ssbi_pmic),
};

static struct resource resources_spi[] = {
	{
		.start	= MSM_SPI_PHYS,
		.end	= MSM_SPI_PHYS + MSM_SPI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.name	= "irq_in",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.name	= "irq_out",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.name	= "irq_err",
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_spi = {
	.name		= "msm_spi",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_spi),
	.resource	= resources_spi,
};

static struct resource msm_vidc_720p_resources[] = {
	{
		.start	= 0xA3B00000,
		.end	= 0xA3B00000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MFC720,
		.end	= INT_MFC720,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_vidc_720p = {
	.name = "msm_vidc_720p",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_vidc_720p_resources),
	.resource = msm_vidc_720p_resources,
};

#ifdef CONFIG_SPI_QSD_NEW
static struct resource qsd_spi_resources[] = {
        {
                .name   = "spi_irq_in",
                .start  = INT_SPI_INPUT,
                .end    = INT_SPI_INPUT,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_irq_out",
                .start  = INT_SPI_OUTPUT,
                .end    = INT_SPI_OUTPUT,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_irq_err",
                .start  = INT_SPI_ERROR,
                .end    = INT_SPI_ERROR,
                .flags  = IORESOURCE_IRQ,
        },
        {
                .name   = "spi_base",
                .start  = 0xA8000000,
                .end    = 0xA8000000 + SZ_4K - 1,
                .flags  = IORESOURCE_MEM,
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

struct platform_device qsdnew_device_spi = {
        .name           = "spi_qsd_new",
        .id             = 0,
        .num_resources  = ARRAY_SIZE(qsd_spi_resources),
        .resource       = qsd_spi_resources,
};
#endif

#ifdef CONFIG_I2C_SSBI
#define MSM_SSBI6_PHYS  0xAD900000
static struct resource msm_ssbi6_resources[] = {
        {
                .name   = "ssbi_base",
                .start  = MSM_SSBI6_PHYS,
                .end    = MSM_SSBI6_PHYS + SZ_4K - 1,
                .flags  = IORESOURCE_MEM,
        },
};

struct platform_device msm_device_ssbi6 = {
        .name           = "i2c_ssbi",
        .id             = 6,
        .num_resources  = ARRAY_SIZE(msm_ssbi6_resources),
        .resource       = msm_ssbi6_resources,
};

#define MSM_SSBI7_PHYS  0xAC800000
static struct resource msm_ssbi7_resources[] = {
        {
                .name   = "ssbi_base",
                .start  = MSM_SSBI7_PHYS,
                .end    = MSM_SSBI7_PHYS + SZ_4K - 1,
                .flags  = IORESOURCE_MEM,
        },
};

struct platform_device msm_device_ssbi7 = {
        .name           = "i2c_ssbi",
        .id             = 7,
        .num_resources  = ARRAY_SIZE(msm_ssbi7_resources),
        .resource       = msm_ssbi7_resources,
};
#endif /* CONFIG_I2C_SSBI */

struct clk msm_clocks_7x30[] = {
	CLK_PCOM("adm_clk",	ADM_CLK,	NULL, 0),
	CLK_PCOM("adsp_clk",	ADSP_CLK,	NULL, 0),
	CLK_PCOM("cam_m_clk",	CAM_M_CLK,	NULL, 0),
	CLK_PCOM("ebi1_clk",	EBI1_CLK,	NULL, CLK_MIN | CLKFLAG_SHARED),
	CLK_PCOM("ecodec_clk",	ECODEC_CLK,	NULL, 0),
	CLK_PCOM("emdh_clk",	EMDH_CLK,	&msm_device_mddi1.dev, OFF | CLK_MINMAX),
	CLK_PCOM("emdh_pclk",	EMDH_P_CLK,	&msm_device_mddi1.dev, OFF),
	CLK_PCOM("gp_clk",	GP_CLK,		NULL, 0),
	CLK_PCOM("grp_2d_clk",	GRP_2D_CLK,	NULL, 0),
	CLK_PCOM("grp_2d_pclk",	GRP_2D_P_CLK,	NULL, 0),
	CLK_PCOM("grp_clk",	GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("grp_pclk",	GRP_3D_P_CLK,	NULL, 0),
	CLK_7X30S("grp_src_clk", GRP_3D_SRC_CLK, GRP_3D_CLK,	NULL, 0),
	CLK_PCOM("hdmi_clk",	HDMI_CLK,	NULL, 0),
	CLK_PCOM("imem_clk",	IMEM_CLK,	NULL, OFF),
	CLK_PCOM("i2c_clk",	I2C_CLK,	&msm_device_i2c.dev, OFF),
	CLK_PCOM("i2c_clk",	I2C_2_CLK,	&msm_device_i2c_2.dev, OFF),
	CLK_PCOM("jpeg_clk",	JPEG_CLK,	NULL, OFF),
	CLK_PCOM("jpeg_pclk",	JPEG_P_CLK,	NULL, OFF),
	CLK_PCOM("lpa_codec_clk",	LPA_CODEC_CLK,		NULL, 0),
	CLK_PCOM("lpa_core_clk",	LPA_CORE_CLK,		NULL, 0),
	CLK_PCOM("lpa_pclk",		LPA_P_CLK,		NULL, 0),
	CLK_PCOM("mdc_clk",	MDC_CLK,	NULL, 0),
	CLK_PCOM("mddi_clk",	PMDH_CLK,	&msm_device_mddi0.dev, OFF | CLK_MINMAX),
	CLK_PCOM("mddi_pclk",	PMDH_P_CLK,	&msm_device_mddi0.dev, OFF | CLK_MINMAX),
	CLK_PCOM("mdp_clk",	MDP_CLK,	&msm_device_mdp.dev, OFF),
	CLK_PCOM("mdp_pclk",	MDP_P_CLK,	&msm_device_mdp.dev, OFF),
	CLK_PCOM("lcdc_pclk_clk", MDP_LCDC_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("lcdc_pad_pclk_clk", MDP_LCDC_PAD_PCLK_CLK, &msm_device_mdp.dev, 0),
	CLK_PCOM("mdp_vsync_clk",	MDP_VSYNC_CLK,  &msm_device_mdp.dev, 0),
	CLK_PCOM("mfc_clk",		MFC_CLK,		NULL, 0),
	CLK_PCOM("mfc_div2_clk",	MFC_DIV2_CLK,		NULL, 0),
	CLK_PCOM("mfc_pclk",		MFC_P_CLK,		NULL, 0),
	CLK_PCOM("mi2s_m_clk",		MI2S_M_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_s_clk",		MI2S_S_CLK,  		NULL, 0),
	CLK_PCOM("mi2s_codec_rx_mclk",	MI2S_CODEC_RX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_rx_sclk",	MI2S_CODEC_RX_S_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_mclk",	MI2S_CODEC_TX_M_CLK,  NULL, 0),
	CLK_PCOM("mi2s_codec_tx_sclk",	MI2S_CODEC_TX_S_CLK,  NULL, 0),
	CLK_PCOM("pbus_clk",	PBUS_CLK,	NULL, CLK_MIN),
	CLK_PCOM("pcm_clk",	PCM_CLK,	NULL, 0),
	CLK_PCOM("rotator_clk",	AXI_ROTATOR_CLK,		NULL, 0),
	CLK_PCOM("rotator_imem_clk",	ROTATOR_IMEM_CLK,	NULL, OFF),
	CLK_PCOM("rotator_pclk",	ROTATOR_P_CLK,		NULL, OFF),
	CLK_PCOM("sdac_clk",	SDAC_CLK,	NULL, OFF),
	CLK_PCOM("sdc_clk",	SDC1_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC1_P_CLK,	&msm_device_sdc1.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC2_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC2_P_CLK,	&msm_device_sdc2.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC3_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC3_P_CLK,	&msm_device_sdc3.dev, OFF),
	CLK_PCOM("sdc_clk",	SDC4_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("sdc_pclk",	SDC4_P_CLK,	&msm_device_sdc4.dev, OFF),
	CLK_PCOM("spi_clk",	SPI_CLK,	&msm_device_spi.dev, 0),
	CLK_PCOM("spi_pclk",	SPI_P_CLK,	&msm_device_spi.dev, 0),
	CLK_7X30S("tv_src_clk",	TV_CLK, 	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("tv_dac_clk",	TV_DAC_CLK,	NULL, 0),
	CLK_PCOM("tv_enc_clk",	TV_ENC_CLK,	NULL, 0),
	CLK_PCOM("uart_clk",	UART1_CLK,	&msm_device_uart1.dev, OFF),
	CLK_PCOM("uart_clk",	UART2_CLK,	&msm_device_uart2.dev, OFF),
	CLK_PCOM("uart_clk",	UART3_CLK,	&msm_device_uart3.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART1DM_CLK,	&msm_device_uart_dm1.dev, OFF),
	CLK_PCOM("uartdm_clk",	UART2DM_CLK,	&msm_device_uart_dm2.dev, OFF),
	CLK_PCOM("usb_hs_clk",		USB_HS_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_pclk",		USB_HS_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs_core_clk",	USB_HS_CORE_CLK,	&msm_device_hsusb.dev, OFF),
	CLK_PCOM("usb_hs2_clk",		USB_HS2_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_pclk",	USB_HS2_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs2_core_clk",	USB_HS2_CORE_CLK,	NULL, OFF),
	CLK_PCOM("usb_hs3_clk",		USB_HS3_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_pclk",	USB_HS3_P_CLK,		NULL, OFF),
	CLK_PCOM("usb_hs3_core_clk",	USB_HS3_CORE_CLK,	NULL, OFF),
	CLK_PCOM("vdc_clk",	VDC_CLK,	NULL, OFF | CLK_MIN),
	CLK_PCOM("vfe_camif_clk",	VFE_CAMIF_CLK, 	NULL, 0),
	CLK_PCOM("vfe_clk",	VFE_CLK,	NULL, 0),
	CLK_PCOM("vfe_mdc_clk",	VFE_MDC_CLK,	NULL, 0),
	CLK_PCOM("vfe_pclk",	VFE_P_CLK,	NULL, OFF),
	CLK_PCOM("vpe_clk",	VPE_CLK,	NULL, 0),
	CLK_PCOM("qup_clk",	QUP_I2C_CLK,	&msm_device_qup_i2c.dev, OFF),
	CLK_PCOM("qup_pclk",	QUP_I2C_P_CLK,	&msm_device_qup_i2c.dev, OFF),

	/* 7x30 v2 hardware only. */
	CLK_PCOM("csi_clk",	CSI0_CLK,	NULL, 0),
	CLK_PCOM("csi_pclk",	CSI0_P_CLK,	NULL, 0),
	CLK_PCOM("csi_vfe_clk",	CSI0_VFE_CLK,	NULL, 0),
};

unsigned msm_num_clocks_7x30 = ARRAY_SIZE(msm_clocks_7x30);

#ifdef CONFIG_MSM_RMT_STORAGE_SERVER
#define RAMFS_INFO_MAGICNUMBER          0x654D4D43
#define RAMFS_INFO_VERSION              0x00000001
#define RAMFS_MODEMSTORAGE_ID           0x4D454653

static void __init msm_register_device(struct platform_device *pdev, void *data)
{
        int ret;

        pdev->dev.platform_data = data;

        ret = platform_device_register(pdev);
        if (ret)
                dev_err(&pdev->dev,
                          "%s: platform_device_register() failed = %d\n",
                          __func__, ret);
}

static struct resource rmt_storage_resources[] = {
       {
                .flags  = IORESOURCE_MEM,
       },
};

static struct platform_device rmt_storage_device = {
       .name           = "rmt_storage",
       .id             = -1,
       .num_resources  = ARRAY_SIZE(rmt_storage_resources),
       .resource       = rmt_storage_resources,
};

int __init rmt_storage_add_ramfs(void)
{
        struct shared_ramfs_table *ramfs_table;
        struct shared_ramfs_entry *ramfs_entry;
        int index;

        ramfs_table = smem_alloc(SMEM_SEFS_INFO,
                        sizeof(struct shared_ramfs_table));

        if (!ramfs_table) {
                printk(KERN_WARNING "%s: No RAMFS table in SMEM\n", __func__);
                return -ENOENT;
        }

        if ((ramfs_table->magic_id != (u32) RAMFS_INFO_MAGICNUMBER) ||
                (ramfs_table->version != (u32) RAMFS_INFO_VERSION)) {
                printk(KERN_WARNING "%s: Magic / Version mismatch:, "
                       "magic_id=%#x, format_version=%#x\n", __func__,
                       ramfs_table->magic_id, ramfs_table->version);
                return -ENOENT;
        }

        for (index = 0; index < ramfs_table->entries; index++) {
                ramfs_entry = &ramfs_table->ramfs_entry[index];

                /* Find a match for the Modem Storage RAMFS area */
                if (ramfs_entry->client_id == (u32) RAMFS_MODEMSTORAGE_ID) {
                        printk(KERN_INFO "%s: RAMFS Info (from SMEM): "
                                "Baseaddr = 0x%08x, Size = 0x%08x\n", __func__,
                                ramfs_entry->base_addr, ramfs_entry->size);
                        rmt_storage_resources[0].start = ramfs_entry->base_addr;
                        rmt_storage_resources[0].end = ramfs_entry->base_addr +
                                                        ramfs_entry->size - 1;
                        msm_register_device(&rmt_storage_device, ramfs_entry);
                        return 0;
                }
        }
        return -ENOENT;
}
#endif

#if defined(CONFIG_ARCH_MSM7X30)
#define GPIO_I2C_CLK 70
#define GPIO_I2C_DAT 71
#elif defined(CONFIG_ARCH_QSD8X50)
#define GPIO_I2C_CLK 95
#define GPIO_I2C_DAT 96
#else
#define GPIO_I2C_CLK 60
#define GPIO_I2C_DAT 61
#endif

void msm_i2c_gpio_init(void)
{
        gpio_request(GPIO_I2C_CLK, "i2c_clk");
        gpio_request(GPIO_I2C_DAT, "i2c_data");
}

static struct platform_device *msm_serial_devices[] __initdata = {
    &msm_device_uart1,
    &msm_device_uart2,
    &msm_device_uart3,
    #ifdef CONFIG_SERIAL_MSM_HS
    &msm_device_uart_dm1,
    &msm_device_uart_dm2,
    #endif
};

int __init msm_add_serial_devices(unsigned num)
{
    return platform_device_register(msm_serial_devices[num]);
}

static struct android_pmem_platform_data pmem_pdata = {
    .name = "pmem",
    .no_allocator = 1,
    .cached = 1,
};

static struct android_pmem_platform_data pmem_adsp_pdata = {
    .name = "pmem_adsp",
    .no_allocator = 0,
#if defined(CONFIG_ARCH_MSM7227)
    .cached = 1,
#else
    .cached = 0,
#endif
};

static struct android_pmem_platform_data pmem_camera_pdata = {
    .name = "pmem_camera",
    .no_allocator = 0,
    .cached = 0,
};

static struct platform_device pmem_device = {
    .name = "android_pmem",
    .id = 0,
    .dev = { .platform_data = &pmem_pdata },
};

static struct platform_device pmem_adsp_device = {
    .name = "android_pmem",
    .id = 1,
    .dev = { .platform_data = &pmem_adsp_pdata },
};

static struct platform_device pmem_camera_device = {
    .name = "android_pmem",
    .id = 4,
    .dev = { .platform_data = &pmem_camera_pdata },
};

static struct resource ram_console_resource[] = {
    {
        .flags  = IORESOURCE_MEM,
    }
};

static struct platform_device ram_console_device = {
    .name = "ram_console",
    .id = -1,
    .num_resources  = ARRAY_SIZE(ram_console_resource),
    .resource       = ram_console_resource,
};

#if defined(CONFIG_MSM_HW3D)
static struct resource resources_hw3d[] = {
    {
        .start  = 0xA0000000,
        .end    = 0xA00fffff,
        .flags  = IORESOURCE_MEM,
        .name   = "regs",
    },
    {
        .flags  = IORESOURCE_MEM,
        .name   = "smi",
    },
    {
        .flags  = IORESOURCE_MEM,
        .name   = "ebi",
    },
    {
        .start  = INT_GRAPHICS,
        .end    = INT_GRAPHICS,
        .flags  = IORESOURCE_IRQ,
        .name   = "gfx",
    },
};

static struct platform_device hw3d_device = {
    .name       = "msm_hw3d",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(resources_hw3d),
    .resource   = resources_hw3d,
};
#endif

#if defined(CONFIG_GPU_MSM_KGSL)
static struct resource msm_kgsl_resources[] = {
    {
        .name   = "kgsl_reg_memory",
        .start  = MSM_GPU_REG_PHYS,
        .end    = MSM_GPU_REG_PHYS + MSM_GPU_REG_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "kgsl_phys_memory",
        .flags  = IORESOURCE_MEM,
    },
    {
#ifdef CONFIG_ARCH_MSM7X30
        .name   = "kgsl_yamato_irq",
        .start  = INT_GRP_3D,
        .end    = INT_GRP_3D,
#else
        .start  = INT_GRAPHICS,
        .end    = INT_GRAPHICS,
#endif
        .flags  = IORESOURCE_IRQ,
    },
#ifdef CONFIG_ARCH_MSM7X30
    {
        .name   = "kgsl_g12_reg_memory",
        .start  = MSM_GPU_2D_REG_PHYS, /* Z180 base address */
        .end    = MSM_GPU_2D_REG_PHYS + MSM_GPU_2D_REG_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    {
        .name   = "kgsl_g12_irq",
        .start  = INT_GRP_2D,
        .end    = INT_GRP_2D,
        .flags  = IORESOURCE_IRQ,
    },
#endif
};

#ifdef CONFIG_ARCH_MSM7X30
static struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
    /* NPA Flow IDs */
    .high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
    .high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
    /* AXI rates in KHz */
    .high_axi_3d = 192000,
    .high_axi_2d = 192000,
#endif
    .max_grp2d_freq = 0,
    .min_grp2d_freq = 0,
    .set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
    .max_grp3d_freq = 245000000,
    .min_grp3d_freq = 192000000,
    .set_grp3d_async = set_grp3d_async,
};
#endif

static struct platform_device msm_kgsl_device = {
    .name       = "kgsl",
    .id     = -1,
    .resource   = msm_kgsl_resources,
    .num_resources  = ARRAY_SIZE(msm_kgsl_resources),
#ifdef CONFIG_ARCH_MSM7X30
    .dev = {
        .platform_data = &kgsl_pdata,
    },
#endif
};

#if !defined(CONFIG_ARCH_MSM7X30)
#define PWR_RAIL_GRP_CLK               8
static int kgsl_power_rail_mode(int follow_clk)
{
       int mode = follow_clk ? 0 : 1;
       int rail_id = PWR_RAIL_GRP_CLK;

       return msm_proc_comm(PCOM_CLKCTL_RPC_RAIL_CONTROL, &rail_id, &mode);
}

static int kgsl_power(bool on)
{
       int cmd;
       int rail_id = PWR_RAIL_GRP_CLK;

       cmd = on ? PCOM_CLKCTL_RPC_RAIL_ENABLE : PCOM_CLKCTL_RPC_RAIL_DISABLE;
       return msm_proc_comm(cmd, &rail_id, NULL);
}
#endif
#endif

void __init msm_add_mem_devices(struct msm_pmem_setting *setting)
{
    if (setting->pmem_size) {
        pmem_pdata.start = setting->pmem_start;
        pmem_pdata.size = setting->pmem_size;
        platform_device_register(&pmem_device);
    }

    if (setting->pmem_adsp_size) {
        pmem_adsp_pdata.start = setting->pmem_adsp_start;
        pmem_adsp_pdata.size = setting->pmem_adsp_size;
        platform_device_register(&pmem_adsp_device);
    }

#if defined(CONFIG_MSM_HW3D)
    if (setting->pmem_gpu0_size && setting->pmem_gpu1_size) {
        struct resource *res;

        res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
                           "smi");
        res->start = setting->pmem_gpu0_start;
        res->end = res->start + setting->pmem_gpu0_size - 1;

        res = platform_get_resource_byname(&hw3d_device, IORESOURCE_MEM,
                           "ebi");
        res->start = setting->pmem_gpu1_start;
        res->end = res->start + setting->pmem_gpu1_size - 1;
        platform_device_register(&hw3d_device);
    }
#endif

    if (setting->pmem_camera_size) {
        pmem_camera_pdata.start = setting->pmem_camera_start;
        pmem_camera_pdata.size = setting->pmem_camera_size;
        platform_device_register(&pmem_camera_device);
    }

    if (setting->ram_console_size) {
        ram_console_resource[0].start = setting->ram_console_start;
        ram_console_resource[0].end = setting->ram_console_start
            + setting->ram_console_size - 1;
        platform_device_register(&ram_console_device);
    }

#if defined(CONFIG_GPU_MSM_KGSL)
    if (setting->kgsl_size) {
        msm_kgsl_resources[1].start = setting->kgsl_start;
        msm_kgsl_resources[1].end = setting->kgsl_start
            + setting->kgsl_size - 1;
/* due to 7x30 gpu hw bug, we have to apply clk
 * first then power on gpu, thus we move power on
 * into kgsl driver
 */
#if !defined(CONFIG_ARCH_MSM7X30)
        kgsl_power_rail_mode(0);
        kgsl_power(true);
#endif
        platform_device_register(&msm_kgsl_device);
    }
#endif
}

