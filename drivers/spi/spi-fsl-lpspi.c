/*
 * Freescale i.MX7ULP LPSPI driver
 *
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define DEBUG

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/platform_data/spi-imx.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#define DRIVER_NAME "fsl_lpspi"

#define FSL_LPSPI_RPM_TIMEOUT 50 /* 50ms */

/* i.MX7ULP LPSPI registers */
#define IMX7ULP_VERID	0x0
#define IMX7ULP_PARAM	0x4
#define IMX7ULP_CR	0x10
#define IMX7ULP_SR	0x14
#define IMX7ULP_IER	0x18
#define IMX7ULP_DER	0x1c
#define IMX7ULP_CFGR0	0x20
#define IMX7ULP_CFGR1	0x24
#define IMX7ULP_DMR0	0x30
#define IMX7ULP_DMR1	0x34
#define IMX7ULP_CCR	0x40
#define IMX7ULP_FCR	0x58
#define IMX7ULP_FSR	0x5c
#define IMX7ULP_TCR	0x60
#define IMX7ULP_TDR	0x64
#define IMX7ULP_RSR	0x70
#define IMX7ULP_RDR	0x74

/* General control register field define */
#define CR_RRF		(1 << 9)
#define CR_RTF		(1 << 8)
#define CR_RST		(1 << 1)
#define CR_MEN		(1 << 0)
#define SR_TCF		(1 << 10)
#define SR_RDF		(1 << 1)
#define SR_TDF		(1 << 0)
#define IER_TCIE	(1 << 10)
#define IER_RDIE	(1 << 1)
#define IER_TDIE	(1 << 0)
#define CFGR1_PCSCFG	(1 << 27)	/* PCS[3:2] disabled */
#define CFGR1_PCSPOL	(1 << 8)	/* PCS active high */
#define CFGR1_NOSTALL   (1 << 3)	/* NO STALL */
#define CFGR1_SAMPLE	(1 << 1)	/* SAMPLE POINT*/
#define CFGR1_MASTER	(1 << 0)	/* MASTER MODE */
#define RSR_RXEMPTY	(1 << 1)
#define TCR_CPOL	(1 << 31)
#define TCR_CPHA	(1 << 30)
#define TCR_CONT	(1 << 21)
#define TCR_CONTC	(1 << 20)
#define TCR_RXMSK	(1 << 19)
#define TCR_TXMSK	(1 << 18)

static int clkdivs[] = {1, 2, 4, 8, 16, 32, 64, 128};

struct lpspi_config {
	u8 bpw;
	u8 chip_select;
	u8 prescale;
	u16 mode;
	u32 speed_hz;
};

struct fsl_lpspi_data {
	struct spi_bitbang bitbang;
	struct device *dev;

	struct completion xfer_done;
	void __iomem *base;
	struct clk *clk_per;
	struct clk *clk_ipg;

	void *rx_buf;
	const void *tx_buf;
	void (*tx)(struct fsl_lpspi_data *);
	void (*rx)(struct fsl_lpspi_data *);
	struct lpspi_config config;

	u8 txfifosize;
	u8 rxfifosize;
	unsigned remain;

	int chipselect[0];
};

static const struct of_device_id fsl_lpspi_dt_ids[] = {
	{ .compatible = "fsl,imx7ulp-spi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_lpspi_dt_ids);

#define LPSPI_BUF_RX(type)						\
static void fsl_lpspi_buf_rx_##type(struct fsl_lpspi_data *fsl_lpspi)	\
{									\
	unsigned int val = readl(fsl_lpspi->base + IMX7ULP_RDR);	\
									\
	if (fsl_lpspi->rx_buf) {					\
		*(type *)fsl_lpspi->rx_buf = val;			\
		fsl_lpspi->rx_buf += sizeof(type);                      \
	}								\
}

#define LPSPI_BUF_TX(type)						\
static void fsl_lpspi_buf_tx_##type(struct fsl_lpspi_data *fsl_lpspi)	\
{									\
	type val = 0;							\
									\
	if (fsl_lpspi->tx_buf) {					\
		val = *(type *)fsl_lpspi->tx_buf;			\
		fsl_lpspi->tx_buf += sizeof(type);			\
	}								\
									\
	fsl_lpspi->remain -= sizeof(type);				\
	writel(val, fsl_lpspi->base + IMX7ULP_TDR);			\
}

LPSPI_BUF_RX(u8)
LPSPI_BUF_TX(u8)
LPSPI_BUF_RX(u16)
LPSPI_BUF_TX(u16)
LPSPI_BUF_RX(u32)
LPSPI_BUF_TX(u32)

static void fsl_lpspi_intctrl(
		struct fsl_lpspi_data *fsl_lpspi, unsigned int enable)
{
	writel(enable, fsl_lpspi->base + IMX7ULP_IER);
}

static void fsl_lpspi_write_tx_fifo(struct fsl_lpspi_data *fsl_lpspi)
{
	u8 txfifo_cnt;
	u32 temp;

	txfifo_cnt = readl(fsl_lpspi->base + IMX7ULP_FSR) & 0xff;

	while (txfifo_cnt < fsl_lpspi->txfifosize) {
		if (!fsl_lpspi->remain)
			break;
		fsl_lpspi->tx(fsl_lpspi);
		txfifo_cnt++;
	}

	if (txfifo_cnt < fsl_lpspi->txfifosize) {
		temp = readl(fsl_lpspi->base + IMX7ULP_TCR);
		temp &= ~TCR_CONTC;
		writel(temp, fsl_lpspi->base + IMX7ULP_TCR);

		fsl_lpspi_intctrl(fsl_lpspi, IER_TCIE);
	} else
		fsl_lpspi_intctrl(fsl_lpspi, IER_TDIE);
}

static void fsl_lpspi_read_rx_fifo(struct fsl_lpspi_data *fsl_lpspi)
{
	while (!(readl(fsl_lpspi->base + IMX7ULP_RSR) & RSR_RXEMPTY))
		fsl_lpspi->rx(fsl_lpspi);
}

static void fsl_lpspi_set_cmd(struct fsl_lpspi_data *fsl_lpspi)
{
	u32 temp = 0;

	temp |= TCR_CONT;
	temp |= fsl_lpspi->config.bpw - 1;
	temp |= fsl_lpspi->config.prescale << 27;
	temp |= (fsl_lpspi->config.mode & 0x3) << 30;
	temp |= (fsl_lpspi->config.chip_select & 0x3) << 24;

	writel(temp, fsl_lpspi->base + IMX7ULP_TCR);

	dev_dbg(fsl_lpspi->dev, "TCR=0x%x\n", temp);
}

static void fsl_lpspi_set_watermark(struct fsl_lpspi_data *fsl_lpspi)
{
	u32 temp;
	u8 txwatermark, rxwatermark;

	temp = readl(fsl_lpspi->base + IMX7ULP_PARAM);

	fsl_lpspi->txfifosize = 1 << (temp & 0x0f);
	fsl_lpspi->rxfifosize = 1 << ((temp >> 8) & 0x0f);

	rxwatermark = fsl_lpspi->txfifosize >> 1;
	txwatermark = fsl_lpspi->rxfifosize >> 1;
	temp = txwatermark | rxwatermark << 16;

	writel(temp, fsl_lpspi->base + IMX7ULP_FCR);

	dev_dbg(fsl_lpspi->dev, "FCR=0x%x\n", temp);
}

static int fsl_lpspi_set_bitrate(struct fsl_lpspi_data *fsl_lpspi)
{
	u8 prescale;
	unsigned int perclk_rate, scldiv;
	struct lpspi_config config = fsl_lpspi->config;

	perclk_rate = clk_get_rate(fsl_lpspi->clk_per);

	for (prescale = 0; prescale < 8; prescale++) {
		scldiv = perclk_rate / (clkdivs[prescale] * config.speed_hz) - 2;
		if (scldiv < 256) {
			fsl_lpspi->config.prescale = prescale;
			break;
		}
	}

	if (prescale == 8 && scldiv >= 256)
		return -EINVAL;

	writel(scldiv | (scldiv << 8), fsl_lpspi->base + IMX7ULP_CCR);

	dev_dbg(fsl_lpspi->dev, "perclk=%d, speed=%d, prescale =%d, scldiv=%d\n",
		perclk_rate, config.speed_hz, prescale, scldiv);

	return 0;
}

static int fsl_lpspi_config(struct fsl_lpspi_data *fsl_lpspi)
{
	int ret;
	u32 temp;

	temp = CR_RST;
	writel(temp, fsl_lpspi->base + IMX7ULP_CR);
	writel(0, fsl_lpspi->base + IMX7ULP_CR);

	ret = fsl_lpspi_set_bitrate(fsl_lpspi);
	if (ret)
		return ret;

	fsl_lpspi_set_watermark(fsl_lpspi);

	temp = CFGR1_PCSCFG | CFGR1_MASTER
		| CFGR1_SAMPLE | CFGR1_NOSTALL;

	/* chip select polarity */
	if (fsl_lpspi->config.mode & SPI_CS_HIGH)
		temp |= CFGR1_PCSPOL;

	writel(temp, fsl_lpspi->base + IMX7ULP_CFGR1);

	temp = readl(fsl_lpspi->base + IMX7ULP_CR);
	temp |= CR_RRF | CR_RTF | CR_MEN;
	writel(temp, fsl_lpspi->base + IMX7ULP_CR);

	return 0;
}

static irqreturn_t fsl_lpspi_isr(int irq, void *dev_id)
{
	u32 temp;
	struct fsl_lpspi_data *fsl_lpspi = dev_id;

	fsl_lpspi_intctrl(fsl_lpspi, 0);
	temp = readl(fsl_lpspi->base + IMX7ULP_SR);

	fsl_lpspi_read_rx_fifo(fsl_lpspi);

	if ((temp & SR_TDF) && !(temp & SR_TCF)) {
		fsl_lpspi_write_tx_fifo(fsl_lpspi);
		return IRQ_HANDLED;
	}

	if (temp & SR_TCF) {
		complete(&fsl_lpspi->xfer_done);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void fsl_lpspi_chipselect(struct spi_device *spi, int is_active)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(spi->master);
	int gpio = fsl_lpspi->chipselect[spi->chip_select];
	int active = is_active != BITBANG_CS_INACTIVE;
	int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

	if (!gpio_is_valid(gpio))
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
}

static int fsl_lpspi_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(spi->master);

	fsl_lpspi->config.mode = spi->mode;
	fsl_lpspi->config.bpw = t ? t->bits_per_word : spi->bits_per_word;
	fsl_lpspi->config.speed_hz = t ? t->speed_hz : spi->max_speed_hz;
	fsl_lpspi->config.chip_select = spi->chip_select;

	if (!fsl_lpspi->config.speed_hz)
		fsl_lpspi->config.speed_hz = spi->max_speed_hz;
	if (!fsl_lpspi->config.bpw)
		fsl_lpspi->config.bpw = spi->bits_per_word;

	/* Initialize the functions for transfer */
	if (fsl_lpspi->config.bpw <= 8) {
		fsl_lpspi->rx = fsl_lpspi_buf_rx_u8;
		fsl_lpspi->tx = fsl_lpspi_buf_tx_u8;
	} else if (fsl_lpspi->config.bpw <= 16) {
		fsl_lpspi->rx = fsl_lpspi_buf_rx_u16;
		fsl_lpspi->tx = fsl_lpspi_buf_tx_u16;
	} else {
		fsl_lpspi->rx = fsl_lpspi_buf_rx_u32;
		fsl_lpspi->tx = fsl_lpspi_buf_tx_u32;
	}

	fsl_lpspi_config(fsl_lpspi);

	return 0;
}

static int fsl_lpspi_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(spi->master);

	fsl_lpspi->tx_buf = transfer->tx_buf;
	fsl_lpspi->rx_buf = transfer->rx_buf;
	fsl_lpspi->remain = transfer->len;

	reinit_completion(&fsl_lpspi->xfer_done);
	fsl_lpspi_set_cmd(fsl_lpspi);
	fsl_lpspi_write_tx_fifo(fsl_lpspi);
	wait_for_completion(&fsl_lpspi->xfer_done);

	return transfer->len;
}

/* The following funs are decided by spi framework */

static int fsl_lpspi_setup(struct spi_device *spi)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(spi->master);
	int gpio = fsl_lpspi->chipselect[spi->chip_select];

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	if (gpio_is_valid(gpio))
		gpio_direction_output(gpio,
				      spi->mode & SPI_CS_HIGH ? 0 : 1);

	fsl_lpspi_chipselect(spi, BITBANG_CS_INACTIVE);

	return 0;
}

static void fsl_lpspi_cleanup(struct spi_device *spi)
{
}

static int
fsl_lpspi_prepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(master);
	int ret;

	ret = pm_runtime_get_sync(fsl_lpspi->dev);
	if (ret < 0) {
		dev_err(fsl_lpspi->dev, "failed to enable clock\n");
		return ret;
	}

	return 0;
}

static int
fsl_lpspi_unprepare_message(struct spi_master *master, struct spi_message *msg)
{
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(master);

	pm_runtime_mark_last_busy(fsl_lpspi->dev);
	pm_runtime_put_autosuspend(fsl_lpspi->dev);

	return 0;
}

int fsl_lpspi_runtime_resume(struct device *dev)
{
	struct fsl_lpspi_data *fsl_lpspi = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(fsl_lpspi->clk_per);
	if (ret)
		return ret;

	ret = clk_prepare_enable(fsl_lpspi->clk_ipg);
	if (ret) {
		clk_disable_unprepare(fsl_lpspi->clk_per);
		return ret;
	}

	return 0;
}

int fsl_lpspi_runtime_suspend(struct device *dev)
{
	struct fsl_lpspi_data *fsl_lpspi = dev_get_drvdata(dev);

	clk_disable_unprepare(fsl_lpspi->clk_per);
	clk_disable_unprepare(fsl_lpspi->clk_ipg);

	return 0;
}

static int fsl_lpspi_init_rpm(struct fsl_lpspi_data *fsl_lpspi)
{
	struct device *dev = fsl_lpspi->dev;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, FSL_LPSPI_RPM_TIMEOUT);
	pm_runtime_use_autosuspend(dev);

	return 0;
}

static int fsl_lpspi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct spi_master *master;
	struct fsl_lpspi_data *fsl_lpspi;
	struct spi_imx_master *lpspi_platform_info =
			dev_get_platdata(&pdev->dev);
	struct resource *res;
	int i, ret, irq;

	master = spi_alloc_master(&pdev->dev, sizeof(struct fsl_lpspi_data));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(8, 32);
	master->bus_num = pdev->id;

	fsl_lpspi = spi_master_get_devdata(master);
	fsl_lpspi->bitbang.master = master;
	fsl_lpspi->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, fsl_lpspi);

	for (i = 0; i < master->num_chipselect; i++) {
		int cs_gpio = of_get_named_gpio(np, "cs-gpios", i);

		if (!gpio_is_valid(cs_gpio) && lpspi_platform_info)
			cs_gpio = lpspi_platform_info->chipselect[i];

		fsl_lpspi->chipselect[i] = cs_gpio;
		if (!gpio_is_valid(cs_gpio))
			continue;

		ret = devm_gpio_request(&pdev->dev, fsl_lpspi->chipselect[i],
					DRIVER_NAME);
		if (ret) {
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_master_put;
		}
	}

	fsl_lpspi->bitbang.chipselect = fsl_lpspi_chipselect;
	fsl_lpspi->bitbang.master->setup = fsl_lpspi_setup;
	fsl_lpspi->bitbang.master->cleanup = fsl_lpspi_cleanup;
	fsl_lpspi->bitbang.setup_transfer = fsl_lpspi_setupxfer;
	fsl_lpspi->bitbang.txrx_bufs = fsl_lpspi_transfer;
	fsl_lpspi->bitbang.master->prepare_message = fsl_lpspi_prepare_message;
	fsl_lpspi->bitbang.master->unprepare_message = fsl_lpspi_unprepare_message;
	fsl_lpspi->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	fsl_lpspi->bitbang.master->flags = SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;

	init_completion(&fsl_lpspi->xfer_done);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	fsl_lpspi->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(fsl_lpspi->base)) {
		ret = PTR_ERR(fsl_lpspi->base);
		goto out_master_put;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto out_master_put;
	}

	fsl_lpspi->clk_per = devm_clk_get(&pdev->dev, "per");
	if (IS_ERR(fsl_lpspi->clk_per)) {
		ret = PTR_ERR(fsl_lpspi->clk_per);
		goto out_master_put;
	}

	fsl_lpspi->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(fsl_lpspi->clk_ipg)) {
		ret = PTR_ERR(fsl_lpspi->clk_ipg);
		goto out_master_put;
	}

	/* enable the clock */
	ret = fsl_lpspi_init_rpm(fsl_lpspi);
	if (ret)
		goto out_master_put;

	ret = devm_request_irq(&pdev->dev, irq, fsl_lpspi_isr, 0,
			       dev_name(&pdev->dev), fsl_lpspi);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", irq, ret);
		goto out_master_put;
	}

	master->dev.of_node = pdev->dev.of_node;
	ret = spi_bitbang_start(&fsl_lpspi->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_master_put;
	}

	dev_info(fsl_lpspi->dev, "lpspi probed\n");

	return ret;

out_master_put:
	spi_master_put(master);

	return ret;
}

static int fsl_lpspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct fsl_lpspi_data *fsl_lpspi = spi_master_get_devdata(master);

	spi_bitbang_stop(&fsl_lpspi->bitbang);
	pm_runtime_disable(fsl_lpspi->dev);

	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int fsl_lpspi_suspend(struct device *dev)
{
	int ret;

	pinctrl_pm_select_sleep_state(dev);
	ret = pm_runtime_force_suspend(dev);
	return ret;
}

static int fsl_lpspi_resume(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret) {
		dev_err(dev, "Error in resume: %d\n", ret);
		return ret;
	}

	pinctrl_pm_select_default_state(dev);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_lpspi_pm_ops = {
	SET_RUNTIME_PM_OPS(fsl_lpspi_runtime_suspend, fsl_lpspi_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_lpspi_suspend, fsl_lpspi_resume)
};

static struct platform_driver fsl_lpspi_driver = {
	.driver = {
			.name = DRIVER_NAME,
			.of_match_table = fsl_lpspi_dt_ids,
			.pm = &fsl_lpspi_pm_ops,
		},
	.probe = fsl_lpspi_probe,
	.remove = fsl_lpspi_remove,
};
module_platform_driver(fsl_lpspi_driver);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_AUTHOR("Gao Pan <pandy.gao@nxp.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
