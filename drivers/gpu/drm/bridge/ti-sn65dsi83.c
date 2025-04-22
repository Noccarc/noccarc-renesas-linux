// SPDX-License-Identifier: GPL-2.0
/*
 * SN65DSI83 DSI to LVDS bridge driver
 *
 * Copyright (C) 2022 Noccarc Robotics Pvt. Ltd.
 * Author: Deepak Kapure  <deepak.k@noccarc.com>
 *
 */
/* #define DEBUG */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <asm/unaligned.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>


/* ID registers */
#define REG_ID(n)				                (0x00 + (n))
/* Reset and clock registers */
#define REG_RC_RESET				            0x09
#define  REG_RC_RESET_SOFT_RESET		        BIT(0)
#define REG_RC_LVDS_PLL				            0x0a
#define  REG_RC_LVDS_PLL_PLL_EN_STAT		    BIT(7)
#define  REG_RC_LVDS_PLL_LVDS_CLK_RANGE(n)	    (((n) & 0x7) << 1)
#define  REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY	    BIT(0)
#define REG_RC_DSI_CLK				            0x0b
#define  REG_RC_DSI_CLK_DSI_CLK_DIVIDER(n)	    (((n) & 0x1f) << 3)
#define  REG_RC_DSI_CLK_REFCLK_MULTIPLIER(n)	((n) & 0x3)
#define REG_RC_PLL_EN				            0x0d
#define  REG_RC_PLL_EN_PLL_EN			        BIT(0)
/* DSI registers */
#define REG_DSI_LANE				            0x10
#define  REG_DSI_LANE_LEFT_RIGHT_PIXELS		    BIT(7)	      /* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_DUAL	    0	          /* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_2SINGLE	BIT(6)	      /* DSI85-only */
#define  REG_DSI_LANE_DSI_CHANNEL_MODE_SINGLE	BIT(5)
#define  REG_DSI_LANE_CHA_DSI_LANES(n)		    (((n) & 0x3) << 3)
#define  REG_DSI_LANE_CHB_DSI_LANES(n)		    (((n) & 0x3) << 1)
#define  REG_DSI_LANE_SOT_ERR_TOL_DIS		    BIT(0)
#define REG_DSI_EQ				                0x11
#define  REG_DSI_EQ_CHA_DSI_DATA_EQ(n)		    (((n) & 0x3) << 6)
#define  REG_DSI_EQ_CHA_DSI_CLK_EQ(n)		    (((n) & 0x3) << 2)
#define REG_DSI_CLK				                0x12
#define  REG_DSI_CLK_CHA_DSI_CLK_RANGE(n)	    ((n) & 0xff)
/* LVDS registers */
#define REG_LVDS_FMT				            0x18
#define  REG_LVDS_FMT_DE_NEG_POLARITY		    BIT(7)
#define  REG_LVDS_FMT_HS_NEG_POLARITY		    BIT(6)
#define  REG_LVDS_FMT_VS_NEG_POLARITY		    BIT(5)
#define  REG_LVDS_FMT_LVDS_LINK_CFG		        BIT(4)	      /* 0:AB 1:A-only */
#define  REG_LVDS_FMT_CHA_24BPP_MODE		    BIT(3)
#define  REG_LVDS_FMT_CHB_24BPP_MODE		    BIT(2)
#define  REG_LVDS_FMT_CHA_24BPP_FORMAT1		    BIT(1)
#define  REG_LVDS_FMT_CHB_24BPP_FORMAT1		    BIT(0)
#define REG_LVDS_VCOM				            0x19
#define  REG_LVDS_VCOM_CHA_LVDS_VOCM		    BIT(6)
#define  REG_LVDS_VCOM_CHB_LVDS_VOCM		    BIT(4)
#define  REG_LVDS_VCOM_CHA_LVDS_VOD_SWING(n)	(((n) & 0x3) << 2)
#define  REG_LVDS_VCOM_CHB_LVDS_VOD_SWING(n)	((n) & 0x3)
#define REG_LVDS_LANE				            0x1a
#define  REG_LVDS_LANE_EVEN_ODD_SWAP		    BIT(6)
#define  REG_LVDS_LANE_CHA_REVERSE_LVDS		    BIT(5)
#define  REG_LVDS_LANE_CHB_REVERSE_LVDS		    BIT(4)
#define  REG_LVDS_LANE_CHA_LVDS_TERM		    BIT(1)
#define  REG_LVDS_LANE_CHB_LVDS_TERM		    BIT(0)
#define REG_LVDS_CM				                0x1b
#define  REG_LVDS_CM_CHA_LVDS_CM_ADJUST(n)	    (((n) & 0x3) << 4)
#define  REG_LVDS_CM_CHB_LVDS_CM_ADJUST(n)	    ((n) & 0x3)
/* Video registers */
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW	    0x20
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH	    0x21
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW	0x24
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH	0x25
#define REG_VID_CHA_SYNC_DELAY_LOW		        0x28
#define REG_VID_CHA_SYNC_DELAY_HIGH		        0x29
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW	    0x2c
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH	    0x2d
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW	    0x30
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH	    0x31
#define REG_VID_CHA_HORIZONTAL_BACK_PORCH	    0x34
#define REG_VID_CHA_VERTICAL_BACK_PORCH		    0x36
#define REG_VID_CHA_HORIZONTAL_FRONT_PORCH	    0x38
#define REG_VID_CHA_VERTICAL_FRONT_PORCH	    0x3a
#define REG_VID_CHA_TEST_PATTERN		        0x3c
/* IRQ registers */
#define REG_IRQ_GLOBAL				            0xe0
#define  REG_IRQ_GLOBAL_IRQ_EN			        BIT(0)
#define REG_IRQ_EN				                0xe1
#define  REG_IRQ_EN_CHA_SYNCH_ERR_EN		    BIT(7)
#define  REG_IRQ_EN_CHA_CRC_ERR_EN		        BIT(6)
#define  REG_IRQ_EN_CHA_UNC_ECC_ERR_EN		    BIT(5)
#define  REG_IRQ_EN_CHA_COR_ECC_ERR_EN		    BIT(4)
#define  REG_IRQ_EN_CHA_LLP_ERR_EN		        BIT(3)
#define  REG_IRQ_EN_CHA_SOT_BIT_ERR_EN		    BIT(2)
#define  REG_IRQ_EN_CHA_PLL_UNLOCK_EN		    BIT(0)
#define REG_IRQ_STAT				            0xe5
#define  REG_IRQ_STAT_CHA_SYNCH_ERR		        BIT(7)
#define  REG_IRQ_STAT_CHA_CRC_ERR		        BIT(6)
#define  REG_IRQ_STAT_CHA_UNC_ECC_ERR		    BIT(5)
#define  REG_IRQ_STAT_CHA_COR_ECC_ERR		    BIT(4)
#define  REG_IRQ_STAT_CHA_LLP_ERR		        BIT(3)
#define  REG_IRQ_STAT_CHA_SOT_BIT_ERR		    BIT(2)
#define  REG_IRQ_STAT_CHA_PLL_UNLOCK		    BIT(0)

#define SINGLE_LINK		                        1		
#define DUAL_LINK		                        2		


enum sn65dsi83_ports {
	SN65DSI83_DSI_IN,
	SN65DSI83_LVDS_OUT0,
	SN65DSI83_LVDS_OUT1,
};

struct ti_data {
	struct i2c_client	*i2c;
	struct device		*dev;

	struct drm_bridge	bridge;
	struct drm_bridge	*panel_bridge;

	struct device_node       *host_node;
	struct mipi_dsi_device   *dsi;
	u8 num_dsi_lanes;

	struct regulator	*vdd;
	struct regulator	*vcc;
	struct gpio_desc	*enable_gpio;
	u8			lvds_link; /* single-link or dual-link */
	u8			bpc;
};

static inline struct ti_data *bridge_to_ti(struct drm_bridge *b)
{
	return container_of(b, struct ti_data, bridge);
}

static void ti_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct ti_data *ti = bridge_to_ti(bridge);
	struct device *dev = &ti->dsi->dev;
	int ret;
    
	ret = regulator_enable(ti->vcc);
	if (ret < 0)
		dev_err(dev, "regulator vcc enable failed, %d\n", ret);

	ret = regulator_enable(ti->vdd);
	if (ret < 0)
		dev_err(dev, "regulator vdd enable failed, %d\n", ret);

	/*
	 * Reset the chip, pull EN line low for t_reset=10ms,
	 * then high for t_en=1ms.
	 */
	gpiod_set_value(ti->enable_gpio, 0);
	usleep_range(1000, 1100);
	gpiod_set_value(ti->enable_gpio, 1);
	usleep_range(10000, 11000);

}

static void ti_bridge_post_disable(struct drm_bridge *bridge)
{
	struct ti_data *ti = bridge_to_ti(bridge);
	struct device *dev = &ti->dsi->dev;
	int ret;

	gpiod_set_value(ti->enable_gpio, 0);
	usleep_range(10, 20);

	ret = regulator_disable(ti->vdd);
	if (ret < 0)
		dev_err(dev, "regulator vdd disable failed, %d\n", ret);
	usleep_range(10000, 11000);

	ret = regulator_disable(ti->vcc);
	if (ret < 0)
		dev_err(dev, "regulator vcc disable failed, %d\n", ret);
	usleep_range(10000, 11000);
}

static u8 sn65dsi83_get_lvds_pll(const struct drm_display_mode *mode)
{
	/*
	 * The encoding of the LVDS_CLK_RANGE is as follows:
	 * 000 - 25 MHz <= LVDS_CLK < 37.5 MHz
	 * 001 - 37.5 MHz <= LVDS_CLK < 62.5 MHz
	 * 010 - 62.5 MHz <= LVDS_CLK < 87.5 MHz
	 * 011 - 87.5 MHz <= LVDS_CLK < 112.5 MHz
	 * 100 - 112.5 MHz <= LVDS_CLK < 137.5 MHz
	 * 101 - 137.5 MHz <= LVDS_CLK <= 154 MHz
	 * which is a range of 12.5MHz..162.5MHz in 50MHz steps, except that
	 * the ends of the ranges are clamped to the supported range. Since
	 * sn65dsi83_mode_valid() already filters the valid modes and limits
	 * the clock to 25..154 MHz, the range calculation can be simplified
	 * as follows:
	 */
	int mode_clock = mode->clock;

	return (mode_clock - 12500) / 25000;
}
static u8 sn65dsi83_get_dsi_clk(struct ti_data *ti,
				  const struct drm_display_mode *mode)
{
	/*
	 * The encoding of the CHA_DSI_CLK_RANGE is as follows:
	 * 0x00 through 0x07 - Reserved
	 * 0x08 - 40 <= DSI_CLK < 45 MHz
	 * 0x09 - 45 <= DSI_CLK < 50 MHz
	 * ...
	 * 0x63 - 495 <= DSI_CLK < 500 MHz
	 * 0x64 - 500 MHz
	 * 0x65 through 0xFF - Reserved
	 * which is DSI clock in 5 MHz steps, clamped to 40..500 MHz.
	 * The DSI clock are calculated as:
	 *  DSI_CLK = mode clock * bpp / dsi_data_lanes / 2
	 *  DSI_CLK = mode clock * bpp / dsi_data_lanes / 2
	 * the 2 is there because the bus is DDR.
	 */
	
	return DIV_ROUND_UP(clamp((unsigned int)mode->clock *
			    mipi_dsi_pixel_format_to_bpp(ti->dsi->format) /
			    ti->num_dsi_lanes / 2, 40000U, 500000U), 5000U);
}

static u8 sn65dsi83_get_dsi_div(struct ti_data *ti)
{
	/* The divider is (DSI_CLK / LVDS_CLK) - 1, which really is: */
	unsigned int dsi_div = mipi_dsi_pixel_format_to_bpp(ti->dsi->format);

	dsi_div /= ti->num_dsi_lanes;

	if (ti->lvds_link == SINGLE_LINK)
		dsi_div /= 2;

	return dsi_div - 1;
}

static void d2l_read(struct i2c_client *i2c, u8 addr, u8 *val)
{
	u8 data[1];
	data[0] = addr;
	int ret;

	ret = i2c_master_send(i2c, data, 1);
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(i2c, (u8 *)val,  sizeof(*val));
	if (ret < 0)
		goto fail;
    
	pr_debug("d2l: I2C : addr:%04x value:%08x\n", addr, *val);
    return ;
fail:
	dev_err(&i2c->dev, "Error %d reading from subaddress 0x%x\n",
		ret, addr);
}

static void d2l_write(struct i2c_client *i2c, u8 addr, u8 val)
{
	u8 data[2];
	data[0] = addr;
	data[1] = val;
	int ret;

	ret = i2c_master_send(i2c, data, ARRAY_SIZE(data));
	if (ret < 0)
		dev_err(&i2c->dev, "Error %d writing to subaddress 0x%x\n",
			ret, addr);
}

/* helper function to access bus_formats */
static struct drm_connector *get_connector(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head)
		if (connector->encoder == encoder)
			return connector;

	return NULL;
}

static void ti_bridge_enable(struct drm_bridge *bridge)
{
	struct ti_data *ti = bridge_to_ti(bridge);
	u32 hback_porch, hsync_len, hfront_porch, hactive, htime1, htime2;
	u32 vback_porch, vsync_len, vfront_porch, vactive, vtime1, vtime2;
	u8 val=0, lvds_pll, dsi_clk;
	u16 dsiclk, clkdiv, byteclk, t1, t2, t3, vsdelay;
	bool lvds_format_24bpp;
	bool lvds_format_jeida;           
	bool pll_en_flag = false;
	int i;
	
	struct drm_display_mode *mode;
	struct drm_connector *connector = get_connector(bridge->encoder);
	
	
	 d2l_read(ti->i2c, REG_ID(0), &val);
	 dev_info(ti->dev, "DSI->LVDS V1 Chip ID %02d\n", val & 0xFF);
	
	switch (connector->display_info.bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		lvds_format_24bpp = false;
		lvds_format_jeida = true;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		lvds_format_24bpp = true;
		lvds_format_jeida = true;
		break;
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
		lvds_format_24bpp = true;
		lvds_format_jeida = false;
		break;
	default:
		/*
		 * Some bridges still don't set the correct
		 * LVDS bus pixel format, use SPWG24 default
		 * format until those are fixed.
		 */
		lvds_format_24bpp = true;
		lvds_format_jeida = false;
		dev_info(ti->dev,
			 "Unsupported LVDS bus format 0x%04x, please check output bridge driver. Falling back to SPWG24.\n",
			 connector->display_info.bus_formats[0]);
		break;
	}

	mode = &bridge->encoder->crtc->state->adjusted_mode;
	
	lvds_pll = sn65dsi83_get_lvds_pll(mode);
	
	lvds_pll = ((lvds_pll<<1) | REG_RC_LVDS_PLL_HS_CLK_SRC_DPHY);
	
	dsi_clk = sn65dsi83_get_dsi_clk(ti, mode);
	
	/* Disable PLL */
	d2l_write(ti->i2c, REG_RC_PLL_EN, 0x00);   //0d
	usleep_range(1000, 1100);
	
	/* Reference clock derived from DSI link clock. */
	d2l_write(ti->i2c, REG_RC_LVDS_PLL, lvds_pll);  //0a
	d2l_write(ti->i2c, REG_DSI_CLK, dsi_clk);      //12
	d2l_write(ti->i2c, REG_RC_DSI_CLK, REG_RC_DSI_CLK_DSI_CLK_DIVIDER(sn65dsi83_get_dsi_div(ti)));   //0b
	d2l_write(ti->i2c, REG_RC_PLL_EN, 0x00);    //0d
	
	/* Set number of DSI lanes and LVDS link config. */
	d2l_write(ti->i2c, REG_DSI_LANE, 0x30);     //10
	
	/* No equalization. */
	d2l_write(ti->i2c, REG_DSI_EQ, 0x00);       //11

	/* Set up sync signal polarity. */
	val = 0x10 | (mode->flags & DRM_MODE_FLAG_NHSYNC ?
	       REG_LVDS_FMT_HS_NEG_POLARITY : 0) |
	      (mode->flags & DRM_MODE_FLAG_NVSYNC ?
	       REG_LVDS_FMT_VS_NEG_POLARITY : 0); //Default value 0x10 based on the sn65dsi83 datasheet for 0x18 register
		   
	/* Set up bits-per-pixel, 18bpp or 24bpp. */
	if (lvds_format_24bpp) {
		val |= REG_LVDS_FMT_CHA_24BPP_MODE;
	}
	
	/* Set up LVDS format, JEIDA/Format 1 or SPWG/Format 2 */
	if (lvds_format_jeida) {
		val |= REG_LVDS_FMT_CHA_24BPP_FORMAT1;
	}
	
	d2l_write(ti->i2c, REG_LVDS_FMT, val);       //18
	d2l_write(ti->i2c, REG_LVDS_VCOM, 0x00);     //19
	d2l_write(ti->i2c, REG_LVDS_LANE, 0x00);     //1a
	d2l_write(ti->i2c, REG_LVDS_CM, 0x00);       //1b
	
	
	hback_porch = mode->htotal - mode->hsync_end;
	hsync_len  = mode->hsync_end - mode->hsync_start;
	vback_porch = mode->vtotal - mode->vsync_end;
	vsync_len  = mode->vsync_end - mode->vsync_start;
	hfront_porch = mode->hsync_start - mode->hdisplay;	
	hactive = mode->hdisplay;
	vfront_porch = mode->vsync_start - mode->vdisplay;
	vactive = mode->vdisplay;

	
	d2l_write(ti->i2c, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, (u8)(hactive&0xff));            //20
	d2l_write(ti->i2c, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, (u8)((hactive>>8)&0xff));      //21
	d2l_write(ti->i2c, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, (u8)(vactive&0xff));         //24
	d2l_write(ti->i2c, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, (u8)((vactive>>8)&0xff));   //25
	
	
	/* 32 + 1 pixel clock to ensure proper operation */
	d2l_write(ti->i2c, REG_VID_CHA_SYNC_DELAY_LOW, 0xff);               //28
	d2l_write(ti->i2c, REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);              //29
	d2l_write(ti->i2c, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, (u8)(hsync_len&0xff));             //2c
	d2l_write(ti->i2c, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, (u8)((hsync_len>>8)&0xff));       //2d
	d2l_write(ti->i2c, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, (u8)(vsync_len&0xff));             //30
	d2l_write(ti->i2c, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, (u8)((vsync_len>>8)&0xff));       //31
	d2l_write(ti->i2c, REG_VID_CHA_HORIZONTAL_BACK_PORCH, (u8)(hback_porch&0xff));           //34
	d2l_write(ti->i2c, REG_VID_CHA_VERTICAL_BACK_PORCH, (u8)(vback_porch&0xff));             //36
	d2l_write(ti->i2c, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, (u8)(hfront_porch&0xff));         //38
	d2l_write(ti->i2c, REG_VID_CHA_VERTICAL_FRONT_PORCH, (u8)(vfront_porch&0xff));           //3a
	d2l_write(ti->i2c, REG_VID_CHA_TEST_PATTERN, 0x00);                 //3c
	
	/* Enable PLL */
	d2l_write(ti->i2c, REG_RC_PLL_EN, REG_RC_PLL_EN_PLL_EN);    
	
	for(i=0; i<10; i++)
	{
		usleep_range(500, 1000);
		val=0;
		d2l_read(ti->i2c, REG_RC_LVDS_PLL, &val);
		if(val & 0x80 == 0x80)
		{
			pll_en_flag = true;
			break;
		}
	}
	
	if (pll_en_flag==false) {
		dev_info(ti->dev, "failed to lock PLL, ret=%i\n", pll_en_flag);
		/* On failure, disable PLL again and exit. */
		d2l_write(ti->i2c, REG_RC_PLL_EN, 0x00);
		return;
	}
	/* Trigger reset after CSR register update. */
	d2l_write(ti->i2c, REG_RC_RESET, REG_RC_RESET_SOFT_RESET);
	usleep_range(10000, 11000);
	
	/* Clear all errors that got asserted during initialization. */
	val=0;
	d2l_read(ti->i2c, REG_IRQ_STAT, &val);
	d2l_write(ti->i2c, REG_IRQ_STAT, val);
	
}

static enum drm_mode_status
ti_mode_valid(struct drm_bridge *bridge,
	      const struct drm_display_info *info,
	      const struct drm_display_mode *mode)
{
	struct ti_data *ti = bridge_to_ti(bridge);

	/*
	 * Maximum pixel clock speed 135MHz for single-link
	 * 270MHz for dual-link
	 */
	if ((mode->clock > 154000 && ti->lvds_link == SINGLE_LINK) ||
	    (mode->clock > 308000 && ti->lvds_link == DUAL_LINK))
		return MODE_CLOCK_HIGH;

	switch (info->bus_formats[0]) {
	case MEDIA_BUS_FMT_RGB888_1X7X4_SPWG:
	case MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA:
		/* RGB888 */
		ti->bpc = 8;
		break;
	case MEDIA_BUS_FMT_RGB666_1X7X3_SPWG:
		/* RGB666 */
		ti->bpc = 6;
		break;
	default:
		dev_warn(ti->dev,
			 "unsupported LVDS bus format 0x%04x\n",
			 info->bus_formats[0]);
		return MODE_NOMODE;
	}

	return MODE_OK;
}

static int sn65dsi83_parse_dt(struct device_node *np, struct ti_data *ti)
{
	struct device_node *endpoint;
	struct device_node *parent;
	struct device_node *remote;
	struct property *prop;
	int len = 0;

	/*
	 * To get the data-lanes of dsi, we need to access the dsi0_out of port1
	 *  of dsi0 endpoint from bridge port0 of d2l_in
	 */
	endpoint = of_graph_get_endpoint_by_regs(ti->dev->of_node,
						 SN65DSI83_DSI_IN, -1);
	if (endpoint) {
		/* dsi0_out node */
		parent = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);
		if (parent) {
			/* dsi0 port 1 */
			endpoint = of_graph_get_endpoint_by_regs(parent, 1, -1);
			of_node_put(parent);
			if (endpoint) {
				prop = of_find_property(endpoint, "data-lanes",
							&len);
				of_node_put(endpoint);
				if (!prop) {
					dev_err(ti->dev,
						"failed to find data lane\n");
					return -EPROBE_DEFER;
				}
			}
		}
	}	
	


	ti->num_dsi_lanes = len / sizeof(u32);

	if (ti->num_dsi_lanes < 1 || ti->num_dsi_lanes > 2)
		return -EINVAL;

	ti->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!ti->host_node)
		return -ENODEV;

		(ti->host_node);

	ti->lvds_link = SINGLE_LINK;
	endpoint = of_graph_get_endpoint_by_regs(ti->dev->of_node,
						 SN65DSI83_LVDS_OUT1, -1);
	if (endpoint) {
		remote = of_graph_get_remote_port_parent(endpoint);
		of_node_put(endpoint);

		if (remote) {
			if (of_device_is_available(remote))
				ti->lvds_link = DUAL_LINK;
			of_node_put(remote);
		}
	}

	dev_dbg(ti->dev, "no.of dsi lanes: %d\n", ti->num_dsi_lanes);
	dev_dbg(ti->dev, "operating in %d-link mode\n",	ti->lvds_link);

	return 0;
}

static int ti_bridge_attach(struct drm_bridge *bridge,
			    enum drm_bridge_attach_flags flags)
{
	struct ti_data *ti = bridge_to_ti(bridge);
	struct device *dev = &ti->i2c->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret;

	const struct mipi_dsi_device_info info = { .type = "sn65dsi83",
							.channel = 0,
							.node = NULL,
						};

	host = of_find_mipi_dsi_host_by_node(ti->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	ti->dsi = dsi;
	dsi->lanes = ti->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO; 
	ret = mipi_dsi_attach(dsi);

	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	/* Attach the panel-bridge to the dsi bridge */
	
	return drm_bridge_attach(bridge->encoder, ti->panel_bridge,
				 &ti->bridge, flags);
err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}

static const struct drm_bridge_funcs ti_bridge_funcs = {
	.attach = ti_bridge_attach,
	.pre_enable = ti_bridge_pre_enable,
	.enable = ti_bridge_enable,
	.mode_valid = ti_mode_valid,
	.post_disable = ti_bridge_post_disable,
};

static int ti_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct drm_panel *panel;
	struct ti_data *ti;
	int ret;

	ti = devm_kzalloc(dev, sizeof(*ti), GFP_KERNEL);

	if (!ti)
	{
		return -ENOMEM;
	}

	ti->dev = dev;
	ti->i2c = client;

	ret = drm_of_find_panel_or_bridge(dev->of_node, SN65DSI83_LVDS_OUT0,
					   0, &panel, NULL);
	 if (ret < 0)
		 return ret;
	 if (!panel)
		return -ENODEV;

	ti->panel_bridge = devm_drm_panel_bridge_add(dev, panel);
	if (IS_ERR(ti->panel_bridge))
		return PTR_ERR(ti->panel_bridge);

	ret = sn65dsi83_parse_dt(dev->of_node, ti);
	if (ret)
		return ret;

	ti->vcc = devm_regulator_get(dev, "vcc-supply");
	if (IS_ERR(ti->vcc)) {
		ret = PTR_ERR(ti->vcc);
		dev_err(dev, "vcc-supply not found\n");
		return ret;
	}

	ti->vdd = devm_regulator_get(dev, "vdd-supply");
	if (IS_ERR(ti->vdd)) {
		ret = PTR_ERR(ti->vdd);
		dev_err(dev, "vdd-supply not found\n");
		return ret;
	}

	ti->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(ti->enable_gpio)) {
		ret = PTR_ERR(ti->enable_gpio);
		dev_err(dev, "cannot get enable-gpios %d\n", ret);
		return ret;
	}

	ti->bridge.funcs = &ti_bridge_funcs;
	ti->bridge.of_node = dev->of_node;
	drm_bridge_add(&ti->bridge);
	i2c_set_clientdata(client, ti);

	return 0;
}

static int ti_remove(struct i2c_client *client)
{
	struct ti_data *ti = i2c_get_clientdata(client);

	drm_bridge_remove(&ti->bridge);

	return 0;
}

static const struct i2c_device_id sn65dsi83_i2c_ids[] = {
	{ "sn65dsi83", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi83_i2c_ids);

static const struct of_device_id sn65dsi83_of_ids[] = {
	{ .compatible = "ti,sn65dsi83", },
	{ }
};
MODULE_DEVICE_TABLE(of, sn65dsi83_of_ids);

static struct i2c_driver sn65dsi83_driver = {
	.driver = {
		.name = "sn65dsi83",
		.of_match_table = sn65dsi83_of_ids,
	},
	.id_table = sn65dsi83_i2c_ids,
	.probe = ti_probe,
	.remove	= ti_remove,
};
module_i2c_driver(sn65dsi83_driver);

MODULE_AUTHOR("Deepak Kapure");
MODULE_DESCRIPTION("SN65DSI83 DSI/LVDS bridge driver");
MODULE_LICENSE("GPL v2");
