#ifndef _TI81XX_HDMI_H_
#define _TI81XX_HDMI_H_


#define HDMI_PER_CNT			(1u)
#define HDMI_CORE_0_REGS		(0x46c00400u)
#define TI814x_HDMI_PHY_0_REGS 	(0x46c00300u)
#define TI816x_HDMI_PHY_0_REGS 	(0x48122000u)
#define PRCM_0_REGS 			(0x48180000u)

#define TI816x_CM_HDMI_CLKSTCTRL_OFF			(0x0408)
#define TI816x_CM_ACTIVE_HDMI_CLKCTRL_OFF		(0x0428)

/* HDMI EDID Length */
//#define HDMI_EDID_MAX_LENGTH			256

/* DM814x HDMI PLL related */
#define TI814x_CM_HDMI_CLKCTRL_OFF             (0x0824)
#define TI814x_CM_ALWON_SDIO_CLKCTRL           (0x15B0)
#define TI814x_HDMI_PLL_BASE_ADDR              (0x481C5200)
#define TI814x_HDMI_PLL_CLKCTRL_OFF			0x4
#define TI814x_HDMI_PLL_TENABLE_OFF			0x8
#define TI814x_HDMI_PLL_TENABLEDIV_OFF			0xC
#define TI814x_HDMI_PLL_M2NDIV_OFF		    	0x10
#define TI814x_HDMI_PLL_MN2DIV_OFF		        0x14
#define TI814x_HDMI_PLL_STATUS_OFF		        0x24

#define VPS_PRCM_MAX_REP_CNT                    (10u)

/* PHY registers for TI816x  */
#define TI816x_PHY_TMDS_CNTL1_OFFSET			(0x00000000)
#define TI816x_PHY_TMDS_CNTL2_OFFSET			(0x00000004)
#define TI816x_PHY_TMDS_CNTL3_OFFSET			(0x00000008)
#define TI816x_PHY_BIST_CNTL_OFFSET			(0x0000000C)
#define TI816x_PHY_BIST_PATTERN_OFFSET			(0x00000010)
#define TI816x_PHY_BIST_INST0_OFFSET			(0x00000014)
#define TI816x_PHY_BIST_INST1_OFFSET			(0x00000018)
#define TI816x_PHY_BIST_CONF0_OFFSET			(0x0000001C)
#define TI816x_PHY_TMDS_CNTL9_OFFSET			(0x00000020)
#define TI816x_PHY_TMDS_CNTL10_OFFSET			(0x00000024)
#define TI816x_PHY_TEST_MUX_CTRL_OFFSET		(0x00000028)

#define TI816x_HDMI_PHY_TMDS_CNTL3_PDB_MASK			(0x00000001u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_DPCOLOR_CTL_MASK		(0x00000006u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_MASK		(0x00000018u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_BIST_SEL_MASK		(0x00000040u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_DPCOLOR_CTL_SHIFT		(0x00000001u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_DPCOLOR_CTL_NO		(0x00000000u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_DPCOLOR_CTL_10BITCHANNEL	(0x00000001u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_DPCOLOR_CTL_12BITCHANNEL	(0x00000002u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_0_5X		(0x00000000u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_1_0X		(0x00000001u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_2_0X		(0x00000002u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_4_0X		(0x00000003u)
#define TI816x_HDMI_PHY_TMDS_CNTL3_CLKMULT_CTL_SHIFT		(0x00000003u)
#define TI816x_HDMI_PHY_BIST_CNTL_BIST_EN_MASK			(0x00000001u)
#define TI816x_HDMI_PHY_BIST_CNTL_BIST_ON_MASK			(0x00000002u)
#define TI816x_HDMI_PHY_BIST_CNTL_ENC_BYP_MASK			(0x00000004u)
#define TI816x_HDMI_PHY_TMDS_CNTL2_TERM_EN_MASK		(0x00000010u)
#define TI816x_HDMI_PHY_TMDS_CNTL2_OE_MASK			(0x00000020u)

/* DM814x HDMI PHY */
#define TI814x_HDMI_TXPHY_TX_CTRL			0x0ul
#define TI814x_HDMI_TXPHY_DIGITAL_CTRL		0x4ul
#define TI814x_HDMI_TXPHY_POWER_CTRL		0x8ul
#define TI814x_HDMI_TXPHY_PAD_CFG_CTRL		0xCul
#define TI814x_HDMI_PHY_TRIM_TEST_CTRL		0x10ul
#define TI814x_HDMI_PHY_ANG_INT_CTRL		0x14ul
#define TI814x_HDMI_PHY_DATA_INT_CTRL		0x18ul
#define TI814x_HDMI_PHY_BIST				0x1Cul

/* DM814x PRCM related */
#define TI814x_CM_HDMI_CLKCTRL_OFF			(0x0824)
#define TI814x_CM_ALWON_SDIO_CLKCTRL			(0x15B0)


/* \brief structure to keep track of pll configurations for a video mode */
struct hdmi_pll_ctrl
{
	u32                  __n;
	/**< Divider N for the PLL.*/
	u32                  __m;
	/**< Multiplier M for the PLL.*/
	u32                  __m2;
	/**< Divider M2 for the PLL.*/
	u32                  clk_ctrl_value;
	/**< For comparison based on the clkOut used */
};

// This data is specific to OMAP /DM814x Library, for HDMI PLL
struct hdmi_pll_info {
	u16 regn;
	u16 regm;
	u32 regmf;
	u16 regm2;
	u16 regsd;
	u16 dcofreq;
};


#endif