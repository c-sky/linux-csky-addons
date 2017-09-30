/*
 * HDMI driver for C-SKY's SoCs.
 *
 * Copyright (C) 2017 C-SKY MicroSystems Co.,Ltd.
 * Author: Huoqing Cai <huoqing_cai@c-sky.com>
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

#ifndef __CSKY_HDMI_H__
#define __CSKY_HDMI_H__

/* HDMI control registers */
#define X00_SYSTEM_CONTROL	0x00	/*Power save and interrupt control */
#define X01_N19_16		0x01	/* 20-bit N used for cycle stamp */
#define X02_N15_8		0x02
#define X03_N7_03		0x03
#define X04_SPDIF_FS4		0x04	/* SPDIF sampling frequency/CTS */
#define X05_CTS_INT		0x05	/* CTS[15:8] internal */
#define X06_CTS_INT		0x06	/* CTS[7:0] internal */
#define X07_CTS_EXT		0x07	/* CTS[19:16] external */
#define X08_CTS_EXT		0x08	/* CTS[15:8] external */
#define X09_CTS_EXT		0x09	/* CTS[7:0] external */
#define X0A_AUDIO_SOURCE	0x0a	/* Audio setting.1 */
#define X0B_AUDIO_SET2		0x0b	/* Audio setting.2 */
#define X0C_I2S_MODE		0x0c	/* I2S audio setting */
#define X0D_DSD_MODE		0x0d	/* DSD audio setting */
#define X0E_DEBUG_MONITOR1	0x0e	/* Reserved */
#define X0F_DEBUG_MONITOR2	0x0f	/* Reserved */
#define X10_I2S_PINMODE		0x10	/* I2S input pin swap */
#define X11_ASTATUS1		0x11	/* Audio status bits setting1 */
#define X12_ASTATUS2		0x12	/* Audio status bits setting2 */
#define X13_CAT_CODE		0x13	/* Category code */
#define X14_A_SOURCE		0x14	/* Source number/ Audio word length */
#define X15_AVSET1		0x15	/* Audio/Video setting.1 */
#define X16_VIDEO1		0x16
#define X17_DC_REG		0x17	/* init 20h Deep color setting */
#define X18_CSC_C0_HI		0x18	/* Color Space Conversion Parameters */
#define X19_CSC_C0_LO		0x19
#define X1A_CSC_C1_HI		0x1a	/* Color Space Conversion Parameters */
#define X1B_CSC_C1_LO		0x1b
#define X1C_CSC_C2_HI		0x1c
#define X1D_CSC_C2_LO		0x1d
#define X1E_CSC_C3_HI		0x1e
#define X1F_CSC_C3_LO		0x1f
#define X20_CSC_C4_HI		0x20
#define X21_CSC_C4_LO		0x21
#define X22_CSC_C5_HI		0x22
#define X23_CSC_C5_LO		0x23
#define X24_CSC_C6_HI		0x24
#define X25_CSC_C6_LO		0x25
#define X26_CSC_C7_HI		0x26
#define X27_CSC_C7_LO		0x27
#define X28_CSC_C8_HI		0x28
#define X29_CSC_C8_LO		0x29
#define X2A_CSC_C9_HI		0x2a
#define X2B_CSC_C9_LO		0x2b
#define X2C_CSC_C10_HI		0x2c
#define X2D_CSC_C10_LO		0x2d
#define X2E_CSC_C11_HI		0x2e
#define X2F_CSC_C11_LO		0x2f
#define X30_EXT_VPARAMS		0x30	/* External video parameter settings */
#define X31_EXT_HTOTAL		0x31	/* External horizontal total */
#define X32_EXT_HTOTAL		0x32
#define X33_EXT_HBLANK		0x33	/* External horizontal blank */
#define X34_EXT_HBLANK		0x34
#define X35_EXT_HDLY		0x35	/* External horizontal delay */
#define X36_EXT_HDLY		0x36
#define X37_EXT_HS_DUR		0x37	/* External horizontal duration */
#define X38_EXT_HS_DUR		0x38
#define X39_EXT_VTOTAL		0x39	/* External vertical total */
#define X3A_EXT_VTOTAL		0x3a
#define X3B_AVSET2		0x3b	/* Audio/Video setting.2 */
#define X3C_EX_VID		0x3c	/* External input Video ID(VID) */
#define X3D_EXT_VBLANK		0x3d	/* External virtical blank */
#define X3E_EXT_VDLY		0x3e	/* External virtical delay */
#define X3F_EXT_VS_DUR		0x3f	/* External virtical durationv */
#define X40_CTRL_PKT_EN		0x40	/* Control packet enable */
#define X41_SEND_CPKT_AUTO	0x41	/* HB0 for generic control packet */
#define X42_AUTO_CHECKSUM	0x42	/* Auto checksum option */
#define X45_VIDEO2		0x45	/* Video setting.2 */
#define X46_OUT_OPTION		0x46	/* Ouput Option*/
#define X51_PHY_CTRL		0x51	/* Revervd[7:4],PHY_OPTION[3:0] */
#define X52_HSYNC_PLACE_656	0x52	/* HSYNC placement at ITU656 */
#define X53_HSYNC_PLACE_656	0x53
#define X54_VSYNC_PLACE_656	0x54	/* VSYNC placement at ITU656 */
#define X55_VSYNC_PLACE_656	0x55
#define X56_PHY_CTRL		0x56	/* SLIPHDMIT parameter settings */
#define X57_PHY_CTRL		0x57
#define X58_PHY_CTRL		0x58
#define X59_PHY_CTRL		0x59
#define X5A_PHY_CTRL		0x5a
#define X5B_PHY_CTRL		0x5b
#define X5C_PHY_CTRL		0x5c
#define X5D_PHY_CTRL		0x5d
#define X5E_PHY_CTRL		0x5e
#define X5F_PACKET_INDEX	0x5f
#define X60_PKT_HB0		0x60
#define X61_PKT_HB1		0x61
#define X62_PKT_HB2		0x62
#define X63_PKT_PB0		0x63
#define X64_PKT_PB1		0x64
#define X65_PKT_PB2		0x65
#define X66_PKT_PB3		0x66
#define X67_PKT_PB4		0x67
#define X68_PKT_PB5		0x68
#define X69_PKT_PB6		0x69
#define X6A_PKT_PB7		0x6a
#define X6B_PKT_PB8		0x6b
#define X6C_PKT_PB9		0x6c
#define X6D_PKT_PB10		0x6d
#define X6E_PKT_PB11		0x6e
#define X6F_PKT_PB12		0x6f
#define X70_PKT_PB13		0x70
#define X71_PKT_PB14		0x71
#define X72_PKT_PB15		0x72
#define X73_PKT_PB16		0x73
#define X74_PKT_PB17		0x74
#define X75_PKT_PB18		0x75
#define X76_PKT_PB19		0x76
#define X77_PKT_PB20		0x77
#define X78_PKT_PB21		0x78
#define X79_PKT_PB22		0x79
#define X7A_PKT_PB23		0x7a
#define X7B_PKT_PB24		0x7b
#define X7C_PKT_PB25		0x7c
#define X7D_PKT_PB26		0x7d
#define X7E_PKT_PB27		0x7e
#define X80_EDID_FIFO		0x80	/* Access window for EDID buffer */
#define X81_DDC_LSB		0x81	/* DDC frequency control LSB */
#define X82_DDC_MSB		0x82	/* DDC frequency control MSB */
#define X92_INT_MASK1		0x92	/* Mask for Interrupt Group1 */
#define X93_INT_MASK2		0x93	/* Mask for Interrupt Group2 */
#define X94_INT_STATUS1		0x94	/* Status for Interrupt Group1 */
#define X95_INT_STATUS2		0x95	/* Status for Interrupt Group2 */
#define X96_INT_MASK3		0x96	/* Mask for Interrupt Group3 */
#define X97_INT_MASK4		0x97	/* Mask for Interrupt Group4 */
#define X98_INT_STATUS3		0x98	/* Status for Interrupt Group3 */
#define X99_INT_STATUS4		0x99	/* Status for Interrupt Group4 */
#define XAF_HDCP_CTRL		0xaf	/* R/W 12h HDCP Control Register */
#define XB8_HDCP_STATUS		0xb8	/* HDCP Status Register */
#define XBE_BCAPS		0xbe	/* BCAPS value */
#define XBF_KSV7_0		0xbf	/* KSV[7:0] - AKSV/BKSV monitor */
#define XC0_KSV15_8		0xc0	/* KSV[15:8] - AKSV/BKSV monitor */
#define XC1_KSV23_16		0xc1	/* KSV[23:16] - AKSV/BKSV monitor */
#define XC2_KSV31_24		0xc2	/* KSV[31:24] - AKSV/BKSV monitor */
#define XC3_KSV39_32		0xc3	/* KSV[39:32] - AKSV/BKSV monitor */
#define XC4_SEG_PTR		0xc4	/* EDID segment pointer */
#define XC5_EDID_WD_ADDR	0xc5	/* EDID word address */
#define XC6_GEN_PB26		0xc6	/* Generic control packet (PB26) */
#define XC8_HDCP_ERR		0xc8	/* HDCP error */
#define XC9_TIMER		0xc0	/*  Timer value for 100ms */
#define XCA_TIMER		0xca	/* init 2ch Timer value for 5sec */
#define XCB_READ_RI_CNT		0xcb	/* Ri read count */
#define XCC_AN_SEED		0xcc	/* An Seed */
#define XCD_MAX_REC_NUM		0xcd	/* maxi number of receivers allowed */
#define XCF_HDCP_MEM_CTRL2	0xcf	/* [1:0] ID_FAX_KEY, ID_HDCP_KEY */
#define XD0_HDCP_CTRL2		0xd0	/* R/W 08h HDCP Control 2 */
#define XD2_HDCP_KEY_CTRL	0xd2	/* HDCP KEY memory control */
#define XD3_CSC_CONFIG1		0xd3	/* CSC/Video Config.1 */
#define XD4_VIDEO3		0xd4	/* Video setting 3 */
#define XD9_GEN_PB27		0xd9	/* Generic InfoFrame PB27 */
#define XDF_HPG_STATUS		0xdf	/* Hot plug/MSENS status */
#define XE0_GAMUT_HB1		0xe0	/* gamut metadata HB1 */
#define XE1_GAMUT_HB2		0xe1	/* gamut metadata HB2 */
#define XE8_AN0			0xe8	/* An[7:0] */

/* Bitfields in HDMI */
#define PLLA_RST_X00		BIT(2)	/* reset PLL_A */
#define PLLB_RST_X00		BIT(3)	/* reset PLL_B */
#define PWR_MOD_A_X00		BIT(4)	/* power mode A */
#define PWR_MOD_B_X00		BIT(5)	/* power mode B */
#define PWR_MOD_D_X00		BIT(6)	/* power mode D */
#define PWR_MOD_E_X00		BIT(7)	/* power mode E */
#define PLL_MASK_X00		GENMASK(7, 4)
#define PLL_RST_X00		(PLLA_RST_X00 | PLLB_RST_X00)
#define PWR_MODB_RST_X00	(PWR_MOD_B_X00 | PLL_RST_X00)
#define PWR_MODB_RSTB_X00	(PWR_MOD_B_X00 | PLLB_RST_X00)

#define FMT_YCC422_X15		BIT(1)
#define FMT_MASK_X15		GENMASK(3, 0)

#define SPACE_YCC_X16		BIT(0)
#define SAV_CHANNEL1_x16	BIT(2)	/* SAV EAV location channel1 */
#define SAV_CHANNEL2_x16	BIT(3)	/* SAV EAV location channel2 */
#define WIDTH_10BITS_X16	BIT(4)
#define VID_YCC422_X16		BIT(7)
#define VID_YCC444_X16		BIT(6)
#define WIDTH_12BITS_X16	0x0
#define VID_RGB_X16		0x0
#define WIDTH_8BITS_X16		(BIT(4) | BIT(5))
#define VID_MASK_X16		GENMASK(7, 6)
#define DAT_WIDTH_MASK_X16	GENMASK(5, 4)

#define SPEED_8BIT_X17		0x0
#define SPEED_10BIT_X17		BIT(6)
#define SPEED_12BIT_X17		BIT(7)
#define SPEED_MASK_X17		GENMASK(7, 6)

#define EXTERANL_VIDEO_X30	BIT(0)
#define INETLACE_X30		BIT(1)
#define HSYNC_POLARITY_X30	BIT(2)
#define VSYNC_POLARITY_X30	BIT(3)
#define HIGH_POLARITY_X30	(HSYNC_POLARITY_X30 | VSYNC_POLARITY_X30)

#define CSC_EN_X3B		BIT(0)	/* Color Space Conversion enable */
#define SEL_FULL_RANGE_X3B	BIT(1)	/* range for Send black video mode */
#define EN_M0_LOAD_X3B		BIT(2)	/* Load M0 into Akey area */
#define EXT_DE_CNT_X3B		BIT(5)	/* External DE control */
#define CD_ZERO_X3B		BIT(6)	/* CD all zero override */
#define CTS_DEBUG_X3B		BIT(7)	/* Debug bit for CTS timing */
#define CD_MASK_X3B		GENMASK(6, 6)

#define NOVIDEO_X45		BIT(0)	/* Send black video */
#define NOAUDIO_X45		BIT(1)	/* Send no audio */
#define AUDIORST_X45		BIT(2)	/* audio capture logic reset */
#define SET_AV_MUTE_X40		BIT(6)	/* Send ?gSet AV mute*/
#define CLEAR_AV_MUTE_X40	BIT(7)	/* Clear AV mute?h */
#define ENVIDEO_X45		GENMASK(0, 0)

#define GENERIC_PKT_X5F		0x0	/* Generic packet */
#define ACP_PKT_X5F		BIT(0)	/* ACP packet */
#define ISRC1_PKT_X5F		BIT(1)	/* ISRC1 packet */
#define ISRC2_PKT_X5F		(BIT(0) | BIT(1))
#define GAMUT_PKT_X5F		BIT(2)	/* Gamut metadata packet */
#define VENDOR_INFO_PKT_X5F	(BIT(0) | BIT(2))
#define AVI_INFO_PKT_X5F	(BIT(1) | BIT(2))
#define PRODUCT_INFO_PKT_X5F	(BIT(0) | BIT(1) | BIT(2))
#define AUDIO_INFO_PKT_X5F	BIT(3)	/* Audio InfoFrame packet */
#define MPEG_SRC_INFO_PKT_X5F	(BIT(0) | BIT(3))

#define HB0_AVI_TYPE_X60	0x82
#define HB1_VERSION_X61		0x02
#define HB2_LENTH_X62		0x0d

#define PB1_YCC422_X64		BIT(5)	/* set PB1 ycc422*/
#define PB1_YCC444_X64		BIT(6)	/* set PB1 ycc444 */
#define PB1_MASK_X64		GENMASK(7, 5)

#define EDID_ERR_MSK_X92	BIT(1)	/* EDID error detect interrupt mask */
#define EDID_RDY_MSK_X92	BIT(2)	/* EDID ready interrupt mask */
#define AFIFO_FULL_MSK_X92	BIT(4)	/* Audio FIFO detect interrupt mask */
#define VSYNC_MSK_X92		BIT(5)	/* VSYNC detect interrupt mask */
#define MSENS_MSK_X92		BIT(6)	/* MSENS detect interrupt mask */
#define HPG_MSK_X92		BIT(7)	/* Hot plug detect interrupt mask */
#define EDID_MSK_X92		(EDID_ERR_MSK_X92 | EDID_RDY_MSK_X92)

#define RDY_AUTH_MSK_X93	BIT(3)	/* Authen ready interrupt mask */
#define AUTH_DONE_MSK_X93	BIT(4)	/* Authen done interrupt mask */
#define BKSV_RCRDY_MSK_X93	BIT(5)	/* BKSV from receiver interrupt mask */
#define BKSV_RPRDY_MSK_X93	BIT(6)	/* BKSV from repeater interrupt mask */
#define HDCP_ERR_MSK_X93	BIT(7)	/* HDCP error detect interrupt mask */

#define EDID_ERR_INT_X94	BIT(1)	/* EDID error detect interrupt */
#define EDID_RDY_INT_X94	BIT(2)	/* EDID ready interrupt */
#define AFIFO_FULL_INT_X94	BIT(4)	/* Audio FIFO full detect interrupt */
#define VSYNC_INT_X94		BIT(5)	/* VSYNC detect interrupt */
#define MSENS_INT_X94		BIT(6)	/* MSENS detect interrupt */
#define HPG_INT_X94		BIT(7)	/* Hot plug detect interrupt */
#define HPG_MSENS_INT_X94	(HPG_INT_X94 | MSENS_INT_X94)
#define EDID_INT_X94		(EDID_RDY_INT_X94 | EDID_ERR_INT_X94)
#define INT_CLR_X94		0xff


#define RDY_AUTH_INT_X95	BIT(3)	/* Authentication ready interrupt */
#define AUTH_DONE_INT_X95	BIT(4)	/* HDCP authen done interrupt */
#define BKSV_RCRDY_INT_X95	BIT(5)	/* BKSV ready  receiver interrupt */
#define BKSV_RPRDY_INT_X95	BIT(6)	/* BKSVs ready  repeater interrupt */
#define HDCP_ERR_INT_X95	BIT(7)	/* HDCP error detect interrupt */
#define INT_CLR_X95		0xff

#define HDCP_RESET_XAF		BIT(0)	/* Reset HDCP */
#define HDMI_MODE_CTRL_XAF	BIT(1)	/* HDMI/DVI mode */
#define ADV_CIPHER_XAF		BIT(2)	/* Advanced cipher mode */
#define STOP_AUTH_XAF		BIT(3)	/* Stop HDCP authentication */
#define FRAME_ENC_XAF		BIT(4)	/* Frame encrypt */
#define BKSV_FAIL_XAF		BIT(5)	/* BKSV check result (FAIL) */
#define BKSV_PASS_XAF		BIT(6)	/* BKSV check result (PASS) */
#define HDCP_REQ_XAF		BIT(7)	/* HDCP authentication start*/
#define HDMI_MODE_MASK_XAF	GENMASK(1, 1)

#define ADV_CIPHERI_STATUS_XB8	BIT(3)	/* Advanced cipher status */
#define HDCP_IDLE_XB8		BIT(4)	/* HDCP state machine status */
#define HDMI_STATUS_XB8		BIT(5)	/* HDMI/DVI status */
#define ENC_XB8			BIT(6)	/* Encryption status */
#define AUTH_XB8		BIT(7)	/* HDCP authentication status*/

#define BAD_BKSV_XC8		BIT(0)	/* BKSV not contain 20 0's 20 1's */

#define LD_HDCP_KEY_XCF		BIT(0)	/* Trigger for loading HDCP key */
#define LD_FAX_KEY_XCF		BIT(1)	/* Trigger for loading fax HDCP key */

#define DELAY_RI_CHK_XD0	BIT(3)
#define DIS_0LEN_HASH_XD0	BIT(4)
#define EN_PJ_CALC_XD0		BIT(5)
#define DIS_127_CHK_XD0		BIT(7)

#define KEY_READY_XD2		BIT(0)
#define KEY_VALID_XD2		BIT(1)
#define KSV_VALID_XD2		BIT(2)
#define KSV_SEL_XD2		BIT(3)
#define LOAD_AKSV_XD2		BIT(4)
#define USE_KSV2_XD2		BIT(5)
#define USE_KSV1_XD2		BIT(6)

#define AUTO_MODE_XD3		BIT(7)

#define BIST_FAIL_XDF		BIT(0)	/* Dual port RAM BIST result fail */
#define BIST_PASS_XDF		BIT(1)	/* Dual port RAM BIST result passed */
#define MSENS_PRT_XDF		BIT(6)	/* MSENS input port status */
#define HPG_PRT_XDF		BIT(7)	/* Hot plug input port status */
#define HPG_MSENS_PRT_XDF	(HPG_PRT_XDF | MSENS_PRT_XDF)

#define EDID_ADDR_80_XC5	0x80	/* word address1 is 0x80 */
#define EDID_ADDR_00_XC5	0x00	/* word address2 is 0x00 */

#define HDMI_I2C_SLAVE_ADDR	0x72
#define HDMI_EDID_RETRY_TIMES	10
#define HDMI_EDID_BLK_SIZE	128
#define HDMI_EDID_EXT_INDEX	126
#define PHY_DATA_SIZE		10
#define HB0_TO_PB27_SIZE	31
#define HDMI_INFO_FRAME_SIZE	0x11

/* Video setting constants */
#define VID_01_640X480P		1
#define VID_02_720X480P		2
#define VID_03_720X480P		3
#define VID_04_1280X720P	4
#define VID_05_1920X1080I	5
#define VID_06_720X480I		6
#define VID_07_720X480I		7
#define VID_08_720X240P		8
#define VID_09_720X240P		9
#define VID_10_2880X480I	10
#define VID_11_2880X480I	11
#define VID_12_2880X240P	12
#define VID_13_2880X240P	13
#define VID_14_1440X480P	14
#define VID_15_1440X480P	15
#define VID_16_1920X1080P	16
#define VID_17_720X576P		17
#define VID_18_720X576P		18
#define VID_19_1280X720P	19
#define VID_20_1920X1080I	20
#define VID_21_720X576I		21
#define VID_22_720X576I		22
#define VID_23_720X288p		23
#define VID_24_720X288p		24
#define VID_25_2880X576I	25
#define VID_26_2880X576I	26
#define VID_27_2880X288p	27
#define VID_28_2880X288p	28
#define VID_29_1440X576P	29
#define VID_30_1440X576P	30
#define VID_31_1920X1080P	31
#define VID_32_1920X1080P	32
#define VID_33_1920X1080P	33
#define VID_34_1920X1080P	34
#define VID_35_2880X480P	35
#define VID_36_2880X480P	36
#define VID_37_2880X576P	37
#define VID_38_2880X567p	38
#define VID_39_1920X1080I	39
#define VID_40_1920X1080I	40
#define VID_41_1280X720P	41
#define VID_42_720X576P		42
#define VID_43_720X576P		43
#define VID_44_720X576I		44
#define VID_45_720X576I		45
#define VID_46_1920X1080I	46
#define VID_47_1280X720P	47
#define VID_48_720X480P		48
#define VID_49_720X480P		49
#define VID_50_720X480I		50
#define VID_51_720X480I		51
#define VID_52_720X576P		52
#define VID_53_720X576P		53
#define VID_54_720X576I		54
#define VID_55_720X576I		55
#define VID_56_720X480P		56
#define VID_57_720X480P		57
#define VID_58_720X480I		58
#define VID_59_720X480I		59

enum DEEP_COLOR {
	DEEP_COLOR_8BIT = 4,
	DEEP_COLOR_10BIT,
	DEEP_COLOR_12BIT
};

enum PHY_TDMS_CLOCK {
	PHY_TMDS_CLOCK_27,
	PHY_TMDS_CLOCK_33,
	PHY_TMDS_CLOCK_40,
	PHY_TMDS_CLOCK_54,
	PHY_TMDS_CLOCK_98,
	PHY_TMDS_CLOCK_111,
	PHY_TMDS_CLOCK_148,
	PHY_TMDS_CLOCK_185,
	PHY_TMDS_CLOCK_222
};

#endif /* __CSKY_HDMI_H__ */
