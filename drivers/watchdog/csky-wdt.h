#ifndef __CSKY_WDT_H__
#define __CSKY_WDT_H__

#include <linux/watchdog.h>

#define DRIVER_NAME	"csky-wdt"

/* WDT control registers */
#define WDT_CR		0x00	/*Control Register*/
#define WDT_TORR	0x04	/*Timeout Range Register*/
#define WDT_CCVR	0x08	/*Current Counter Value Register*/
#define WDT_CRR		0x0c	/*Current Counter Value Register*/
#define WDT_STAT	0x10	/*Interrupt Status Register*/
#define WDT_EOI		0x14	/*Interrupt Clear Register*/

/* Bitfields in WDT */
#define WDTCNF_TORR_DEFAULT	0x0f
#define WDTCNF_CCR_EN		0x76
#define WDTCNF_CR_EN		BIT(0)
#define WDTCNF_CR_RMOD_INT	BIT(1)
#define WDTCNF_CR_DIS		(~ WDTCNF_CR_EN)
#define WDTCNF_CR_RMOD_RST	(~ WDTCNF_CR_RMOD_INT)


struct csky_wdt_priv {
	struct device		*dev;
	struct watchdog_device	wdd;
	void __iomem		*iobase;
	struct clk		*clk_apb;
	int			irq;
	u32			wdt_period;
	unsigned long		wdt_cnts;
	unsigned long		wdt_freq;
};

#endif /* __CSKY_WDT_H__ */

