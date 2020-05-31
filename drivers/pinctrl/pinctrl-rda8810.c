#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "pinctrl-utils.h"

#define DRIVER_NAME "pinctrl-rda8810"

/**
 * struct rda8810_pin_conf - information about configuring a pin
 * @pin: the pin number
 * @reg: config register
 * @mask: the bits affecting the configuration of the pin
 */
struct rda8810_pin_conf {
	unsigned int pin;
	u32 reg;
	u32 mask;
};

/**
 * struct rda8810_pmx - state holder for the gemini pin controller
 * @dev: a pointer back to containing device
 * @pctl: the offset to the controller in virtual memory
 * @map: regmap to access registers
 * @confs: pin config information
 * @nconfs: number of pin config information items
 */
struct rda8810_pmx {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct regmap *map;
	const struct gemini_pin_conf *confs;
	unsigned int nconfs;
};

/**
 * struct rda8810_pin_group - describes a RDA8810 pin group
 * @name: the name of this specific pin group
 * @pins: an array of discrete physical pins used in this group, taken
 *	from the driver-local pin enumeration space
 * @num_pins: the number of pins in this group array, i.e. the number of
 *	elements in .pins so we can iterate over that array
 * @mask: bits to clear to enable this when doing pin muxing
 * @value: bits to set to enable this when doing pin muxing
 * @driving_mask: bitmask for the IO Pad driving register for this
 *	group, if it supports altering the driving strength of
 *	its lines.
 */
struct rda8810_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned int num_pins;
	u32 mask;
	u32 value;
	u32 driving_mask;
};

//#define RDA_PIN(PORT, num) \
//			#define PORT##num  (PORT##_base + num)

#define RDA_PINCTRL_PIN(PORT, num) \
			PINCTRL_PIN(PORT ## _base + num, #PORT #num),

#define RDA_M8( NAME , PORT ) \
	RDA_##NAME(PORT,0) RDA_##NAME(PORT,1) RDA_##NAME(PORT,2) RDA_##NAME(PORT,3) \
	RDA_##NAME(PORT,4) RDA_##NAME(PORT,5) RDA_##NAME(PORT,6) RDA_##NAME(PORT,7)  
	
#define RDA_M32( NAME , PORT ) \
	RDA_M8( NAME , PORT )														  \
	RDA_##NAME(PORT,8) RDA_##NAME(PORT,9) RDA_##NAME(PORT,10) RDA_##NAME(PORT,11) \
	RDA_##NAME(PORT,12) RDA_##NAME(PORT,13) RDA_##NAME(PORT,14) RDA_##NAME(PORT,15) \
	RDA_##NAME(PORT,16) RDA_##NAME(PORT,17) RDA_##NAME(PORT,18) RDA_##NAME(PORT,19) \
	RDA_##NAME(PORT,20) RDA_##NAME(PORT,21) RDA_##NAME(PORT,22) RDA_##NAME(PORT,23) \
	RDA_##NAME(PORT,24) RDA_##NAME(PORT,25) RDA_##NAME(PORT,26) RDA_##NAME(PORT,27) \
	RDA_##NAME(PORT,28) RDA_##NAME(PORT,29) RDA_##NAME(PORT,30) RDA_##NAME(PORT,31) 


static const struct pinctrl_pin_desc rda8810_pins[] = {
	RDA_M32(PINCTRL_PIN,GPIO_A)
	RDA_M32(PINCTRL_PIN,GPIO_B)
	RDA_M32(PINCTRL_PIN,GPIO_C)
	RDA_M8(PINCTRL_PIN,GPIO_D)
	RDA_M8(PINCTRL_PIN,GPO_)
}


static int rda8810_pmx_probe(struct platform_device *pdev)
{
	struct rda8810_pmx *pmx;
	struct regmap *map;
	struct device *dev = &pdev->dev;
	struct device *parent;
	unsigned long tmp;
	u32 val;
	int ret;
	int i;

	/* Create state holders etc for this driver */
	pmx = devm_kzalloc(&pdev->dev, sizeof(*pmx), GFP_KERNEL);
	if (!pmx)
		return -ENOMEM;

	pmx->dev = &pdev->dev;
	parent = dev->parent;
	if (!parent) {
		dev_err(dev, "No Syscon parent\n");
		return -ENODEV;
	}

	map = syscon_node_to_regmap(parent->of_node);
	if (IS_ERR(map)) {
		dev_err(dev, "no syscon regmap\n");
		return PTR_ERR(map);
	}
	pmx->map = map;

	/* Check that regmap works at first call, then no more */
	ret = regmap_read(map, CHIP_ID, &val);
	if (ret) {
		dev_err(dev, "cannot access regmap\n");
		return ret;
	}
	val >>= 16;

	if (val == 0x8810) {
		pmx->confs = gemini_confs_3512;
		pmx->nconfs = ARRAY_SIZE(gemini_confs_3512);
		rda8810_pmx_desc.pins = rda8810_pins;
		rda8810_pmx_desc.npins = ARRAY_SIZE(rda8810_pins);
		dev_info(dev, "detected rda8810 chip variant\n");
	} else {
		dev_err(dev, "unknown chip ID: %04x\n", val);
		return -ENODEV;
	}

	pmx->pctl = devm_pinctrl_register(dev, &rda8810_pmx_desc, pmx);
	if (IS_ERR(pmx->pctl)) {
		dev_err(dev, "could not register pinmux driver\n");
		return PTR_ERR(pmx->pctl);
	}

	dev_info(dev, "Initialized RDA Micro RDA881- pin control driver\n");

	return 0;
}

static const struct of_device_id rda8810_pinctrl_match[] = {
	{ .compatible = "rdamicro,rda8810-pinctrl" },
	{},
};

static struct platform_driver rda8810_pmx_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = rda8810_pinctrl_match,
	},
	.probe = rda8810_pmx_probe,
};

static int __init rda8810_pmx_init(void)
{
	return platform_driver_register(&rda8810_pmx_driver);
}

arch_initcall(rda8810_pmx_init);

