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
	const struct rda8810_pin_conf *confs;
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


struct rda8810_pmx_func {
	const char *name;
	const char * const *groups;
	const unsigned int num_groups;
};


#define RDA_PIN(PORT, num) 	(PORT##_base + num)

#define RDA_PINCTRL_PIN(PORT, NUM) \
	PINCTRL_PIN(PORT ## _base + NUM, #PORT #num),

#define RDA_GP_NEW_GROUP(PORT, NUM) \
	static const unsigned PORT ## NUM ## _grp_pins[] = { PORT ## _base + NUM };\
	static const char * const PORT ## NUM ## _groups[] = { #PORT #NUM  "_grp" };


#define RDA_M8( MACRO , PORT ) \
	RDA_##MACRO(PORT,0) RDA_##MACRO(PORT,1) RDA_##MACRO(PORT,2) RDA_##MACRO(PORT,3) \
	RDA_##MACRO(PORT,4) RDA_##MACRO(PORT,5) RDA_##MACRO(PORT,6) RDA_##MACRO(PORT,7)  

#define RDA_M32( MACRO , PORT ) \
	RDA_M8( MACRO , PORT )  \
	RDA_##MACRO(PORT,8) RDA_##MACRO(PORT,9) RDA_##MACRO(PORT,10) RDA_##MACRO(PORT,11) \
	RDA_##MACRO(PORT,12) RDA_##MACRO(PORT,13) RDA_##MACRO(PORT,14) RDA_##MACRO(PORT,15) \
	RDA_##MACRO(PORT,16) RDA_##MACRO(PORT,17) RDA_##MACRO(PORT,18) RDA_##MACRO(PORT,19) \
	RDA_##MACRO(PORT,20) RDA_##MACRO(PORT,21) RDA_##MACRO(PORT,22) RDA_##MACRO(PORT,23) \
	RDA_##MACRO(PORT,24) RDA_##MACRO(PORT,25) RDA_##MACRO(PORT,26) RDA_##MACRO(PORT,27) \
	RDA_##MACRO(PORT,28) RDA_##MACRO(PORT,29) RDA_##MACRO(PORT,30) RDA_##MACRO(PORT,31) 

#define RDA_NEW_GROUP(GRPF,...)	\
	static const unsigned GRPF ## _grp_pins[] = { __VA_ARGS__ };
	static const char * const  GRPF ## _grp[] = { #GRPF "_grp" };

#define RDA_NEW_GROUP_M(GRPF,GRPNUM,...)	\
	static const unsigned GRPF ## _grp ## GRPNUM ## _pins[] = { __VA_ARGS__ };
	static const char * const  GRPF ## _grp ## GRPNUM ## [] = { #GRPF "_grp" #GRPNUM};

static const struct pinctrl_pin_desc rda8810_pins[] = {
	RDA_M32(PINCTRL_PIN,GPIO_A)
	RDA_M32(PINCTRL_PIN,GPIO_B)
	RDA_M32(PINCTRL_PIN,GPIO_C)
	RDA_M8(PINCTRL_PIN,GPIO_D)
	RDA_M8(PINCTRL_PIN,GPO_)
};

RDA_M32(GP_NEW_GROUP, GPIO_A)
RDA_M32(GP_NEW_GROUP, GPIO_B)
RDA_M32(GP_NEW_GROUP, GPIO_C)
RDA_M8(GP_NEW_GROUP, GPIO_D)

RDA_NEW_GROUP(GPO_0, RDA_PIN(GPO_,0) )
RDA_NEW_GROUP(GPO_1, RDA_PIN(GPO_,1) )
RDA_NEW_GROUP(GPO_2, RDA_PIN(GPO_,2) )
RDA_NEW_GROUP(GPO_3, RDA_PIN(GPO_,3) )
RDA_NEW_GROUP(GPO_4, RDA_PIN(GPO_,4) )
RDA_NEW_GROUP(SPI1,\
					RDA_PIN(GPIO_C,21),
					RDA_PIN(GPIO_C,22),
					RDA_PIN(GPIO_C,23),
					RDA_PIN(GPIO_C,24)
			 )
RDA_NEW_GROUP(SPI1_CS_1, RDA_PIN(GPIO_A,17) )
RDA_NEW_GROUP(SPI1_CS_2, RDA_PIN(GPIO_B,1) )
RDA_NEW_GROUP(SPI2,\
					RDA_PIN(GPIO_A,2),
					RDA_PIN(GPIO_A,3),
					RDA_PIN(GPIO_A,4),
					RDA_PIN(GPIO_A,5)
			 )
RDA_NEW_GROUP(SPI2_CS_1,RDA_PIN(GPIO_A,6) )
RDA_NEW_GROUP(BB_SPI1,\
					RDA_PIN(GPIO_C,21),
					RDA_PIN(GPIO_C,22),
					RDA_PIN(GPIO_C,23),
					RDA_PIN(GPIO_C,24)
			 )
RDA_NEW_GROUP(I2C1,\
					RDA_PIN(GPIO_B,30),
					RDA_PIN(GPIO_B,31)
			 )
RDA_NEW_GROUP_M(I2C2, 0,\
					RDA_PIN(GPIO_A,0),
					RDA_PIN(GPIO_A,1)
			 )
// Cam / I2C - bit 3 : 0 / 1
RDA_NEW_GROUP_M(I2C2, 1,\
					RDA_PIN(GPIO_B,10),
					RDA_PIN(GPIO_B,11)
				)
// I2C3 / KeyOut 34 bit 17 :  0 / 1
RDA_NEW_GROUP(I2C3,\
					RDA_PIN(GPIO_B,6),
					RDA_PIN(GPIO_B,7)
			 )
RDA_NEW_GROUP(UART1_RXD_TXD,\
					RDA_PIN(GPIO_A,14),
					RDA_PIN(GPIO_C,6)
			 )
RDA_NEW_GROUP(UART1_RTS_CTS,\
					RDA_PIN(GPIO_A,15),
					RDA_PIN(GPIO_A,16)
			 )
RDA_NEW_GROUP(UART1_DTR_DCD_DSR_RI, \
					RDA_PIN(GPIO_C,7),
					RDA_PIN(GPIO_C,8),
					RDA_PIN(GPIO_B,8),
					RDA_PIN(GPIO_B,9)
			 )
RDA_NEW_GROUP(UART2_RXD_TXD,\
					RDA_PIN(GPIO_C,7),
					RDA_PIN(GPIO_C,8)
			 )
RDA_NEW_GROUP(UART2_RTS_CTS,\
					RDA_PIN(GPIO_B,8),
					RDA_PIN(GPIO_B,9)
			 )
RDA_NEW_GROUP(UART3_RXD_TXD,\
					RDA_PIN(GPIO_D,0),
					RDA_PIN(GPIO_D,1)
			 )
RDA_NEW_GROUP(UART3_RTS_CTS,\
					RDA_PIN(GPIO_D,3),
					RDA_PIN(GPIO_D,2)
			 )
RDA_NEW_GROUP(BB_UART_RXD_TXD,\
					RDA_PIN(GPIO_C,7),
					RDA_PIN(GPIO_C,8)
			 )
RDA_NEW_GROUP(KEYIN_0,\
					RDA_PIN(GPIO_B,0)
			 )
RDA_NEW_GROUP(KEYIN_1,\
					RDA_PIN(GPIO_B,1)
			 )
RDA_NEW_GROUP(KEYIN_2,\
					RDA_PIN(GPIO_B,2)
			 )
RDA_NEW_GROUP(KEYIN_3,\
					RDA_PIN(GPIO_A,6)
			 )
RDA_NEW_GROUP(KEYIN_4,\
					RDA_PIN(GPIO_A,7)
			 )
RDA_NEW_GROUP(KEYIN_5,\
					RDA_PIN(GPO_,0)
			 )
RDA_NEW_GROUP(KEYOUT_0,\
					RDA_PIN(GPIO_B,3)
			 )
RDA_NEW_GROUP(KEYOUT_1,\
					RDA_PIN(GPIO_B,4)
			 )
RDA_NEW_GROUP(KEYOUT_2,\
					RDA_PIN(GPIO_B,5)
			 )
RDA_NEW_GROUP(KEYOUT_34,\
					RDA_PIN(GPIO_B,6),
					RDA_PIN(GPIO_B,7)
			 )
RDA_NEW_GROUP(KEYOUT_5,\
					RDA_PIN(GPO_,1)
			 )
RDA_NEW_GROUP(KEYINOUT_6,\
					RDA_PIN(GPIO_B,8),
					RDA_PIN(GPIO_B,9)
			 )
RDA_NEW_GROUP(KEYINOUT_7,\
					RDA_PIN(GPIO_A,15),
					RDA_PIN(GPIO_A,16)
			 )
RDA_NEW_GROUP(TCO_0,\
					RDA_PIN(GPIO_B,3)
			 )
RDA_NEW_GROUP(TCO_1,\
					RDA_PIN(GPIO_B,4)
			 )
RDA_NEW_GROUP(TCO_2,\
					RDA_PIN(GPIO_B,5)
			 )
RDA_NEW_GROUP(I2S_DI_2,\
					RDA_PIN(GPIO_B,2)
			 )
RDA_NEW_GROUP(I2S,\
					RDA_PIN(GPIO_A,9),
					RDA_PIN(GPIO_A,10),
					RDA_PIN(GPIO_A,11),
					RDA_PIN(GPIO_A,12),
					RDA_PIN(GPIO_A,13)
			 )
RDA_NEW_GROUP(DAI, 0, \
				RDA_PIN(GPIO_A,10),
				RDA_PIN(GPIO_A,11),
				RDA_PIN(GPIO_A,12),
				RDA_PIN(GPIO_A,13)
			 )
RDA_NEW_GROUP(DAI_SIMPLE, 0, \
				RDA_PIN(GPIO_A,10),
				RDA_PIN(GPIO_A,11),
				RDA_PIN(GPIO_A,12),
				RDA_PIN(GPIO_A,13)
			 )

// --------------------------------------------------------------------
#define RDA_ADD_GROUP(GRPF)	\
	{ \
		.name = #GRPF "_grp", \
		.pins = GRPF ## _grp0_pins,\
		.npins = ARRAY_SIZE(GRPF ## _grp0_pins), \
	}

#define RDA_ADD_GROUP_M(GRPF,GRPNUM)	\
	{ \
		.name = #GRPF "_grp" #GRPNUM, \
		.pins = GRPF ## _grp ## GRPNUM ## _pins, \
		.npins = ARRAY_SIZE(GRPF ## _grp ## GRPNUM ## _pins), \
	}

#define RDA_GP_ADD_GROUP(PORT, NUM) \
	{ \
		.name = #PORT #NUM "_grp", \
		.pins = PORT ## NUM ## _grp_pins, \
		.npins = ARRAY_SIZE(PORT ## NUM ## _grp_pins), \
	}

static const struct rda8810_pin_group rda8810_pin_groups[] = {
	RDA_M32(GP_ADD_GROUP, GPIO_A)
	RDA_M32(GP_ADD_GROUP, GPIO_B)
	RDA_M32(GP_ADD_GROUP, GPIO_C)
	RDA_M8(GP_ADD_GROUP, GPIO_D)

	RDA_ADD_GROUP(GPO_0, 0)
	RDA_ADD_GROUP(GPO_1, 0)
	RDA_ADD_GROUP(GPO_2, 0)
	RDA_ADD_GROUP(GPO_3, 0)
	RDA_ADD_GROUP(GPO_4, 0)
	RDA_ADD_GROUP(SPI1, 0)
	RDA_ADD_GROUP(SPI1_CS_1, 0)
	RDA_ADD_GROUP(SPI1_CS_2, 0)
	RDA_ADD_GROUP(SPI2, 0)
	RDA_ADD_GROUP(SPI2_CS_1, 0)
	RDA_ADD_GROUP(BB_SPI1, 0)
	RDA_ADD_GROUP(I2C1, 0)
	RDA_ADD_GROUP(I2C2, 0)
	// Cam / I2C - bit 3 : 0 / 1
	RDA_ADD_GROUP(I2C2, 1)
	// I2C3 / KeyOut 34 bit 17 :  0 / 1
	RDA_ADD_GROUP(I2C3, 0)
	RDA_ADD_GROUP(UART1_RXD_TXD, 0)
	RDA_ADD_GROUP(UART1_RTS_CTS, 0)
	RDA_ADD_GROUP(UART1_DTR_DCD_DSR_RI, 0)
	RDA_ADD_GROUP(UART2_RXD_TXD, 0)
	RDA_ADD_GROUP(UART2_RTS_CTS, 0)
	RDA_ADD_GROUP(UART3_RXD_TXD, 0)
	RDA_ADD_GROUP(UART3_RTS_CTS, 0)
	RDA_ADD_GROUP(BB_UART_RXD_TXD, 0)
	RDA_ADD_GROUP(KEYIN_0, 0)
	RDA_ADD_GROUP(KEYIN_1, 0)
	RDA_ADD_GROUP(KEYIN_2, 0)
	RDA_ADD_GROUP(KEYIN_3, 0)
	RDA_ADD_GROUP(KEYIN_4, 0)
	RDA_ADD_GROUP(KEYIN_5, 0)
	RDA_ADD_GROUP(KEYOUT_0, 0)
	RDA_ADD_GROUP(KEYOUT_1, 0)
	RDA_ADD_GROUP(KEYOUT_2, 0)
	RDA_ADD_GROUP(KEYOUT_34, 0)
	RDA_ADD_GROUP(KEYOUT_5, 0)
	RDA_ADD_GROUP(KEYINOUT_6, 0)
	RDA_ADD_GROUP(KEYINOUT_7, 0)
	RDA_ADD_GROUP(TCO_0, 0)
	RDA_ADD_GROUP(TCO_1, 0)
	RDA_ADD_GROUP(TCO_2, 0)
	RDA_ADD_GROUP(I2S_DI_2, 0)
	RDA_ADD_GROUP(I2S, 0)
	RDA_ADD_GROUP(DAI, 0)
	RDA_ADD_GROUP(DAI_SIMPLE, 0)
};

static int rda8810_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	return ARRAY_SIZE(rda8810_pin_groups);
}

static const char *rda8810_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	return rda8810_pin_groups[selector].name;
}

static int rda8810_get_group_pins(struct pinctrl_dev *pctldev,
				 unsigned int selector,
				 const unsigned int **pins,
				 unsigned int *num_pins)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);

	*pins = rda8810_pin_groups[selector].pins;
	*num_pins = rda8810_pin_groups[selector].npins;

	return 0;
}

static void rda8810_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
				unsigned int offset)
{
	seq_printf(s, " " DRIVER_NAME);
}

static const struct pinctrl_ops rda8810_pctrl_ops = {
	.get_groups_count = rda8810_get_groups_count,
	.get_group_name = rda8810_get_group_name,
	.get_group_pins = rda8810_get_group_pins,
	.pin_dbg_show = rda8810_pin_dbg_show,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

// ---------------------------------------------------------------------
#define RDA_GP_FUNCT_GROUPS()
	static const char * const [] = { "dramgrp" };


#define RDA_FUNCT_GROUPS(FUNC,)
	static const char * const dramgrps[] = { "dramgrp" };




static const struct rda8810_pmx_func rda8810_pmx_funcs[] = {
	
	
	RDA_M32(GP_ADD_GROUP, GPIO_A)
	RDA_M32(GP_ADD_GROUP, GPIO_B)
	RDA_M32(GP_ADD_GROUP, GPIO_C)
	RDA_M8(GP_ADD_GROUP, GPIO_D)

	RDA_ADD_GROUP(GPO_0, 0)
	RDA_ADD_GROUP(GPO_1, 0)
	RDA_ADD_GROUP(GPO_2, 0)
	RDA_ADD_GROUP(GPO_3, 0)
	RDA_ADD_GROUP(GPO_4, 0)
	RDA_ADD_GROUP(SPI1, 0)
	RDA_ADD_GROUP(SPI1_CS_1, 0)
	RDA_ADD_GROUP(SPI1_CS_2, 0)
	RDA_ADD_GROUP(SPI2, 0)
	RDA_ADD_GROUP(SPI2_CS_1, 0)
	RDA_ADD_GROUP(BB_SPI1, 0)
	RDA_ADD_GROUP(I2C1, 0)
	RDA_ADD_GROUP(I2C2, 0)
	// Cam / I2C - bit 3 : 0 / 1
	RDA_ADD_GROUP(I2C2, 1)
	// I2C3 / KeyOut 34 bit 17 :  0 / 1
	RDA_ADD_GROUP(I2C3, 0)
	RDA_ADD_GROUP(UART1_RXD_TXD, 0)
	RDA_ADD_GROUP(UART1_RTS_CTS, 0)
	RDA_ADD_GROUP(UART1_DTR_DCD_DSR_RI, 0)
	RDA_ADD_GROUP(UART2_RXD_TXD, 0)
	RDA_ADD_GROUP(UART2_RTS_CTS, 0)
	RDA_ADD_GROUP(UART3_RXD_TXD, 0)
	RDA_ADD_GROUP(UART3_RTS_CTS, 0)
	RDA_ADD_GROUP(BB_UART_RXD_TXD, 0)
	RDA_ADD_GROUP(KEYIN_0, 0)
	RDA_ADD_GROUP(KEYIN_1, 0)
	RDA_ADD_GROUP(KEYIN_2, 0)
	RDA_ADD_GROUP(KEYIN_3, 0)
	RDA_ADD_GROUP(KEYIN_4, 0)
	RDA_ADD_GROUP(KEYIN_5, 0)
	RDA_ADD_GROUP(KEYOUT_0, 0)
	RDA_ADD_GROUP(KEYOUT_1, 0)
	RDA_ADD_GROUP(KEYOUT_2, 0)
	RDA_ADD_GROUP(KEYOUT_34, 0)
	RDA_ADD_GROUP(KEYOUT_5, 0)
	RDA_ADD_GROUP(KEYINOUT_6, 0)
	RDA_ADD_GROUP(KEYINOUT_7, 0)
	RDA_ADD_GROUP(TCO_0, 0)
	RDA_ADD_GROUP(TCO_1, 0)
	RDA_ADD_GROUP(TCO_2, 0)
	RDA_ADD_GROUP(I2S_DI_2, 0)
	RDA_ADD_GROUP(I2S, 0)
	RDA_ADD_GROUP(DAI, 0)
	RDA_ADD_GROUP(DAI_SIMPLE, 0)



};

static int rda8810_pmx_set_mux(struct pinctrl_dev *pctldev,
			      unsigned int selector,
			      unsigned int group)
{
	struct rda8810_pmx *pmx;
	const struct rda8810_pmx_func *func;
	const struct rda8810_pin_group *grp;
	u32 before, after, expected;
	unsigned long tmp;
	int i;

	pmx = pinctrl_dev_get_drvdata(pctldev);

	func = &rda8810_pmx_functions[selector];
	grp = &rda8810_pin_groups[group];

	dev_dbg(pmx->dev,
		"ACTIVATE function \"%s\" with group \"%s\"\n",
		func->name, grp->name);

	regmap_read(pmx->map, GLOBAL_MISC_CTRL, &before);
	regmap_update_bits(pmx->map, GLOBAL_MISC_CTRL,
			   grp->mask | grp->value,
			   grp->value);
	regmap_read(pmx->map, GLOBAL_MISC_CTRL, &after);

	/* Which bits changed */
	before &= PADS_MASK;
	after &= PADS_MASK;
	expected = before &= ~grp->mask;
	expected |= grp->value;
	expected &= PADS_MASK;

	/* Print changed states */
	tmp = grp->mask;
	for_each_set_bit(i, &tmp, PADS_MAXBIT) {
		bool enabled = !(i > 3);

		/* Did not go low though it should */
		if (after & BIT(i)) {
			dev_err(pmx->dev,
				"pin group %s could not be %s: "
				"probably a hardware limitation\n",
				gemini_padgroups[i],
				enabled ? "enabled" : "disabled");
			dev_err(pmx->dev,
				"GLOBAL MISC CTRL before: %08x, after %08x, expected %08x\n",
				before, after, expected);
		} else {
			dev_dbg(pmx->dev,
				"padgroup %s %s\n",
				gemini_padgroups[i],
				enabled ? "enabled" : "disabled");
		}
	}

	tmp = grp->value;
	for_each_set_bit(i, &tmp, PADS_MAXBIT) {
		bool enabled = (i > 3);

		/* Did not go high though it should */
		if (!(after & BIT(i))) {
			dev_err(pmx->dev,
				"pin group %s could not be %s: "
				"probably a hardware limitation\n",
				gemini_padgroups[i],
				enabled ? "enabled" : "disabled");
			dev_err(pmx->dev,
				"GLOBAL MISC CTRL before: %08x, after %08x, expected %08x\n",
				before, after, expected);
		} else {
			dev_dbg(pmx->dev,
				"padgroup %s %s\n",
				gemini_padgroups[i],
				enabled ? "enabled" : "disabled");
		}
	}

	return 0;
}

static int rda8810_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(rda8810_pmx_funcs);
}

static const char *rda8810_pmx_get_func_name(struct pinctrl_dev *pctldev,
					    unsigned int selector)
{
	return rda8810_pmx_funcs[selector].name;
}

static int rda8810_pmx_get_groups(struct pinctrl_dev *pctldev,
				 unsigned int selector,
				 const char * const **groups,
				 unsigned int * const num_groups)
{
	*groups = rda8810_pmx_functs[selector].groups;
	*num_groups = rda8810_pmx_functions[selector].num_groups;
	return 0;
}

static const struct pinmux_ops rda8810_pmx_ops = {
	.get_functions_count = rda8810_pmx_get_funcs_count,
	.get_function_name = rda8810_pmx_get_func_name,
	.get_function_groups = rda8810_pmx_get_groups,
	.set_mux = rda8810_pmx_set_mux,
};
// ---------------------------------------------------------------------

static const struct rda8810_pin_conf *rda8810_get_pin_conf(struct rda8810_pmx *pmx,
							 unsigned int pin)
{
	const struct rda8810_pin_conf *retconf;
	int i;

	for (i = 0; i < pmx->nconfs; i++) {
		retconf = &pmx->confs[i];
		if (retconf->pin == pin)
			return retconf;
	}
	return NULL;
}

static int rda8810_pinconf_get(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *config)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	const struct rda8810_pin_conf *conf;
	u32 val;

	switch (param) {
	case PIN_CONFIG_SKEW_DELAY:
		conf = rda8810_get_pin_conf(pmx, pin);
		if (!conf)
			return -ENOTSUPP;
		regmap_read(pmx->map, conf->reg, &val);
		val &= conf->mask;
		val >>= (ffs(conf->mask) - 1);
		*config = pinconf_to_config_packed(PIN_CONFIG_SKEW_DELAY, val);
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static int rda8810_pinconf_set(struct pinctrl_dev *pctldev, unsigned int pin,
			      unsigned long *configs, unsigned int num_configs)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	const struct rda8810_pin_conf *conf;
	enum pin_config_param param;
	u32 arg;
	int ret = 0;
	int i;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_SKEW_DELAY:
			if (arg > 0xf)
				return -EINVAL;
			conf = rda8810_get_pin_conf(pmx, pin);
			if (!conf) {
				dev_err(pmx->dev,
					"invalid pin for skew delay %d\n", pin);
				return -ENOTSUPP;
			}
			arg <<= (ffs(conf->mask) - 1);
			dev_dbg(pmx->dev,
				"set pin %d to skew delay mask %08x, val %08x\n",
				pin, conf->mask, arg);
			regmap_update_bits(pmx->map, conf->reg, conf->mask, arg);
			break;
		default:
			dev_err(pmx->dev, "Invalid config param %04x\n", param);
			return -ENOTSUPP;
		}
	}

	return ret;
}

static int rda8810_pinconf_group_set(struct pinctrl_dev *pctldev,
				    unsigned selector,
				    unsigned long *configs,
				    unsigned num_configs)
{
	struct rda8810_pmx *pmx = pinctrl_dev_get_drvdata(pctldev);
	const struct rda8810_pin_group *grp = NULL;
	enum pin_config_param param;
	u32 arg;
	u32 val;
	int i;

	grp = &rda8810_pin_groups[selector];
	
	/* First figure out if this group supports configs */
	if (!grp->driving_mask) {
		dev_err(pmx->dev, "pin config group \"%s\" does "
			"not support drive strength setting\n",
			grp->name);
		return -EINVAL;
	}

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_STRENGTH:
			switch (arg) {
			case 4:
				val = 0;
				break;
			case 8:
				val = 1;
				break;
			case 12:
				val = 2;
				break;
			case 16:
				val = 3;
				break;
			default:
				dev_err(pmx->dev,
					"invalid drive strength %d mA\n",
					arg);
				return -ENOTSUPP;
			}
			val <<= (ffs(grp->driving_mask) - 1);
			regmap_update_bits(pmx->map, GLOBAL_IODRIVE,
					   grp->driving_mask,
					   val);
			dev_dbg(pmx->dev,
				"set group %s to %d mA drive strength mask %08x val %08x\n",
				grp->name, arg, grp->driving_mask, val);
			break;
		default:
			dev_err(pmx->dev, "invalid config param %04x\n", param);
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops rda8810_pinconf_ops = {
	.pin_config_get = rda8810_pinconf_get,
	.pin_config_set = rda8810_pinconf_set,
	.pin_config_group_set = rda8810_pinconf_group_set,
	.is_generic = true,
};

// ---------------------------------------------------------------------
static struct pinctrl_desc rda8810_pmx_desc = {
	.name = DRIVER_NAME,
	.pctlops = &rda8810_pctrl_ops,
	.pmxops = &rda8810_pmx_ops,
	.confops = &rda8810_pinconf_ops,
	.owner = THIS_MODULE,
};

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

	dev_info(dev, "Initialized RDA Micro RDA8810- pin control driver\n");

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

