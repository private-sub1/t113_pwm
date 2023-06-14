// SPDX-License-Identifier: GPL-2.0
/*
 * PWM Controller Driver for sunxi platforms (D1, T113-S3 and R329)
 *
 * Copyright (c) 2023 Aleksandr Shubin <privatesub2@gmail.com>
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/reset.h>

#define PWM_CLK_CFG_REG(chan)		(0x20 + ((chan >> 1) * 0x4))
#define PWM_CLK_SRC			7
#define PWM_CLK_SRC_MASK		GENMASK(8, PWM_CLK_SRC)
#define PWM_CLK_DIV_M			0
#define PWM_CLK_DIV_M_MASK		GENMASK(3, PWM_CLK_DIV_M)

#define PWM_CLK_GATE_REG		0x40
#define PWM_CLK_GATING(chan)		BIT(chan)

#define PWM_ENABLE_REG			0x80
#define PWM_EN(chan)			BIT(chan)

#define PWM_CTL_REG(chan)		(0x100 + chan * 0x20)
#define PWM_ACT_STA			BIT(8)
#define PWM_PRESCAL_K			0
#define PWM_PRESCAL_K_MASK		GENMASK(7, PWM_PRESCAL_K)

#define PWM_PERIOD_REG(chan)		(0x104 + chan * 0x20)
#define PWM_ENTIRE_CYCLE		16
#define PWM_ENTIRE_CYCLE_MASK		GENMASK(31, PWM_ENTIRE_CYCLE)
#define PWM_ACT_CYCLE			0
#define PWM_ACT_CYCLE_MASK		GENMASK(15, PWM_ACT_CYCLE)

#define SET_VALUE(reg_val, val, name) \
		reg_val = (((reg_val) & ~name##_MASK) | (val << name))
#define GET_VALUE(reg_val, name) \
		(((reg_val) & ~name##_MASK) >> name)


struct sun20i_pwm_chip {
	struct pwm_chip chip;
	struct clk *clk_bus, *clk_hosc;
	struct reset_control *rst;
	void __iomem *base;
	/* Mutex to protect pwm apply state */
	struct mutex mutex;
};

static inline struct sun20i_pwm_chip *to_sun20i_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct sun20i_pwm_chip, chip);
}

static inline u32 sun20i_pwm_readl(struct sun20i_pwm_chip *chip,
				   unsigned long offset)
{
	return readl(chip->base + offset);
}

static inline void sun20i_pwm_writel(struct sun20i_pwm_chip *chip,
				     u32 val, unsigned long offset)
{
	writel(val, chip->base + offset);
}

static int sun20i_pwm_get_state(struct pwm_chip *chip,
				struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);
	u64 clk_rate, tmp;
	u32 val;
	u16 clk_div, act_cycle;
	u8 prescal, div_id;
	u8 chn = pwm->hwpwm;

	might_sleep();
	mutex_lock(&sun20i_chip->mutex);

	val = sun20i_pwm_readl(sun20i_chip, PWM_CLK_CFG_REG(pwm->hwpwm));
	div_id = GET_VALUE(val, PWM_CLK_DIV_M);
	if (GET_VALUE(val, PWM_CLK_SRC) == 0)
		clk_rate = clk_get_rate(sun20i_chip->clk_hosc);
	else
		clk_rate = clk_get_rate(sun20i_chip->clk_bus);

	val = sun20i_pwm_readl(sun20i_chip, PWM_CTL_REG(pwm->hwpwm));
	if (PWM_ACT_STA & val)
		state->polarity = PWM_POLARITY_NORMAL;
	else
		state->polarity = PWM_POLARITY_INVERSED;

	prescal = PWM_PRESCAL_K & val;

	val = sun20i_pwm_readl(sun20i_chip, PWM_ENABLE_REG);
	if (PWM_EN(chn) & val)
		state->enabled = true;
	else
		state->enabled = false;

	val = sun20i_pwm_readl(sun20i_chip, PWM_PERIOD_REG(pwm->hwpwm));
	act_cycle = GET_VALUE(val, PWM_ACT_CYCLE);
	clk_div = GET_VALUE(val, PWM_ENTIRE_CYCLE);

	tmp = act_cycle * prescal * (1U << div_id) * NSEC_PER_SEC;
	state->duty_cycle = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);
	tmp = clk_div * prescal * (1U << div_id) * NSEC_PER_SEC;
	state->period = DIV_ROUND_CLOSEST_ULL(tmp, clk_rate);

	mutex_unlock(&sun20i_chip->mutex);

	return 0;
}

static int sun20i_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			    const struct pwm_state *state)
{
	struct pwm_state curstate;
	int ret = 0;
	u32 clk_gate, clk_cfg, pwm_en, ctl, period;
	u64 bus_rate, hosc_rate, clk_div, val, clk_rate;
	u16 prescaler, div_m;
	bool use_bus_clk;
	struct sun20i_pwm_chip *sun20i_chip = to_sun20i_pwm_chip(chip);

	mutex_lock(&sun20i_chip->mutex);
	pwm_get_state(pwm, &curstate);

	pwm_en = sun20i_pwm_readl(sun20i_chip, PWM_ENABLE_REG);
	if (state->polarity != curstate.polarity ||
	    state->duty_cycle != curstate.duty_cycle ||
	    state->period != curstate.period) {
			
		ctl = sun20i_pwm_readl(sun20i_chip, PWM_CTL_REG(pwm->hwpwm));
		clk_cfg = sun20i_pwm_readl(sun20i_chip, PWM_CLK_CFG_REG(pwm->hwpwm));
		hosc_rate = clk_get_rate(sun20i_chip->clk_hosc);
		bus_rate = clk_get_rate(sun20i_chip->clk_bus);

		if (pwm_en & PWM_EN(pwm->hwpwm ^ 1)) {
			/* check period only */
			if (GET_VALUE(clk_cfg, PWM_CLK_SRC) == 0)
				clk_rate = hosc_rate;
			else
				clk_rate = bus_rate;

			val = state->period * clk_rate;
			do_div(val, NSEC_PER_SEC);

			div_m = GET_VALUE(clk_cfg, PWM_CLK_DIV_M);

			/* calculate and set prescaler, PWM entire cycle */
			clk_div = val;
			for(prescaler = 0; clk_div > 65535; prescaler++) {
				if (prescaler >= 256) {
					dev_err(sun20i_chip->chip.dev, "Period is too long\n");
					ret = -EINVAL;
					goto unlock_mutex;
				}

				clk_div = val;
				do_div(clk_div, 1U << div_m);
				do_div(clk_div, prescaler + 1);
			}
		} else {
			/* check period and select clock source */
			use_bus_clk = false;
			val = state->period * hosc_rate;
			do_div(val, NSEC_PER_SEC);
			if (val <= 1) {
				use_bus_clk = true;
				val = state->period * bus_rate;
				do_div(val, NSEC_PER_SEC);
				if (val <= 1) {
					dev_err(sun20i_chip->chip.dev, "Period is too small\n");
					ret = -EINVAL;
					goto unlock_mutex;
				}
			}

			if (use_bus_clk)
				SET_VALUE(clk_cfg, 1, PWM_CLK_SRC);
			else
				SET_VALUE(clk_cfg, 0, PWM_CLK_SRC);

			/* calculate and set prescaler, M factor, PWM entire cycle */
			clk_div = val;
			for(prescaler = div_m = 0; clk_div > 65535; prescaler++) {
				if (prescaler >= 256) {
					prescaler = 0;
					div_m++;
					if (div_m >= 9) {
						dev_err(sun20i_chip->chip.dev, "Period is too long\n");
						ret = -EINVAL;
						goto unlock_mutex;
					}
				}

				clk_div = val;
				do_div(clk_div, 1U << div_m);
				do_div(clk_div, prescaler + 1);
			}

			/* set up the M factor */
			SET_VALUE(clk_cfg, div_m, PWM_CLK_DIV_M);

			sun20i_pwm_writel(sun20i_chip, clk_cfg, PWM_CLK_CFG_REG(pwm->hwpwm));
		}

		period = sun20i_pwm_readl(sun20i_chip, PWM_PERIOD_REG(pwm->hwpwm));

		SET_VALUE(period, clk_div, PWM_ENTIRE_CYCLE);
		SET_VALUE(ctl, prescaler, PWM_PRESCAL_K);

		/* set duty cycle */
		val = state->period;
		do_div(val, clk_div);
		clk_div = state->duty_cycle;
		do_div(clk_div, val);
		if (clk_div > 65535)
			clk_div = 65535;

		SET_VALUE(period, clk_div, PWM_ACT_CYCLE);
		sun20i_pwm_writel(sun20i_chip, period, PWM_PERIOD_REG(pwm->hwpwm));

		if (state->polarity == PWM_POLARITY_NORMAL)
			ctl |= PWM_ACT_STA;
		else
			ctl &= ~PWM_ACT_STA;

		sun20i_pwm_writel(sun20i_chip, ctl, PWM_CTL_REG(pwm->hwpwm));
	}

	if (state->enabled != curstate.enabled) {
		clk_gate = sun20i_pwm_readl(sun20i_chip, PWM_CLK_GATE_REG);

		if (state->enabled) {
			clk_gate |= PWM_CLK_GATING(pwm->hwpwm);
			pwm_en |= PWM_EN(pwm->hwpwm);
		} else {
			clk_gate &= ~PWM_CLK_GATING(pwm->hwpwm);
			pwm_en &= ~PWM_EN(pwm->hwpwm);
		}
		sun20i_pwm_writel(sun20i_chip, pwm_en, PWM_ENABLE_REG);
		sun20i_pwm_writel(sun20i_chip, clk_gate, PWM_CLK_GATE_REG);
	}

unlock_mutex:
	mutex_unlock(&sun20i_chip->mutex);

	return ret;
}

static const struct pwm_ops sun20i_pwm_ops = {
	.get_state = sun20i_pwm_get_state,
	.apply = sun20i_pwm_apply,
	.owner = THIS_MODULE,
};

static const struct of_device_id sun20i_pwm_dt_ids[] = {
	{ .compatible = "allwinner,sun20i-d1-pwm" },
	{ },
};
MODULE_DEVICE_TABLE(of, sun20i_pwm_dt_ids);

static int sun20i_pwm_probe(struct platform_device *pdev)
{
	struct sun20i_pwm_chip *sun20i_chip;
	int ret;

	sun20i_chip = devm_kzalloc(&pdev->dev, sizeof(*sun20i_chip), GFP_KERNEL);
	if (!sun20i_chip)
		return -ENOMEM;

	sun20i_chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sun20i_chip->base))
		return PTR_ERR(sun20i_chip->base);

	sun20i_chip->clk_bus = devm_clk_get(&pdev->dev, "bus");
	if (IS_ERR(sun20i_chip->clk_bus)) {
		dev_err(&pdev->dev, "Failed to get bus clock\n");
		return PTR_ERR(sun20i_chip->clk_bus);
	}

	sun20i_chip->clk_hosc = devm_clk_get(&pdev->dev, "hosc");
	if (IS_ERR(sun20i_chip->clk_hosc)) {
		dev_err(&pdev->dev, "Failed to get hosc clock\n");
		return PTR_ERR(sun20i_chip->clk_hosc);
	}

	sun20i_chip->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(sun20i_chip->rst)) {
		dev_err(&pdev->dev, "Failed to get bus reset\n");
		return PTR_ERR(sun20i_chip->rst);
	}

	/* Deassert reset */
	ret = reset_control_deassert(sun20i_chip->rst);
	if (ret) {
		dev_err(&pdev->dev, "Failed to deassert reset\n");
		return ret;
	}

	ret = clk_prepare_enable(sun20i_chip->clk_bus);
	if (ret) {
		dev_err(&pdev->dev, "Failed to ungate bus clock\n");
		goto err_bus;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
				   "allwinner,pwm-channels",
				   &sun20i_chip->chip.npwm);
	if (ret) {
		dev_err(&pdev->dev, "Can't get pwm-channels\n");
		goto err_pwm_add;
	}

	sun20i_chip->chip.dev = &pdev->dev;
	sun20i_chip->chip.ops = &sun20i_pwm_ops;

	mutex_init(&sun20i_chip->mutex);

	ret = pwmchip_add(&sun20i_chip->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to add PWM chip: %d\n", ret);
		goto err_pwm_add;
	}

	platform_set_drvdata(pdev, sun20i_chip);

	return 0;

err_pwm_add:
	clk_disable_unprepare(sun20i_chip->clk_bus);
err_bus:
	reset_control_assert(sun20i_chip->rst);
	return ret;
}

static void sun20i_pwm_remove(struct platform_device *pdev)
{
	struct sun20i_pwm_chip *sun20i_chip = platform_get_drvdata(pdev);
	int ret;

	pwmchip_remove(&sun20i_chip->chip);

	clk_disable_unprepare(sun20i_chip->clk_bus);
	reset_control_assert(sun20i_chip->rst);
}

static struct platform_driver sun20i_pwm_driver = {
	.driver = {
		.name = "sun20i-pwm",
		.of_match_table = sun20i_pwm_dt_ids,
	},
	.probe = sun20i_pwm_probe,
	.remove_new = sun20i_pwm_remove,
};
module_platform_driver(sun20i_pwm_driver);

MODULE_AUTHOR("Aleksandr Shubin <privatesub2@gmail.com>");
MODULE_DESCRIPTION("Allwinner sun20i PWM driver");
MODULE_LICENSE("GPL v2");
