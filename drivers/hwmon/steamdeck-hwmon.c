// SPDX-License-Identifier: GPL-2.0+
/*
 * Steam Deck EC sensors driver
 *
 * Copyright (C) 2021-2022 Valve Corporation
 */

#include <linux/acpi.h>
#include <linux/hwmon.h>
#include <linux/platform_device.h>

#define STEAMDECK_HWMON_NAME	"steamdeck-hwmon"

struct steamdeck_hwmon {
	struct acpi_device *adev;
};

static ssize_t
steamdeck_hwmon_simple_store(struct device *dev, const char *buf, size_t count,
			     const char *method,
			     unsigned long lower_limit,
			     unsigned long upper_limit,
			     unsigned long *parsed_value)
{
	struct steamdeck_hwmon *sd = dev_get_drvdata(dev);
	unsigned long value;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	value = clamp_val(value, lower_limit, upper_limit);
	if (parsed_value)
		*parsed_value = value;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	if (ACPI_FAILURE(acpi_execute_simple_method(sd->adev->handle,
						    (char *)method, value)))
		return -EIO;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	return count;
}

static int steamdeck_hwmon_ec_fan_recalculate(struct steamdeck_hwmon *sd,
					      bool on)
{
	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);
	/*
	 * We need to call SCHG(1) to notify EC that one of the PID
	 * loop attributes has been updated and to enable PID control
	 * if it's not enabled yet.
	 *
	 * Writing 0 to any of those attributes will disable PID loop
	 * by calling SCHG(0)
	 */
	if (ACPI_FAILURE(acpi_execute_simple_method(sd->adev->handle,
						    "SCHG", on)))
		return -EIO;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	return 0;
}

static ssize_t
steamdeck_hwmon_pid_attr_store(struct device *dev, const char *buf,
			       size_t count, const char *method,
			       unsigned long lower_limit,
			       unsigned long upper_limit)
{
	struct steamdeck_hwmon *sd = dev_get_drvdata(dev);
	unsigned long value;
	ssize_t ret;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	ret = steamdeck_hwmon_simple_store(dev, buf, count, method,
					   lower_limit, upper_limit,
					   &value);
	if (ret < 0)
		return ret;

	pr_info("XXXXXXXXXXXXXXXXXXXX: %s %d\n", __func__, __LINE__);

	return steamdeck_hwmon_ec_fan_recalculate(sd, value > 0);
}

#define STEAMDECK_HWMON_PID_ATTR_WO(_name, _method, _upper_limit)	\
	static ssize_t _name##_store(struct device *dev,		\
				     struct device_attribute *attr,	\
				     const char *buf, size_t count)	\
	{								\
		return steamdeck_hwmon_pid_attr_store(dev, buf, count,	\
						      _method,		\
						      0, _upper_limit);	\
	}								\
	static DEVICE_ATTR_WO(_name)

STEAMDECK_HWMON_PID_ATTR_WO(target_cpu_temp, "STCT", U8_MAX / 2);
STEAMDECK_HWMON_PID_ATTR_WO(gain, "SGAN", U8_MAX);
STEAMDECK_HWMON_PID_ATTR_WO(ramp_rate, "SFRR",  U8_MAX);
STEAMDECK_HWMON_PID_ATTR_WO(hysteresis, "SHTS", U8_MAX);

static ssize_t maximum_battery_charge_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	return steamdeck_hwmon_simple_store(dev, buf, count,
					    "CHGR",
					    250, 2500, NULL);
}
static DEVICE_ATTR_WO(maximum_battery_charge_rate);

static struct attribute *steamdeck_hwmon_attrs[] = {
	&dev_attr_target_cpu_temp.attr,
	&dev_attr_gain.attr,
	&dev_attr_ramp_rate.attr,
	&dev_attr_hysteresis.attr,
	&dev_attr_maximum_battery_charge_rate.attr,
	NULL
};
ATTRIBUTE_GROUPS(steamdeck_hwmon);

static long
steamdeck_hwmon_get(struct steamdeck_hwmon *sd, const char *method)
{
	unsigned long long val;
	if (ACPI_FAILURE(acpi_evaluate_integer(sd->adev->handle,
					       (char *)method, NULL, &val)))
		return -EIO;

	return val;
}

static int
steamdeck_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
		     u32 attr, int channel, long *out)
{
	struct steamdeck_hwmon *sd = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_curr:
		if (attr != hwmon_curr_input)
			return -EOPNOTSUPP;

		*out = steamdeck_hwmon_get(sd, "PDAM");
		if (*out < 0)
			return *out;
		break;
	case hwmon_in:
		if (attr != hwmon_in_input)
			return -EOPNOTSUPP;

		*out = steamdeck_hwmon_get(sd, "PDVL");
		if (*out < 0)
			return *out;
		break;
	case hwmon_temp:
		if (attr != hwmon_temp_input)
			return -EOPNOTSUPP;

		*out = steamdeck_hwmon_get(sd, "BATT");
		if (*out < 0)
			return *out;
		/*
		 * Assuming BATT returns deg C we need to mutiply it
		 * by 1000 to convert to mC
		 */
		*out *= 1000;
		break;
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			*out = steamdeck_hwmon_get(sd, "FANR");
			if (*out < 0)
				return *out;
			break;
		case hwmon_fan_target:
			*out = steamdeck_hwmon_get(sd, "FSSR");
			if (*out < 0)
				return *out;
			break;
		case hwmon_fan_fault:
			*out = steamdeck_hwmon_get(sd, "FANC");
			if (*out < 0)
				return *out;
			/*
			 * FANC (Fan check):
			 * 0: Abnormal
			 * 1: Normal
			 */
			*out = !*out;
			break;
		default:
			return -EOPNOTSUPP;
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int
steamdeck_hwmon_read_string(struct device *dev, enum hwmon_sensor_types type,
			    u32 attr, int channel, const char **str)
{
	switch (type) {
		/*
		 * These two aren't, strictly speaking, measured. EC
		 * firmware just reports what PD negotiation resulted
		 * in.
		 */
	case hwmon_curr:
		*str = "PD Contract Current";
		break;
	case hwmon_in:
		*str = "PD Contract Voltage";
		break;
	case hwmon_temp:
		*str = "Battery Temp";
		break;
	case hwmon_fan:
		*str = "System Fan";
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int
steamdeck_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
		      u32 attr, int channel, long val)
{
	struct steamdeck_hwmon *sd = dev_get_drvdata(dev);

	if (type != hwmon_fan ||
	    attr != hwmon_fan_target)
		return -EOPNOTSUPP;

	val = clamp_val(val, 0, 7300);

	if (ACPI_FAILURE(acpi_execute_simple_method(sd->adev->handle,
						    "FANS", val)))
		return -EIO;

	return steamdeck_hwmon_ec_fan_recalculate(sd, val > 0);
}

static umode_t
steamdeck_hwmon_is_visible(const void *data, enum hwmon_sensor_types type,
			   u32 attr, int channel)
{
	if (type == hwmon_fan &&
	    attr == hwmon_fan_target)
		return 0644;

	return 0444;
}

static const struct hwmon_channel_info *steamdeck_hwmon_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LABEL),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_LABEL),
	HWMON_CHANNEL_INFO(fan,
			   HWMON_F_INPUT | HWMON_F_LABEL |
			   HWMON_F_TARGET | HWMON_F_FAULT),
	NULL
};

static const struct hwmon_ops steamdeck_hwmon_ops = {
	.is_visible = steamdeck_hwmon_is_visible,
	.read = steamdeck_hwmon_read,
	.read_string = steamdeck_hwmon_read_string,
	.write = steamdeck_hwmon_write,
};

static const struct hwmon_chip_info steamdeck_hwmon_chip_info = {
	.ops = &steamdeck_hwmon_ops,
	.info = steamdeck_hwmon_info,
};

static int steamdeck_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct steamdeck_hwmon *sd;
	struct device *hwmon;

	sd = devm_kzalloc(dev, sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->adev = ACPI_COMPANION(dev->parent);
	hwmon = devm_hwmon_device_register_with_info(dev,
						     STEAMDECK_HWMON_NAME,
						     sd,
						     &steamdeck_hwmon_chip_info,
						     steamdeck_hwmon_groups);
	if (IS_ERR(hwmon)) {
		dev_err(dev, "Failed to register HWMON device");
		return PTR_ERR(hwmon);
	}

	return 0;
}

static const struct platform_device_id steamdeck_hwmon_id_table[] = {
	{ .name = STEAMDECK_HWMON_NAME },
	{}
};
MODULE_DEVICE_TABLE(platform, steamdeck_hwmon_id_table);

static struct platform_driver steamdeck_hwmon_driver = {
	.probe = steamdeck_hwmon_probe,
	.driver = {
		.name = STEAMDECK_HWMON_NAME,
	},
	.id_table = steamdeck_hwmon_id_table,
};
module_platform_driver(steamdeck_hwmon_driver);

MODULE_AUTHOR("Andrey Smirnov <andrew.smirnov@gmail.com>");
MODULE_DESCRIPTION("Steam Deck EC sensors driver");
MODULE_LICENSE("GPL");
