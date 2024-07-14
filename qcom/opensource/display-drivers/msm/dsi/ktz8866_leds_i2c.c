/*
 * Copyright (C) 2011 ST-Ericsson SA.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * License Terms: GNU General Public License v2
 *
 * Simple driver for National Semiconductor bkl Backlight driver chip
 *
 * Author: Shreshtha Kumar SAHU <shreshthakumar.sahu@stericsson.com>
 * based on leds-bkl.c by Dan Murphy <D.Murphy@motorola.com>
 */

#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/backlight.h>
//#include <linux/moduleparam.h>

/**
 * struct bkl_data
 * @led_dev: led class device
 * @client: i2c client
 * @pdata: bkl platform data
 * @mode: mode of operation - manual, ALS, PWM
 * @regulator: regulator
 * @brightness: previous brightness value
 * @enable: regulator is enabled
 */

#define bkl_NAME "bkl-bl"

#define bkl_LED_DEV "bkl-bl"

#define BKL_LED_DEBUG    1
#define BKL_LOG(fmt, args...)    pr_info("[ktz8866_leds] %s %d: " fmt, __func__, __LINE__, ##args)
#define BKL_ERR(fmt, args...)    pr_err("[ktz8866_leds] %s %d: " fmt, __func__, __LINE__, ##args)
#if BKL_LED_DEBUG
#define BKL_DEBUG(fmt, args...)    pr_err("[ktz8866_leds] %s %d: " fmt, __func__, __LINE__, ##args)
#endif

struct bkl_data {
	struct led_classdev led_dev;
	struct i2c_client *client;
	struct device dev;
	struct i2c_adapter *adapter;
	unsigned short addr;
	struct mutex lock;
	struct work_struct work;
	enum led_brightness brightness;
	bool enable;
	u8 pwm_cfg;
	u8 full_scale_current;
	bool brt_code_enable;
	u16 *brt_code_table;
	int hwen_gpio;
	unsigned int  pwm_mode;
	bool using_lsb;
	unsigned int pwm_period;
	unsigned int full_scale_led;
	unsigned int ramp_on_time;
	unsigned int ramp_off_time;
	unsigned int pwm_trans_dim;
	unsigned int i2c_trans_dim;
	unsigned int channel;
	unsigned int ovp_level;
	unsigned int frequency;
	unsigned int default_brightness;
	unsigned int max_brightness;
	unsigned int induct_current;
	unsigned int flash_current;
	unsigned int flash_timeout;
	unsigned int bl_map;
	struct backlight_device *bl_dev;

};

#define MAX_BRIGHTNESS 2047
#define DEFAULT_BRIGHTNESS 1140
struct bkl_data *g_bkl_data;
struct bkl_data *drvdata;
static int bkl_temp = -1;

int i2c_bkl_write(struct i2c_client *client, uint8_t command, uint8_t data)
{
	int retry/*, loop_i*/;
	uint8_t buf[1 + 1];
	uint8_t toRetry = 5;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1 + 1,
			.buf = buf,
		}
	};
	buf[0] = command;
	buf[1] = data;
	for (retry = 0; retry < toRetry; retry++) {
		if (i2c_transfer(client->adapter, msg, 1) == 1)
			break;
		//msleep(20);
	}

	if (retry == toRetry) {
		BKL_ERR("i2c_write_block retry over %d\n", toRetry);
		return -EIO;
	}
	return 0;

}

static int bkl_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		BKL_ERR("read err: ret= %d\n", ret);
		return ret;
	}

	*val = ret;

	//BKL_LOG("Reading 0x%02x=0x%02x\n", reg, *val);
	return ret;
}

static int bkl_init_registers(struct bkl_data *drvdata)
{
	int ret = 0;
	ret |= i2c_bkl_write(drvdata->client,0x0C, 0x30);
	ret |= i2c_bkl_write(drvdata->client,0x0D, 0x28);
	ret |= i2c_bkl_write(drvdata->client,0x0E, 0x28);
	ret |= i2c_bkl_write(drvdata->client,0x09, 0x9E);
	msleep(10);
	//ret |= i2c_bkl_write(drvdata->client,0x02, 0x5A);
	ret |= i2c_bkl_write(drvdata->client,0x03, 0xCD);
	ret |= i2c_bkl_write(drvdata->client,0x04, 0x07);
	ret |= i2c_bkl_write(drvdata->client,0x05, 0xFF);
	ret |= i2c_bkl_write(drvdata->client,0x08, 0x5F);

	return ret;
}

void bkl_power_switch(int enable)
{
	if(enable == true)
	{
		i2c_bkl_write(g_bkl_data->client, 0x09, 0x9C);
		msleep(5);
		i2c_bkl_write(g_bkl_data->client, 0x09, 0x9E);
		//BKL_LOG("into power on bkl\n");
	}
	else
	{
		i2c_bkl_write(g_bkl_data->client, 0x09, 0x9C);
		i2c_bkl_write(g_bkl_data->client, 0x09, 0x98);
		//BKL_LOG("into power off bkl\n");
	}
}
EXPORT_SYMBOL(bkl_power_switch);

void bkl_hbm_switch(int enable)
{
	if(enable == true)
	{
		//i2c_bkl_write(g_bkl_data->client, 0x015, 0xC8);
		i2c_bkl_write(g_bkl_data->client, 0x004, 0x07);
		i2c_bkl_write(g_bkl_data->client, 0x005, 0xFF);//25.2MA
		BKL_LOG("bkl hbm open\n");
	}
	else
	{
		//i2c_bkl_write(g_bkl_data->client, 0x015, 0xB0);
		i2c_bkl_write(g_bkl_data->client, 0x004, 0x02);
		i2c_bkl_write(g_bkl_data->client, 0x005, 0xE6);//22.68MA
		BKL_LOG("bkl hbm close\n");
	}
}
EXPORT_SYMBOL(bkl_hbm_switch);

static void bkl_hwen_pin_ctrl(struct bkl_data *drvdata, int en)
{
	if (gpio_is_valid(drvdata->hwen_gpio)) {
		if (en) {
			BKL_LOG("hwen pin is going to be high!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, true);
			usleep_range(3500, 4000);
		} else {
			BKL_LOG("hwen pin is going to be low!---<%d>\n", en);
			gpio_set_value(drvdata->hwen_gpio, false);
			usleep_range(1000, 2000);
		}
	}
}

static int bkl_gpio_init(struct bkl_data *drvdata)
{

	int ret;

	if (gpio_is_valid(drvdata->hwen_gpio)) {
		ret = gpio_request(drvdata->hwen_gpio, "hwen_gpio");
		if (ret < 0) {
			BKL_ERR("failed to request gpio, ret= %d\n", ret);
			return -1;
		}
		ret = gpio_direction_output(drvdata->hwen_gpio, 1);
		if (ret < 0) {
			gpio_free(drvdata->hwen_gpio);
			BKL_ERR("failed to set output, ret= %d\n", ret);
			return ret;
		} else {
			BKL_LOG("gpio is valid, success!\n");
		}
		bkl_hwen_pin_ctrl(drvdata, 1);
	}

	return 0;
}


static int bkl_backlight_enable(struct bkl_data *drvdata)
{
	BKL_LOG("enter.\n");
	drvdata->enable = true;

	return 0;
}

int bkl_brightness_set(int brt_val)
{
	int err = 0;
	int brightness_lsb;
	int brightness_msb;

	if(bkl_temp == brt_val) {
		return err;
	}
	bkl_temp = brt_val;

	if (g_bkl_data->enable == false) {
		//bkl_init_registers(drvdata);
		bkl_backlight_enable(g_bkl_data);
	}
	BKL_LOG("backlight brightness is 0x%x\n", brt_val);
	if (brt_val < 0)
		brt_val =0;

	if (brt_val > MAX_BRIGHTNESS)
		brt_val = MAX_BRIGHTNESS;
	brightness_lsb = brt_val & 0x7;
	brightness_msb = (brt_val >> 3) & 0xff;
	//BKL_LOG("bkl brightness_lsb=%x , brightness_msb=%x\n", brightness_lsb, brightness_msb);

	if (brt_val > 0){
		err = i2c_bkl_write(g_bkl_data->client, 0x04, brightness_lsb);
		err = i2c_bkl_write(g_bkl_data->client, 0x05, brightness_msb);
	}else{
		err = i2c_bkl_write(g_bkl_data->client, 0x04, 0x00);
		err = i2c_bkl_write(g_bkl_data->client, 0x05, 0x00);

	}


	/* set the brightness in brightness control register*/
	//err |= i2c_bkl_write(drvdata->client, 0x04, (brt_val >> 4));
	//err |= i2c_bkl_write(drvdata->client, 0x05, (brt_val & 0xf));

	g_bkl_data->brightness = brt_val;
	if (g_bkl_data->brightness == 0)
		g_bkl_data->enable = false;
	return err;
}
EXPORT_SYMBOL(bkl_brightness_set);

static int bkl_bl_get_brightness(struct backlight_device *bl_dev)
{
		return bl_dev->props.brightness;
}

static inline int bkl_bl_update_status(struct backlight_device *bl_dev)
{
		//struct bkl_data *drvdata = bl_get_data(bl_dev);
		int brt;

		if (bl_dev->props.state & BL_CORE_SUSPENDED)
				bl_dev->props.brightness = 0;

		brt = bl_dev->props.brightness;
		/*
		 * Brightness register should always be written
		 * not only register based mode but also in PWM mode.
		 */
		return bkl_brightness_set(brt);
}

int bkl_backlight_device_set_brightness(struct backlight_device *bl_dev,
				    unsigned long brightness)
{
	int rc = -ENXIO;
	//struct bkl_data *drvdata = bl_get_data(bl_dev);

	mutex_lock(&bl_dev->ops_lock);
	if (bl_dev->ops) {
			if (brightness > bl_dev->props.max_brightness)
			brightness = bl_dev->props.max_brightness;

			BKL_DEBUG("set brightness to %lu\n", brightness);
			//brightness = brightness * 9 / 10;
			bl_dev->props.brightness = brightness;
			rc = bkl_bl_update_status(bl_dev);
		}
	mutex_unlock(&bl_dev->ops_lock);

	//backlight_generate_event(bl_dev, BACKLIGHT_UPDATE_SYSFS);

	return rc;
}
EXPORT_SYMBOL(bkl_backlight_device_set_brightness);

static const struct backlight_ops bkl_bl_ops = {
		.update_status = bkl_bl_update_status,
		.get_brightness = bkl_bl_get_brightness,
};

static ssize_t bkl_i2c_reg_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct bkl_data *drvdata = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < 0x20; i++) {
	bkl_read_reg(drvdata->client, i, &reg_val);
	len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%02x=0x%02x\n",i, reg_val);
	}
	return len;
}

static ssize_t bkl_i2c_reg_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
        struct bkl_data *drvdata = dev_get_drvdata(dev);

        unsigned int databuf[2] = {0, 0};

        if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
                i2c_bkl_write(drvdata->client,
                                (unsigned char)databuf[0],
                                (unsigned char)databuf[1]);
        }

        return count;
}

static DEVICE_ATTR(bkl_reg, 0664, bkl_i2c_reg_show, bkl_i2c_reg_store);

static struct attribute *bkl_attributes[] = {
	&dev_attr_bkl_reg.attr,
	NULL
};

static struct attribute_group bkl_attribute_group = {
	.attrs = bkl_attributes
};

static void __bkl_work(struct bkl_data *led,
				enum led_brightness value)
{
	mutex_lock(&led->lock);
	bkl_brightness_set(value);
	mutex_unlock(&led->lock);
}

static void bkl_work(struct work_struct *work)
{
	struct bkl_data *drvdata = container_of(work,
					struct bkl_data, work);

	__bkl_work(drvdata, drvdata->led_dev.brightness);
}

static void bkl_set_brightness(struct led_classdev *led_cdev,
			enum led_brightness brt_val)
{
	struct bkl_data *drvdata;

	drvdata = container_of(led_cdev, struct bkl_data, led_dev);
	schedule_work(&drvdata->work);
}

static void bkl_get_dt_data(struct device *dev, struct bkl_data *drvdata)
{
	struct device_node *np = dev->of_node;
	//u32 bl_channel, temp;

	drvdata->hwen_gpio = of_get_named_gpio(np, "bkl,hwen-gpio", 0);
	BKL_LOG("drvdata->hwen_gpio --<%d>\n", drvdata->hwen_gpio);

}



static int bkl_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bkl_data *drvdata;
	struct backlight_device *bl_dev;
	struct backlight_properties props;

	int err = 0;

	BKL_LOG("bkl bkl_probe start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		BKL_ERR("I2C_FUNC_I2C not supported\n");
		err = -EIO;
		goto err_out;
	}

	if (!client->dev.of_node) {
		BKL_ERR("no device node\n");
		err = -ENOMEM;
		goto err_out;
	}

	drvdata = kzalloc(sizeof(struct bkl_data), GFP_KERNEL);
	if (drvdata == NULL) {
		BKL_ERR("kzalloc failed\n");
		err = -ENOMEM;
		goto err_out;
	}

	drvdata->client = client;
	drvdata->adapter = client->adapter;
	drvdata->addr = client->addr;
	drvdata->brightness = LED_OFF;
	drvdata->enable = true;
	drvdata->led_dev.default_trigger = "bkl-trigger";
	drvdata->led_dev.name = bkl_LED_DEV;
	drvdata->led_dev.brightness_set = bkl_set_brightness;
	drvdata->led_dev.max_brightness = MAX_BRIGHTNESS;
	mutex_init(&drvdata->lock);
	INIT_WORK(&drvdata->work, bkl_work);
	bkl_get_dt_data(&client->dev, drvdata);
	i2c_set_clientdata(client, drvdata);
	bkl_gpio_init(drvdata);

#if 0
	err = bkl_read_chipid(drvdata);
	if (err < 0) {
		BKL_ERR("ID idenfy failed, err= %d\n", err);
		goto err_init;
	}
#endif
	err = led_classdev_register(&client->dev, &drvdata->led_dev);
	if (err < 0) {
		BKL_ERR("Register led class failed, err= %d\n", err);
		err = -ENODEV;
		goto err_init;
	} else {
		BKL_LOG("Register led class successful\n");
	}


	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.brightness = DEFAULT_BRIGHTNESS;
	props.max_brightness = MAX_BRIGHTNESS;
	bl_dev = backlight_device_register(bkl_NAME, &client->dev,
					drvdata, &bkl_bl_ops, &props);


	g_bkl_data = drvdata;
	bkl_init_registers(drvdata);
	bkl_backlight_enable(drvdata);

	//bkl_brightness_set(drvdata, DEFAULT_BRIGHTNESS);

	err = sysfs_create_group(&client->dev.kobj, &bkl_attribute_group);
	if (err < 0) {
		BKL_ERR("error creating sysfs attr fileserr= %d\n", err);
		goto err_sysfs;
	}

	BKL_LOG("probe success, exit\n");
	return 0;

err_sysfs:
err_init:
	kfree(drvdata);
err_out:
	return err;
}

 static int bkl_remove(struct i2c_client *client)
{
		struct bkl_data *drvdata = i2c_get_clientdata(client);
		led_classdev_unregister(&drvdata->led_dev);
		kfree(drvdata);
		return 0;
}

static const struct i2c_device_id bkl_id[] = {
	{bkl_NAME, 0},
	{}
};
static struct of_device_id match_table[] = {
			{.compatible = "bkl,bkl-bl",}
};

MODULE_DEVICE_TABLE(i2c, bkl_id);

static struct i2c_driver bkl_i2c_driver = {
	.probe = bkl_probe,
	.remove = bkl_remove,
	.id_table = bkl_id,
	.driver = {
		.name = bkl_NAME,
		.owner = THIS_MODULE,
		.of_match_table = match_table,
	},
};
/*
int leds_bkl_init(void)
{
	int ret = 0;
	BKL_LOG("bkl leds_bkl_init start\n");
	ret = i2c_add_driver(&bkl_i2c_driver);
	if (ret < 0) {
		BKL_ERR("bkl leds_bkl_init failed, ret = %d\n", ret);
	} else {
		BKL_LOG("bkl leds_bkl_init success\n");
	}
	return ret;
}
EXPORT_SYMBOL(leds_bkl_init);

void leds_bkl_exit(void)
{
	i2c_del_driver(&bkl_i2c_driver);
	BKL_LOG("bkl leds_bkl_exit \n");

}
EXPORT_SYMBOL(leds_bkl_exit);
*/
//MODULE_SOFTDEP("post: msm_drm");
module_i2c_driver(bkl_i2c_driver);

MODULE_DESCRIPTION("Back Light driver for bkl");
MODULE_LICENSE("GPL v2");
