/*
 * Driver for the Sony ECX337AF OLED
 * Preliminary - only performs hardware reset and then
 * wakes up the device from power saving mode by writing to SPI registers.
 * Copyright 2017 FLIR Systems, Inc.
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>

#undef TEST_REGISTER_READ

typedef enum
{
    Register_0x00,
    Register_0x01,
    Register_0x02,
    Register_0x03,
    Register_0x04,
    Register_0x05,
    Register_0x06,
    Register_0x07,
    Register_0x08,
    Register_0x09,
    Register_0x0A,
    Register_0x0B,
    Register_0x0C,
    Register_0x0D,
    Register_0x0E,
    Register_0x0F,
    Register_0x10,
    Register_0x11,
    Register_0x12,
    Register_0x13,
    Register_0x14,
    Register_0x15,
    Register_0x16,
    Register_0x17,
    Register_0x18,
    Register_0x19,
    Register_0x1A,
    Register_0x1B,
    Register_0x1C,
    Register_0x1D,
    Register_0x1E,
    Register_0x1F,
    Register_0x20,
    Register_0x21,
    Register_0x22,
    Register_0x23,
    Register_0x24,
    Register_0x25,
    Register_0x26,
    Register_0x27,
    Register_0x28,
    Register_0x29,
    Register_0x2A,
    Register_0x2B,
    Register_0x2C,
    Register_0x2D,
    Register_0x2E,
    Register_0x2F,
    Register_0x30,
    Register_0x31,
    Register_0x32,
    Register_0x33,
    Register_0x34,
    Register_0x35,
    Register_0x36,
    Register_0x37,
    Register_0x38,
    Register_0x39,
    Register_0x3A,
    Register_0x3B,
    Register_0x3C,
    Register_0x3D,
    Register_0x3E,
    Register_0x3F,
    Register_0x40,
    Register_0x41,
    Register_0x42,
    Register_0x43,
    Register_0x44,
    Register_0x45,
    Register_0x46,
    Register_0x47,
    Register_0x48,
    Register_0x49,
    Register_0x4A,
    Register_0x4B,
    Register_0x4C,
    Register_0x4D,
    Register_0x4E,
    Register_0x4F,
    Register_0x50,
    Register_0x51,
    Register_0x52,
    Register_0x53,
    Register_0x54,
    Register_0x55,
    Register_0x56,
    Register_0x57,
    Register_0x58,
    Register_0x59,
    Register_0x5A,
    Register_0x5B,
    Register_0x5C,
    Register_0x5D,
    Register_0x5E,
    Register_0x5F,
    Register_0x60,
    Register_0x61,
    Register_0x62,
    Register_0x63,
    Register_0x64,
    Register_0x65,
    Register_0x66,
    Register_0x67,
    Register_0x68,
    Register_0x69,
    Register_0x6A,
    Register_0x6B,
    Register_0x6C,
    Register_0x6D,
    Register_0x6E,
    Register_0x6F,
    Register_0x70,
    Register_0x71,
    Register_0x72,
    Register_0x73,
    Register_0x74,
    Register_0x75,
    Register_0x76,
    Register_0x77,
    Register_0x78,
    Register_0x79,
    Register_0x7A,
    Register_0x7B,
    Register_0x7C,
    Register_0x7D,
    Register_0x7E,
    Register_0x7F,
    Register_0x80,
    Register_0x81
}Register;


#define ECX337_


struct ecx337af_data {
	unsigned		reset;
	struct spi_device	*spi;
	int			state;
};


static uint8_t reorder_bits(uint8_t in)
{
	uint8_t ret = 0;
	uint8_t x;
	
	for (x = 0; x < 8; x++)
	{
		if (in & (1 << x))
		{
			ret |= (1 << (7-x));
		}
	}
	return (ret);
}


static int ecx337af_spi_write_then_read(struct lcd_device *lcdev,
				u8 *txbuf, u16 txlen,
				u8 *rxbuf, u16 rxlen)
{
	struct ecx337af_data *lcd = lcd_get_data(lcdev);
	struct spi_message msg;
	struct spi_transfer xfer[2];
	int ret = 0;

	memset(xfer, 0, sizeof(xfer));
	spi_message_init(&msg);

	if (txlen)
        {
                xfer[0].len = txlen;
                xfer[0].bits_per_word = 8;
		xfer[0].tx_buf = txbuf;
		spi_message_add_tail(&xfer[0], &msg);
	}

	if (rxlen)
        {
		xfer[1].len = rxlen;
		xfer[1].bits_per_word = 8;
		xfer[1].rx_buf = rxbuf;
		spi_message_add_tail(&xfer[1], &msg);
	}

	ret = spi_sync(lcd->spi, &msg);
	if (ret < 0)
		dev_err(&lcdev->dev, "Couldn't send SPI data\n");


	return ret;
}

static inline int ecx337af_spi_write_array(struct lcd_device *lcdev,
					u8 *value, u8 len)
{
	return ecx337af_spi_write_then_read(lcdev, value, len, NULL, 0);
}

static inline int ecx337af_spi_write_byte(struct lcd_device *lcdev,
					u8 value)
{
	return ecx337af_spi_write_then_read(lcdev, &value, 1, NULL, 0);
}
static inline int ecx337af_spi_read_register(struct lcd_device *lcdev, u8 reg, u8 *pvalue)
{
        u8 rd_addr = Register_0x81;
        u8 rd_enable = Register_0x80;
        u8 tx[2];
        u8 rx = 0;
        int ret = 0;

        /* Enable register read */
        tx[0] = reorder_bits(rd_enable);
        tx[1] = reorder_bits(0x1);
        ret = ecx337af_spi_write_array(lcdev,tx, sizeof(tx));
        if (ret == 0)
        {
            /* Select the register to be read */
            tx[0] = reorder_bits(rd_addr);
            tx[1] = reorder_bits(reg);
            rx = 0;
            ret = ecx337af_spi_write_then_read(lcdev, tx, sizeof(tx), NULL, 0);
            if (ret == 0)
            {
                // Read the data from the rd_addr register
                ret = ecx337af_spi_write_then_read(lcdev, &tx[0], sizeof(tx[0]), &rx, sizeof(rx));
            }
        }
        *pvalue = reorder_bits(rx);
        return ret;
}
static inline int ecx337af_spi_write_register(struct lcd_device *lcdev, u8 reg, u8 value)
{
	return ecx337af_spi_write_then_read(lcdev, &reg, sizeof(reg), &value, 1);
}

static int ecx337af_enter_standby(struct lcd_device *lcdev)
{
	int ret = 0;
	return ret;
}

static int ecx337af_exit_standby(struct lcd_device *lcdev)
{
	int ret = 0;
	return ret;
}

static void ecx337af_lcd_reset(struct lcd_device *lcdev)
{
	struct ecx337af_data *lcd = lcd_get_data(lcdev);
	/* Reset the device */
        dev_info(&lcdev->dev,"%s\n",__FUNCTION__);
        dev_info(&lcdev->dev,"pulling reset low\n");
	gpio_set_value(lcd->reset, 0);
	usleep_range(10000, 12000);
        dev_info(&lcdev->dev,"pulling reset high\n");
	gpio_set_value(lcd->reset, 1);
}

static int ecx337af_lcd_init(struct lcd_device *lcdev)
{
        u8 cmd[2] = {0,0};
        #ifdef TEST_REGISTER_READ
        u8 reg = 0;
        u8 regnum = 0;
        #endif
        dev_info(&lcdev->dev,"%s\n",__FUNCTION__);

        /* Power on sequence: Set power saving bit PS0 to 1, wait at least 200 microseconds
         * and set power saving bit PS1 to 1
         */

        /* Set power saving bit PS1 to 1 */
        cmd[0] = reorder_bits(0);
        cmd[1] = reorder_bits(0x4d);
        if (ecx337af_spi_write_array(lcdev,cmd,sizeof(cmd)) != 0)
        {
            dev_info(&lcdev->dev,"ecx337af_spi_write_array 0,0x4d failed\n");
        }
        msleep(1);
        cmd[0] = reorder_bits(0);
        cmd[1] = reorder_bits(0x4f);
        if (ecx337af_spi_write_array(lcdev,cmd,sizeof(cmd)) != 0)
        {
            dev_info(&lcdev->dev,"ecx337af_spi_write_array 0,0x4f failed\n");
        }
        #ifdef TEST_REGISTER_READ
        for (regnum = 0; regnum <= 0x14; regnum++)
        {
            if (ecx337af_spi_read_register(lcdev, regnum, &reg) == 0)
            {
                dev_info(&lcdev->dev,"reg %02x = %02x\n",regnum,reg);
            }
            msleep(5);
        }
        #endif



	return 0;
}


#define POWER_IS_ON(pwr)	((pwr) <= FB_BLANK_NORMAL)

static int ecx337af_set_power(struct lcd_device *lcdev, int power)
{
	struct ecx337af_data *lcd = lcd_get_data(lcdev);
	int ret = 0;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->state))
		ret = ecx337af_exit_standby(lcdev);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->state))
		ret = ecx337af_enter_standby(lcdev);

	if (ret == 0)
		lcd->state = power;
	else
		dev_warn(&lcdev->dev, "failed to set power mode %d\n", power);

	return ret;
}

static int ecx337af_get_power(struct lcd_device *lcdev)
{
	struct ecx337af_data *lcd = lcd_get_data(lcdev);

	return lcd->state;
}

static struct lcd_ops ecx337af_ops = {
	.set_power	= ecx337af_set_power,
	.get_power	= ecx337af_get_power,
};

static const struct of_device_id ecx337af_dt_ids[] = {
	{
		.compatible = "sony,ecx337af",
		.data = ecx337af_lcd_init,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ecx337af_dt_ids);

static int ecx337af_probe(struct spi_device *spi)
{
	struct lcd_device *lcdev;
	struct ecx337af_data *lcd;
	const struct of_device_id *match;
	int ret;

	lcd = devm_kzalloc(&spi->dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;
        spi->mode = SPI_MODE_0;
        spi->bits_per_word = 8;
        if (!spi->max_speed_hz)
        {
                spi->max_speed_hz = 500000;
        }

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "SPI setup failed.\n");
		return ret;
	}

	lcd->spi = spi;

	match = of_match_device(ecx337af_dt_ids, &spi->dev);
	if (!match || !match->data)
		return -EINVAL;

	lcd->reset = of_get_named_gpio(spi->dev.of_node, "gpios-reset", 0);
	if (!gpio_is_valid(lcd->reset)) {
		dev_err(&spi->dev, "Missing dt property: gpios-reset\n");
		return -EINVAL;
	}
	ret = devm_gpio_request_one(&spi->dev, lcd->reset,
				    GPIOF_OUT_INIT_HIGH,
				    "ecx337af-reset");
	if (ret) {
		dev_err(&spi->dev,
			"failed to request gpio %d: %d\n",
			lcd->reset, ret);
		return -EINVAL;
	}
	lcdev = devm_lcd_device_register(&spi->dev, "mxsfb", &spi->dev, lcd,
					&ecx337af_ops);
	if (IS_ERR(lcdev)) {
		ret = PTR_ERR(lcdev);
		return ret;
	}
	spi_set_drvdata(spi, lcdev);

	ecx337af_lcd_reset(lcdev);

	ret = ((int (*)(struct lcd_device *))match->data)(lcdev);
	if (ret) {
		dev_err(&spi->dev, "Couldn't initialize panel\n");
		return ret;
	}

	dev_info(&spi->dev, "Panel probed\n");

	return 0;
}

static struct spi_driver ecx337af_driver = {
	.probe  = ecx337af_probe,
	.driver = {
		.name = "ecx337af",
		.of_match_table = ecx337af_dt_ids,
	},
};

module_spi_driver(ecx337af_driver);

MODULE_AUTHOR("George Rouse <george.rouse@flir.com>");
MODULE_DESCRIPTION("Sony ECX337AF LCD Driver");
MODULE_LICENSE("GPL");
