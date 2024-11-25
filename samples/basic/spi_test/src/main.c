#include "zephyr/kernel.h"
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

const struct device *SpiDevice = DEVICE_DT_GET(DT_ALIAS(mt_display_spi)); 
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct spi_config spi_cfg = {
    .frequency = 2000000U,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave = 0,
    .cs = NULL,
};

static void spi_callback(const struct device *dev, int status, void *user_data)
{
    gpio_pin_toggle_dt(&led);
    printk("SPI transfer completed with status: %d\n", status);
}

int main(void)
{
    const struct device *spi = SpiDevice;
    int ret, ret_led;
    uint8_t testdata[255];
    for (ret = 0; ret < 255; ret++) {
        testdata[ret] = ret;
    }

    printk("spi_demo\n");
    if (!device_is_ready(SpiDevice)) {
        printk("SPI device %s is not ready\n", SpiDevice->name);
        return -1;
    }
    if (!gpio_is_ready_dt(&led)) {
        return 0;
    }

    ret_led = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret_led < 0) {
        return 0;
    }

    while (1) {
        struct spi_buf tx_buf = {
            .buf = testdata,
            .len = sizeof(testdata),
        };

        struct spi_buf_set tx_bufs = {
            .buffers = &tx_buf,
            .count = 1,
        };

        ret_led = gpio_pin_toggle_dt(&led);
        if (ret_led < 0) {
            return 0;
        }

        ret = spi_transceive_cb(spi, &spi_cfg, &tx_bufs, NULL, spi_callback, NULL);
        if (ret) {
            printk("SPI write failed: %d\n", ret);
            return 0;
        }
        gpio_pin_toggle_dt(&led); // Toggle LED after SPI transfer completes
        printk("SPI write started\n");


        

        printk("SPI transfer completed\n");

        k_msleep(1000);
    }
}