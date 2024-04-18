#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/spi_master.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "wifiudp.h"
#include "ads1292r.h"

#define UART_PORT_NUM 1

extern spi_device_handle_t spi;

static int adc_raw;
static int voltage;
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);

void app_main(void)
{
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // wifi_init_sta();

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &config));
    adc_cali_handle_t adc_cali_chan0_handle = NULL;
    bool do_calibration_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_12, &adc_cali_chan0_handle);

    gpio_config_t low_power = {
        .pin_bit_mask = BIT64(GPIO_LOW_POWER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&low_power);
    gpio_set_level(GPIO_LOW_POWER, 1);

    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_MISO,
        .mosi_io_num = GPIO_MOSI,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 1,
        .spics_io_num = GPIO_CS,
        .queue_size = 10,
    };
    ret = spi_bus_initialize(ADS1292R_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(ADS1292R_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ads1292r_init(spi);

    while (1)
    {
        adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw);
        if (do_calibration_chan0)
        {
            adc_cali_raw_to_voltage(adc_cali_chan0_handle, adc_raw, &voltage);
        }
        if (voltage < 3710 / 2)
        {
            gpio_set_level(GPIO_LOW_POWER, !gpio_get_level(GPIO_LOW_POWER));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;
    if (!calibrated)
    {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
    *out_handle = handle;
    return calibrated;
}