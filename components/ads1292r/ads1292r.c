#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "unistd.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>

#include "ads1292r.h"

spi_device_handle_t spi;
static const char *TAG = "ADS1292R";

uint8_t ads1292r_sample_data[9];

static QueueHandle_t gpio_evt_queue = NULL;
#define ESP_INTR_FLAG_DEFAULT 0
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
static void intr_task(void *arg)
{
    uint32_t io_num;
    // char rx_buffer[128];
    // char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    while (1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0)
        {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        while (1)
        {
            if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
            {
                ads1292r_data(spi, ads1292r_sample_data);
                int err = sendto(sock, ads1292r_sample_data, 9, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                printf("data is %x %x %x %x %x %x %x %x %x\n",
                       ads1292r_sample_data[0], ads1292r_sample_data[1],
                       ads1292r_sample_data[2], ads1292r_sample_data[3],
                       ads1292r_sample_data[4], ads1292r_sample_data[5],
                       ads1292r_sample_data[6], ads1292r_sample_data[7],
                       ads1292r_sample_data[8]);
                // vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
    }
}

void ads1292r_cmd(spi_device_handle_t spi_handle, const uint8_t cmd, bool keep_cs_active)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    if (keep_cs_active)
    {
        t.flags = SPI_TRANS_CS_KEEP_ACTIVE;
    }
    ret = spi_device_polling_transmit(spi_handle, &t);
    assert(ret == ESP_OK);
}

void ads1292r_send_data(spi_device_handle_t spi_handle, uint8_t data)
{
    esp_err_t ret;
    spi_transaction_t t;

    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    ret = spi_device_polling_transmit(spi_handle, &t);
    assert(ret == ESP_OK);
}

uint8_t ads1292r_read_reg(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t num)
{
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    ads1292r_cmd(spi_handle, RREG | reg_addr, true);
    ads1292r_cmd(spi_handle, num - 1, true);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.flags = SPI_TRANS_USE_RXDATA;
    spi_device_polling_transmit(spi_handle, &t);
    spi_device_release_bus(spi_handle);
    return t.rx_data[0];
}
void ads1292r_write_reg(spi_device_handle_t spi_handle, uint8_t reg_addr, uint8_t data)
{
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    ads1292r_cmd(spi_handle, WREG | reg_addr, true);
    ads1292r_cmd(spi_handle, 0x00, true);
    ads1292r_send_data(spi_handle, data);
    spi_device_release_bus(spi_handle);
}

void ads1292r_init(spi_device_handle_t spi_handle)
{
    gpio_config_t output_cfg = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
    };
    gpio_config(&output_cfg);
    gpio_config_t drdy_cfg = {
        .pin_bit_mask = 1ULL << GPIO_DRDY,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&drdy_cfg);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(intr_task, "drdy_low", 2048, NULL, 5, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_DRDY, gpio_isr_handler, (void *)GPIO_DRDY);

    gpio_set_level(GPIO_START, 0);
    gpio_set_level(GPIO_CS, 1);
    gpio_set_level(GPIO_RESET, 0);
    sleep(1);
    gpio_set_level(GPIO_RESET, 1);
    sleep(1);
    gpio_set_level(GPIO_RESET, 0);
    usleep(10);
    gpio_set_level(GPIO_RESET, 1);
    sleep(1);

    gpio_set_level(GPIO_START, 0);
    gpio_set_level(GPIO_CS, 0);
    ads1292r_cmd(spi_handle, SDATAC, false);
    ads1292r_write_reg(spi_handle, ADS1292_REG_CONFIG2, 0xE0);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_CONFIG1, 0x01);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_LOFF, 0xF0);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_CH1SET, 0x60);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_CH2SET, 0x00);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_RLDSENS, 0xEF);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_LOFFSENS, 0x0F);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_LOFFSTAT, 0x00);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_RESP1, 0xEA);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_RESP2, 0x03);
    usleep(10);
    ads1292r_write_reg(spi_handle, ADS1292_REG_GPIO, 0x0C);
    usleep(10);
    uint8_t reg_val;
    for (int i = 0; i < 12; i++)
    {
        reg_val = ads1292r_read_reg(spi_handle, i, 1);
        printf("reg %d value is %x\n", i, reg_val);
    }
    ads1292r_cmd(spi_handle, RDATAC, false);
    gpio_set_level(GPIO_START, 1);
}
void ads1292r_data(spi_device_handle_t spi_handle, uint8_t *data)
{
    spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 9;
    t.rx_buffer = data;
    spi_device_polling_transmit(spi_handle, &t);
    spi_device_release_bus(spi_handle);
}

void cs_low()
{
    gpio_set_level(GPIO_CS, 0);
    usleep(2);
}
void cs_high()
{
    gpio_set_level(GPIO_CS, 1);
    usleep(2);
}
