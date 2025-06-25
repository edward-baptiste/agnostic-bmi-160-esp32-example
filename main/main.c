/**
 * @file main.c
 * @author Edward Baptiste (edward.baptiste.work@gmail.com)
 * @brief This file is an example of how to use the simplified driver for the BMI160 sensor from Bosch. 
 * It uses the ESP IDF framework to demonstrate how this driver could be used with an ESP32
 * @version 1.0
 * @date 2025-06-23
 * 
 */

#include <stdio.h>
#include "bmi160.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

// Set Pinout Configuration
#define SCL_IO_PIN 32 
#define SDA_IO_PIN 33 
#define PORT_NUMBER I2C_NUM_0
#define BMI160_VDD_PIN 0

i2c_master_dev_handle_t dev_handle;

i2c_master_dev_handle_t i2c_bus_init() {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PORT_NUMBER,
        .scl_io_num = SCL_IO_PIN, 
        .sda_io_num = SDA_IO_PIN, 
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    return dev_handle;
}


uint8_t write_byte_to_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev) {
    uint8_t res;

    uint8_t write_buff[] = {reg_addr, *data}; 
    esp_err_t ret = i2c_master_transmit(
            dev_handle,
            write_buff,
            sizeof(write_buff),
            -1
    );
    if (ret != ESP_OK) {
        res = BMI_E_ERROR;
        printf("send error \n");
    } else {
        res = BMI_L_SUCCESS;
        printf("send success\n");
    }

    return res;
}

uint8_t read_byte_from_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev) {
    uint8_t res;
    *data = 0x01;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, sizeof(data), -1);

    if (ret != ESP_OK) {
        res = BMI_E_ERROR;
        printf("read error \n");
    } else {
        res = BMI_L_SUCCESS;
        printf("read success\n");
    }
    return res;

}

void delay(uint8_t ms) {
    vTaskDelay(ms / portTICK_PERIOD_MS);
}


int app_main(void) {
    dev_handle = i2c_bus_init();
    
    struct bmi160_dev_t dev = {
        .read = read_byte_from_reg,
        .write = write_byte_to_reg,
        .delay_ms = delay
    };

    uint8_t res = bmi160_open(&dev);
    printf("open: %d \n", res);
    res = bmi160_ioctl(&dev, BMI_IOCTL_SELF_TEST);
    printf("self test: %d \n", res);
    printf("Completed!\n");
    return 0;
}
