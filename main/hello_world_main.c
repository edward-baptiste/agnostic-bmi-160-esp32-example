/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// #include <stdio.h>
// #include <bmi160.h>


// void app_main(void)
// {
//     printf("Hello world!\n");

// }



#include <stdio.h>
#include "bmi160.h"
#include "driver/i2c_master.h"

void write_byte_to_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t write_data) {
    // https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
    uint8_t write_buf[2] = {reg_addr, write_data};
    printf("writing: ");
    print_data(&write_data, sizeof(write_data));

    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), -1);
    if (ret != ESP_OK) {
        printf("send error \n");
    } else {
        printf("send success\n");
    }
}

void read_byte_from_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *read_data) {
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, read_data, sizeof(read_data), -1);
    if (ret != ESP_OK) {
        printf("read error \n");
    } else {
        printf("read success. recieved: %.2X \n", *read_data);
        // print_data(&read_data, sizeof(read_data));
    }

}



i2c_master_dev_handle_t i2c_bus_init() {
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT, // could also be I2C_CLK_SRC_APB. has an effect on power consumption
        .i2c_port = PORT_NUMBER, // -1 for auto selecting
        .scl_io_num = SCL_IO_PIN, // any gpio?
        .sda_io_num = SDA_IO_PIN, // any gpio
        .glitch_ignore_cnt = 7, // If the glitch period on the line is less than this value, it can be filtered out, typically value is 7 
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


int app_main(void) {
    // Example init (with dummy function pointers)
    i2c_master_dev_handle_t dev_handle = i2c_bus_init();

    struct bmi160_dev_t dev = {
        .read = NULL,
        .write = NULL,
        .delay_ms = NULL
    };

    uint8_t res = my_bmi160_open(&dev);
    printf("%d", res);
    printf("Completed!\n");
    return 0;
}
