/**
 * @file bmi160.c
 * @author Edward Baptiste (edward.baptiste.work@gmail.com)
 * @brief This file is a simplified driver for the BMI160 sensor from Bosch. 
 * It is designed to be platform agnostic. This driver is not meant to replace the original Bosch driver,
 * rather it is a simplified version written to facilitate learning about driver development.
 * @version 1.0
 * @date 2025-06-23
 * 
 */


 #include "bmi160.h"
 #include <stdint.h>
 #include <stdio.h>
 #include <stdlib.h> // For abs()
 
 /*********************************************************************/
 /* Internal Constants (Registers, Values)                           */
 /*********************************************************************/
 
 #define CHIP_ID_REG           0x00
 #define CMD_REG               0x7E
 #define ACC_CONF_REG          0x40
 #define ACC_RANGE_REG         0x41
 #define GYRO_CONF_REG         0x42
 #define GYRO_RANGE_REG        0x43
 #define ACC_X_LSB             0x12
 #define ACC_X_MSB             0x13
 #define ACC_Y_LSB             0x14
 #define ACC_Y_MSB             0x15
 #define ACC_Z_LSB             0x16
 #define ACC_Z_MSB             0x17
 #define GYRO_X_LSB            0x0C
 #define GYRO_X_MSB            0x0D
 #define GYRO_Y_LSB            0x0E
 #define GYRO_Y_MSB            0x0F
 #define GYRO_Z_LSB            0x10
 #define GYRO_Z_MSB            0x11
 
 #define CMD_ACCEL_POWER_NORMAL   0x11
 #define CMD_ACCEL_POWER_SUSPEND  0x10
 #define CMD_GYRO_POWER_NORMAL    0x15
 #define CMD_GYRO_POWER_SUSPEND   0x14
 #define CMD_SOFT_RESET           0xB6
 #define ACC_RANGE_8G             0x08
 #define ACC_CONF_ODR_1600HZ      0x2C
 #define SELF_TEST_ACC_POS        0x0D
 #define SELF_TEST_ACC_NEG        0x09

/*********************************************************************/
/* bmi160 registers */
/*********************************************************************/

#define CHIP_ID_REG 0x00 // chip id register
#define CMD_REG 0x7E // command register
#define ACC_CONF_REG 0x40 // accelerometer configuration register
#define ACC_RANGE_REG 0x41 // allows selection of accel g range
#define ACC_X_LSB 0x12 // accel sensor data x lsb
#define ACC_X_MSB 0x13 // accel sensor data x msb
#define ACC_Y_LSB 0x14 // accel sensor data y lsb
#define ACC_Y_MSB 0x15 // accel sensor data y msb
#define ACC_Z_LSB 0x16 // accel sensor data z lsb
#define ACC_Z_MSB 0x17 // accel sensor data z msb  
#define GYRO_CONF_REG 0x42 // gyro configuration register
#define GYRO_RANGE_REG 0x43 // allows selection of gyro g range
#define GYRO_X_LSB 0x0C // gyro sensor data x lsb
#define GYRO_X_MSB 0x0D // gyro sensor data x msb
#define GYRO_Y_LSB 0x0E // gyro sensor data y lsb
#define GYRO_Y_MSB 0x0F // gyro sensor data y msb
#define GYRO_Z_LSB 0x10 // gyro sensor data z lsb
#define GYRO_Z_MSB 0x11 // gyro sensor data z msb

/*********************************************************************/
/* bmi160 values */
/*********************************************************************/

#define CMD_ACCEL_POWER_NORMAL 0x11 // normal power mode
#define CMD_ACCEL_POWER_SUSPEND 0x10 // suspend power mode
#define CMD_GYRO_POWER_NORMAL 0x15 // normal power mode
#define CMD_GYRO_POWER_SUSPEND 0x14 // suspend power mode
#define CMD_SOFT_RESET 0xB6 // soft reset command
#define CMD_SELF_TEST 0x6D // self test command
#define ACC_RANGE_8G 0x08 // +- 8g range
#define ACC_CONF_ODR_1600HZ 0x2C // 1600Hz output data rate
#define SELF_TEST_ACC_POS 0x0D // accel self test positive value 
#define SELF_TEST_ACC_NEG 0x09 // accel self test negative value

/*********************************************************************/
/* ioctl commands */
/*********************************************************************/

#define BMI_IOCTL_SOFT_RESET 0 // soft reset command
#define BMI_IOCTL_POWER_MODE_NORMAL 1 // set power mode to normal
#define BMI_IOCTL_POWER_MODE_SUSPEND 2 // set power mode to suspend
#define BMI_IOCTL_SELF_TEST 3 // self test command
#define BMI_IOCTL_SET_ACCEL_CONFIG 4 // set accel config command
#define BMI_IOCTL_SET_GYRO_CONFIG 5 // set gyro config command

/*********************************************************************/
/* static function declarations */
/*********************************************************************/


/*********************************************************************/
/* user function definitions */
/*********************************************************************/


uint8_t bmi160_open(struct bmi160_dev_t *dev){
    uint8_t result;
    if(dev == NULL){
        return BMI_E_NULL_PTR;
    }
    dev->chip_id = 0;

    result = bmi160_read_reg(CHIP_ID_REG, &(dev->chip_id), 1, dev);
    if (result != BMI_L_SUCCESS && dev->chip_id != CHIP_ID_DEFAULT) {
        return result;
    }

    result = bmi160_ioctl(dev, BMI_IOCTL_SOFT_RESET);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    result = bmi160_ioctl(dev, BMI_IOCTL_POWER_MODE_NORMAL); 

    return result;
};


uint8_t bmi160_close(struct bmi160_dev_t *dev){
    uint8_t result;
    if(dev == NULL){
        return BMI_E_NULL_PTR;
    }

    result = bmi160_ioctl(dev, BMI_IOCTL_SOFT_RESET);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    result = bmi160_ioctl(dev, BMI_IOCTL_POWER_MODE_SUSPEND); 

    return result;

};


uint8_t bmi160_read_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev){
    uint8_t result;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
    if (len <= 0){
        return BMI_E_INVALID_LEN;
    }

    result = dev->read(reg_addr, data, len, dev);

    return result;
};


uint8_t bmi160_write_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev){
    uint8_t result;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
    if (len <= 0){
        return BMI_E_INVALID_LEN;
    }

    result = dev->write(reg_addr, data, len, dev);

    return result;
};


uint8_t bmi160_read_accel(struct bmi160_accel_data_t *accel_data, struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;

    if (dev == NULL || accel_data == NULL){
        return BMI_E_NULL_PTR;
    }

    result = bmi160_read_reg(ACC_X_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->x_msb = data;

    result = bmi160_read_reg(ACC_X_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->x_lsb = data;

    result = bmi160_read_reg(ACC_Y_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->y_msb = data;

    result = bmi160_read_reg(ACC_Y_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->y_lsb = data;

    result = bmi160_read_reg(ACC_Z_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->z_msb = data;

    result = bmi160_read_reg(ACC_Z_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    accel_data->z_lsb = data;

    return BMI_L_SUCCESS;
};


uint8_t bmi160_read_gyro(struct bmi160_gyro_data_t *gyro_data, struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;

    if (dev == NULL || gyro_data == NULL){
        return BMI_E_NULL_PTR;
    }

    result = bmi160_read_reg(GYRO_X_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->x_msb = (int8_t)data;

    result = bmi160_read_reg(GYRO_X_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->x_lsb = (int8_t)data;

    result = bmi160_read_reg(GYRO_Y_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->y_msb = (int8_t)data;

    result = bmi160_read_reg(GYRO_Y_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->y_lsb = (int8_t)data;

    result = bmi160_read_reg(GYRO_Z_MSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->z_msb = (int8_t)data;

    result = bmi160_read_reg(GYRO_Z_LSB, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    gyro_data->z_lsb = (int8_t)data;

    return BMI_L_SUCCESS;
};



/*********************************************************************/
/* static function definitions */
/*********************************************************************/


static uint8_t bmi160_set_power_normal(struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
 
    data = CMD_ACCEL_POWER_NORMAL;
    result = bmi160_write_reg(CMD_REG, &data, 0x01, dev); 
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(5);

    data = CMD_GYRO_POWER_NORMAL;
    result = bmi160_write_reg(CMD_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(80);

    return result;

};


static uint8_t bmi160_set_power_suspend(struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
 
    data = CMD_ACCEL_POWER_SUSPEND;
    result = bmi160_write_reg(CMD_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(5);

    data = CMD_GYRO_POWER_SUSPEND;
    result = bmi160_write_reg(CMD_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(80);

    return result;

};


static uint8_t bmi160_set_accel_config(struct bmi160_dev_t *dev){
    uint8_t result;
    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
    result = bmi160_write_reg(ACC_CONF_REG, &(dev->accel_config), 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    result = bmi160_write_reg(ACC_RANGE_REG, &(dev->accel_range), 0x01, dev);

    return result;
}; 


static uint8_t bmi160_set_gyro_config(struct bmi160_dev_t *dev){
    uint8_t result;
    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
    result = bmi160_write_reg(GYRO_CONF_REG, &(dev->gyro_config), 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
    result = bmi160_write_reg(GYRO_RANGE_REG, &(dev->gyro_range), 0x01, dev);

    return result;
}; 


static uint8_t bmi160_soft_reset(struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }
 
    data = CMD_SOFT_RESET;
    result = bmi160_write_reg(CMD_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(50);

    return result;
};


static uint8_t bmi160_self_test(struct bmi160_dev_t *dev){
    uint8_t result;
    uint8_t data;
    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }

    result = bmi160_soft_reset(dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    result = bmi160_set_power_normal(dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    data = ACC_RANGE_8G;
    result = bmi160_write_reg(ACC_RANGE_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    data = ACC_CONF_ODR_1600HZ;
    result = bmi160_write_reg(ACC_CONF_REG, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }
 
    data = SELF_TEST_ACC_POS;
    result = bmi160_write_reg(CMD_SELF_TEST, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(50);

    struct bmi160_accel_data_t accel_data_pos;
    result = bmi160_read_accel(&accel_data_pos, dev);

    data = SELF_TEST_ACC_NEG;
    result = bmi160_write_reg(CMD_SELF_TEST, &data, 0x01, dev);
    if (result != BMI_L_SUCCESS) {
        return result;
    }

    dev->delay_ms(50);

    struct bmi160_accel_data_t accel_data_neg;
    result = bmi160_read_accel(&accel_data_neg, dev);

    int16_t x_pos = ((int16_t)accel_data_pos.x_msb << 8) | accel_data_pos.x_lsb;
    int16_t y_pos = ((int16_t)accel_data_pos.y_msb << 8) | accel_data_pos.y_lsb;
    int16_t z_pos = ((int16_t)accel_data_pos.z_msb << 8) | accel_data_pos.z_lsb;
    int16_t x_neg = ((int16_t)accel_data_neg.x_msb << 8) | accel_data_neg.x_lsb;
    int16_t y_neg = ((int16_t)accel_data_neg.y_msb << 8) | accel_data_neg.y_lsb;
    int16_t z_neg = ((int16_t)accel_data_neg.z_msb << 8) | accel_data_neg.z_lsb;


    if (abs(x_pos - x_neg) >= 8192 && abs(y_pos - y_neg) >= 8192 && abs(z_pos - z_neg) >= 8192) {
        result = BMI_L_SELF_TEST_PASS;
    } else {
        result = BMI_L_SELF_TEST_FAIL;
    }

    bmi160_soft_reset(dev);

    return result;
};


uint8_t bmi160_ioctl(struct bmi160_dev_t *dev, uint8_t cmd){
    uint8_t result;

    if (dev == NULL){
        return BMI_E_NULL_PTR;
    }

    switch(cmd){
        case BMI_IOCTL_SOFT_RESET:
            result = bmi160_soft_reset(dev);
            break;
        
        case BMI_IOCTL_POWER_MODE_NORMAL:
            result = bmi160_set_power_normal(dev); 
            break;

        case BMI_IOCTL_POWER_MODE_SUSPEND:
            result = bmi160_set_power_suspend(dev); 
            break;

        case BMI_IOCTL_SELF_TEST:
            result = bmi160_self_test(dev);
            break;

        case BMI_IOCTL_SET_ACCEL_CONFIG:
            result = bmi160_set_accel_config(dev); 
            break;

        case BMI_IOCTL_SET_GYRO_CONFIG:
            result = bmi160_set_gyro_config(dev); 
            break;

        default:
            result = BMI_E_INVALID_IOCTL_CMD;
    }

    return result;
};