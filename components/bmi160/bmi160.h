#ifndef BMI160_H
#define BMI160_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*********************************************************************/
/* Constants: Result and error codes */
/*********************************************************************/
#define BMI_L_SUCCESS 0 // function success
#define BMI_L_SELF_TEST_PASS 1 // self test pass
#define BMI_L_SELF_TEST_FAIL 2 // self test fail
#define BMI_E_NULL_PTR 3 // null pointer error
#define BMI_E_INVALID_LEN 4 // invalid length error
#define BMI_E_INVALID_IOCTL_CMD 5 // invalid ioctl command error
#define BMI_E_ERROR 6 // function error

#define CHIP_ID_DEFAULT 0xD1 // default chip id

/*********************************************************************/
/* IOCTL Commands */
/*********************************************************************/
#define BMI_IOCTL_SOFT_RESET          0
#define BMI_IOCTL_POWER_MODE_NORMAL   1
#define BMI_IOCTL_POWER_MODE_SUSPEND  2
#define BMI_IOCTL_SELF_TEST           3
#define BMI_IOCTL_SET_ACCEL_CONFIG    4
#define BMI_IOCTL_SET_GYRO_CONFIG     5

/*********************************************************************/
/* BMI160 Register Configuration Constants */
/*********************************************************************/
#define BMI160_ACCEL_RANGE_8G         0x08
#define BMI160_ACCEL_ODR_1600HZ       0x2C

/*********************************************************************/
/* Typedefs: Device I/O function pointer types */
/*********************************************************************/
struct bmi160_dev_t;

typedef uint8_t (*bmi160_read)(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev);
typedef uint8_t (*bmi160_write)(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160_dev_t *dev);
typedef void (*bmi160_delay_ms)(uint8_t ms);

/*********************************************************************/
/* Device Struct */
/*********************************************************************/
typedef struct bmi160_dev_t {
    bmi160_read read;
    bmi160_write write;
    bmi160_delay_ms delay_ms;

    uint8_t chip_id;

    uint8_t accel_config;
    uint8_t accel_range;
    uint8_t gyro_config;
    uint8_t gyro_range;
} bmi160_dev_t;

/*********************************************************************/
/* Sensor Output Data Structs */
/*********************************************************************/
struct bmi160_accel_data_t {
    uint8_t x_lsb;
    uint8_t y_lsb;
    uint8_t z_lsb;
    uint8_t x_msb;
    uint8_t y_msb;
    uint8_t z_msb;
};
struct bmi160_gyro_data_t {
    uint8_t x_lsb;
    uint8_t y_lsb;
    uint8_t z_lsb;
    uint8_t x_msb;
    uint8_t y_msb;
    uint8_t z_msb;
};

/*********************************************************************/
/* API Function Prototypes */
/*********************************************************************/

uint8_t bmi160_open(bmi160_dev_t *dev);
uint8_t bmi160_close(bmi160_dev_t *dev);

uint8_t bmi160_read_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, bmi160_dev_t *dev);
uint8_t bmi160_write_reg(uint8_t reg_addr, uint8_t *data, uint16_t len, bmi160_dev_t *dev);

uint8_t bmi160_read_accel(struct bmi160_accel_data_t *accel_data, bmi160_dev_t *dev);
uint8_t bmi160_read_gyro(struct bmi160_gyro_data_t *gyro_data, bmi160_dev_t *dev);

uint8_t bmi160_ioctl(bmi160_dev_t *dev, uint8_t cmd);

#ifdef __cplusplus
}
#endif

#endif /* BMI160_H */
