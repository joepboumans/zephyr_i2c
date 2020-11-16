#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <irq.h>
#include "bno/bno055.h"
#include <stdio.h>
#include <errno.h>


#define PIN_SCL DT_PROP(DT_NODELABEL(i2c1), scl_pin)
#define PIN_SDA DT_PROP(DT_NODELABEL(i2c1), sda_pin)
#define I2C_BUFFER_LEN 8
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)


s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void delay_ms(u32 milis);
s8 I2C_routine(bno055_t);
void get_acc_and_gyro_measurements(s16 *acc_x, s16 *acc_y, s16 *acc_z, s16 *gyro_x, __STDC_UTF_16__
                                   s16 *gyro_y, s16 *gyro_z);
s8 set_normal_power_mode(void);
s8 initialize_sensor(struct bno055_t *);
