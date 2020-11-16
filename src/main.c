#include "bno/bno055_support.h"

struct bno055_t sensor;
const struct device *p_master ;

s16 *accelx_data, *accely_data, *accelz_data;
s16 *gyrox_data, *gyroy_data, *gyroz_data;
int main(void)
{   

    p_master = device_get_binding("I2C_1");

    if (p_master == NULL) {
		printk("No device \"%s\" found; did initialization fail?\n", "I2C_1");
	} else {
		printk("Found device \"%s\"\n", "I2C_1");
	}
    uint32_t config = I2C_SPEED_SET(I2C_SPEED_FAST);
    int master = i2c_configure(p_master, config);

    if (master)
    {
        return master;
    }
    
    I2C_routine(p_master);
    u8 op_mode;
    s8 get_op_mode_ret = bno055_get_operation_mode(&op_mode);
    printk("Current Operation Mode %02X \n" , op_mode);
    get_acc_and_gyro_measurements(&gyrox_data, &gyroy_data, &gyroz_data, 
                                  &accelx_data, &accely_data, &accelz_data);

    return 0;
}
