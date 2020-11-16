#include "bno/bno055_support.h"

s8 I2C_routine(struct bno055_t *device)
{
    device.bus_write = bno055_write;
    device.bus_read = bno055_read;
    device.dev_addr = BNO055_I2C_ADDR1;
    device.delay_msec = delay_ms;
    return BNO055_INIT_VALUE;
}

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{   
    
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    } 
    int errnum = i2c_write(p_master, &array, cnt + BNO055_I2C_BUS_WRITE_ARRAY_INDEX, dev_addr);
    return errnum;
}

void delay_ms(u32 msek){
    k_msleep(msek);

}


s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{   
    u8 stringpos = BNO055_INIT_VALUE;
    int errnum = i2c_write_read(p_master, dev_addr, &reg_addr, 1, reg_data, cnt);
    return errnum;
}

s8 initialize_sensor(struct bno055_t *sensor){

    //Initialize the bno055 struct
    int bno055_ret =I2C_routine(sensor);
    // printk("Return value from I2C_routine(): %02X \n",bno055_ret);

    //Initialize the BNO055 sensor using bno055_init();
    int bno055_init_ret = bno055_init(&sensor);
    printk("Sensor Init values \n");
    printk("%02X \n",sensor.chip_id);
    printk("%02X \n",sensor.accel_rev_id);
    printk("%02X \n",sensor.mag_rev_id);
    printk("%02X \n",sensor.gyro_rev_id);
    printk("%02X \n",sensor.sw_rev_id);
    printk("%02X \n",sensor.page_id);
    // printk("Return value from bno055_init(): %02X \n", bno055_init_ret);
    return 0;
}

s8 set_normal_power_mode(void){
    //Set up NORMAL power mode for the sensor
    u8 power_mode = BNO055_POWER_MODE_NORMAL;
    s8 power_mode_setup_ret = bno055_set_power_mode(power_mode);
    printk("POW MODE RET %02X \n", power_mode_setup_ret);

    //Set up OPERATION MODE to ACCONLY
    u8 operation_mode = BNO055_OPERATION_MODE_ACCGYRO;
    s8 operation_mode_ret = bno055_set_operation_mode(operation_mode);
    printk("OP MODE RET %02X \n", operation_mode_ret);
    return 0;
}

void get_acc_and_gyro_measurements(s16 *acc_x, s16 *acc_y, s16 *acc_z, s16 *gyro_x, s16 *gyro_y, s16 *gyro_z)
{
    printk("GYRO MEASUREMENTS \n");
        for(int j=0; j<=5; j++){
            for(int i= 0; i <=50; i++){
                bno055_read_gyro_x(&gyro_x);
                bno055_read_gyro_y(&gyro_y);
                bno055_read_gyro_z(&gyro_z);
                printk("%02X, ", gyro_x);
                printk("%02X, ",gyro_y);
                printk("%02X \n", gyro_z);
                k_msleep(100);
            }
            printk("Change Orientation \n");
            k_msleep(5000);
        }

        k_msleep(200);
        printk("ACCEL MEASUREMENTS \n");
        for(int j=0; j<=5; j++){
            for(int i= 0; i <=50; i++){
                bno055_read_accel_x(&accel_x);
                bno055_read_accel_y(&accel_y);
                bno055_read_accel_z(&accel_z);

                printk("%02X, ", &accel_x);
                printk("%02X, ", &accel_y);
                printk("%02X \n", &accel_z);
                k_msleep(100);
            }
            printk("Change Movement\n");
            k_msleep(5000);
        }
        k_msleep(100);
}