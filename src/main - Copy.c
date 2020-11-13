#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <irq.h>
#include <nrfx_twim.h>
#include "bno/bno055.h"
#include <stdio.h>
#include <time.h>

#define PIN_SCL DT_PROP(DT_NODELABEL(i2c1), scl_pin)
#define PIN_SDA DT_PROP(DT_NODELABEL(i2c1), sda_pin)
#define I2C_BUFFER_LEN 8
#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)


// //BNO055 Power og Operation Mode Registers
// uint8_t power_mode_normal = BNO055_POWER_MODE_NORMAL;
// uint8_t pow_mode_reg = BNO055_POWER_MODE_REG;

// //BNO055 OPERATION_MODE registers
// uint8_t operation_mode_reg = BNO055_OPERATION_MODE_REG;
// uint8_t operation_mode_config = BNO055_OPERATION_MODE_CONFIG;
// uint8_t operation_mode_acc_only = BNO055_OPERATION_MODE_ACCONLY;

// //BNO055 ACC RAW DATA registers
// uint8_t accel_datax_lsb_reg_addr = BNO055_ACCEL_DATA_X_LSB_ADDR;
// uint8_t accel_datax_msb_reg_addr = BNO055_ACCEL_DATA_X_MSB_ADDR;
// uint8_t accel_datax_lsb_reg_addr = BNO055_ACCEL_DATA_X_LSB_ADDR;
// uint8_t accel_datax_lsb_reg_addr = BNO055_ACCEL_DATA_X_LSB_ADDR;

static const nrfx_twim_t m_twim = NRFX_TWIM_INSTANCE(1);
static nrfx_twim_config_t m_twim_config;
struct bno055_t sensor;

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void delay_ms(u32 milis);

s8 I2C_routine(void)
{
    sensor.bus_write = bno055_write;
    sensor.bus_read = bno055_read;
    sensor.dev_addr = BNO055_I2C_ADDR1;
    sensor.delay_msec = delay_ms;
    return BNO055_INIT_VALUE;
}

s8 bno055_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{   
    
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[I2C_BUFFER_LEN];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
        array[stringpos + BNO055_I2C_BUS_WRITE_ARRAY_INDEX] = *(reg_data + stringpos);
    }

    nrfx_twim_xfer_desc_t descr = NRFX_TWIM_XFER_DESC_TXTX(dev_addr, &reg_addr, 1, reg_data, cnt);
    int twim_xfer_ret = nrfx_twim_xfer(&m_twim, &descr, 0);
    printk("XFER WRITE return: %02X \n", twim_xfer_ret);
    return (s8)BNO055_iERROR;
}

void delay_ms(u32 msek){
    k_msleep(msek);

}

s8 bno055_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{   
    nrfx_twim_xfer_desc_t descr = NRFX_TWIM_XFER_DESC_TXRX(dev_addr, &reg_addr, 1, reg_data, cnt);
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 stringpos = BNO055_INIT_VALUE;
    int twim_xfer_ret = nrfx_twim_xfer(&m_twim, &descr, 0);
    printk("XFER READ return: %02X \n", twim_xfer_ret);
    return (s8)BNO055_iERROR;
}



static void twim_event_handler(nrfx_twim_evt_t const *const p_event)
{
    switch (p_event->type)
    {
    case NRFX_TWIM_EVT_DONE:
        // printk("NRFX_TWIM_EVT_DONE \n");
        break;
    case NRFX_TWIM_EVT_ADDRESS_NACK:
        printk("NRFX_TWIM_EVT_ADDRESS_NACK \n");
        break;
    case NRFX_TWIM_EVT_OVERRUN:
        printk("NRFX_TWIM_EVT_OVERRUN \n");
        break;
    case NRFX_TWIM_EVT_DATA_NACK:
        printk("NRFX_TWIM_EVT_DATA_NACK \n");
        break;
    case NRFX_TWIM_EVT_BUS_ERROR:
        printk("NRFX_TWIM_EVT_BUS_ERROR \n");
    default:
        break;
    }
}

/* Application main Thread */
int main(void)
{   
    //Equal to NRFX_TWIM_DEFAULT_CONFIG with exception of SDA and SCL pins
    m_twim_config.scl = PIN_SCL;
    m_twim_config.sda = PIN_SDA;
    m_twim_config.frequency = NRF_TWIM_FREQ_400K;
    m_twim_config.interrupt_priority = NRFX_TWIM_DEFAULT_CONFIG_IRQ_PRIORITY;
    m_twim_config.hold_bus_uninit = false;

    //Initialization of the TWIM MASTER
    int ret = nrfx_twim_init(&m_twim, &m_twim_config, twim_event_handler, NULL);
    // printk("Return from nrfx_twim_init(): %02X \n", ret);

    //JEG IKKE VITE HVA SKJER HER
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(i2c1)),
                DT_IRQ(DT_NODELABEL(i2c1), priority),
                nrfx_isr, nrfx_twim_1_irq_handler, 0);

    //Enables TWIM_MASTER after initial configuration
    nrfx_twim_enable(&m_twim);

    
    //Initialize the bno055 struct
    int bno055_ret =I2C_routine();
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


    //Set up NORMAL power mode for the sensor
    u8 power_mode = BNO055_POWER_MODE_NORMAL;
    s8 power_mode_setup_ret = bno055_set_power_mode(power_mode);
    printk("POW MODE RET %02X \n", power_mode_setup_ret);

    //Set up OPERATION MODE to ACCONLY
    u8 operation_mode = BNO055_OPERATION_MODE_AMG;
    s8 operation_mode_ret = bno055_set_operation_mode(operation_mode);
    printk("OP MODE RET %02X \n", operation_mode_ret);

    s16 accelx_data;
    s16 accely_data;
    s16 accelz_data;

    s16 gyrox_data;
    s16 gyroy_data;
    s16 gyroz_data;

    u8 op_mode;
    for(int i =0; i < 3; i++) {
        s8 operation_mode_ret = bno055_set_operation_mode(operation_mode);
    }
    
    s8 get_op_mode_ret = bno055_get_operation_mode(&op_mode);
    printk("Current Operation Mode %02X \n" , op_mode);
    while (1)
    {
        // printk("Writing 0x60, 0x70, 0x80, 0x90 to slave at address 0x20 \n");
        int accelx_read_ret = bno055_read_accel_x(&accelx_data);
        int accely_read_ret = bno055_read_accel_y(&accely_data);
        int accelz_read_ret = bno055_read_accel_z(&accelz_data);
        // printk("Return from communication with accelerometer x-axis: %02X \n", accel_read_ret);
        printk("The acceleration x data is: %02X \n", accelx_data);
        printk("The acceleration y data is: %02X \n", accely_data);
        printk("The acceleration z data is: %02X \n", accelz_data);
        k_msleep(500);
    }
    return 0;
}