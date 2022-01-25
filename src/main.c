#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <sys/printk.h>
#include <zephyr.h>


#include <shtcx.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(demo);

#define LED0_NODE DT_ALIAS(led0)
const struct device *led_dev;
#if DT_NODE_HAS_STATUS(LED0_NODE,okay)
    #define LED0    DT_GPIO_LABEL(LED0_NODE, gpios)
    #define PIN     DT_GPIO_PIN(LED0_NODE, gpios)
    #define FLAGS   DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
    //Can't blink
    #error "Unsupported board: led0 devicetree alias is not defined"
    #define LED0 ""
    #define PIN 0
    #define FLAGS 0
#endif

// #define SHTC3 DT_INST(0, sensirion_shtc3)
// #define CONFIG_SHTC3_DEV_NAME DT_LABEL(SHTC3)


// #define SHTC3 DT_NODELABEL(shtc3_0)
// const struct device *shtc3;

// void shtc3_init(void){
//     int config_result = false;
//     shtc3 = device_get_binding(DT_LABEL(SHTC3));
//     if (shtc3 == NULL) {
//         printk("\n\nI2C slave: Device driver not found.\n");
//     } else {
//         printk("\nI2C device 1: %s\n", DT_PROP(SHTC3, label));

//         // config_result = i2c_configure(shtc3, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER);

//     if (!config_result) {
//       printk("I2C Master: Slave ADDR: 0x%x SCL: %d, SDA: %d, CLK(Hz): %u\n\n",
//           DT_REG_ADDR(SHTC3), 
//           DT_PROP(SHTC3, scl_pin),
//           DT_PROP(SHTC3, sda_pin),
//           DT_PROP(SHTC3, clock_frequency));
//     } else
//       printk("\n\nI2C: Configuration error code: %d\n", config_result);
//   }
// }

#define MP_TWIM DT_NODELABEL(i2c0)
// #define MP_TWIM DT_NODELABEL(twis_device2)
// #define MP_TWIM DT_INST(1,nordic,nrf-twim)
const struct device *nrfx_twis_dev2;

static void twim_init(void) {
  int config_result = false;

  nrfx_twis_dev2 = device_get_binding(DT_LABEL(MP_TWIM));

  if (nrfx_twis_dev2 == NULL) {
    printk("\n\nI2C Slave: Device driver not found.\n");
  } else {
    printk("\nI2C device 2: %s\n", DT_PROP(MP_TWIM, label));

    config_result = i2c_configure(nrfx_twis_dev2, I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_MASTER);

    if (!config_result) {
      printk("I2C Master: Slave ADDR: 0x%x SCL: %d, SDA: %d, CLK(Hz): %u\n\n",
          DT_REG_ADDR(MP_TWIM), 
          DT_PROP(MP_TWIM, scl_pin),
          DT_PROP(MP_TWIM, sda_pin),
          DT_PROP(MP_TWIM, clock_frequency));
    } else
      printk("\n\nI2C: Configuration error code: %d\n", config_result);
  }
}

// struct shtc3_data {
// 	uint16_t temp;
// 	uint16_t humidity;
// } __packed __aligned(2);

// void shtc3_fetch(void)
// {
// 	sensor_sample_fetch(shtc3);
// 	return;
// }

// int32_t shtc3_get_temp(void){
// 	struct shtc3_data *data = shtc3->data;
// 	return data->temp;
// }


// uint32_t shtc3_get_humidity(void){
// 	struct shtc3_data *data = shtc3->data;
// 	return data->humidity;
// }

// static sensor_data_t sensor_data = {0};

// static void update_shtc3(void){
// 	shtc3_fetch();
// 	sensor_data.temperature = shtc3_get_temp();
// 	sensor_data.humidity    = shtc3_get_humidity();
// 	LOG_DBG("Temperature: %d, Humidity: %d",sensor_data.temperature, sensor_data.humidity);
// }

#define TWI_BUFFER_SIZE 14
static unsigned char i2c_tx_buffer[TWI_BUFFER_SIZE] = {'M', 'S','G',' ','F', 'R', 'O', 'M', ' ', 'T', 'W', 'I', 'M', '\0'};

////////////////////////////////////////////////////////////

#define ECHO_WRITES false
#define MSG_EXCHANGE_CNT 5
#define INFINITE_MSG_EXCHANGE false
#define TWI_LOOPBACK false

void main(void){
    printk("Startup\n");
    // shtc3_init();
    twim_init();
    // uint8_t bufferi2c[TWI_BUFFER_SIZE] = {0xA};
	// // bufferi2c = {0xA};
	// uint8_t bufferi2crx[TWI_BUFFER_SIZE];
	// bufferi2crx = {0x00,0x00};
	

    while(1){
        k_msleep(1000);
        if(!i2c_write(nrfx_twis_dev2,i2c_tx_buffer,sizeof(i2c_tx_buffer), DT_REG_ADDR(MP_TWIM)))
            printk("ERROR: Not writing!\n");
        // shtc3_fetch();
        // printk("%x",shtc3_get_humidity());

    }
}