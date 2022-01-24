/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/printk.h>
#include <zephyr.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

const struct device *led_dev;

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0 ""
#define PIN 0
#define FLAGS 0
#endif

////////////////////////////////////////////////////////////

// TWI Test Configuration

////////////////////////////////////////////////////////////

#define ECHO_WRITES false
#define MSG_EXCHANGE_CNT 5
#define INFINITE_MSG_EXCHANGE false
#define TWI_LOOPBACK false
// In non-loopback mode, the tx message is unchanged.
// In loopback mode the rx buffer acts as both the tx and rx buffer. The rx buffer is updated with each read.


////////////////////////////////////////////////////////////

// Adding TWI Functionality

////////////////////////////////////////////////////////////


#define TWI_BUFFER_SIZE 14
static unsigned char i2c_tx_buffer[TWI_BUFFER_SIZE] = {'M', 'S','G',' ','F', 'R', 'O', 'M', ' ', 'T', 'W', 'I', 'M', '\0'};
static unsigned char i2c_rx_buffer[TWI_BUFFER_SIZE] = "Begin Loopback";

// TWI Master Setup

#define MY_TWIM DT_NODELABEL(i2c1)
const struct device *nrfx_twis_dev1;// = device_get_binding(DT_LABEL(MY_TWIM));

static void twim_init(void) {
  int config_result = false;

  nrfx_twis_dev1 = device_get_binding(DT_LABEL(MY_TWIM));

  if (nrfx_twis_dev1 == NULL) {
    printk("\n\nI2C Slave: Device driver not found.\n");
  } else {
    printk("\nI2C device 1: %s\n", DT_PROP(DT_NODELABEL(twis_device1), label));

    config_result = i2c_configure(nrfx_twis_dev1, I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER);

    if (!config_result) {
      printk("I2C Master: Slave ADDR: 0x%x SCL: %d, SDA: %d, CLK(Hz): %u\n\n",
          DT_REG_ADDR(DT_NODELABEL(twis_device1)), 
          DT_PROP(MY_TWIM, scl_pin),
          DT_PROP(MY_TWIM, sda_pin),
          DT_PROP(MY_TWIM, clock_frequency));
    } else
      printk("\n\nI2C: Configuration error code: %d\n", config_result);
  }
}

// TWI Master Write

void twi_write_tst(void){
  int rtn_code  = 0;
  uint8_t incr  = 0;
  bool loopback = TWI_LOOPBACK;

  if (nrfx_twis_dev1 != NULL) {
       // printk("\nTWIM TX/writing.");
      if (!loopback) { //write repeated default message
        rtn_code = i2c_write(nrfx_twis_dev1, i2c_tx_buffer, sizeof(i2c_tx_buffer), DT_REG_ADDR(DT_NODELABEL(twis_device1)));
      }else{ //use read buffer as write buffer
        rtn_code = i2c_write(nrfx_twis_dev1, i2c_rx_buffer, sizeof(i2c_rx_buffer), DT_REG_ADDR(DT_NODELABEL(twis_device1)));
      }

      if (ECHO_WRITES && (rtn_code == 0)){
        printk("\nTWIM TX:");
        while (incr < TWI_BUFFER_SIZE)
          printk("%c", i2c_rx_buffer[incr++]);
        printk("\n");
      } //rtn_code == 0

      if (rtn_code){
        printk("twi return code %u\n\n", rtn_code);
      }
  } else // twis_dev1 == NULL
      printk("TWIS device is not initialized correctly.\n");
}

void twi_read_tst(void){
  int rtn_code  = 0;
  uint8_t incr  = 0;

  if (nrfx_twis_dev1 != NULL) {
      // printk("\nTWIM RX/reading -->");
      rtn_code = i2c_read(nrfx_twis_dev1, i2c_rx_buffer, sizeof(i2c_rx_buffer), DT_REG_ADDR(DT_NODELABEL(twis_device1)));

      if (rtn_code == 0) {
        printk("TWIM RX:");
        while (incr < TWI_BUFFER_SIZE) {
          printk("%c", i2c_rx_buffer[incr++]);
        };
      printk("\n");
      }
    
      if (rtn_code)
        printk("twi return code %u\n\n", rtn_code);

  } else //twim_dev1 == NULL
      printk("TWI is not initialized correctly.\n");
}


void main(void) {
  uint16_t twi_test_cnt = MSG_EXCHANGE_CNT;
  bool led_is_on = true;
  uint32_t ret;

  led_dev = device_get_binding(LED0);
  if (led_dev == NULL) {
    return;
  }

  ret = gpio_pin_configure(led_dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
  if (ret < 0) {
    return;
  }

  twim_init();
	uint8_t *bufferi2c;
	bufferi2c = (uint8_t []){0x80,0x5D};
	uint8_t *bufferi2crx;
	bufferi2crx = (uint8_t []){0x00,0x00};
	if(!i2c_write(nrfx_twis_dev1,bufferi2c,sizeof(bufferi2c), DT_REG_ADDR(DT_NODELABEL(twis_device1))))
    printk("ERROR: Not writing!\n");

  bufferi2c = (uint8_t []){0x35,0x17};
	k_msleep(10);
  	if(!i2c_write(nrfx_twis_dev1,bufferi2c,sizeof(bufferi2c), DT_REG_ADDR(DT_NODELABEL(twis_device1))))
    printk("ERROR: Not writing!\n");

	printk("PROM: ");
	bufferi2c = (uint8_t []){0x78,0x86};
	i2c_write(nrfx_twis_dev1,bufferi2c,sizeof(bufferi2c), DT_REG_ADDR(DT_NODELABEL(twis_device1)));
  k_msleep(15);
	if(!i2c_read(nrfx_twis_dev1,bufferi2crx,sizeof(bufferi2crx),DT_REG_ADDR(DT_NODELABEL(twis_device1))))
		printk("\n ERROR not reading\n");
	printk("%x",*bufferi2crx);

  while(1){

    gpio_pin_set(led_dev, PIN, (int)led_is_on);
    led_is_on = !led_is_on;
    bufferi2c = (uint8_t []){0x78,0x86};
    if((twi_test_cnt) || (INFINITE_MSG_EXCHANGE)){
		if(!i2c_write_read(nrfx_twis_dev1,DT_REG_ADDR(DT_NODELABEL(twis_device1)), bufferi2c,sizeof(bufferi2c),bufferi2crx,sizeof(bufferi2crx) ))
			printk("\n ERROR not reading\n");

    k_msleep(100);
		printk("\n Hallo:");
		printk("%x",*bufferi2crx);
        if(twi_test_cnt){
          twi_test_cnt--;
        }

        if((!twi_test_cnt) && (!INFINITE_MSG_EXCHANGE)){
              printk("\n\nTo rerun this test, reset the master.\n\n");
        }

    } 
  
    k_msleep(SLEEP_TIME_MS);
  }
}