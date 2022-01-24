#ifndef MS5637_H_
#def MS5637_H_

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/printk.h>
#include <zephyr.h>

// Init i2c device with the given configurations
void ms5637_init(uint32_t dev_config);
// Write data to the device
void ms5637_write(uint8_t *buffer, uint8_t size);
// Read data from the device
uint8_t *ms5637_read(uint8_t *buffer, uint8_t size);
// Write and then read data from the device
uint8_t *ms5637_write_read(uint8_t *buffer, uint8_t size);

#endif