/*******************************************************
 *
 *  @file   : i2c.h
 *
 *  @brief  : This header file provides abstraction of the
 *            I2c functions
 *
 *  @date   : September 11, 2022
 *  @author : Sanish Kharade
 *
 ******************************************************/

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#define SI7021_POWERUP_TIME_US      80000
#define SI7021_CONVERSION_TIME_US   10800


#define MAX30105_ADDRESS          0x57

/**
 * @brief   : Function to initialize I2C0
 *
 * @param   : none
 *
 * @return  : void
 */
void i2c_init();

/**
 * @brief   : Function to send read command to the temperature sensor
 *
 * @param   : none
 *
 * @return  : void
 */
void sendReadCommandToSi7021(void);

/**
 * @brief   : Function to read the temperature value
 *
 * @param   : none
 *
 * @return  : void
 */
void readTemperature(void);

/**
 * @brief   : Function to power the temperature sensor ON/OFF
 *
 * @param   : state : true  - ON
 *                    false - OFF
 *
 * @return  : void
 */
void Si7021_power(bool state);

/**
 * @brief   : Function to print the temperature obtained from Si7021
 *
 * @param   : none
 *
 * @return  : void
 */
void printTemperature(void);


bool i2c_write_read_apds9960(uint8_t reg, uint8_t *data);
bool i2c_write_write_apds9960(uint8_t reg, uint8_t data);
int i2c_read_data_block_apds9960(uint8_t reg, uint8_t *data, unsigned int len);


int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len);
bool wireReadDataByte(uint8_t reg, uint8_t *val);
bool wireWriteDataByte(uint8_t reg, uint8_t val);


uint8_t max30101_i2c_read_reg(uint8_t reg);
void max30101_i2c_write_reg(uint8_t reg, uint8_t value);
//void max30101_i2c_write_reg(uint8_t reg, uint8_t *value);
uint8_t max30101_i2c_read();
void max30101_i2c_write(uint8_t reg);
uint8_t max30101_i2c_burst_read(uint8_t reg, uint8_t *data, unsigned int len);

#endif /* SRC_I2C_H_ */
