/******************************************************************
 *
 *  @file   : i2c.c
 *  @brief  : This source file contains the I2C functions
 *
 *  @date   : September 11, 2022
 *  @author : Sanish Kharade
 *
 *  @note   : Please see i2c.h for function explanations
 *
 *****************************************************************/

#include "sl_i2cspm.h"
#include "em_gpio.h"
#include "gpio.h"
#include "app.h"
#include "i2c.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

//#define SI7021_ADDRESS      (0x40)
#define APDS9960_ADDRESS    (0x39)

#define SI7021_I2C0_SCL_PIN   (10)
#define SI7021_I2C0_SDA_PIN   (11)

#define SI7021_I2C0_SCL_PORT_LOCATION   (14)
#define SI7021_I2C0_SDA_PORT_LOCATION   (16)

#define SI7021_COMMAND 0xF3

I2C_TransferSeq_TypeDef transferSequence;
//uint8_t cmd_data;
uint16_t read_data;

#define I2CSPM 0
//uint32_t write_read(uint8_t reg, uint8_t *data) {
//  uint8_t comd_data[1];
//  I2C_TransferReturn_TypeDef transferStatus;
//
//  i2c_init();
//
//  comd_data[0] = reg;
//
//  //structure to write command from master to slave
//  transferSequence.addr = APDS9960_ADDRESS<<1;
//  transferSequence.flags = I2C_FLAG_WRITE_READ;
//  transferSequence.buf[0].data = comd_data;
//  transferSequence.buf[0].len = 1;
//  transferSequence.buf[1].data = data;
//  transferSequence.buf[1].len = 1;
//
//    //enable I2C interrupt
//     // NVIC_EnableIRQ(I2C0_IRQn);
//
//      //initialize I2C transfer
//      transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
//
//      //check transfer function return status
//      if(transferStatus != i2cTransferDone) {
//          LOG_ERROR("I2C_TransferInit status %d write: failed\n\r", (uint32_t)transferStatus);
//          *data = 0xff;
//          return (uint32_t)transferStatus;
//      }
//
//      return (uint32_t)1;
//}

uint8_t max30101_i2c_read()
{

  uint8_t read_data;
  transferSequence.flags = I2C_FLAG_READ, // Write command
  transferSequence.addr = (MAX30105_ADDRESS<<1), // Slave address needs to be left shift by one bit
  transferSequence.buf[0].data = &read_data, // Passing the pointer that has the command data stored
  transferSequence.buf[0].len = sizeof(read_data); // Length of the command data


  I2C_TransferReturn_TypeDef trans_ret = I2CSPM_Transfer(I2C0, &transferSequence);
//  LOG_INFO("max30101_i2c_read: %d\r", trans_ret);
  // Checking if the transfer is done or no.
  if(trans_ret != i2cTransferDone)
      {
        LOG_ERROR("I2C Write error: %d", trans_ret); // If transfer is not done then we will log error message
//        createEventSystemError();
      }

  return read_data;

//  I2C_TransferSeq_TypeDef transferSequence = {
//      .addr = SI7021_ADDRESS << 1,
//      .flags = I2C_FLAG_READ,
//      .buf[0].data = (uint8_t*)(&read_data),
//      .buf[0].len = sizeof(read_data)
//  };
//
//  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
//  if (transferStatus != i2cTransferDone) {
//      LOG_ERROR("I2CSPM Transfer: I2C Read failed with error = %d\n\r", transferStatus);
//  }
//
//  uint16_t new_data =  (read_data << 8) | ((read_data >> 8) & 0x00FF);
//
//  float temp = ( (175.72 * new_data) / 65536 ) - 46.85;
//
//  LOG_INFO("Temperature = %f\n\r", temp);

}

uint8_t max30101_i2c_read_reg(uint8_t reg)
{

  uint8_t read_data;
  transferSequence.flags = I2C_FLAG_WRITE_READ, // Write command
  transferSequence.addr = (MAX30105_ADDRESS<<1), // Slave address needs to be left shift by one bit
  transferSequence.buf[0].data = &reg, // Passing the pointer that has the command data stored
  transferSequence.buf[0].len = sizeof(reg); // Length of the command data
  transferSequence.buf[1].data = &read_data, // Passing the pointer that has the command data stored
//  transferSequence.buf[1].len = nbytes_read_data;
  transferSequence.buf[1].len = 1;

  // This will initialize the write command on to the bus
  I2C_TransferReturn_TypeDef trans_ret = I2CSPM_Transfer(I2C0,&transferSequence);
//  LOG_INFO("i2c_Read_blocking: %d\r", trans_ret);
  // Checking if the transfer is done or no.
  if(trans_ret != i2cTransferDone)
      {
        LOG_ERROR("I2C Write error: %d", trans_ret); // If transfer is not done then we will log error message
//        createEventSystemError();
      }
//  for (int i = 0; i < nbytes_read_data; i++)
//    {
//      printf("Reg 0x%02x :: Read %d:0x%02x\t", reg, i+1, *(read_data+i));
//    }
//  printf("\n\n");
  return read_data;
}
void max30101_i2c_write_reg(uint8_t reg, uint8_t value)
{
  transferSequence.flags = I2C_FLAG_WRITE_WRITE, // Write command
  transferSequence.addr = (MAX30105_ADDRESS<<1), // Slave address needs to be left shift by one bit
  transferSequence.buf[0].data = &reg, // Passing the pointer that has the command data stored
  transferSequence.buf[0].len = sizeof(reg); // Length of the command data
  transferSequence.buf[1].data = &value, // Passing the pointer that has the command data stored
//  transferSequence.buf[1].len = nbytes_write_data;
  transferSequence.buf[1].len = 1;

  // This will initialize the write command on to the bus
  I2C_TransferReturn_TypeDef trans_ret = I2CSPM_Transfer(I2C0,&transferSequence);
//  LOG_INFO("i2c_Write_Write_blocking: %d\r", trans_ret);
  // Checking if the transfer is done or no.
  if(trans_ret != i2cTransferDone)
      {
        LOG_ERROR("I2C Write error: %d", trans_ret); // If transfer is not done then we will log error message
//        createEventSystemError();
      }

}
bool wireReadDataByte(uint8_t reg, uint8_t *val)
{
  return i2c_write_read_apds9960(reg, val);
}
bool wireWriteDataByte(uint8_t reg, uint8_t val)
{
  return i2c_write_write_apds9960(reg, val);
}
int wireReadDataBlock(uint8_t reg, uint8_t *val, unsigned int len)
{
  return i2c_read_data_block_apds9960(reg, val, len);
}
bool i2c_write_read_apds9960(uint8_t reg, uint8_t *data)
{
//  uint8_t cmd_data;
  uint8_t cmd_data[1];
  I2C_TransferReturn_TypeDef transferStatus;

  i2c_init();

  cmd_data[0] = reg;
//  cmd_data = reg;

  // Sequence to send over I2C
  transferSequence.addr = APDS9960_ADDRESS << 1;
  transferSequence.flags = I2C_FLAG_WRITE_READ;

  transferSequence.buf[0].data = cmd_data;
  transferSequence.buf[0].len = 1;

  transferSequence.buf[1].data = data;
  transferSequence.buf[1].len = 1;

  // Enable NVIC for I2C0
  //NVIC_EnableIRQ(I2C0_IRQn);

  //transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
//  if (transferStatus < 0) {
  if(transferStatus != i2cTransferDone) {
      LOG_ERROR("i2c_write_read_apds9960 failed: reg=0x%x, data=0x%x, error = 0x%d\n\r", cmd_data[0], *data, transferStatus);
      *data = 0xFF;
//      return (uint32_t)transferStatus;
      return false;

  }

//  return (uint32_t)1;
  return true;
}

bool i2c_write_write_apds9960(uint8_t reg, uint8_t data)
{
  uint8_t cmd_data[2];
  uint8_t no_data[1];
  I2C_TransferReturn_TypeDef transferStatus;

  i2c_init();

  cmd_data[0] = reg;
  cmd_data[1] = data;

  // Sequence to send over I2C
  transferSequence.addr = APDS9960_ADDRESS << 1;
  transferSequence.flags = I2C_FLAG_WRITE;

  transferSequence.buf[0].data = cmd_data;
  transferSequence.buf[0].len = 2;

  transferSequence.buf[1].data = no_data;
  transferSequence.buf[1].len = 0;

  // Enable NVIC for I2C0
  //NVIC_EnableIRQ(I2C0_IRQn);

  //transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
//  if (transferStatus < 0) {
  if(transferStatus != i2cTransferDone) {
      LOG_ERROR("i2c_write_write_apds9960 failed: reg=0x%x, data=0x%x, error = 0x%x\n\r", cmd_data, data, transferStatus);
      //*data = 0xFF;
//      return (uint32_t)transferStatus;
      return false;

  }

//  return (uint32_t)1;
  return true;
}

int i2c_read_data_block_apds9960(uint8_t reg, uint8_t *data, unsigned int len)
{
  //uint8_t cmd_data[2];
  uint8_t cmd_data;
  //uint8_t no_data[1];
  I2C_TransferReturn_TypeDef transferStatus;

  i2c_init();

  cmd_data = reg;

  // Sequence to send over I2C
  transferSequence.addr = APDS9960_ADDRESS << 1;
  transferSequence.flags = I2C_FLAG_WRITE_READ;

  transferSequence.buf[0].data = &cmd_data;
  transferSequence.buf[0].len = sizeof(cmd_data);

  transferSequence.buf[1].data = data;
  transferSequence.buf[1].len = len;

  // Enable NVIC for I2C0
  //NVIC_EnableIRQ(I2C0_IRQn);

  //transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
  if (transferStatus < 0) {
      LOG_ERROR("i2c_read_data_block_apds9960 failed: reg=0x%x, data=0x%x, len=%d, error = 0x%x\n\r", cmd_data, *data, len, transferStatus);
      //*data = 0xFF;
//      return (uint32_t)transferStatus;
      return -1;

  }

//  return (uint32_t)1;
  return (int)len;
}
#if (I2CSPM)
void i2c_init()
{

  I2CSPM_Init_TypeDef i2c_config = {
      .port = I2C0,
      .sclPort = gpioPortC,
      .sclPin = SI7021_I2C0_SCL_PIN,
      .sdaPort = gpioPortC,
      .sdaPin = SI7021_I2C0_SDA_PIN,
      .portLocationScl = SI7021_I2C0_SCL_PORT_LOCATION,
      .portLocationSda = SI7021_I2C0_SDA_PORT_LOCATION,
      .i2cRefFreq = 0,
      .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
      .i2cClhr = i2cClockHLRStandard
  };

  /*
   * I2CSPM_Init
   * - Does the clock initializations
   * - Does the gpio initializations
   * - Sets the required I2C bits and uses the emlib function I2C_Init() for
   *    finally initializing I2C
   * - Does not return anything hence no error check condition here
   * */
  I2CSPM_Init(&i2c_config);

#if DEBUG
  uint32_t i2c_bus_freq = 0;
  i2c_bus_freq = I2C_BusFreqGet(I2C0);
  LOG_INFO("I2C Bus Freq = %u\n\r", i2c_bus_freq);
#endif

}
#else
void i2c_init()
{

  I2CSPM_Init_TypeDef i2c_config = {
      .port = I2C0,
      .sclPort = gpioPortC,
      .sclPin = SI7021_I2C0_SCL_PIN,
      .sdaPort = gpioPortC,
      .sdaPin = SI7021_I2C0_SDA_PIN,
      .portLocationScl = SI7021_I2C0_SCL_PORT_LOCATION,
      .portLocationSda = SI7021_I2C0_SDA_PORT_LOCATION,
      .i2cRefFreq = 0,
      .i2cMaxFreq = I2C_FREQ_STANDARD_MAX,
      .i2cClhr = i2cClockHLRStandard
  };

  /*
   * I2CSPM_Init
   * - Does the clock initializations
   * - Does the gpio initializations
   * - Sets the required I2C bits and uses the emlib function I2C_Init() for
   *    finally initializing I2C
   * - Does not return anything hence no error check condition here
   * */
  I2CSPM_Init(&i2c_config);

#if DEBUG
  uint32_t i2c_bus_freq = 0;
  i2c_bus_freq = I2C_BusFreqGet(I2C0);
  LOG_INFO("I2C Bus Freq = %u\n\r", i2c_bus_freq);
#endif

}
#endif

//void Si7021_power(bool state)
//{
//  if(state) {
//      // Turn ON
//      GPIO_PinOutSet(SI7021_SENSOR_ENABLE_PORT,SI7021_SENSOR_ENABLE_PIN);
//  }
//  else {
//      // Turn OFF
//      GPIO_PinOutClear(SI7021_SENSOR_ENABLE_PORT,SI7021_SENSOR_ENABLE_PIN);
//  }
//
//}
//
//void printTemperature(void)
//{
//  // Swap the bytes to get correct data
//  uint16_t new_data =  (read_data << 8) | ((read_data >> 8) & 0x00FF);
//
//  float temp = ( (175.72 * new_data) / 65536 ) - 46.85;
//
//  LOG_INFO("Temperature = %f\n\r", temp);
//}
#if (I2CSPM)
void sendReadCommandToSi7021(void)
{
  cmd_data = SI7021_COMMAND;

  // Sequence to send over I2C
  I2C_TransferSeq_TypeDef transferSequence = {
      .addr = SI7021_ADDRESS << 1,
      .flags = I2C_FLAG_WRITE,
      .buf[0].data = &cmd_data,
      .buf[0].len = sizeof(cmd_data)
  };

  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
  if (transferStatus != i2cTransferDone) {
      LOG_ERROR("I2CSPM Transfer: I2C Write of Command %x failed with error = %d\n\r", cmd_data, transferStatus);
  }

}
void readTemperature(void)
{

  I2C_TransferSeq_TypeDef transferSequence = {
      .addr = SI7021_ADDRESS << 1,
      .flags = I2C_FLAG_READ,
      .buf[0].data = (uint8_t*)(&read_data),
      .buf[0].len = sizeof(read_data)
  };

  transferStatus = I2CSPM_Transfer(I2C0, &transferSequence);
  if (transferStatus != i2cTransferDone) {
      LOG_ERROR("I2CSPM Transfer: I2C Read failed with error = %d\n\r", transferStatus);
  }

  uint16_t new_data =  (read_data << 8) | ((read_data >> 8) & 0x00FF);

  float temp = ( (175.72 * new_data) / 65536 ) - 46.85;

  LOG_INFO("Temperature = %f\n\r", temp);

}
#else
//void sendReadCommandToSi7021(void)
//{
//  uint8_t cmd_data;
//  I2C_TransferReturn_TypeDef transferStatus;
//
//  i2c_init();
//
//  cmd_data = SI7021_COMMAND;
//
//  // Sequence to send over I2C
//  transferSequence.addr = SI7021_ADDRESS << 1;
//  transferSequence.flags = I2C_FLAG_WRITE;
//  transferSequence.buf[0].data = &cmd_data;
//  transferSequence.buf[0].len = sizeof(cmd_data);
//
//  // Enable NVIC for I2C0
//  NVIC_EnableIRQ(I2C0_IRQn);
//
//  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
//  if (transferStatus < 0) {
//      LOG_ERROR("I2C Transfer: I2C Write of Command %x failed with error = %d\n\r", cmd_data, transferStatus);
//  }
//
//}
//void readTemperature(void)
//{
//  I2C_TransferReturn_TypeDef transferStatus;
//
//  i2c_init();
//  transferSequence.addr = SI7021_ADDRESS << 1;
//  transferSequence.flags = I2C_FLAG_READ;
//  transferSequence.buf[0].data = (uint8_t*)(&read_data);
//  transferSequence.buf[0].len = sizeof(read_data);
//
//  // Enable NVIC for I2C0
//  NVIC_EnableIRQ(I2C0_IRQn);
//
//  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
//  if (transferStatus < 0) {
//      LOG_ERROR("I2C Transfer: I2C Read failed with error = %d\n\r", transferStatus);
//  }
//
//}
#endif
