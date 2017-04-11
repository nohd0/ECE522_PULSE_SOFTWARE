#include "hal_i2c.h"
#include "OSAL.h"
#include "stdio.h"
#include "i2c_wrapper.h"

void writeReg_I2C( uint8 registerAddress, uint8 registerValue )
{
    uint8 writeData[2];
    writeData[0] = registerAddress;
    writeData[1] = registerValue;
    HalI2CWrite( 2, writeData , TRUE );
}

void writeReg16_I2C( uint8 registerAddress, uint16 registerValue )
{
    uint8 writeData[3];
    writeData[0] = registerAddress;
    writeData[1] = (registerValue >> 8);
    writeData[2] = registerValue;
    HalI2CWrite( 3, writeData , TRUE );
}



uint8 readReg_I2C(uint8 SlaveAddr, uint8 registerAddress)
{
    uint8 writeData[2];
    uint8 readData[2];
    HalI2CInit( SlaveAddr, i2cClock_533KHZ );
    writeData[0] = registerAddress;
    HalI2CWrite( 1, writeData , FALSE );
    readData[0] = 0xFF;
    HalI2CRead( 1, readData );
    return readData[0];
}

void Reset_I2C( void ) {
    // Global Reset Function for All Slaves
    HalI2CInit( 0x00, i2cClock_533KHZ );
    uint8 writeData[2];
    writeData[0] = 0x06;
    HalI2CWrite( 1, writeData, TRUE );
}

void FetchSI1145Data( uint8 *SI1145UV ) {
  // Reads Registers 0x22 through 0x25 and 0x2C through 0x2D [ALS_VIS, ALS_IR, UVINDEX  with 2 bytes each]
  uint8 writeData[1] = { 0x2C };
  HalI2CInit( 0x60, i2cClock_533KHZ );
  HalI2CWrite( 1, writeData, FALSE );
  HalI2CRead( 2, SI1145UV );
}

void FetchAccelData( uint8 *AccelData ) {
  uint8 readData[6];
  uint8 writeData[1] = { 0x06 }; //XOUT, YOUT, ZOUT
  HalI2CInit( 0x0F, i2cClock_267KHZ );
  HalI2CWrite( 1, writeData, FALSE ); 
  HalI2CRead( 6, readData );
  AccelData[0] = ( readData[0] >> 4 & 0x0F ) | (readData[1] << 4 & 0xF0 );
  AccelData[1] = ( readData[2] >> 4 & 0x0F ) | (readData[3] << 4 & 0xF0 );
  AccelData[2] = ( readData[4] >> 4 & 0x0F ) | (readData[5] << 4 & 0xF0 );
}
