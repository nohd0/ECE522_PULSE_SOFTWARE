// I2C
void writeReg_I2C( uint8 registerAddress, uint8 registerValue ); //Allie
void writeReg16_I2C( uint8 registerAddress, uint16 registerValue ); //Allie
uint8 readReg_I2C(uint8 SlaveAddr, uint8 registerAddress); //Allie
void FetchSI1145Data( uint8 *SI1145UV ); //Allie
void FetchAccelData( uint8 *AccelData ); //Allie
void Reset_I2C( void ); //Allie

// Slave Addresses
/*
SI1143 = 0x5A;
SI1145 = 0x60;
TMP006 = 0x40;
Accelerometer = 0x0F;
*/