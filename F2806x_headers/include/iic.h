#ifndef IIC_H_
#define IIC_H_

#include "DSP28x_Project.h"  // Device Headerfile and Examples Include File

#define setSCL GpioDataRegs.GPADAT.bit.GPIO29 = 1
#define resetSCL GpioDataRegs.GPADAT.bit.GPIO29 = 0
#define setSDA GpioDataRegs.GPADAT.bit.GPIO28 = 1
#define resetSDA GpioDataRegs.GPADAT.bit.GPIO28 = 0

void I2C_Ack3(void);
void I2C_NAck3(void);
void I2C_Start3(void);
void I2C_Stop3(void);
void OLED_WriteByte(Uint8 mcmd);
void I2C_ReleaseBus(void);

#endif /* IIC_H_ */
