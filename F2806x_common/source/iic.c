#include "iic.h"
void I2C_Ack3(void) {
    resetSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
}

void I2C_NAck3(void) {
    setSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
}

void I2C_Start3(void) {
    setSDA;
    DELAY_US(1);
    setSCL;
    DELAY_US(1);
    resetSDA;
    DELAY_US(1);
    resetSCL;
    DELAY_US(1);
    OLED_WriteByte(0x78);
    I2C_Ack3();  //*这句很重要
}

void I2C_Stop3(void) {
    setSCL;
    DELAY_US(1);
    resetSDA;
    DELAY_US(1);
    setSDA;
    DELAY_US(1);
}

void OLED_WriteByte(Uint8 mcmd) {
    Uint8 length = 8;  // Send Command

    while (length--) {
        if (mcmd & 0x80) {
            setSDA;
        } else {
            resetSDA;
        }
        DELAY_US(1);
        setSCL;
        DELAY_US(1);
        resetSCL;
        DELAY_US(1);
        mcmd = mcmd << 1;
    }
}

/*
 * I2C总线释放函数：发送9个时钟脉冲
 * 用于释放卡在低电平的SDA线（从机可能处于错误状态）
 */
void I2C_ReleaseBus(void) {
    Uint8 i;

    /* 先确保SCL和SDA均为高 */
    setSDA;
    DELAY_US(5);
    setSCL;
    DELAY_US(5);

    /* 发送最多9个时钟脉冲，尝试释放总线 */
    for (i = 0; i < 9; i++) {
        resetSCL;
        DELAY_US(5);
        setSCL;
        DELAY_US(5);
    }

    /* 产生一个STOP条件使总线回到空闲状态 */
    resetSCL;
    DELAY_US(5);
    resetSDA;
    DELAY_US(5);
    setSCL;
    DELAY_US(5);
    setSDA;
    DELAY_US(5);
}

// 2024.3.15 由搁浅飞云改进
