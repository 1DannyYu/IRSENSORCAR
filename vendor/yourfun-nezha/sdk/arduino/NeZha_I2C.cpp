#include "NeZha_I2C.h"
#include <Arduino.h>

#define UNIT_T 2

#define NeZha_SCL(x) digitalWrite(6, x)
#define NeZha_SDA(x) digitalWrite(7, x)

void NeZha_I2C_Init()
{
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
}

void NeZha_I2C_SdaDir(unsigned char dir)
{
    if (!dir) {
       pinMode(7, OUTPUT);
    } else {
       pinMode(7, INPUT);
    }
}

void NeZha_I2C_Start(void)
{
    NeZha_SDA(1);
    NeZha_SCL(1);
    NeZha_SDA(0);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(0);
}

void NeZha_I2C_Stop(void)
{
    NeZha_SDA(0);
    NeZha_SCL(1);
    NeZha_Delay_us(UNIT_T);
    NeZha_SDA(1);
}

void NeZha_I2C_SendByte(unsigned char Byte)
{
    NeZha_I2C_SdaDir(0);
    NeZha_SCL(0);
    NeZha_Delay_us(UNIT_T);

    for (unsigned char i = 0; i < 8; i++)
    {
        NeZha_SDA(Byte & (0x80 >> i));
        NeZha_SCL(1);
        NeZha_Delay_us(UNIT_T);
        NeZha_SCL(0);
        NeZha_Delay_us(UNIT_T);
    }

    NeZha_SCL(1);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(0);
    NeZha_Delay_us(UNIT_T);
}

unsigned char NeZha_I2C_ReadByte()
{
    unsigned char Byte = 0x00;
    NeZha_I2C_SdaDir(1);

    for (unsigned char i = 0; i < 8; i++)
    {
        NeZha_SCL(0);
        NeZha_Delay_us(UNIT_T);
        NeZha_SCL(1);

        if (digitalRead(7))
        {
          Byte |= (0X80 >> i);
        }

        NeZha_Delay_us(UNIT_T);
    }

return Byte;
}

void NeZha_I2C_ACK()
{
    NeZha_SCL(0);
    NeZha_I2C_SdaDir(0);
    NeZha_SDA(0);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(1);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(0);
}

void NeZha_I2C_NACK()
{
    NeZha_SCL(0);
    NeZha_I2C_SdaDir(0);
    NeZha_SDA(1);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(1);
    NeZha_Delay_us(UNIT_T);
    NeZha_SCL(0);
}

void NeZha_Delay_ms(unsigned int xms) 
{
    delay(xms);
}

void NeZha_Delay_us(unsigned int xus) 
{
    delayMicroseconds(xus);
}