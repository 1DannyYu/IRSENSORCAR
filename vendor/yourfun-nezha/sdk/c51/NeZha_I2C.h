#ifndef _NeZha_I2C_H_
#define _NeZha_I2C_H_

void NeZha_I2C_Init(void);
void NeZha_I2C_Start(void);
void NeZha_I2C_Stop(void);
void NeZha_I2C_SendByte(unsigned char Byte);
unsigned char NeZha_I2C_ReadByte(void);
void NeZha_I2C_SdaDir(unsigned char dir);
void NeZha_I2C_ACK(void);
void NeZha_I2C_NACK(void);
void NeZha_Delay_us(unsigned int xus);
void NeZha_Delay_ms(unsigned int xms);
#endif
