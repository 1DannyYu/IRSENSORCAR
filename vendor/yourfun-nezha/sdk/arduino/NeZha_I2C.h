
void NeZha_I2C_Init();
void NeZha_I2C_SdaDir(unsigned char dir);
void NeZha_I2C_Start(void);
void NeZha_I2C_Stop(void);
void NeZha_I2C_SendByte(unsigned char Byte);
unsigned char NeZha_I2C_ReadByte();
void NeZha_I2C_ACK();
void NeZha_I2C_NACK();
void NeZha_Delay_us(unsigned int xus) ;
void NeZha_Delay_ms(unsigned int xms) ;
