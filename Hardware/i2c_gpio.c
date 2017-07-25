#include "i2c_gpio.h"

#define TRUE  0
#define FALSE 1
#define I2C_Direction_Transmitter      ((uint8_t)0x00)	//写
#define I2C_Direction_Receiver         ((uint8_t)0x01)	//读

/***User's config***/
//SDA IO配置函数
#define SDB_IN()  {GPIOB->MODER&=0XFFFF0FFF;GPIOB->MODER|=0X00000000;}
#define SDB_OUT() {GPIOB->MODER&=0XFFFF0FFF;GPIOB->MODER|=0X00004000;}


//IO操作函数	 
#define I2C_SCL    PBout(6) //SCL-Output
#define I2C_SDA    PBout(7) //SDA-Output
#define READ_SDA   PBin(7)  //SDA-Input

/***User's config end...***/

static void I2C_Start(void);
static void I2C_Send_Byte(unsigned char txd);
static unsigned char I2C_Read_Byte(unsigned char data);
static void I2C_Ack(void);
static void I2C_NAck(void);
static void I2C_Stop(void);
static unsigned char I2C_Wait_Ack(void);

static void delay_us(int num)
{
  unsigned char i,j; 
  for(i=0;i<num;i++)
    for(j=100;j>0;j--);
}

void I2C_Start(void)
{
//  SDB_OUT();     //sda线输出
  I2C_SDA=1;	  	  
  I2C_SCL=1;
  delay_us(4);
  I2C_SDA=0;		//START:when CLK is high,DATA change form high to low 
  delay_us(4);
  I2C_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void I2C_Stop(void)
{
//  SDB_OUT();//sda线输出
  I2C_SCL=0;
  I2C_SDA=0;//STOP:when CLK is high DATA change form low to high
  delay_us(4);
  I2C_SCL=1; 
  //delay???
  I2C_SDA=1;//发送I2C总线结束信号
  delay_us(4);							   	
}

//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char I2C_Wait_Ack(void)
{
  uint16_t ucErrTime=0;
//  SDB_IN();      //SDA设置为输入  
  I2C_SDA=1;delay_us(1);	  //?SDA已经没有输出能力了，为什么还要输出？ 
  I2C_SCL=1;delay_us(1);
  ucErrTime = 0;
  while(READ_SDA)
  {
    ucErrTime++;
    if(ucErrTime>250)
    {
      I2C_Stop();
      return 1;
    }
  }
  I2C_SCL=0;//时钟输出0 	   
  return 0;  
} 

//产生ACK应答
void I2C_Ack(void)
{
  I2C_SCL=0;
//  SDB_OUT();
  I2C_SDA=0;
  delay_us(2);
  I2C_SCL=1;
  delay_us(2);
  I2C_SCL=0;
}
//不产生ACK应答		    
void I2C_NAck(void)
{
  I2C_SCL=0;
//  SDB_OUT();
  I2C_SDA=1;
  delay_us(2);
  I2C_SCL=1;
  delay_us(2);
  I2C_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void I2C_Send_Byte(unsigned char txd)
{
  unsigned char t;   
//  SDB_OUT(); 	    
  I2C_SCL=0;//拉低时钟开始数据传输
  for(t=0;t<8;t++)
  {
    I2C_SDA=(txd&0x80)>>7;
    txd<<=1; 	  
    delay_us(2);   //对TEA5767这三个延时都是必须的
    I2C_SCL=1;
    delay_us(2); 
    I2C_SCL=0;	
    delay_us(2);
  }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
unsigned char I2C_Read_Byte(unsigned char ack)
{
  unsigned char i,receive=0;
//  SDB_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
  {
    I2C_SCL=0; 
    delay_us(2);
    I2C_SCL=1;
    receive<<=1;
    if(READ_SDA)
      receive++;   
    delay_us(1); 
  }
  if(ack == 0)
    I2C_NAck();//发送nACK
  else if(ack == 1)
    I2C_Ack(); //发送ACK   
  return receive;
}


//addr：器件slave_address
//reg ：从器件将要写入数据的首地址
//data：将要写入的一串数据	  
//len ：写入数据的长度

unsigned char I2C_Write(unsigned char addr, unsigned char reg, unsigned char * data, unsigned char len)
{
  I2C_Start();
  
  I2C_Send_Byte(addr << 1 | I2C_Direction_Transmitter);//7位器件从地址+读写位
  if (I2C_Wait_Ack()) 
    return 1;
  
  I2C_Send_Byte(reg);
  if (I2C_Wait_Ack()) 
    return 1;
  
  while (len--)
  {
    I2C_Send_Byte(*data);
    if (I2C_Wait_Ack()) 
      return 1;
    data++;
  }
  I2C_Stop();
  return 0;
}


//addr：器件slave_address
//reg ：从器件将要读的数据的首地址
//len ：读出数据的长度
//buf ：将要读出的数据存储位置
unsigned char I2C_Read(unsigned char addr, unsigned char reg, unsigned char* buf, unsigned char len)
{
  I2C_Start();
  
  I2C_Send_Byte(addr << 1 | I2C_Direction_Transmitter);
  if(I2C_Wait_Ack())
    return 1;
  
  I2C_Send_Byte(reg);
  if(I2C_Wait_Ack())
    return 1;  
  
  I2C_Start();
  I2C_Send_Byte(addr << 1 | I2C_Direction_Receiver);
  if(I2C_Wait_Ack())
    return 1;
  
  while (len--)
  {
    if (len == 0)
      *buf = I2C_Read_Byte(0);
    else
      *buf = I2C_Read_Byte(1);
    buf++;
  }
  
  I2C_Stop();
  return 0;
}


