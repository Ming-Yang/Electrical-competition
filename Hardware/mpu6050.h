#ifndef _MPU6050_H_
#define _MPU6050_H_


/*所有寄存器复位后值都为0x00，除了下面两个寄存器.
*寄存器(PWR_MGMT_1)107(0x6B): 0x40.
*寄存器(WHO_AM_I)117(0x75): 0x68.
*/

typedef struct
{
  signed short acc_x;
  signed short acc_y;
  signed short acc_z;
  signed short gyr_x;
  signed short gyr_y;
  signed short gyr_z;
  signed short temp;
}MPU6050_DATA_STRUCT;

/********用户可修改值 开始***********/

/********用户可修改值 结束***********/

#define MPU6050_SELF_TEST_X 	        0x0D//R/W [7:5]XA_TEST[4-2] [4:0]XG_TEST[4-0]
#define MPU6050_SELF_TEST_Y 	        0x0E//R/W [7:5]XA_TEST[4-2] [4:0]XG_TEST[4-0]
#define MPU6050_SELF_TEST_Z 	        0x0F//R/W [7:5]XA_TEST[4-2] [4:0]XG_TEST[4-0]
#define MPU6050_SELF_TEST_A 	        0x10//R/W [5:4]XA_TEST[1-0] [3:2]YA_TEST[1-0] [1:0]ZA_TEST[1-0]

//=========MPU6050 功能参数==================//
#define MPU6050_ADDR    0x68 //Normally,can range 0x08 to 0xEF
#define MPU6050_ID      MPU6050_ADDR
#define MPU6050_WRITE   MPU6050_ADDR<<1 | 0x00
#define MPU6050_READ    MPU6050_ADDR<<1 | 0x01

//定义SCL Bus Speed取值，外设总线为50Mhz时的计算结果
#define MPU6050_SCL_50KHZ                   (0x33) 
#define MPU6050_SCL_100KHZ                  (0x2B)  
#define MPU6050_SCL_150KHZ                  (0x28)
#define MPU6050_SCL_200KHZ                  (0x23)
#define MPU6050_SCL_250KHZ                  (0x21)
#define MPU6050_SCL_300KHZ                  (0x20)
#define MPU6050_SCL_400KHZ                  (0x17)  



/*Self Test Registers(自检寄存器)
*These registers are used for gyroscope and accelerometer self-tests 
*that permit the user to test the mechanical and electrical portions 
*of the gyroscope and the accelerometer. The following sections 
describe the self-test process.
*/

#define MPU6050_SMPLRT_DIV 	        0x19//R/W [7:0]SMPLRT_DIV
/*Sample Rate Divider(采样频率分频寄存器)
*This register specifies the divider from the gyroscope output rate 
*used to generate the Sample Rate for the MPU-60X0.
*/

#define MPU6050_CONFIG 		        0x1A//R/W [5:4]EXT_SYNC_SET[2-0]  [2:0]DLPF_CFG[2-0]
/*Configuration(配置寄存器)

*/

#define MPU6050_GYRO_CONFIG 	        0x1B//R/W [4:3]FS_SEL[1-0]
#define MPU6050_ACCEL_CONFIG 	        0x1C//R/W [7]XA_ST [6]YA_ST [5]ZA_ST [4:3]AFS_SEL[1-0]
#define MPU6050_MOT_THR 	        0x1F//R/W [7:0]MOT_THR[7-0]
/*运动检测阀值

*/
#define MPU6050_FIFO_EN 	        0x23//R/W [7]TEMP_FIFO_EN  [6]XG_FIFO_EN   [5]YG_FIFO_EN    [4]ZG_FIFO_EN 
//    [3]ACCEL_FIFO_EN [2]SLV2_FIFO_EN [1]SLV1_FIFO_EN  [0]SLV0_FIFO_EN
#define MPU6050_I2C_MST_CTRL	        0x24//R/W [7]MULT_MST_EN   [6]WAIT_FOR_ES  [5]SLV_3_FIFO_EN [4]I2C_MST_P_NSR
//    [3:0]I2C_MST_CLK[3-0]
#define MPU6050_I2C_SLV0_ADDR	        0x25//R/W [7]I2C_SLV0_RW [6:0]I2C_SLV0_ADDR[6-0]
#define MPU6050_I2C_SLV0_REG	        0x26//R/W [7:0]I2C_SLV0_REG[7-0]
#define MPU6050_I2C_SLV0_CTRL	        0x27//R/W [7]I2C_SLV0_EN [6]I2C_SLV0_BYTE_SW [5]I2C_SLV0_REG_DIS [4]I2C_SLV0_GRP
//    [3:0]I2C_SLV0_LEN[3-0]
#define MPU6050_I2C_SLV1_ADDR	        0x28//R/W [7]I2C_SLV1_RW [6:0]I2C_SLV1_ADDR[6-0]
#define MPU6050_I2C_SLV1_REG	        0x29//R/W [7:0]I2C_SLV1_REG[7-0]
#define MPU6050_I2C_SLV1_CTRL	        0x2A//R/W [7]I2C_SLV1_EN [6]I2C_SLV1_BYTE_SW [5]I2C_SLV1_REG_DIS [4]I2C_SLV1_GRP
//    [3:0]I2C_SLV1_LEN[3-0]
#define MPU6050_I2C_SLV2_ADDR	        0x2B//R/W [7]I2C_SLV2_RW [6:0]I2C_SLV2_ADDR[6-0]
#define MPU6050_I2C_SLV2_REG	        0x2C//R/W [7:0]I2C_SLV2_REG[7-0]
#define MPU6050_I2C_SLV2_CTRL	        0x2D//R/W [7]I2C_SLV2_EN [6]I2C_SLV2_BYTE_SW [5]I2C_SLV2_REG_DIS [4]I2C_SLV2_GRP
//    [3:0]I2C_SLV2_LEN[3-0]
#define MPU6050_I2C_SLV3_ADDR	        0x2E//R/W [7]I2C_SLV3_RW [6:0]I2C_SLV3_ADDR[6-0]
#define MPU6050_I2C_SLV3_REG	        0x2F//R/W [7:0]I2C_SLV3_REG[7-0]
#define MPU6050_I2C_SLV3_CTRL	        0x30//R/W [7]I2C_SLV3_EN [6]I2C_SLV3_BYTE_SW [5]I2C_SLV3_REG_DIS [4]I2C_SLV3_GRP
//    [3:0]I2C_SLV3_LEN[3-0]
#define MPU6050_I2C_SLV4_ADDR	        0x31//R/W [7]I2C_SLV4_RW [6:0]I2C_SLV4_ADDR[6-0]
#define MPU6050_I2C_SLV4_REG	        0x32//R/W [7:0]I2C_SLV4_REG[7-0]
#define MPU6050_I2C_SLV4_DO	        0x33//R/W [7:0]I2C_SLV4_DO[7-0]
#define MPU6050_I2C_SLV4_CTRL	        0x34//R/W [7]I2C_SLV4_EN [6]I2C_SLV4_INT_EN [5]I2C_SLV4_REG_DIS I2C_MST_DLY[4:0]
#define MPU6050_I2C_SLV4_DI	        0x35//R   [7:0]I2C_SLV4_DI[7-0]
#define MPU6050_I2C_MST_STATUS	        0x36//R   [7]PASS_THROUGH  [6]I2C_SLV4_DONE [5]I2C_LOST_ARB  [4]I2C_SLV4_NACK
//    [3]I2C_SLV3_NACK [2]I2C_SLV2_NACK [1]I2C_SLV1_NACK [0]I2C_SLV0_NACK
#define MPU6050_INT_PIN_CFG	        0x37//R/W [7]INT_LEVEL       [6]INT_OPEN     [5]LATCH_INT_EN [4]INT_RD_CLEAR 
//    [3]FSYNC_INT_LEVEL [2]FSYNC_INT_EN [1]I2C_BYPASS_EN
#define MPU6050_INT_ENABLE	        0x38//R/W [6]MOT_EN  [4]FIFO_OFLOW_EN   [3]I2C_MST_INT_EN [0]DATA_RDY_EN
#define MPU6050_INT_STATUS	        0x3A//R   [6]MOT_INT [4]FIFO_OFLOW _INT [3]I2C_MST_INT    [0]DATA_RDY_INT
#define MPU6050_ACCEL_XOUT_H	        0x3B//R   [7:0]ACCEL_XOUT[15-8]
#define MPU6050_ACCEL_XOUT_L	        0x3C//R   [7:0]ACCEL_XOUT[7-0]
#define MPU6050_ACCEL_XOUT              MPU6050_ACCEL_XOUT_H
#define MPU6050_ACCEL_YOUT_H	        0x3D//R   [7:0]ACCEL_YOUT[15-8]
#define MPU6050_ACCEL_YOUT_L	        0x3E//R   [7:0]ACCEL_YOUT[7-0]
#define MPU6050_ACCEL_YOUT              MPU6050_ACCEL_YOUT_H
#define MPU6050_ACCEL_ZOUT_H	        0x3F//R   [7:0]ACCEL_ZOUT[15-8]
#define MPU6050_ACCEL_ZOUT_L	        0x40//R   [7:0]ACCEL_ZOUT[7-0]
#define MPU6050_ACCEL_ZOUT              MPU6050_ACCEL_ZOUT_H
#define MPU6050_TEMP_OUT_H	        0x41//R   [7:0]TEMP_OUT[15-8]
#define MPU6050_TEMP_OUT_L	        0x42//R   [7:0]TEMP_OUT[7-0]
#define MPU6050_TEMP_OUT                MPU6050_TEMP_OUT_H
#define MPU6050_GYRO_XOUT_H	        0x43//R   [7:0]GYRO_XOUT[15-8]
#define MPU6050_GYRO_XOUT_L	        0x44//R   [7:0]GYRO_XOUT[7-0]
#define MPU6050_GYRO_XOUT               MPU6050_GYRO_XOUT_H
#define MPU6050_GYRO_YOUT_H	        0x45//R   [7:0]GYRO_YOUT[15-8]
#define MPU6050_GYRO_YOUT_L	        0x46//R   [7:0]GYRO_YOUT[7-0]
#define MPU6050_GYRO_YOUT               MPU6050_GYRO_YOUT_H
#define MPU6050_GYRO_ZOUT_H	        0x47//R   [7:0]GYRO_ZOUT[15-8]
#define MPU6050_GYRO_ZOUT_L	        0x48//R   [7:0]GYRO_ZOUT[7-0]
#define MPU6050_GYRO_ZOUT               MPU6050_GYRO_ZOUT_H
#define MPU6050_EXT_SENS_DATA_00	0x49//R   [7:0]EXT_SENS_DATA_00
#define MPU6050_EXT_SENS_DATA_01	0x4A//R   [7:0]EXT_SENS_DATA_01
#define MPU6050_EXT_SENS_DATA_02	0x4B//R   [7:0]EXT_SENS_DATA_02
#define MPU6050_EXT_SENS_DATA_03	0x4C//R   [7:0]EXT_SENS_DATA_03
#define MPU6050_EXT_SENS_DATA_04	0x4D//R   [7:0]EXT_SENS_DATA_04
#define MPU6050_EXT_SENS_DATA_05	0x4E//R   [7:0]EXT_SENS_DATA_05
#define MPU6050_EXT_SENS_DATA_06	0x4F//R   [7:0]EXT_SENS_DATA_06
#define MPU6050_EXT_SENS_DATA_07	0x50//R   [7:0]EXT_SENS_DATA_07
#define MPU6050_EXT_SENS_DATA_08	0x51//R   [7:0]EXT_SENS_DATA_08
#define MPU6050_EXT_SENS_DATA_09	0x52//R   [7:0]EXT_SENS_DATA_09
#define MPU6050_EXT_SENS_DATA_10	0x53//R   [7:0]EXT_SENS_DATA_10
#define MPU6050_EXT_SENS_DATA_11	0x54//R   [7:0]EXT_SENS_DATA_11
#define MPU6050_EXT_SENS_DATA_12	0x55//R   [7:0]EXT_SENS_DATA_12
#define MPU6050_EXT_SENS_DATA_13	0x56//R   [7:0]EXT_SENS_DATA_13
#define MPU6050_EXT_SENS_DATA_14	0x57//R   [7:0]EXT_SENS_DATA_14
#define MPU6050_EXT_SENS_DATA_15	0x58//R   [7:0]EXT_SENS_DATA_15
#define MPU6050_EXT_SENS_DATA_16	0x59//R   [7:0]EXT_SENS_DATA_16
#define MPU6050_EXT_SENS_DATA_17	0x5A//R   [7:0]EXT_SENS_DATA_17
#define MPU6050_EXT_SENS_DATA_18	0x5B//R   [7:0]EXT_SENS_DATA_18
#define MPU6050_EXT_SENS_DATA_19	0x5C//R   [7:0]EXT_SENS_DATA_19
#define MPU6050_EXT_SENS_DATA_20	0x5D//R   [7:0]EXT_SENS_DATA_20
#define MPU6050_EXT_SENS_DATA_21	0x5E//R   [7:0]EXT_SENS_DATA_21
#define MPU6050_EXT_SENS_DATA_22	0x5F//R   [7:0]EXT_SENS_DATA_22
#define MPU6050_EXT_SENS_DATA_23	0x60//R   [7:0]EXT_SENS_DATA_23
#define MPU6050_I2C_SLV0_DO		0x63//R/W [7:0]I2C_SLV0_DO
#define MPU6050_I2C_SLV1_DO		0x64//R/W [7:0]I2C_SLV1_DO
#define MPU6050_I2C_SLV2_DO		0x65//R/W [7:0]I2C_SLV2_DO
#define MPU6050_I2C_SLV3_DO		0x66//R/W [7:0]I2C_SLV3_DO
#define MPU6050_I2C_MST_DELAY_CTRL	0x67//R/W [7]DELAY_ES_SHADOW [4]I2C_SLV4_DLY_EN [3]I2C_SLV3_DLY_EN 
//    [2]I2C_SLV2_DLY_EN [1]I2C_SLV1_DLY_EN [0]I2C_SLV0_DLY_EN
#define MPU6050_SIGNAL_PATH_RESET	0x68//R/W [2]GYRO_RESET [1]ACCEL_RESET [0]TEMP_RESET
#define MPU6050_MOT_DETECT_CTRL		0x69//R/W [5:4]ACCEL_ON_DELAY[1-0]
#define MPU6050_USER_CTRL		0x6A//R/W [6]FIFO_EN    [5]I2C_MST_EN    [4]I2C_IF_DIS 
//    [2]FIFO_RESET [1]I2C_MST_RESET [0]SIG_COND_RESET
#define MPU6050_PWR_MGMT_1		0x6B//R/W [7]DEVICE_RESET [6]SLEEP          [5]CYCLE 
//    [4]TEMP_DIS     [2:0]CLKSEL[2-0]
#define MPU6050_PWR_MGMT_2		0x6C//R/W [7:6]LP_WAKE_CTRL[1-0] [5]STBY_XA  [4]STBY_YA 
//    [3]STBY_ZA             [2]STBY_XG  [1]STBY_YG  [0]STBY_ZG
#define MPU6050_FIFO_COUNTH		0x72//R/W [7:0]FIFO_COUNT[15-8]
#define MPU6050_FIFO_COUNTL		0x73//R/W [7:0]FIFO_COUNT[7-0]
#define MPU6050_FIFO_R_W		0x74//R/W [7:0]FIFO_DATA[7-0]
#define MPU6050_WHO_AM_I		0x75//R   [6:1]WHO_AM_I[6-1]     [0]depend on AD0


#define MPU6050_ADDRESS_AD0_LOW         0x68 // address pin low (GND), default
#define MPU6050_ADDRESS_AD0_HIGH        0x69 // address pin high (VCC)
#define MPU6050_ADDRESS                 MPU6050_ADDRESS_AD0_LOW
#define MPU6050_READ_ADDRESS            ( MPU6050_ADDRESS << 1 ) | 0x00
#define MPU6050_WRITE_ADDRESS           ( MPU6050_ADDRESS << 1 ) | 0x01

#include "userinc.h"
#include "i2c_gpio.h"

u8 MPU6050_Init(void);
void MPU6050_GetData(MPU6050_DATA_STRUCT*);

#endif /* _MPU6050_H_ */