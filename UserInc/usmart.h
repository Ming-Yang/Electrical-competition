#ifndef __USMART_H
#define __USMART_H	 
//CubeMX 串口配置使用
//基于正点原子USMART修改
#include "userinc.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2011/6/13
//版本：V2.1
//版权所有，盗版必究。
//Copyright(C) 正点原子 2011-2021
//All rights reserved
//********************************************************************************
//升级说明
//V1.4
//增加了对参数为string类型的函数的支持.适用范围大大提高.
//优化了内存占用,静态内存占用为79个字节@10个参数.动态适应数字及字符串长度
//V2.0 
//1,修改了list指令,打印函数的完整表达式.
//2,增加了id指令,打印每个函数的入口地址.
//3,修改了参数匹配,支持函数参数的调用(输入入口地址).
//4,增加了函数名长度宏定义.	
//V2.1 20110707		 
//1,增加dec,hex两个指令,用于设置参数显示进制,及执行进制转换.
//注:当dec,hex不带参数的时候,即设定显示参数进制.当后跟参数的时候,即执行进制转换.
//如:"dec 0XFF" 则会将0XFF转为255,由串口返回.
//如:"hex 100" 	则会将100转为0X64,由串口返回
//2,新增usmart_get_cmdname函数,用于获取指令名字.
/////////////////////////////////////////////////////////////////////////////////////
//USMART资源占用情况@MDK 3.80A@2.0版本：
//FLASH:2492字节~3152字节(通过USMART_USE_HELP设置)
//SRAM:72字节(最少的情况下)
//SRAM计算公式:   SRAM=PARM_LEN+72-4  其中PARM_LEN必须大于等于4.
//应该保证堆栈不小于100个字节.
////////////////////////////////////////////用户配置参数////////////////////////////////////////////////////	  
#define MAX_FNAME_LEN 	30	//函数名最大长度，应该设置为不小于最长函数名的长度。											   
#define MAX_PARM 		10	//最大为10个参数 ,修改此参数,必须修改usmart_exe与之对应.
#define PARM_LEN 		200	//所有参数之和的长度不超过PARM_LEN个字节,注意串口接收部分要与之对应(不小于PARM_LEN)
#define USART_REC_LEN PARM_LEN 	//接收缓存长度

#define USMART_USE_HELP		1	//使用帮助，该值设为0，可以节省近700个字节，但是将导致无法显示帮助信息。
///////////////////////////////////////////////END///////////////////////////////////////////////////////////

#define USMART_OK 			0  //无错误
#define USMART_FUNCERR 		1  //函数错误
#define USMART_PARMERR 		2  //参数错误
#define USMART_PARMOVER 	3  //参数溢出
#define USMART_NOFUNCFIND 	4  //未找到匹配函数

#define SP_TYPE_DEC      	0  //10进制参数显示
#define SP_TYPE_HEX       	1  //16进制参数显示


 //函数名列表	 
struct _m_usmart_nametab
{
	void* func;			//函数指针
	const u8* name;		//函数名(查找串)	 
};
//usmart控制管理器
struct _m_usmart_dev
{
	struct _m_usmart_nametab *funs;	//函数名指针
	u8 (*cmd_rec)(u8*str);			//识别函数名及参数
	void (*exe)(void); 				//执行 
	void (*scan)(void);             //扫描
	u8 fnum; 				  		//函数数量
	u8 pnum;                        //参数数量
	u8 id;							//函数id
	u16 parmtype;					//参数的类型
	u8  plentbl[MAX_PARM];  		//每个参数的长度暂存表
	u8  parm[PARM_LEN];  			//函数的参数
	u8 sptype;						//参数显示类型(非字符串参数):0,10进制;1,16进制;
};
extern struct _m_usmart_nametab usmart_nametab[];	//在usmart_config.c里面定义
extern struct _m_usmart_dev usmart_dev;				//在usmart_config.c里面定义

u8 usmart_cmd_rec(u8*str);	//识别
void usmart_exe(void);		//执行
void usmart_scan(void);     //扫描

void init_usmart();
#endif


