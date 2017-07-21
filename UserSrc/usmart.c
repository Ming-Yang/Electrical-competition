#include "usmart.h"
//CubeMX 串口配置使用
//基于正点原子USMART修改

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

u8 USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.

//字符串操作
//对比字符串str1和str2
//*str1:字符串1指针
//*str2:字符串2指针
//返回值:0，不相等;1，相等;
u8 usmart_strcmp(u8 *str1, u8 *str2)
{
  while (1)
  {
    if (*str1 != *str2)
      return 0; //不相等
    if (*str1 == '\0')
      break; //对比完成了.
    str1++;
    str2++;
  }
  return 1; //两个字符串相等
}
//把str1的内容copy到str2
//*str1:字符串1指针
//*str2:字符串2指针
void usmart_strcopy(u8 *str1, u8 *str2)
{
  while (1)
  {
    *str2 = *str1; //拷贝
    if (*str1 == '\0')
      break; //拷贝完成了.
    str1++;
    str2++;
  }
}
//得到字符串的长度(字节)
//*str:字符串指针
//返回值:字符串的长度
u8 usmart_strlen(u8 *str)
{
  u8 len = 0;
  while (1)
  {
    if (*str == '\0')
      break; //拷贝完成了.
    len++;
    str++;
  }
  return len;
}
//m^n函数
//返回值:m^n次方
u32 usmart_pow(u8 m, u8 n)
{
  u32 result = 1;
  while (n--)
    result *= m;
  return result;
}
//把字符串转为数字
//支持16进制转换,但是16进制字母必须是大写的,且格式为以0X开头的.
//不支持负数
//*str:数字字符串指针
//*res:转换完的结果存放地址.
//返回值:0，成功转换完成.其他,错误代码.
//1,数据格式错误.2,16进制位数为0.3,起始格式错误.4,十进制位数为0.
u8 usmart_str2num(u8 *str, u32 *res)
{
  u32 t;
  u8 bnum = 0; //数字的位数
  u8 *p;
  u8 hexdec = 10; //默认为十进制数据
  p = str;
  *res = 0; //清零.
  while (1)
  {
    if ((*p <= '9' && *p >= '0') || (*p <= 'F' && *p >= 'A') || (*p == 'X' && bnum == 1)) //参数合法
    {
      if (*p >= 'A')
        hexdec = 16; //字符串中存在字母,为16进制格式.
      bnum++;        //位数增加.
    }
    else if (*p == '\0')
      break; //碰到结束符,退出.
    else
      return 1; //不全是十进制或者16进制数据.
    p++;
  }
  p = str;          //重新定位到字符串开始的地址.
  if (hexdec == 16) //16进制数据
  {
    if (bnum < 3)
      return 2;                         //位数小于3，直接退出.因为0X就占了2个,如果0X后面不跟数据,则该数据非法.
    if (*p == '0' && (*(p + 1) == 'X')) //必须以'0X'开头.
    {
      p += 2;    //偏移到数据起始地址.
      bnum -= 2; //减去偏移量
    }
    else
      return 3; //起始头的格式不对
  }
  else if (bnum == 0)
    return 4; //位数为0，直接退出.
  while (1)
  {
    if (bnum)
      bnum--;
    if (*p <= '9' && *p >= '0')
      t = *p - '0'; //得到数字的值
    else
      t = *p - 'A' + 10; //得到A~F对应的值
    *res += t * usmart_pow(hexdec, bnum);
    p++;
    if (*p == '\0')
      break; //数据都查完了.
  }
  return 0; //成功转换
}
//得到指令名
//*str:源字符串
//*cmdname:指令名
//*nlen:指令名长度
void usmart_get_cmdname(u8 *str, u8 *cmdname, u8 *nlen)
{
  *nlen = 0;
  while (*str != ' ' && *str != '\0') //找到空格或者结束符则认为结束了
  {
    *cmdname = *str;
    str++;
    cmdname++;
    (*nlen)++; //统计命令长度
  }
  *cmdname = '\0'; //加入结束符
}
//从str中得到函数名
//*str:源字符串指针
//*fname:获取到的函数名字指针
//pnum:函数的参数个数
//返回值:0,成功;其他,错误代码.
u8 usmart_get_fname(u8 *str, u8 *fname, u8 *pnum)
{
  u8 res;
  u8 fover = 0; //括号深度
  u8 *strtemp;
  u8 offset = 0;
  u8 parmnum = 0;
  u8 temp = 1;
  strtemp = str;

  res = 0;
  while (*strtemp != '(') //此代码找到函数名的真正起始位置,函数名后紧跟(,如果增加空格,将导致失败
  {
    strtemp++;
    res++;
    if (*strtemp == ' ')
      offset = res; //跳过空格.
  }
  strtemp = str;
  if (offset)
    strtemp += offset + 1; //跳到函数名开始的地方
  res = 0;
  while (1)
  {
    if (*strtemp == 0)
    {
      res = USMART_FUNCERR; //函数错误
      break;
    }
    else if (*strtemp == '(')
      fover++;                //括号深度增加一级
    else if (*strtemp == ')') //
    {
      if (fover)
        fover--;
      else
        res = USMART_FUNCERR; //错误结束,没收到'('
      if (fover == 0)
        break; //到末尾了,退出
    }
    if (fover == 0) //函数名还没接收完
    {
      *fname = *strtemp; //得到函数名
      fname++;
    }
    else //已经接受完了函数名了.
    {
      if (*strtemp == ',')
        temp = 1; //使能增加一个参数
      else if (*strtemp != ' ')
        temp++; //得到有效参数(非空格)
      if (fover == 1 && temp == 2)
      {
        temp++;    //防止重复增加
        parmnum++; //参数增加一个
      }
    }
    strtemp++;
  }
  *pnum = parmnum; //记录参数个数
  *fname = '\0';   //加入结束符
  return res;      //返回执行结果
}

//从str中得到一个函数的参数
//*str:源字符串指针
//*fparm:参数字符串指针
//*ptype:参数类型 0，数字;1，字符串;0XFF，参数错误
//返回值:0,已经无参数了;其他,下一个参数的偏移量.
u8 usmart_get_aparm(u8 *str, u8 *fparm, u8 *ptype)
{
  u8 i = 0;
  u8 enout = 0;
  u8 type = 0;   //默认是数字
  u8 string = 0; //标记str是否正在读
  while (1)
  {
    if (*str == ',' && string == 0)
      enout = 1; //暂缓立即退出,目的是寻找下一个参数的起始地址
    if ((*str == ')' || *str == '\0') && string == 0)
      break;       //立即退出标识符
    if (type == 0) //默认是数字的
    {
      if ((*str >= '0' && *str <= '9') || (*str >= 'a' && *str <= 'f') || (*str >= 'A' && *str <= 'F') || *str == 'X' || *str == 'x') //数字串检测
      {
        if (enout)
          break; //找到了下一个参数,直接退出.
        if (*str >= 'a')
          *fparm = *str - 0X20; //小写转换为大写
        else
          *fparm = *str; //小写或者数字保持不变
        fparm++;
      }
      else if (*str == '"') //找到字符串的开始标志
      {
        if (enout)
          break; //找到,后才找到",认为结束了.
        type = 1;
        string = 1; //登记STRING 正在读了
      }
      else if (*str != ' ' && *str != ',') //发现非法字符,参数错误
      {
        type = 0XFF;
        break;
      }
    }
    else //string类
    {
      if (*str == '"')
        string = 0;
      if (enout)
        break;    //找到了下一个参数,直接退出.
      if (string) //字符串正在读
      {
        *fparm = *str; //小写或者数字保持不变
        fparm++;
      }
    }
    i++; //偏移量增加
    str++;
  }
  *fparm = '\0'; //加入结束符
  *ptype = type; //返回参数类型
  return i;      //返回参数长度
}
//得到指定参数的起始地址
//num:第num个参数,范围0~9.
//返回值:该参数的起始地址
u8 usmart_get_parmpos(u8 num)
{
  u8 temp = 0;
  u8 i;
  for (i = 0; i < num; i++)
    temp += usmart_dev.plentbl[i];
  return temp;
}
//从str中得到函数参数
//str:源字符串;
//parn:参数的多少.0表示无参数 void类型
//返回值:0,成功;其他,错误代码.
u8 usmart_get_fparam(u8 *str, u8 *parn)
{
  u8 i, type;
  u32 res;
  u8 n = 0;
  u8 len;
  u8 tstr[PARM_LEN + 1]; //字节长度的缓存,最多可以存放PARM_LEN个字符的字符串
  for (i = 0; i < MAX_PARM; i++)
    usmart_dev.plentbl[i] = 0; //清空参数长度表
  while (*str != '(')          //偏移到参数开始的地方
  {
    str++;
    if (*str == '\0')
      return USMART_FUNCERR; //遇到结束符了
  }
  str++; //偏移到"("之后的第一个字节
  while (1)
  {
    i = usmart_get_aparm(str, tstr, &type); //得到第一个参数
    str += i;                               //偏移
    switch (type)
    {
    case 0:                //数字
      if (tstr[0] != '\0') //接收到的参数有效
      {
        i = usmart_str2num(tstr, &res); //记录该参数
        if (i)
          return USMART_PARMERR;                                 //参数错误.
        *(u32 *)(usmart_dev.parm + usmart_get_parmpos(n)) = res; //记录转换成功的结果.
        usmart_dev.parmtype &= ~(1 << n);                        //标记数字
        usmart_dev.plentbl[n] = 4;                               //该参数的长度为4
        n++;                                                     //参数增加
        if (n > MAX_PARM)
          return USMART_PARMOVER; //参数太多
      }
      break;
    case 1:                                                          //字符串
      len = usmart_strlen(tstr) + 1;                                 //包含了结束符'\0'
      usmart_strcopy(tstr, &usmart_dev.parm[usmart_get_parmpos(n)]); //拷贝tstr数据到usmart_dev.parm[n]
      usmart_dev.parmtype |= 1 << n;                                 //标记字符串
      usmart_dev.plentbl[n] = len;                                   //该参数的长度为len
      n++;
      if (n > MAX_PARM)
        return USMART_PARMOVER; //参数太多
      break;
    case 0XFF:               //错误
      return USMART_PARMERR; //参数错误
    }
    if (*str == ')' || *str == '\0')
      break; //查到结束标志了.
  }
  *parn = n;        //记录参数的个数
  return USMART_OK; //正确得到了参数
}

//USMART

//系统命令
u8 *sys_cmd_tab[] =
    {
        "?",
        "help",
        "ls",
        "id",
        "hex",
        "dec",
        "exit",
};
//处理系统指令
//0,成功处理;其他,错误代码;
u8 usmart_sys_cmd_exe(u8 *str)
{
  u8 i;
  u8 sfname[MAX_FNAME_LEN]; //存放本地函数名
  u8 pnum;
  u32 res;

  usmart_get_cmdname(str, sfname, &i); //得到指令及指令长度
  str += i;
  for (i = 0; i < 7; i++) //支持7个系统指令
  {
    if (usmart_strcmp(sfname, sys_cmd_tab[i]))
      break;
  }
  switch (i)
  {
  case 0:
  case 1: //帮助指令
    printf("\r\n");
#if USMART_USE_HELP
    printf("------------------------USMART V2.1------------------------ \r\n");
    printf("?:    获取帮助信息\r\n");
    printf("help: 获取帮助信息\r\n");
    printf("ls:   可用的函数列表\r\n\n");
    printf("id:   可用函数的ID列表\r\n\n");
    printf("hex:  参数16进制显示,后跟空格+数字即执行进制转换\r\n\n");
    printf("dec:  参数10进制显示,后跟空格+数字即执行进制转换\r\n\n");
    printf("exit: 退出\r\n\n");
    printf("--------------------------ALIENTEK------------------------- \r\n");
#else
    printf("指令失效\r\n");
#endif
    break;
  case 2: //查询指令
    printf("\r\n");
    printf("-------------------------函数清单--------------------------- \r\n");
    for (i = 0; i < usmart_dev.fnum; i++)
      printf("%s\r\n", usmart_dev.funs[i].name);
    printf("\r\n");
    break;
  case 3: //查询ID
    printf("\r\n");
    printf("-------------------------函数 ID --------------------------- \r\n");
    for (i = 0; i < usmart_dev.fnum; i++)
    {
      usmart_get_fname((u8 *)usmart_dev.funs[i].name, sfname, &pnum);                   //得到本地函数名
      printf("%s id is:\r\n0X%08X\r\n", sfname, (unsigned int)usmart_dev.funs[i].func); //显示ID
    }
    printf("\r\n");
    break;
  case 4:
    printf("\r\n");
    usmart_get_aparm(str, sfname, &i);
    if (i == 0) //参数正常
    {
      i = usmart_str2num(sfname, &res); //记录该参数
      if (i == 0)                       //进制转换功能
      {
        printf("HEX:0X%X\r\n", res); //转为16进制
      }
      else if (i != 4)
        return USMART_PARMERR; //参数错误.
      else                     //参数显示设定功能
      {
        printf("16进制参数显示!\r\n");
        usmart_dev.sptype = SP_TYPE_HEX;
      }
    }
    else
      return USMART_PARMERR; //参数错误.
    printf("\r\n");
    break;
  case 5:
    printf("\r\n");
    usmart_get_aparm(str, sfname, &i);
    if (i == 0) //参数正常
    {
      i = usmart_str2num(sfname, &res); //记录该参数
      if (i == 0)                       //进制转换功能
      {
        printf("DEC:%lu\r\n", (unsigned long)res); //转为10进制
      }
      else if (i != 4)
        return USMART_PARMERR; //参数错误.
      else                     //参数显示设定功能
      {
        printf("10进制参数显示!\r\n");
        usmart_dev.sptype = SP_TYPE_DEC;
      }
      return USMART_PARMERR; //参数错误.
    }
    else
      printf("\r\n");
    break;
  case 6:
    ExitUsmart();
    break;
  default: //非法指令
    return USMART_FUNCERR;
  }
  return 0;
}

//从str中获取函数名,id,及参数信息
//*str:字符串指针.
//返回值:0,识别成功;其他,错误代码.
u8 usmart_cmd_rec(u8 *str)
{
  u8 sta, i; //状态
  u8 rpnum, spnum;
  u8 rfname[MAX_FNAME_LEN];                    //暂存空间,用于存放接收到的函数名
  u8 sfname[MAX_FNAME_LEN];                    //存放本地函数名
  sta = usmart_get_fname(str, rfname, &rpnum); //得到接收到的数据的函数名及参数个数
  if (sta)
    return sta; //错误
  for (i = 0; i < usmart_dev.fnum; i++)
  {
    sta = usmart_get_fname((u8 *)usmart_dev.funs[i].name, sfname, &spnum); //得到本地函数名及参数个数
    if (sta)
      return sta;                      //本地解析有误
    if (usmart_strcmp(sfname, rfname)) //相等
    {
      if (spnum > rpnum)
        return USMART_PARMERR; //参数错误(输入参数比源函数参数少)
      usmart_dev.id = i;       //记录函数ID.
      break;                   //跳出.
    }
  }
  if (i == usmart_dev.fnum)
    return USMART_NOFUNCFIND;       //未找到匹配的函数
  sta = usmart_get_fparam(str, &i); //得到函数参数个数
  if (sta)
    return sta;        //返回错误
  usmart_dev.pnum = i; //参数个数记录
  return USMART_OK;
}
//usamrt执行函数
//该函数用于最终执行从串口收到的有效函数.
//最多支持10个参数的函数,更多的参数支持也很容易实现.不过用的很少.一般5个左右的参数的函数已经很少见了.
//该函数会在串口打印执行情况.以:"函数名(参数1，参数2...参数N)=返回值".的形式打印.
//当所执行的函数没有返回值的时候,所打印的返回值是一个无意义的数据.
void usmart_exe(void)
{
  u8 id, i;
  u32 res;
  u32 temp[MAX_PARM];       //参数转换,使之支持了字符串
  u8 sfname[MAX_FNAME_LEN]; //存放本地函数名
  u8 pnum;
  id = usmart_dev.id;
  if (id >= usmart_dev.fnum)
    return;                                                        //不执行.
  usmart_get_fname((u8 *)usmart_dev.funs[id].name, sfname, &pnum); //得到本地函数名,及参数个数
  printf("\r\n%s(", sfname);                                       //输出正要执行的函数名
  for (i = 0; i < pnum; i++)                                       //输出参数
  {
    if (usmart_dev.parmtype & (1 << i)) //参数是字符串
    {
      printf("%c", '"');
      printf("%s", usmart_dev.parm + usmart_get_parmpos(i));
      printf("%c", '"');
      temp[i] = (u32) & (usmart_dev.parm[usmart_get_parmpos(i)]);
    }
    else //参数是数字
    {
      temp[i] = *(u32 *)(usmart_dev.parm + usmart_get_parmpos(i));
      if (usmart_dev.sptype == SP_TYPE_DEC)
        printf("%lu", (unsigned long)temp[i]); //10进制参数显示
      else
        printf("0X%X", temp[i]); //16进制参数显示
    }
    if (i != pnum - 1)
      printf(",");
  }
  printf(")");
  switch (usmart_dev.pnum)
  {
  case 0: //无参数(void类型)
    res = (*(u32(*)())usmart_dev.funs[id].func)();
    break;
  case 1: //有1个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0]);
    break;
  case 2: //有2个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1]);
    break;
  case 3: //有3个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2]);
    break;
  case 4: //有4个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3]);
    break;
  case 5: //有5个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4]);
    break;
  case 6: //有6个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5]);
    break;
  case 7: //有7个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6]);
    break;
  case 8: //有8个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7]);
    break;
  case 9: //有9个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7], temp[8]);
    break;
  case 10: //有10个参数
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7], temp[8], temp[9]);
    break;
  }
  if (usmart_dev.sptype == SP_TYPE_DEC)
    printf("=%d;\r\n", (int)res); //输出执行结果(10进制参数显示)
  else
    printf("=0X%X", res); //输出执行结果(16进制参数显示)
}
//usmart扫描函数
//通过调用该函数,实现usmart的各个控制.该函数需要每隔一定时间被调用一次
//以及时执行从串口发过来的各个函数.
//本函数可以在中断里面调用,从而实现自动管理.
//如果非ALIENTEK用户,则USART_RX_STA和USART_RX_BUF[]需要用户自己实现
void usmart_scan()
{
  u8 sta;
  u8 len = -1;
  memset(&USART_RX_BUF,0,sizeof(USART_RX_BUF));
  while (uart1_rx_buff[++len] != '\0') //得到此次接收到的数据长度
  {
    USART_RX_BUF[len] = uart1_rx_buff[len];
  }
  sta = usmart_dev.cmd_rec(USART_RX_BUF); //得到函数各个信息
  if (sta == 0)
    usmart_dev.exe(); //执行函数
  else
  {
    len = usmart_sys_cmd_exe(USART_RX_BUF);
    if (len != USMART_FUNCERR)
      sta = len;
    if (sta)
    {
      switch (sta)
      {
      case USMART_FUNCERR:
        printf("函数错误!\r\n");
        break;
      case USMART_PARMERR:
        printf("参数错误!\r\n");
        break;
      case USMART_PARMOVER:
        printf("参数太多!\r\n");
        break;
      case USMART_NOFUNCFIND:
        printf("未找到匹配的函数!\r\n");
        break;
      }
    }
  }
}

