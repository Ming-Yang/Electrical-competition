#include "usmart.h"
//CubeMX ��������ʹ��
//��������ԭ��USMART�޸�

//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2011/6/13
//�汾��V2.1
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2011-2021
//All rights reserved
//********************************************************************************
//����˵��
//V1.4
//�����˶Բ���Ϊstring���͵ĺ�����֧��.���÷�Χ������.
//�Ż����ڴ�ռ��,��̬�ڴ�ռ��Ϊ79���ֽ�@10������.��̬��Ӧ���ּ��ַ�������
//V2.0
//1,�޸���listָ��,��ӡ�������������ʽ.
//2,������idָ��,��ӡÿ����������ڵ�ַ.
//3,�޸��˲���ƥ��,֧�ֺ��������ĵ���(������ڵ�ַ).
//4,�����˺��������Ⱥ궨��.
//V2.1 20110707
//1,����dec,hex����ָ��,�������ò�����ʾ����,��ִ�н���ת��.
//ע:��dec,hex����������ʱ��,���趨��ʾ��������.�����������ʱ��,��ִ�н���ת��.
//��:"dec 0XFF" ��Ὣ0XFFתΪ255,�ɴ��ڷ���.
//��:"hex 100" 	��Ὣ100תΪ0X64,�ɴ��ڷ���
//2,����usmart_get_cmdname����,���ڻ�ȡָ������.
/////////////////////////////////////////////////////////////////////////////////////

u8 USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.

//�ַ�������
//�Ա��ַ���str1��str2
//*str1:�ַ���1ָ��
//*str2:�ַ���2ָ��
//����ֵ:0�������;1�����;
u8 usmart_strcmp(u8 *str1, u8 *str2)
{
  while (1)
  {
    if (*str1 != *str2)
      return 0; //�����
    if (*str1 == '\0')
      break; //�Ա������.
    str1++;
    str2++;
  }
  return 1; //�����ַ������
}
//��str1������copy��str2
//*str1:�ַ���1ָ��
//*str2:�ַ���2ָ��
void usmart_strcopy(u8 *str1, u8 *str2)
{
  while (1)
  {
    *str2 = *str1; //����
    if (*str1 == '\0')
      break; //���������.
    str1++;
    str2++;
  }
}
//�õ��ַ����ĳ���(�ֽ�)
//*str:�ַ���ָ��
//����ֵ:�ַ����ĳ���
u8 usmart_strlen(u8 *str)
{
  u8 len = 0;
  while (1)
  {
    if (*str == '\0')
      break; //���������.
    len++;
    str++;
  }
  return len;
}
//m^n����
//����ֵ:m^n�η�
u32 usmart_pow(u8 m, u8 n)
{
  u32 result = 1;
  while (n--)
    result *= m;
  return result;
}
//���ַ���תΪ����
//֧��16����ת��,����16������ĸ�����Ǵ�д��,�Ҹ�ʽΪ��0X��ͷ��.
//��֧�ָ���
//*str:�����ַ���ָ��
//*res:ת����Ľ����ŵ�ַ.
//����ֵ:0���ɹ�ת�����.����,�������.
//1,���ݸ�ʽ����.2,16����λ��Ϊ0.3,��ʼ��ʽ����.4,ʮ����λ��Ϊ0.
u8 usmart_str2num(u8 *str, u32 *res)
{
  u32 t;
  u8 bnum = 0; //���ֵ�λ��
  u8 *p;
  u8 hexdec = 10; //Ĭ��Ϊʮ��������
  p = str;
  *res = 0; //����.
  while (1)
  {
    if ((*p <= '9' && *p >= '0') || (*p <= 'F' && *p >= 'A') || (*p == 'X' && bnum == 1)) //�����Ϸ�
    {
      if (*p >= 'A')
        hexdec = 16; //�ַ����д�����ĸ,Ϊ16���Ƹ�ʽ.
      bnum++;        //λ������.
    }
    else if (*p == '\0')
      break; //����������,�˳�.
    else
      return 1; //��ȫ��ʮ���ƻ���16��������.
    p++;
  }
  p = str;          //���¶�λ���ַ�����ʼ�ĵ�ַ.
  if (hexdec == 16) //16��������
  {
    if (bnum < 3)
      return 2;                         //λ��С��3��ֱ���˳�.��Ϊ0X��ռ��2��,���0X���治������,������ݷǷ�.
    if (*p == '0' && (*(p + 1) == 'X')) //������'0X'��ͷ.
    {
      p += 2;    //ƫ�Ƶ�������ʼ��ַ.
      bnum -= 2; //��ȥƫ����
    }
    else
      return 3; //��ʼͷ�ĸ�ʽ����
  }
  else if (bnum == 0)
    return 4; //λ��Ϊ0��ֱ���˳�.
  while (1)
  {
    if (bnum)
      bnum--;
    if (*p <= '9' && *p >= '0')
      t = *p - '0'; //�õ����ֵ�ֵ
    else
      t = *p - 'A' + 10; //�õ�A~F��Ӧ��ֵ
    *res += t * usmart_pow(hexdec, bnum);
    p++;
    if (*p == '\0')
      break; //���ݶ�������.
  }
  return 0; //�ɹ�ת��
}
//�õ�ָ����
//*str:Դ�ַ���
//*cmdname:ָ����
//*nlen:ָ��������
void usmart_get_cmdname(u8 *str, u8 *cmdname, u8 *nlen)
{
  *nlen = 0;
  while (*str != ' ' && *str != '\0') //�ҵ��ո���߽���������Ϊ������
  {
    *cmdname = *str;
    str++;
    cmdname++;
    (*nlen)++; //ͳ�������
  }
  *cmdname = '\0'; //���������
}
//��str�еõ�������
//*str:Դ�ַ���ָ��
//*fname:��ȡ���ĺ�������ָ��
//pnum:�����Ĳ�������
//����ֵ:0,�ɹ�;����,�������.
u8 usmart_get_fname(u8 *str, u8 *fname, u8 *pnum)
{
  u8 res;
  u8 fover = 0; //�������
  u8 *strtemp;
  u8 offset = 0;
  u8 parmnum = 0;
  u8 temp = 1;
  strtemp = str;

  res = 0;
  while (*strtemp != '(') //�˴����ҵ���������������ʼλ��,�����������(,������ӿո�,������ʧ��
  {
    strtemp++;
    res++;
    if (*strtemp == ' ')
      offset = res; //�����ո�.
  }
  strtemp = str;
  if (offset)
    strtemp += offset + 1; //������������ʼ�ĵط�
  res = 0;
  while (1)
  {
    if (*strtemp == 0)
    {
      res = USMART_FUNCERR; //��������
      break;
    }
    else if (*strtemp == '(')
      fover++;                //�����������һ��
    else if (*strtemp == ')') //
    {
      if (fover)
        fover--;
      else
        res = USMART_FUNCERR; //�������,û�յ�'('
      if (fover == 0)
        break; //��ĩβ��,�˳�
    }
    if (fover == 0) //��������û������
    {
      *fname = *strtemp; //�õ�������
      fname++;
    }
    else //�Ѿ��������˺�������.
    {
      if (*strtemp == ',')
        temp = 1; //ʹ������һ������
      else if (*strtemp != ' ')
        temp++; //�õ���Ч����(�ǿո�)
      if (fover == 1 && temp == 2)
      {
        temp++;    //��ֹ�ظ�����
        parmnum++; //��������һ��
      }
    }
    strtemp++;
  }
  *pnum = parmnum; //��¼��������
  *fname = '\0';   //���������
  return res;      //����ִ�н��
}

//��str�еõ�һ�������Ĳ���
//*str:Դ�ַ���ָ��
//*fparm:�����ַ���ָ��
//*ptype:�������� 0������;1���ַ���;0XFF����������
//����ֵ:0,�Ѿ��޲�����;����,��һ��������ƫ����.
u8 usmart_get_aparm(u8 *str, u8 *fparm, u8 *ptype)
{
  u8 i = 0;
  u8 enout = 0;
  u8 type = 0;   //Ĭ��������
  u8 string = 0; //���str�Ƿ����ڶ�
  while (1)
  {
    if (*str == ',' && string == 0)
      enout = 1; //�ݻ������˳�,Ŀ����Ѱ����һ����������ʼ��ַ
    if ((*str == ')' || *str == '\0') && string == 0)
      break;       //�����˳���ʶ��
    if (type == 0) //Ĭ�������ֵ�
    {
      if ((*str >= '0' && *str <= '9') || (*str >= 'a' && *str <= 'f') || (*str >= 'A' && *str <= 'F') || *str == 'X' || *str == 'x') //���ִ����
      {
        if (enout)
          break; //�ҵ�����һ������,ֱ���˳�.
        if (*str >= 'a')
          *fparm = *str - 0X20; //Сдת��Ϊ��д
        else
          *fparm = *str; //Сд�������ֱ��ֲ���
        fparm++;
      }
      else if (*str == '"') //�ҵ��ַ����Ŀ�ʼ��־
      {
        if (enout)
          break; //�ҵ�,����ҵ�",��Ϊ������.
        type = 1;
        string = 1; //�Ǽ�STRING ���ڶ���
      }
      else if (*str != ' ' && *str != ',') //���ַǷ��ַ�,��������
      {
        type = 0XFF;
        break;
      }
    }
    else //string��
    {
      if (*str == '"')
        string = 0;
      if (enout)
        break;    //�ҵ�����һ������,ֱ���˳�.
      if (string) //�ַ������ڶ�
      {
        *fparm = *str; //Сд�������ֱ��ֲ���
        fparm++;
      }
    }
    i++; //ƫ��������
    str++;
  }
  *fparm = '\0'; //���������
  *ptype = type; //���ز�������
  return i;      //���ز�������
}
//�õ�ָ����������ʼ��ַ
//num:��num������,��Χ0~9.
//����ֵ:�ò�������ʼ��ַ
u8 usmart_get_parmpos(u8 num)
{
  u8 temp = 0;
  u8 i;
  for (i = 0; i < num; i++)
    temp += usmart_dev.plentbl[i];
  return temp;
}
//��str�еõ���������
//str:Դ�ַ���;
//parn:�����Ķ���.0��ʾ�޲��� void����
//����ֵ:0,�ɹ�;����,�������.
u8 usmart_get_fparam(u8 *str, u8 *parn)
{
  u8 i, type;
  u32 res;
  u8 n = 0;
  u8 len;
  u8 tstr[PARM_LEN + 1]; //�ֽڳ��ȵĻ���,�����Դ��PARM_LEN���ַ����ַ���
  for (i = 0; i < MAX_PARM; i++)
    usmart_dev.plentbl[i] = 0; //��ղ������ȱ�
  while (*str != '(')          //ƫ�Ƶ�������ʼ�ĵط�
  {
    str++;
    if (*str == '\0')
      return USMART_FUNCERR; //������������
  }
  str++; //ƫ�Ƶ�"("֮��ĵ�һ���ֽ�
  while (1)
  {
    i = usmart_get_aparm(str, tstr, &type); //�õ���һ������
    str += i;                               //ƫ��
    switch (type)
    {
    case 0:                //����
      if (tstr[0] != '\0') //���յ��Ĳ�����Ч
      {
        i = usmart_str2num(tstr, &res); //��¼�ò���
        if (i)
          return USMART_PARMERR;                                 //��������.
        *(u32 *)(usmart_dev.parm + usmart_get_parmpos(n)) = res; //��¼ת���ɹ��Ľ��.
        usmart_dev.parmtype &= ~(1 << n);                        //�������
        usmart_dev.plentbl[n] = 4;                               //�ò����ĳ���Ϊ4
        n++;                                                     //��������
        if (n > MAX_PARM)
          return USMART_PARMOVER; //����̫��
      }
      break;
    case 1:                                                          //�ַ���
      len = usmart_strlen(tstr) + 1;                                 //�����˽�����'\0'
      usmart_strcopy(tstr, &usmart_dev.parm[usmart_get_parmpos(n)]); //����tstr���ݵ�usmart_dev.parm[n]
      usmart_dev.parmtype |= 1 << n;                                 //����ַ���
      usmart_dev.plentbl[n] = len;                                   //�ò����ĳ���Ϊlen
      n++;
      if (n > MAX_PARM)
        return USMART_PARMOVER; //����̫��
      break;
    case 0XFF:               //����
      return USMART_PARMERR; //��������
    }
    if (*str == ')' || *str == '\0')
      break; //�鵽������־��.
  }
  *parn = n;        //��¼�����ĸ���
  return USMART_OK; //��ȷ�õ��˲���
}

//USMART

//ϵͳ����
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
//����ϵͳָ��
//0,�ɹ�����;����,�������;
u8 usmart_sys_cmd_exe(u8 *str)
{
  u8 i;
  u8 sfname[MAX_FNAME_LEN]; //��ű��غ�����
  u8 pnum;
  u32 res;

  usmart_get_cmdname(str, sfname, &i); //�õ�ָ�ָ���
  str += i;
  for (i = 0; i < 7; i++) //֧��7��ϵͳָ��
  {
    if (usmart_strcmp(sfname, sys_cmd_tab[i]))
      break;
  }
  switch (i)
  {
  case 0:
  case 1: //����ָ��
    printf("\r\n");
#if USMART_USE_HELP
    printf("------------------------USMART V2.1------------------------ \r\n");
    printf("?:    ��ȡ������Ϣ\r\n");
    printf("help: ��ȡ������Ϣ\r\n");
    printf("ls:   ���õĺ����б�\r\n\n");
    printf("id:   ���ú�����ID�б�\r\n\n");
    printf("hex:  ����16������ʾ,����ո�+���ּ�ִ�н���ת��\r\n\n");
    printf("dec:  ����10������ʾ,����ո�+���ּ�ִ�н���ת��\r\n\n");
    printf("exit: �˳�\r\n\n");
    printf("--------------------------ALIENTEK------------------------- \r\n");
#else
    printf("ָ��ʧЧ\r\n");
#endif
    break;
  case 2: //��ѯָ��
    printf("\r\n");
    printf("-------------------------�����嵥--------------------------- \r\n");
    for (i = 0; i < usmart_dev.fnum; i++)
      printf("%s\r\n", usmart_dev.funs[i].name);
    printf("\r\n");
    break;
  case 3: //��ѯID
    printf("\r\n");
    printf("-------------------------���� ID --------------------------- \r\n");
    for (i = 0; i < usmart_dev.fnum; i++)
    {
      usmart_get_fname((u8 *)usmart_dev.funs[i].name, sfname, &pnum);                   //�õ����غ�����
      printf("%s id is:\r\n0X%08X\r\n", sfname, (unsigned int)usmart_dev.funs[i].func); //��ʾID
    }
    printf("\r\n");
    break;
  case 4:
    printf("\r\n");
    usmart_get_aparm(str, sfname, &i);
    if (i == 0) //��������
    {
      i = usmart_str2num(sfname, &res); //��¼�ò���
      if (i == 0)                       //����ת������
      {
        printf("HEX:0X%X\r\n", res); //תΪ16����
      }
      else if (i != 4)
        return USMART_PARMERR; //��������.
      else                     //������ʾ�趨����
      {
        printf("16���Ʋ�����ʾ!\r\n");
        usmart_dev.sptype = SP_TYPE_HEX;
      }
    }
    else
      return USMART_PARMERR; //��������.
    printf("\r\n");
    break;
  case 5:
    printf("\r\n");
    usmart_get_aparm(str, sfname, &i);
    if (i == 0) //��������
    {
      i = usmart_str2num(sfname, &res); //��¼�ò���
      if (i == 0)                       //����ת������
      {
        printf("DEC:%lu\r\n", (unsigned long)res); //תΪ10����
      }
      else if (i != 4)
        return USMART_PARMERR; //��������.
      else                     //������ʾ�趨����
      {
        printf("10���Ʋ�����ʾ!\r\n");
        usmart_dev.sptype = SP_TYPE_DEC;
      }
      return USMART_PARMERR; //��������.
    }
    else
      printf("\r\n");
    break;
  case 6:
    ExitUsmart();
    break;
  default: //�Ƿ�ָ��
    return USMART_FUNCERR;
  }
  return 0;
}

//��str�л�ȡ������,id,��������Ϣ
//*str:�ַ���ָ��.
//����ֵ:0,ʶ��ɹ�;����,�������.
u8 usmart_cmd_rec(u8 *str)
{
  u8 sta, i; //״̬
  u8 rpnum, spnum;
  u8 rfname[MAX_FNAME_LEN];                    //�ݴ�ռ�,���ڴ�Ž��յ��ĺ�����
  u8 sfname[MAX_FNAME_LEN];                    //��ű��غ�����
  sta = usmart_get_fname(str, rfname, &rpnum); //�õ����յ������ݵĺ���������������
  if (sta)
    return sta; //����
  for (i = 0; i < usmart_dev.fnum; i++)
  {
    sta = usmart_get_fname((u8 *)usmart_dev.funs[i].name, sfname, &spnum); //�õ����غ���������������
    if (sta)
      return sta;                      //���ؽ�������
    if (usmart_strcmp(sfname, rfname)) //���
    {
      if (spnum > rpnum)
        return USMART_PARMERR; //��������(���������Դ����������)
      usmart_dev.id = i;       //��¼����ID.
      break;                   //����.
    }
  }
  if (i == usmart_dev.fnum)
    return USMART_NOFUNCFIND;       //δ�ҵ�ƥ��ĺ���
  sta = usmart_get_fparam(str, &i); //�õ�������������
  if (sta)
    return sta;        //���ش���
  usmart_dev.pnum = i; //����������¼
  return USMART_OK;
}
//usamrtִ�к���
//�ú�����������ִ�дӴ����յ�����Ч����.
//���֧��10�������ĺ���,����Ĳ���֧��Ҳ������ʵ��.�����õĺ���.һ��5�����ҵĲ����ĺ����Ѿ����ټ���.
//�ú������ڴ��ڴ�ӡִ�����.��:"������(����1������2...����N)=����ֵ".����ʽ��ӡ.
//����ִ�еĺ���û�з���ֵ��ʱ��,����ӡ�ķ���ֵ��һ�������������.
void usmart_exe(void)
{
  u8 id, i;
  u32 res;
  u32 temp[MAX_PARM];       //����ת��,ʹ֧֮�����ַ���
  u8 sfname[MAX_FNAME_LEN]; //��ű��غ�����
  u8 pnum;
  id = usmart_dev.id;
  if (id >= usmart_dev.fnum)
    return;                                                        //��ִ��.
  usmart_get_fname((u8 *)usmart_dev.funs[id].name, sfname, &pnum); //�õ����غ�����,����������
  printf("\r\n%s(", sfname);                                       //�����Ҫִ�еĺ�����
  for (i = 0; i < pnum; i++)                                       //�������
  {
    if (usmart_dev.parmtype & (1 << i)) //�������ַ���
    {
      printf("%c", '"');
      printf("%s", usmart_dev.parm + usmart_get_parmpos(i));
      printf("%c", '"');
      temp[i] = (u32) & (usmart_dev.parm[usmart_get_parmpos(i)]);
    }
    else //����������
    {
      temp[i] = *(u32 *)(usmart_dev.parm + usmart_get_parmpos(i));
      if (usmart_dev.sptype == SP_TYPE_DEC)
        printf("%lu", (unsigned long)temp[i]); //10���Ʋ�����ʾ
      else
        printf("0X%X", temp[i]); //16���Ʋ�����ʾ
    }
    if (i != pnum - 1)
      printf(",");
  }
  printf(")");
  switch (usmart_dev.pnum)
  {
  case 0: //�޲���(void����)
    res = (*(u32(*)())usmart_dev.funs[id].func)();
    break;
  case 1: //��1������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0]);
    break;
  case 2: //��2������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1]);
    break;
  case 3: //��3������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2]);
    break;
  case 4: //��4������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3]);
    break;
  case 5: //��5������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4]);
    break;
  case 6: //��6������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5]);
    break;
  case 7: //��7������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6]);
    break;
  case 8: //��8������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7]);
    break;
  case 9: //��9������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7], temp[8]);
    break;
  case 10: //��10������
    res = (*(u32(*)())usmart_dev.funs[id].func)(temp[0], temp[1], temp[2], temp[3], temp[4],
                                                temp[5], temp[6], temp[7], temp[8], temp[9]);
    break;
  }
  if (usmart_dev.sptype == SP_TYPE_DEC)
    printf("=%d;\r\n", (int)res); //���ִ�н��(10���Ʋ�����ʾ)
  else
    printf("=0X%X", res); //���ִ�н��(16���Ʋ�����ʾ)
}
//usmartɨ�躯��
//ͨ�����øú���,ʵ��usmart�ĸ�������.�ú�����Ҫÿ��һ��ʱ�䱻����һ��
//�Լ�ʱִ�дӴ��ڷ������ĸ�������.
//�������������ж��������,�Ӷ�ʵ���Զ�����.
//�����ALIENTEK�û�,��USART_RX_STA��USART_RX_BUF[]��Ҫ�û��Լ�ʵ��
void usmart_scan()
{
  u8 sta;
  u8 len = -1;
  memset(&USART_RX_BUF,0,sizeof(USART_RX_BUF));
  while (uart1_rx_buff[++len] != '\0') //�õ��˴ν��յ������ݳ���
  {
    USART_RX_BUF[len] = uart1_rx_buff[len];
  }
  sta = usmart_dev.cmd_rec(USART_RX_BUF); //�õ�����������Ϣ
  if (sta == 0)
    usmart_dev.exe(); //ִ�к���
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
        printf("��������!\r\n");
        break;
      case USMART_PARMERR:
        printf("��������!\r\n");
        break;
      case USMART_PARMOVER:
        printf("����̫��!\r\n");
        break;
      case USMART_NOFUNCFIND:
        printf("δ�ҵ�ƥ��ĺ���!\r\n");
        break;
      }
    }
  }
}

