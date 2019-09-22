/** \file max30102.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: max30102.cpp
* Description: This module is an embedded controller driver for the MAX30102
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/
//#include "mbed.h"
#include "MAX30102.h"
#include "myiic.h"

//I2C i2c(I2C_SDA, I2C_SCL);//SDA-PB9,SCL-PB8

void max30102_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//INT引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;   //悬空输入，因为MAX30102的INT已经有上拉电阻
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}
int8_t maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
/**
* \brief        Write a value to a MAX30102 register
* \par          Details
*               This function writes a value to a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[in]    uch_data    - register data
*
* \retval       true on success
*/
{
    //char ach_i2c_data[2];
    //u16 data_temp = 0;
    //u16 data_temp1 = 0;
    uint8_t Ack1,Ack2,Ack3;//Ack4;
    //Ack1=Ack2=Ack3=Ack4=2;
//  ach_i2c_data[0]=uch_addr;
//  ach_i2c_data[1]=uch_data;



    IIC_Start();
    IIC_Send_Byte(I2C_WRITE_ADDR);//发送设备写地址
    Ack1 = IIC_Wait_Ack();

    IIC_Send_Byte(uch_addr);	//发送寄存器地址
    Ack2 = IIC_Wait_Ack();

    IIC_Send_Byte(uch_data);	//发送数据
    Ack3 = IIC_Wait_Ack();

    IIC_Stop();//产生一个停止条件

    if(Ack1 || Ack2 || Ack3)//如果
        return -1; //发送失败
    else
        return 0; //发送成功
//  if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 2, false)==0)
//    return true;
//  else
//    return false;
}

int8_t maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
/**
* \brief        Read a MAX30102 register
* \par          Details
*               This function reads a MAX30102 register
*
* \param[in]    uch_addr    - register address
* \param[out]   puch_data    - pointer that stores the register data
*
* \retval       true on success
*/
{
    //u16 data_temp = 0;
    //u16 data_temp1 = 0;
    uint8_t Ack1,Ack2,Ack3;//Ack4;
    //Ack1=Ack2=Ack3=Ack4=2;

    IIC_Start();
    IIC_Send_Byte(I2C_WRITE_ADDR);	//发送设备写地址
    Ack1 = IIC_Wait_Ack();
    IIC_Send_Byte(uch_addr);	//发送寄存器地址
    Ack2 = IIC_Wait_Ack();

    IIC_Start();
    IIC_Send_Byte(I2C_READ_ADDR);//发送设备读地址
    Ack3 = IIC_Wait_Ack();
    *puch_data = IIC_Read_Byte(0);//IIC读取字节

    IIC_Stop();//产?桓鐾Ｖ固跫

    if(Ack1 || Ack2 || Ack3)//如果
        return -1; //发送失败
    else
        return 0; //发送成功


//  char ch_i2c_data;
//  ch_i2c_data=uch_addr;
//  if(i2c.write(I2C_WRITE_ADDR, &ch_i2c_data, 1, true)!=0)
//    return false;
//  if(i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1, false)==0)
//  {
//    *puch_data=(uint8_t) ch_i2c_data;
//    return true;
//  }
//  else
//    return false;
}

int8_t maxim_max30102_init(void)
/**
* \brief        Initialize the MAX30102
* \par          Details
*               This function initializes the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    max30102_GPIO_Init();//初始化INT引脚
    if(maxim_max30102_write_reg(REG_INTR_ENABLE_1,0xc0)) // INTR setting
        return -1;
    if(maxim_max30102_write_reg(REG_INTR_ENABLE_2,0x00))
        return -1;
    if(maxim_max30102_write_reg(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
        return -1;
    if(maxim_max30102_write_reg(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
        return -1;
    if(maxim_max30102_write_reg(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
        return -1;
    if(maxim_max30102_write_reg(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=-1, fifo almost full = 17
        return -1;
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
        return -1;
    if(maxim_max30102_write_reg(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
        return -1;

    if(maxim_max30102_write_reg(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
        return -1;
    if(maxim_max30102_write_reg(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
        return -1;
    if(maxim_max30102_write_reg(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
        return -1;
    return 0;  //返回成功
}

int8_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
/**
* \brief        Read a set of samples from the MAX30102 FIFO register
* \par          Details
*               This function reads a set of samples from the MAX30102 FIFO register
*
* \param[out]   *pun_red_led   - pointer that stores the red LED reading data
* \param[out]   *pun_ir_led    - pointer that stores the IR LED reading data
*
* \retval       true on success
*/
{
    uint32_t un_temp;
    unsigned char uch_temp;
    char ach_i2c_data[6];
    uint8_t Ack1,Ack2,Ack3;//,Ack4;
    *pun_red_led=0;
    *pun_ir_led=0;


    //read and clear status register
    maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
    maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);

    ach_i2c_data[0]=REG_FIFO_DATA;


    IIC_Start();
    IIC_Send_Byte(I2C_WRITE_ADDR);	//发送设备写地址
    Ack1 = IIC_Wait_Ack();
    IIC_Send_Byte(ach_i2c_data[0]);	//发送寄存器地址
    Ack2 = IIC_Wait_Ack();

//  if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 1, true)!=0)
//    return false;

    IIC_Start();
    IIC_Send_Byte(I2C_READ_ADDR);	//发送设备读地址
    Ack3 = IIC_Wait_Ack();


//  if(i2c.read(I2C_READ_ADDR, ach_i2c_data, 6, false)!=0)
//  {
//    return false;
//  }




    //un_temp=(unsigned char) ach_i2c_data[0];
    un_temp = IIC_Read_Byte(0);//读取
    un_temp<<=16;
    *pun_red_led+=un_temp;
    //un_temp=(unsigned char) ach_i2c_data[1];
    un_temp = IIC_Read_Byte(0);//读取
    un_temp<<=8;
    *pun_red_led+=un_temp;
    //un_temp=(unsigned char) ach_i2c_data[2];
    un_temp = IIC_Read_Byte(0);//读取
    *pun_red_led+=un_temp;

// un_temp=(unsigned char) ach_i2c_data[3];
    un_temp = IIC_Read_Byte(0);//读取
    un_temp<<=16;
    *pun_ir_led+=un_temp;
    //un_temp=(unsigned char) ach_i2c_data[4];
    un_temp = IIC_Read_Byte(0);//读取
    un_temp<<=8;
    *pun_ir_led+=un_temp;
    //un_temp=(unsigned char) ach_i2c_data[5];
    un_temp = IIC_Read_Byte(0);//读取
    IIC_Stop();//产生停止

    *pun_ir_led+=un_temp;
    *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
    *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]

    if(Ack1 || Ack2 || Ack3)//如果
        return -1; //发送失败
    else
        return 0; //发送成功

}

int8_t maxim_max30102_reset()
/**
* \brief        Reset the MAX30102
* \par          Details
*               This function resets the MAX30102
*
* \param        None
*
* \retval       true on success
*/
{
    if(maxim_max30102_write_reg(REG_MODE_CONFIG,0x40))
        return -1;
    else
        return 0; //返回成功
}
