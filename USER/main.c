
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "myiic.h"
#include "MAX30102.h"
#include "algorithm.h"

#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;



 int main(void)
 { 
    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int i;
    int32_t n_brightness;
    float f_temp;
    
    //uint8_t IIC_Flag=1;//IIC通信状态，0为成功1为失败
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);	 	//串口初始化为9600
    IIC_Init();
    
    
    if(maxim_max30102_reset())//复位 MAX30102
        printf("max30102_reset failed!\r\n");
    if(maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy))//read and clear status register
        printf("read_reg REG_INTR_STATUS_1 failed!\r\n");
    if(maxim_max30102_init())//初始化MAX30102
        printf("max30102_init failed!\r\n");
    
    //printf("指示灯亮了吗？\r\n");
    
    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
    
    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    
    printf("采集500个样本\r\n");
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(max30102_INTPin==1);   //等待MAX30102中断引脚拉低

        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max
            
        printf("red=");
        printf("%i", aun_red_buffer[i]);
        printf(", ir=");
        printf("%i\r\n", aun_ir_buffer[i]);

    }
    un_prev_data=aun_red_buffer[i];
    
    
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
        i=0;
        un_min=0x3FFFF;
        un_max=0;
        
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
        
        //take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(max30102_INTPin==1);   //等待MAX30102中断引脚拉低
            
            maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
        
            if(aun_red_buffer[i]>un_prev_data)//just to determine the brightness of LED according to the deviation of adjacent two AD data
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }


            //send samples and calculation result to terminal program through UART
            printf("red=");
            printf("%i", aun_red_buffer[i]);
            printf(", ir=");
            printf("%i", aun_ir_buffer[i]);
            printf(", HR=%i, ", n_heart_rate); 
            printf("HRvalid=%i, ", ch_hr_valid);
            printf("SpO2=%i, ", n_sp02);
            printf("SPO2Valid=%i\r\n", ch_spo2_valid);
        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    }

}
