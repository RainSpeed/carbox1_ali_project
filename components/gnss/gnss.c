#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "minmea.h"
#include "gnss.h"
#include "Led.h"
#include "RtcUsr.h"

#define UART2_TXD  (UART_PIN_NO_CHANGE)
#define UART2_RXD  (GPIO_NUM_16)
#define UART2_RTS  (UART_PIN_NO_CHANGE)
#define UART2_CTS  (UART_PIN_NO_CHANGE)

#define BUF_SIZE    1024

const char tag[] = "GNSS";


void GNSS_READ_task(void* arg)
{
	static char data_u2[BUF_SIZE]="\0";
	static char rmc[256]="\0";
	int rmc_len=0;
    static char gga[256]="\0";
	int gga_len=0;

	while(1) 
	{
		int u2_len = uart_read_bytes(UART_NUM_2, (uint8_t *)data_u2, BUF_SIZE, 300 / portTICK_RATE_MS);
		if(u2_len>0)
		{
			
			//ESP_LOGI(tag, "u2=%s,len=%d", data_u2,u2_len);
            u2_len=0;
            if(data_u2[0]=='$')
            {
                char* rmc_start=strstr(data_u2,"$GNRMC");
                if(rmc_start!=NULL)
                {
                    char* rmc_end=strchr(rmc_start,'\n');
                    if(rmc_end!=NULL)
                    {
                        rmc_len=rmc_end-rmc_start;
                        strncpy(rmc,rmc_start,rmc_len-1);
                    }
                }

                //printf("rmc_len=%d\n",rmc_len);			
			    printf("rmc=%s\n",rmc);

                char* gga_start=strstr(data_u2,"$GNGGA");
                if(gga_start!=NULL)
                {
                    char* gga_end=strchr(gga_start,'\n');
                    if(gga_end!=NULL)
                    {
                        gga_len=gga_end-gga_start;
                        strncpy(gga,gga_start,gga_len-1);
                    }	
                }
			    printf("gga=%s\n",gga);

            }

            if(MINMEA_SENTENCE_RMC==minmea_sentence_id(rmc, false))
            {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, rmc)) 
                {
                    latitude=minmea_tocoord(&frame.latitude);
                    longitude=minmea_tocoord(&frame.longitude);
                    ESP_LOGI(tag, "latitude=%f,longitude=%f",latitude,longitude);
                    speed=minmea_tofloat(&frame.speed);
                    speed=speed*1.852;
                    valid=frame.valid;
                    if(speed<2)
                    {
                        speed=0;
                    }
                    if(valid==1) //A有效定位
                    {
                        /*printf( "TIME: %d:%d:%d %d-%d-%d\n",
                                frame.time.hours,
                                frame.time.minutes,
                                frame.time.seconds,
                                frame.date.day,
                                frame.date.month,
                                frame.date.year);*/
                        Rtc_Set(2000+frame.date.year,frame.date.month,frame.date.day,frame.time.hours,frame.time.minutes,frame.time.seconds);
                        Led_R_On();
                        vTaskDelay(200 / portTICK_RATE_MS);
                        Led_R_Off();
                    }
                    //ESP_LOGI(tag, "latitude=%f,longitude=%f,speed=%f",latitude,longitude,speed);
                }
                else 
                {
                    ESP_LOGI(tag, "$xxRMC sentence is not parsed\n");
                }                
            }

            if(MINMEA_SENTENCE_GGA==minmea_sentence_id(gga, false))
            {
                struct minmea_sentence_gga frame;
                if(minmea_parse_gga(&frame, gga)) 
                {
                    Altitude=minmea_tofloat(&frame.altitude);
                    ESP_LOGI(tag, "Altitude=%f",Altitude);
                }
                else 
                {
                    ESP_LOGI(tag, "$xxGGA sentence is not parsed\n");
                } 
            }

            memset(data_u2,0,sizeof(data_u2));
            memset(rmc,0,rmc_len);
            memset(gga,0,gga_len);
		}
		//vTaskDelay(5 / portTICK_RATE_MS);
	}
} 


void GNSS_init(void)
{
     //配置GPIO
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = 1 << UART2_RXD;
    io_conf.mode = GPIO_MODE_INPUT;

    gpio_config(&io_conf);
    
    
    uart_config_t uart_config = 
	{
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    gpio_set_pull_mode(UART2_RXD, 0);

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, UART2_TXD, UART2_RXD, UART2_RTS, UART2_CTS);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);
	xTaskCreate(&GNSS_READ_task, "GNSS_READ_task", 1024*10, NULL, 12, NULL);

}