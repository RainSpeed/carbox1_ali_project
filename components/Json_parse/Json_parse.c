#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cJSON.h>
#include "esp_system.h"
#include "Json_parse.h"
#include "Nvs.h"
#include "ServerTimer.h"
#include "esp_log.h"

#include "esp_wifi.h"
#include "Smartconfig.h"
#include "E2prom.h"
#include "sht31.h"
#include "gnss.h"
#include "RtcUsr.h"







esp_err_t parse_Uart0(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_ProductKey = NULL;
    cJSON *json_data_parse_DeviceName = NULL;
    cJSON *json_data_parse_DeviceSecret = NULL;

    if(json_data[0]!='{')
    {
        printf("uart0 Json Formatting error1\n");
        return 0;
    }

    json_data_parse = cJSON_Parse(json_data);
    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {
        printf("uart0 Json Formatting error\n");
        cJSON_Delete(json_data_parse);

        return 0;
    }
    else
    {
        /*
        {
            "Command": "SetupALi",
            "ProductKey": "a18hJfuxArE",
            "DeviceName": "AIR2V001",
            "DeviceSecret": "kc0tfij2RbXdbOmiHSXnwmaZgR3CDE85",
            
        }
        */
        char zero_data[256];
        bzero(zero_data,sizeof(zero_data));
        
        json_data_parse_ProductKey = cJSON_GetObjectItem(json_data_parse, "ProductKey");
        if(json_data_parse_ProductKey!=NULL) 
        {
            E2prom_Write(PRODUCTKEY_ADDR, (uint8_t *)zero_data, PRODUCTKEY_LEN);            
            sprintf(ProductKey,"%s%c",json_data_parse_ProductKey->valuestring,'\0');
            E2prom_Write(PRODUCTKEY_ADDR, (uint8_t *)ProductKey, strlen(ProductKey));            
            printf("ProductKey= %s\n", json_data_parse_ProductKey->valuestring);
        }

        json_data_parse_DeviceName = cJSON_GetObjectItem(json_data_parse, "DeviceName"); 
        if(json_data_parse_DeviceName!=NULL) 
        {
            E2prom_Write(DEVICENAME_ADDR, (uint8_t *)zero_data, DEVICENAME_LEN);   
            sprintf(DeviceName,"%s%c",json_data_parse_DeviceName->valuestring,'\0');
            E2prom_Write(DEVICENAME_ADDR, (uint8_t *)DeviceName, strlen(DeviceName)); 
            printf("DeviceName= %s\n", json_data_parse_DeviceName->valuestring);
        }

        json_data_parse_DeviceSecret = cJSON_GetObjectItem(json_data_parse, "DeviceSecret"); 
        if(json_data_parse_DeviceSecret!=NULL) 
        {
            E2prom_Write(DEVICESECRET_ADDR, (uint8_t *)zero_data, DEVICESECRET_LEN);   
            sprintf(DeviceSecret,"%s%c",json_data_parse_DeviceSecret->valuestring,'\0');
            E2prom_Write(DEVICESECRET_ADDR, (uint8_t *)DeviceSecret, strlen(DeviceSecret));
            printf("DeviceSecret= %s\n", json_data_parse_DeviceSecret->valuestring);
        }  

        printf("{\"status\":\"success\",\"err_code\": 0}");
        cJSON_Delete(json_data_parse);
        fflush(stdout);//使stdout清空，就会立刻输出所有在缓冲区的内容。
        esp_restart();//芯片复位 函数位于esp_system.h
        return 1;

    }
}




/*esp_err_t parse_objects_mqtt(char *json_data)
{
    cJSON *json_data_parse = NULL;
    cJSON *json_data_parse_value = NULL;

    json_data_parse = cJSON_Parse(json_data);

    if(json_data[0]!='{')
    {
        printf("mqtt Json Formatting error\n");

        return 0;       
    }

    if (json_data_parse == NULL) //如果数据包不为JSON则退出
    {

        printf("Json Formatting error4\n");

        cJSON_Delete(json_data_parse);
        return 0;
    }
    else
    {
        json_data_parse_value = cJSON_GetObjectItem(json_data_parse, "switch"); 
        printf("switch= %s\n", json_data_parse_value->valuestring);
        if(strcmp(json_data_parse_value->valuestring,"on")==0)
        {
            printf("switch on\n");
            Beep_On();
        }
        else if(strcmp(json_data_parse_value->valuestring,"off")==0)
        {
            printf("switch off\n");
            Beep_Off();
        }

    }

    cJSON_Delete(json_data_parse);
    

    return 1;
}*/





/*
{
	"method": "thing.event.property.post",
	"params": {
		"CO2": 1,
		"CurrentHumidity": 5,
		"CurrentTemperature": 3,
		"HCHO": 2,
		"PM25": 4,
		"TVOC": 1,
		"RSSI": 0,
        "DeviceNumber": "AIR2V001",
        "WeatherLocationNumber":"CN888"
	}
}
*/


/*
{
	"method": "thing.event.property.post",
	"params": {
		"Speed": 12,
		"Temperature": 24.3,
		"SerialNumber": "CAR1V001",
		"Humidity": 34.9,
		"UpdateTime": "2019-04-29 08:03:45",
		"GeoLocation": {
			"CoordinateSystem": 1,
			"Latitude": 39.12,
			"Longitude": 121.34,
			"Altitude": 12.5
		}
	}
}
*/

void create_mqtt_json(creat_json *pCreat_json)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *params = cJSON_CreateObject();
    

    cJSON_AddItemToObject(root, "method", cJSON_CreateString("thing.event.property.post"));
    cJSON_AddItemToObject(root, "params", params);

    if (sht31_readTempHum()) 
    {		
        double Temperature = sht31_readTemperature();
        double Humidity = sht31_readHumidity();

        cJSON_AddItemToObject(params, "Temperature", cJSON_CreateNumber(Temperature));
        cJSON_AddItemToObject(params, "Humidity", cJSON_CreateNumber(Humidity)); 
        ESP_LOGI("SHT30", "Temperature=%.1f, Humidity=%.1f", Temperature, Humidity);
	} 
    cJSON_AddItemToObject(params, "Speed", cJSON_CreateNumber((int)speed));
    cJSON_AddItemToObject(params, "SerialNumber", cJSON_CreateString(DeviceName));
    if(valid==1) //A有效定位
    {
        cJSON *GeoLocation = cJSON_CreateObject();
        Rtc_Read(&year,&month,&day,&hour,&min,&sec);
        char UpdateTime[50];
        sprintf(UpdateTime,"%d-%02d-%02d %02d:%02d:%02d",year,month,day,hour,min,sec);
        cJSON_AddItemToObject(params, "UpdateTime", cJSON_CreateString(UpdateTime));

        cJSON_AddItemToObject(params, "GeoLocation", GeoLocation);
        cJSON_AddItemToObject(GeoLocation, "CoordinateSystem", cJSON_CreateNumber(1));
        cJSON_AddItemToObject(GeoLocation, "Latitude", cJSON_CreateNumber(latitude));
        cJSON_AddItemToObject(GeoLocation, "Longitude", cJSON_CreateNumber(longitude));
        cJSON_AddItemToObject(GeoLocation, "Altitude", cJSON_CreateNumber(Altitude));
    }



    char *cjson_printunformat;
    cjson_printunformat=cJSON_PrintUnformatted(root);
    pCreat_json->creat_json_c=strlen(cjson_printunformat);
    bzero(pCreat_json->creat_json_b,sizeof(pCreat_json->creat_json_b));
    memcpy(pCreat_json->creat_json_b,cjson_printunformat,pCreat_json->creat_json_c);
    printf("len=%d,mqtt_json=%s\n",pCreat_json->creat_json_c,pCreat_json->creat_json_b);
    free(cjson_printunformat);
    cJSON_Delete(root);
    
}


/*void create_mqtt_json(creat_json *pCreat_json)
{

    cJSON *root = cJSON_CreateObject();
   

    if (sht31_readTempHum()) 
    {		
        double Temperature = sht31_readTemperature();
        double Humidity = sht31_readHumidity();

        cJSON_AddItemToObject(root, "Temperature", cJSON_CreateNumber(Temperature));
        cJSON_AddItemToObject(root, "Humidity", cJSON_CreateNumber(Humidity)); 
        ESP_LOGI("SHT30", "Temperature=%.1f, Humidity=%.1f", Temperature, Humidity);
	} 
    else 
    {
		ESP_LOGI("SHT30", "SHT31_ReadTempHum : failed");
	}

    cJSON_AddItemToObject(root, "speed", cJSON_CreateNumber((int)speed));  
    if(valid==1) 
    {
        cJSON *next = cJSON_CreateObject();
        cJSON_AddItemToObject(root, "position", next);
        cJSON_AddItemToObject(next, "lon", cJSON_CreateNumber(longitude));
        cJSON_AddItemToObject(next, "lat", cJSON_CreateNumber(latitude));
        Rtc_Read(&year,&month,&day,&hour,&min,&sec);
    }
    


    char *cjson_printunformat;
    cjson_printunformat=cJSON_PrintUnformatted(root);
    pCreat_json->creat_json_c=strlen(cjson_printunformat);
    bzero(pCreat_json->creat_json_b,sizeof(pCreat_json->creat_json_b));
    memcpy(pCreat_json->creat_json_b,cjson_printunformat,pCreat_json->creat_json_c);
    printf("len=%d,mqtt_json=%s\n",pCreat_json->creat_json_c,pCreat_json->creat_json_b);
    free(cjson_printunformat);
    cJSON_Delete(root);
    
}*/


