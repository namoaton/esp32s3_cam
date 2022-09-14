#include <string.h>
#include<stdint.h>
#include "mqtt_client.h"

const char* INFOTOPIC =  "/dogcam/info";
const char* ALARMTOPIC = "/dogcam/alarm";
const char*  LIGHT_ON = "light_on";
const char*  LIGHT_OFF = "light_off";
const char*  START_STREAM = "start_stream";
const char*  STOP_STREAM = "stop_stream";
const char*  VIBRO_ON = "vibro_on";
const char*  VIBRO_OFF = "vibro_off";
const char*  PLAY_COMMAND_1 = "play_command_1";
const char*  PLAY_COMMAND_2 = "play_command_2";
const char*  PLAY_COMMAND_3 = "play_command_3";
const char*  PLAY_COMMAND_4 = "play_command_4";
const char*  PLAY_COMMAND_5 = "play_command_5";

int mqtt_data_processing(char* data)
{
   int result = 0;
   if (!strcmp(data, LIGHT_ON))
    {
      result = 1;
      return result;
    }
    if (!strcmp(data, LIGHT_OFF))
    {
      result = 2;
    } 
    if (!strcmp(data, START_STREAM))
    {

      result = 3;
      return result;
    }
    if (!strcmp(data, STOP_STREAM))
    {
      result = 4;
      return result;
    }
    if (!strcmp(data, VIBRO_ON))
    {
      result = 5;
      return result;
    }
    if (!strcmp(data, VIBRO_OFF))
    {
      result = 6;
      return result;
    }
    if (!strcmp(data, PLAY_COMMAND_1))
    {
      result = 7;
      return result;
    }
    if (!strcmp(data, PLAY_COMMAND_2))
    {
      result = 8;
      return result;
    }
    if (!strcmp(data, PLAY_COMMAND_3))
    {
      result = 9;
      return result;
    }
    if (!strcmp(data, PLAY_COMMAND_4))
    {
      result = 10;
      return result;
    }
    if (!strcmp(data, PLAY_COMMAND_5))
    {
      result = 11;
      return result;
    }
    return result;
}

const char* id_to_string(uint8_t command_id){
 switch (command_id)
 {
 case 1:
   return LIGHT_ON;
   break;
 case 2:
   return LIGHT_OFF ;
   break;
 
 case 3:
   return  START_STREAM;
   break;
 
 case 4:
   return  STOP_STREAM;
   break;
 
 case 5:
   return VIBRO_ON ;
   break;
 
 case 6:
   return  VIBRO_OFF;
   break;
 
 case 7:
   return  PLAY_COMMAND_1;
   break;
 
 case 8:
   return  PLAY_COMMAND_2;
   break;
 
 case 9:
   return  PLAY_COMMAND_3;
   break;
 
 case 10:
   return  PLAY_COMMAND_4;
   break;
 
 case 11:
   return  PLAY_COMMAND_5;
   break;
 
 default:
   break;
 }
 return "command_not_found";
}

void mqtt_command_respond(esp_mqtt_client_handle_t client,uint8_t command_id, bool status){
  char mqtt_data[200];
  sprintf(mqtt_data, "{\"command\":\"%s\", \"status\":\"%s\"}",
                      id_to_string(command_id), (status? "ok":"error"));
  esp_mqtt_client_publish(client, INFOTOPIC,mqtt_data, 0, 1, 0);
}

void mqtt_alarm(esp_mqtt_client_handle_t client,uint8_t class_id, uint8_t score){
  char mqtt_data[200];
  sprintf(mqtt_data, "{\"alarm\":\"detection\", \"class\":\"%s\", \"score\":\"%d\"}",
                     "person", score);
  esp_mqtt_client_publish(client, ALARMTOPIC,mqtt_data, 0, 1, 0);
}

