#include <Arduino.h>
#include <stdio.h>
#include "driver/uart.h"
#include <HardwareSerial.h>
#include "time.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
#define TOPIC_TEMP "IoTHydroponic/temperature"
#define TOPIC_LIGHT "IoTHydroponic/light"
#define TOPIC_PH "IoTHydroponic/pH"
#define TOPIC_WARNING "IoTHydroponic/warning" /*Topic where temperature warnings will be sent*/
#define BROKER_IP "192.168.0.14"
#define BROKER_PORT 1883
#define RX_BUFFER_SIZE 13
#define READINGS_SIZE 4

/*UART communication related variables*/

HardwareSerial serialUart(2);
std::string dataLecture;
char* dataLectureCharArray;
int temperature;
int light;
int pH;

/*WiFi related variables*/

const char* ssid = "vodafoneD840";
const char* password = "UDZ2CDZJJZWETZ";

//const char* ssid = "Galaxy A02s9111";
//const char* password = "yxiz0168";

WiFiClient espClient;
PubSubClient client(espClient);

/*NTP Server related variables*/

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
int hour;

/*Buzzer related variables*/

const int buzzerFrequency = 5000;
const int buzzerResolution = 8;
const int buzzerChannel = 0;
const int buzzerPin = 32;
const int freqSoundCnst = 255;

QueueHandle_t xMutexData;
QueueHandle_t xMutexHour;

void IRAM_ATTR readingInterrupt() {

  String dataLecture = serialUart.readString();
  Serial.println(dataLecture);
}

void wifiConnect()
{
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void mqttConnect() {
  client.setServer(BROKER_IP, BROKER_PORT);
  while (!client.connected()) {
    Serial.print("MQTT connecting ...");

    if (client.connect("ESP32Client1")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, status code =");
      Serial.print(client.state());
      Serial.println("try again in 5 seconds");

      delay(5000);  //* Wait 5 seconds before retrying
    }
  }
}

void vTaskGetHour( void *pvParameters ){
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;

  for(;;) {
    struct tm timeinfo;

    if(getLocalTime(&timeinfo)){

      xSemaphoreTake(xMutexHour, 2000/portTICK_PERIOD_MS);
      hour = timeinfo.tm_hour;
      Serial.println("Hour obtained: " + hour);
      xSemaphoreGive(xMutexHour);

    }else Serial.println("Hour could not be obtained");
    
    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); // NULL indica que nos referimos a esta tarea
}

void vTaskSendHour( void *pvParameters ){
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  for(;;) {

    if (serialUart.availableForWrite()) {

      xSemaphoreTake(xMutexHour, 2000/portTICK_PERIOD_MS);
      serialUart.print(hour);
      Serial.println("Hour sent: " + hour);
      xSemaphoreGive(xMutexHour);
      
    }else Serial.println("Hour could not be sent");
  
    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); // NULL indica que nos referimos a esta tarea
}

void vTaskActivateAlarm( void *pvParameters )
{
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  for(;;) {
    
    int temp;

    xSemaphoreTake(xMutexHour, 2000/portTICK_PERIOD_MS);
    temp = temperature;
    xSemaphoreGive(xMutexHour);

    if(temp >= 25 && temp <= 28){

      //Aviso dashboard
      Serial.println("Temperature between 25 and 28 degrees");
      ledcWrite(buzzerChannel, 0);

    } else if (temp > 28){

      //Activar alarma
      ledcWrite(buzzerChannel, freqSoundCnst);
      Serial.println("Temperature over 28 degrees");

    }else {
      ledcWrite(buzzerChannel, 0);
      Serial.println("NOIF");
    }

    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); // NULL indica que nos referimos a esta tarea
}

void vTaskConvertReadData( void *pvParameters )
{
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  for(;;) {
    
    int temp, lightAux, pHAux;

    xSemaphoreTake(xMutexHour, 2000/portTICK_PERIOD_MS);
    temp = temperature;
    lightAux = light;
    pHAux = pH;
    dataLectureCharArray = new char[dataLecture.length() + 1];
    std::strcpy(dataLectureCharArray, dataLecture.c_str());
    xSemaphoreGive(xMutexHour);


    Serial.printf("Data obtained: %c", dataLectureCharArray);

    const char* delimiter = ",";
    char* token = strtok(dataLectureCharArray, delimiter);

    if(token != NULL){
      if((temp = atoi(token)) == 0) Serial.println("Error converting temperature");
      else token = strtok(NULL, delimiter);

      if((lightAux = atoi(token)) == 0) Serial.println("Error converting light");
      else token = strtok(NULL, delimiter);

      if((pHAux = atoi(token)) == 0) Serial.println("Error converting pH");
      else token = strtok(NULL, delimiter);
    }else{
      Serial.println("Error converting data");
    }
    

    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); // NULL indica que nos referimos a esta tarea
}

void vTaskPublicData( void * pvParameters ){
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  for(;;) {
    
    char cTemp[READINGS_SIZE];
    char cLight[READINGS_SIZE];
    char cPH[READINGS_SIZE];

    int temp, lightAux, pHAux;

    xSemaphoreTake(xMutexHour, 2000/portTICK_PERIOD_MS);
    temp = temperature;
    lightAux = light;
    pHAux = pH;
    xSemaphoreGive(xMutexHour);

    sprintf(cTemp, "%d", temp);
    sprintf(cLight, "%d", lightAux);
    sprintf(cPH, "%d", pHAux);

    Serial.printf("Data sent %c, %c, %c\r\n", cTemp, cLight, cPH);

    client.publish(TOPIC_TEMP, cTemp);
    client.publish(TOPIC_LIGHT, cLight);
    client.publish(TOPIC_PH, cPH);

    vTaskDelay(5000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL); // NULL indica que nos referimos a esta tarea
}

void app_main(){

  vSemaphoreCreateBinary( xMutexData );
  vSemaphoreCreateBinary( xMutexHour );

  xTaskCreate(vTaskGetHour, "Task Get Hour", 2500, NULL, 3, NULL);
  xTaskCreate(vTaskSendHour, "Task Send Hour", 2500, NULL, 3, NULL);
  xTaskCreate(vTaskActivateAlarm, "Task Activate Alarm", 2500, NULL, 2, NULL);
  xTaskCreate(vTaskPublicData, "Task Public Data", 2500, NULL, 2, NULL);
  xTaskCreate(vTaskConvertReadData, "Task Convert Read Data", 2500, NULL, 1, NULL);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  serialUart.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);
  serialUart.onReceive(readingInterrupt);
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  ledcSetup(buzzerChannel, buzzerFrequency, buzzerResolution);

  delay(4000);
  wifiConnect();
  mqttConnect();

  app_main();
}
void loop() {
  
}
