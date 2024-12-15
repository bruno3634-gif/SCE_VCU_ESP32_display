#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "esp_freertos_hooks.h"
#include "ESP32_CAN.h"

  // Data structure to hold the data received from the CAN bus

struct data
{
  float temperature;
  float batteryVoltage;
  int power;
  float brakePressure;
  int fan;
  bool readyToDrive;
  bool ignition;
};

TWAI_Interface CAN1(1000, 4, 5); // argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN

SemaphoreHandle_t xMutex;  // This mutex is used to protect the CAN bus

QueueHandle_t display_Queue; // Queue to send data to the display task

#define TFT_DC 2
#define TFT_CS 15
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Placeholder variables
bool readyToDrive = false;
bool ignition = false;
int fan = 50;
int temperature = 75;
int batteryVoltage = 12;
int power = 60;
int brakePressure = 50;

// Function prototypes

void display_task(void *pvParameters);
void read_can(void *pvParameters);
void send_can(void *pvParameters);


// Forward declarations for display functions
void drawStatus();
void drawGauges();
void drawGauge(int x, int y, const char *label, int value, int maxValue = 100);

void setup()
{
  pinMode(17,OUTPUT);
  Serial.begin(115200);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(3);


  xMutex = xSemaphoreCreateMutex();
  display_Queue = xQueueCreate(1, sizeof(data));

  xTaskCreatePinnedToCore(display_task, "display_task", 8192, NULL, 2, NULL, 1); // Pin to core 1
  xTaskCreate(read_can, "read_can", 8192, NULL, 3, NULL);
  xTaskCreate(send_can, "send_can", 8192, NULL, 1, NULL);

}

void loop()
{
  vTaskDelete(NULL);
}



void display_task(void *pvParameters)
{
  struct data data_received;
  while (1)
  {
    
    if (xQueueReceive(display_Queue, &data_received, portMAX_DELAY) == pdTRUE)
    {
      temperature = data_received.temperature;
      batteryVoltage = data_received.batteryVoltage;
      power = data_received.power;
      brakePressure = data_received.brakePressure;
      readyToDrive = data_received.readyToDrive;
      ignition = data_received.ignition;
      fan = data_received.fan;

      // Update the display
      drawStatus();
      drawGauges();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void read_can(void *pvParameters)
{
  struct data data_received;
  uint8_t prev_rxmessage[8] = {0};
  int prev_id = -1;

  while (1)
  {
    int send = 1;
    int id = 0;
    uint8_t rxmessage[8] = {0};
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      id = CAN1.RXpacketBegin();
      if (id == 0)
      {
        Serial.println("Waiting for CAN message");
      }
      else
      {
        uint8_t CAN_DLC = CAN1.RXgetDLC();
        for (int i = 0; i < CAN_DLC; i++)
        {
          rxmessage[i] = CAN1.RXpacketRead(i);
        }
        switch (id)
        {
        case 0x14:
          Serial.println("Received 0x14");
          readyToDrive = (rxmessage[0] == 0x01);
          ignition = (rxmessage[1] == 0x01);
          brakePressure = 0;
          brakePressure = (rxmessage[2] << 8) | rxmessage[3];
          brakePressure = brakePressure / 10;
          break;
        case 0x24:
          Serial.println("Received 0x24");
          power = 0;
          power = (rxmessage[5] << 8) | rxmessage[4];
          power = power / 10;
          break;
        case 0x54:
          Serial.println("Received 0x54");
          batteryVoltage = 0;
          temperature = 0;
          batteryVoltage = (rxmessage[0] << 8) | rxmessage[1];
          batteryVoltage = batteryVoltage / 10;
          temperature = (rxmessage[2] << 8) | rxmessage[3];
          temperature = temperature / 10;
          break;
        default:
          send = 0;
          break;
        }

        data_received.batteryVoltage = batteryVoltage;
        data_received.temperature = temperature;
        data_received.power = power;
        data_received.brakePressure = brakePressure;
        data_received.readyToDrive = readyToDrive;
        data_received.ignition = ignition;

        int fanSpeed = map(temperature, 30, 40, 0, 255);
        fanSpeed = constrain(fanSpeed, 0, 255);
        analogWrite(17, fanSpeed);
        data_received.fan = fanSpeed;

        if (send == 1)
        {
         xQueueOverwrite(display_Queue, &data_received);
        }

        prev_id = id;
        memcpy(prev_rxmessage, rxmessage, CAN_DLC);
        
      }
    }
    xSemaphoreGive(xMutex);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void send_can(void *pvParameters)
{
  while (1)
  {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      Serial.println("Sending CAN message");
      CAN1.TXpacketBegin(0x44, 0);
      CAN1.TXpacketLoad(0x01);
      CAN1.TXpacketLoad(0x01);
      CAN1.TXpackettransmit();
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}


// Function to draw the status information on the display
void drawStatus()
{
  tft.fillRect(0, 0, tft.width(), 60, ILI9341_BLACK);

  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("R2D:");
  tft.print(readyToDrive ? "ON" : "OFF");

  tft.setCursor(115, 10);
  tft.print("IGN:");
  tft.print(ignition ? "ON" : "OFF");

  tft.setCursor(220, 10);
  tft.print("Fan:");
  tft.print(fan);
  tft.print("%");
}
//Function to draw the gauges on the display
void drawGauges()
{
  tft.fillRect(0, 60, tft.width(), tft.height() - 60, ILI9341_BLACK);

  drawGauge(75, 100, "Temp", temperature);
  drawGauge(220, 100, "Bat Volt", batteryVoltage, 30); // Adjust maxValue to 30
  drawGauge(150, 200, "Power", power);
  //drawGauge(220, 200, "Br Pr", brakePressure);
}
// Function to draw a gauge on the display
void drawGauge(int x, int y, const char *label, int value, int maxValue)
{
  int radius = 50;
  int angle = map(value, 0, maxValue, -180, 0);

  for (int i = -180; i <= 0; i += 1)
  {
    float x0 = x + radius * cos(i * 3.14 / 180);
    float y0 = y + radius * sin(i * 3.14 / 180);
    float x1 = x + (radius + 10) * cos(i * 3.14 / 180);
    float y1 = y + (radius + 10) * sin(i * 3.14 / 180);
    tft.drawLine(x0, y0, x1, y1, ILI9341_WHITE);
  }

  float x2 = x + radius * cos(angle * 3.14 / 180);
  float y2 = y + radius * sin(angle * 3.14 / 180);
  tft.drawLine(x, y, x2, y2, ILI9341_RED);

  tft.setCursor(x - 60, y + 5);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(label);
  tft.print(":");
  tft.print(value);
}
