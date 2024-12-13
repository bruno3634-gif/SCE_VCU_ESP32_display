#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "esp_freertos_hooks.h"
#include "ESP32_CAN.h"


// Declare a global mutex handle

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

SemaphoreHandle_t xMutex;

QueueHandle_t display_Queue;

// Declare the emergency variable
uint8_t emergency = 0;

// Create a queue handle
QueueHandle_t emergencyQueue;

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

void display_task(void *pvParameters);
void read_can(void *pvParameters);
void send_can(void *pvParameters);
bool myIdleHook(void);

// Forward declarations for display functions
void drawStatus();
void drawGauges();
void drawGauge(int x, int y, const char *label, int value, int maxValue = 100);

// Interrupt Service Routine
void IRAM_ATTR emergencyISR() {
  // Toggle the emergency variable
  emergency ^= 1;
  
  // Send the variable to the queue
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(emergencyQueue, &emergency, &xHigherPriorityTaskWoken);
  
  // Yield to higher priority task if necessary
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void setup()
{
  Serial.begin(115200);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(3);

  // Create the mutex
  xMutex = xSemaphoreCreateMutex();
  display_Queue = xQueueCreate(1, sizeof(data));
  
  // Initialize the queue
  emergencyQueue = xQueueCreate(10, sizeof(uint8_t));
  
  // Configure pin 16 as input with pulldown resistor
  pinMode(16, INPUT_PULLDOWN);
  
  // Attach interrupt to pin 16
  attachInterrupt(digitalPinToInterrupt(16), emergencyISR, CHANGE);
  
  // Create tasks with increased stack size
  xTaskCreatePinnedToCore(display_task, "display_task", 8192, NULL, 1, NULL, 1); // Pin to core 1
  xTaskCreate(read_can, "read_can", 8192, NULL, 1, NULL);
  xTaskCreate(send_can, "send_can", 8192, NULL, 1, NULL);

  // Register the custom idle hook
  esp_register_freertos_idle_hook(myIdleHook);
}

void loop()
{
  // Start the CAN sender task
  vTaskDelete(NULL);
  
  // Other loop code...
}

void display_task(void *pvParameters)
{
  while (1)
  {
    struct data data_received;
    if (xQueueReceive(display_Queue, &data_received, portMAX_DELAY) == pdTRUE)
    {
      temperature = data_received.temperature;
      batteryVoltage = data_received.batteryVoltage;
      power = data_received.power;
      brakePressure = data_received.brakePressure;
      readyToDrive = data_received.readyToDrive;
      ignition = data_received.ignition;
      fan = data_received.fan;
    }

    drawStatus();
    drawGauges();

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
        /*
        if (prev_id != id || memcmp(rxmessage, prev_rxmessage, CAN_DLC) != 0)
        {*/
        switch (id)
        {
        case 0x14:
          Serial.println("Received 0x14");
          readyToDrive = (rxmessage[0] == 0x01);
          ignition = (rxmessage[1] == 0x01);
          brakePressure = 0;
          brakePressure = (rxmessage[2] << 8) | rxmessage[3];
          brakePressure = brakePressure / 10;
          if(brakePressure < 0){
            brakePressure = 0;
          }
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

        if (readyToDrive == false)
        {
          power = 0;
        }
        

        data_received.batteryVoltage = batteryVoltage;
        data_received.temperature = temperature;
        data_received.power = power;
        data_received.brakePressure = brakePressure;
        data_received.readyToDrive = readyToDrive;
        data_received.ignition = ignition;
        data_received.fan = fan;

        if (send == 1)
        {
         // xQueueSend(display_Queue, &data_received, portMAX_DELAY);
         xQueueOverwrite(display_Queue, &data_received);
        }

        prev_id = id;
        memcpy(prev_rxmessage, rxmessage, CAN_DLC);
        //}
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
    uint8_t emergencia = 0;
    xQueueReceive(emergencyQueue, &emergencia, portMAX_DELAY);
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      if(emergencia == 1){
        CAN1.TXpacketBegin(0x502, 0);
        CAN1.TXpacketLoad(0x04);
        CAN1.TXpackettransmit();
      }
      else
      {
        CAN1.TXpacketBegin(0x502, 0);
        CAN1.TXpacketLoad(0x00);
        CAN1.TXpackettransmit();
      }
      
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

bool myIdleHook(void)
{
  static unsigned long idleCounter = 0;
  idleCounter++;
  return true;
}

void drawStatus()
{
  tft.fillRect(0, 0, tft.width(), 60, ILI9341_BLACK);

  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("IGN:");
  tft.print(readyToDrive ? "ON" : "OFF");

  tft.setCursor(115, 10);
  tft.print("R2D:");
  tft.print(ignition ? "ON" : "OFF");

  tft.setCursor(220, 10);
  tft.print("Fan:");
  tft.print(fan);
  tft.print("%");
}

void drawGauges()
{
  tft.fillRect(0, 60, tft.width(), tft.height() - 60, ILI9341_BLACK);

  drawGauge(75, 100, "Temp", temperature);
  drawGauge(220, 100, "Bat Volt", batteryVoltage, 30); // Adjust maxValue to 30
  drawGauge(75, 200, "Power", power);
  drawGauge(220, 200, "Br Pr", brakePressure);
}

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

void canSenderTask(void *pvParameters) {
  uint8_t emergencyState;
  while (1) {
    // Wait for emergency variable from the queue
    if (xQueueReceive(emergencyQueue, &emergencyState, portMAX_DELAY) == pdPASS) {
      // Send emergencyState via CAN
      // CAN sending code here...
    }
  }
}
