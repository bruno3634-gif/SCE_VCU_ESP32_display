#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "esp_freertos_hooks.h"

// Declare a global mutex handle
SemaphoreHandle_t xMutex;

#define TFT_DC 2
#define TFT_CS 15
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Placeholder variables
bool readyToDrive = false; // Change this value to true/false to test R2D display
bool ignition = false; // Change this value to true/false to test Ignition display
int fan = 50; // Example fan percentage value (0-100)
int temperature = 75; // Example temperature value (0-100)
int batteryVoltage = 12; // Example battery voltage value (0-100)
int power = 60; // Example power value (0-100)
int brakePressure = 50; // Example brake pressure value (0-100)

void display_task(void *pvParameters);
void read_can(void *pvParameters);
void send_can(void *pvParameters);
bool myIdleHook(void);

// Forward declarations for display functions
void drawStatus();
void drawGauges();
void drawGauge(int x, int y, const char* label, int value);

void setup() {
  Serial.begin(115200);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(3);
  
  // Create the mutex
  xMutex = xSemaphoreCreateMutex();
  
  // Create tasks with increased stack size
  xTaskCreatePinnedToCore(display_task, "display_task", 8192, NULL, 1, NULL, 1); // Pin to core 1
  xTaskCreate(read_can, "read_can", 8192, NULL, 1, NULL);
  xTaskCreate(send_can, "send_can", 8192, NULL, 1, NULL);
  
  // Register the custom idle hook
  esp_register_freertos_idle_hook(myIdleHook);
}

void loop() {
  // Empty loop
}

void display_task(void *pvParameters) {
  while(1) {
    // Attempt to take the mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      drawStatus();
      drawGauges();
      
      // Release the mutex
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to avoid flooding the display
  }
}

void read_can(void *pvParameters) {
  while(1) {
    // Attempt to take the mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // Read CAN bus
      // Parse CAN message
      // Update display
      Serial.println("Reading CAN bus");
      
      // Release the mutex
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void send_can(void *pvParameters) {
  while(1) {
    // Attempt to take the mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // Send CAN message
      Serial.println("Sending CAN message");
      
      // Release the mutex
      xSemaphoreGive(xMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

bool myIdleHook(void) {
  // Simple idle hook function
  static unsigned long idleCounter = 0;
  idleCounter++;
  return true; // Return true to keep the hook registered
}

void drawStatus() {
  // Clear the status area
  tft.fillRect(0, 0, tft.width(), 60, ILI9341_BLACK);

  // Draw R2D status
  tft.setCursor(10, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("R2D:");
  tft.print(readyToDrive ? "ON" : "OFF");

  // Draw Ignition status
  tft.setCursor(115, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("IGN:");
  tft.print(ignition ? "ON" : "OFF");

  // Draw Fan percentage
  tft.setCursor(220, 10);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("Fan:");
  tft.print(fan);
  tft.print("%");
}

void drawGauges() {
  // Clear the gauges area
  tft.fillRect(0, 60, tft.width(), tft.height() - 60, ILI9341_BLACK);

  // Draw Temperature gauge
  drawGauge(70, 90, "Temp", temperature);

  // Draw Battery Voltage gauge
  drawGauge(220, 90, "Bat Volt", batteryVoltage);

  // Draw Power gauge
  drawGauge(60, 220, "Power", power);

  // Draw Brake Pressure gauge
  drawGauge(180, 220, "Br Pr", brakePressure);
}

void drawGauge(int x, int y, const char* label, int value) {
  int radius = 55;
  int angle = map(value, 0, 100, -180, 0); // Rotate -90 degrees

  // Draw the semi-circle gauge frame
  for (int i = -180; i <= 0; i += 1) {
    float x0 = x + radius * cos(i * 3.14 / 180);
    float y0 = y + radius * sin(i * 3.14 / 180);
    float x1 = x + (radius + 10) * cos(i * 3.14 / 180);
    float y1 = y + (radius + 10) * sin(i * 3.14 / 180);
    tft.drawLine(x0, y0, x1, y1, ILI9341_WHITE);
  }

  // Draw the value indicator
  float x2 = x + radius * cos(angle * 3.14 / 180);
  float y2 = y + radius * sin(angle * 3.14 / 180);
  tft.drawLine(x, y, x2, y2, ILI9341_RED);

  // Add gauge label and value
  //tft.setCursor(x - radius, y + radius + 10);
  tft.setCursor(x - 60, y);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(label);
  //tft.setCursor(x, y);
  tft.print(": ");
  tft.print(value);
  tft.print("%");
}