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
int brakePressure = 50; // Example brake pressure value (0-100)
int sensorValue = 0;
int lastAngle = -999; // Initialize with an invalid angle

void display_task(void *pvParameters);
void read_can(void *pvParameters);
void send_can(void *pvParameters);
bool myIdleHook(void);

// Forward declarations for display functions
void drawGauge(int angle);
void drawR2D();
void drawBP();
void drawWatermark();

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
      // Analog value read (e.g., from a potentiometer)
      sensorValue = sensorValue + 10;
      if(sensorValue > 100){
        sensorValue = 0;
      }
      int angle = map(sensorValue, 0, 100, -90, 10); // Map sensor value to -90 to 90 degrees

      if (angle != lastAngle) {
        drawGauge(angle);
        lastAngle = angle;
      }
      drawR2D();
      drawBP();
      drawWatermark();
      
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

void drawGauge(int angle) {
  // Clear only the previous indicator
  if (lastAngle != -999) {
    float x2 = 120 + 90 * cos((lastAngle - 50) * 3.14 / 180);
    float y2 = 240 + 90 * sin((lastAngle - 50) * 3.14 / 180);
    tft.drawLine(120, 240, x2, y2, ILI9341_BLACK);
  }

  // Draw the semi-circle gauge frame shifted to the left and slightly down
  int centerX = 120; // Center shifted to the left on the X-axis
  int centerY = 240; // Center shifted down on the Y-axis
  int radius = 90; // Radius of the semi-circle

  for (int i = -50; i <= 50; i += 1) {
    float x0 = centerX + radius * cos((i - 90) * 3.14 / 180);
    float y0 = centerY + radius * sin((i - 90) * 3.14 / 180);
    float x1 = centerX + (radius + 10) * cos((i - 90) * 3.14 / 180);
    float y1 = centerY + (radius + 10) * sin((i - 90) * 3.14 / 180);
    tft.drawLine(x0, y0, x1, y1, ILI9341_WHITE);
  }

  // Draw and label the scale markers
  tft.setCursor(centerX - 90, centerY - 70);
  tft.print("0");

  tft.setCursor(centerX - 60, centerY - 105);
  tft.print("25");

  tft.setCursor(centerX - 10, centerY - 120);
  tft.print("50");

  tft.setCursor(centerX + 50, centerY - 105);
  tft.print("75");

  tft.setCursor(centerX + 80, centerY - 70);
  tft.print("100");

  // Draw background for gauge value text
  int textX = centerX - 25;
  int textY = centerY - 10;
  int textWidth = 50;
  int textHeight = 20;
  tft.fillRect(textX, textY, textWidth, textHeight, ILI9341_BLACK); // Background color

  // Add gauge value text on top of the background
  tft.setCursor(textX, textY);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(sensorValue);

  // Draw the value indicator
  float x2 = centerX + radius * cos((angle - 50) * 3.14 / 180);
  float y2 = centerY + radius * sin((angle - 50) * 3.14 / 180);
  tft.drawLine(centerX, centerY, x2, y2, ILI9341_RED);
}

void drawR2D() {
  int boxSize = 50;
  int x = 70; // Box position
  int y = 10;

  // Box color and content
  uint16_t boxColor = readyToDrive ? ILI9341_GREEN : ILI9341_DARKGREY;
  const char* text = readyToDrive ? "ON" : "OFF";

  // Add "R2D:" text
  tft.setCursor(x - 50, y + 15); // Position of "R2D:" text
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("R2D:");

  // Draw the box and its content
  tft.fillRect(x, y, boxSize, boxSize, boxColor); // Corrected function call
  tft.setCursor(x + 10, y + 20);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print(text);
}

void drawBP() {
  // Update brake pressure value for demonstration
  brakePressure = (brakePressure + 5) % 101;

  int barWidth = 30;
  int barHeight = map(brakePressure, 0, 100, 0, 100); // Height based on BP value
  int x = 250; // Position of BP box shifted to the left
  int y = 120 - barHeight; // Fill from bottom to top

  // Draw the outer frame of the BP box
  tft.drawRect(x, 20, barWidth, 100, ILI9341_WHITE);

  // Draw the inner fill of the BP box
  tft.fillRect(x, 120 - barHeight, barWidth, barHeight, ILI9341_BLUE);

  // Add BP value text
  tft.setCursor(x - 15, 130); // Position of BP value text shifted to the left
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("BP: ");
  tft.println(brakePressure);
}

void drawWatermark() {
  // Draw FreeRTOS watermark at the bottom of the display
  tft.setCursor(10, tft.height() - 20);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.print("FreeRTOS");
} 