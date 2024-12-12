#include <Arduino.h>
#include <ESP32_CAN.h>

//GPIOS:  CANTX: GPIO4   CANRX: GPIO5 
//BaudRates: 1000 -> 1MBps
//            800 -> 800Kbps
//            500 -> 500Kbps
//            250 -> 250Kbps
//            125 -> 125Kbps
//            100 -> 100Kbps
//             50 -> 50Kbps
//             25 -> 25Kbps  

TWAI_Interface CAN1(1000,4,5);  //argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN


uint32_t CAN_ID = 0;

uint8_t var1 = 0;

void setup() {
  Serial.begin(115200);

}

void loop() {
//Frame Setup:
//ID: 0x600
//DLC: 8 -- Automatically handled by the library
 CAN1.TXpacketBegin(0x600,0); //Parameter 1 -> Frame ID  Parameter 2 -> Extended ID: 0 - Normal 11 bit, 1 - Extended 29 bit
CAN1.TXpacketLoad(10);  //Decimal Value to be transmited
CAN1.TXpacketLoad(20);  //Decimal Value to be transmited
CAN1.TXpacketLoad(30);  //Decimal Value to be transmited
CAN1.TXpacketLoad(40);  //Decimal Value to be transmited
CAN1.TXpacketLoad(50);  //Decimal Value to be transmited
CAN1.TXpacketLoad(60);  //Decimal Value to be transmited
CAN1.TXpacketLoad(70);  //Decimal Value to be transmited
CAN1.TXpacketLoad(80);  //Decimal Value to be transmited
 CAN1.TXpackettransmit();
 Serial.println("Message 1 Sent...");
delay(500);
//Frame Setup:
//ID: 0x601
//DLC: 3 -- Automatically handled by the library
uint16_t var1 = 15000;

 CAN1.TXpacketBegin(0x601,0); //Parameter 1 -> Frame ID  Parameter 2 -> Extended ID: 0 - Normal 11 bit, 1 - Extended 29 bit
CAN1.TXpacketLoad(var1>>8);
CAN1.TXpacketLoad(var1);
CAN1.TXpacketLoad(15);

 CAN1.TXpackettransmit();
 Serial.println("Message 2 Sent...");
delay(500);
}
