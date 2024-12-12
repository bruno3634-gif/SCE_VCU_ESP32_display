#ifndef TWAI_DRIVER_H
#define TWAI_DRIVER_H

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/twai.h>


class TWAI_Interface{
    private:
    twai_message_t CanTxMessage;
    twai_message_t CanRxMessage;
    uint8_t TXdatactrl = 0;
    uint8_t TXdatactrl_OVF = 0;
    gpio_num_t _rxPin;
    gpio_num_t _txPin;
    public:
    
    TWAI_Interface (uint16_t TWAI_BAUD,uint8_t TX_PIN, uint8_t RX_PIN);
    void TXpacketBegin (uint16_t TWAITX_ID,uint8_t TWAITX_EXTD_EN);
    void TXpacketLoad (uint8_t TWAITX_DATA);
    uint8_t TXpackettransmit ();
    
    uint32_t RXpacketBegin();
    uint8_t RXpacketRead(uint8_t RX_INDEX);
    uint8_t RXgetDLC();
};

#endif