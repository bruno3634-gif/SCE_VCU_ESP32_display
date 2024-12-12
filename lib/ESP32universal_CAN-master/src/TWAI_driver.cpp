#include "TWAI_driver.h"

TWAI_Interface :: TWAI_Interface(uint16_t TWAI_BAUD,uint8_t TX_PIN, uint8_t RX_PIN){

    twai_timing_config_t t_config;
    _txPin = (gpio_num_t)TX_PIN;
    _rxPin = (gpio_num_t)RX_PIN;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);

  switch(TWAI_BAUD){
    case 25:
    t_config = TWAI_TIMING_CONFIG_25KBITS();    
    break;
    case 50:
    t_config = TWAI_TIMING_CONFIG_50KBITS();    
    break;
    case 100:
     t_config = TWAI_TIMING_CONFIG_100KBITS();    
    break;
    case 125:
     t_config = TWAI_TIMING_CONFIG_125KBITS();
    break;
    case 250:
    t_config = TWAI_TIMING_CONFIG_250KBITS();    
    break;
    case 500:
     t_config = TWAI_TIMING_CONFIG_500KBITS(); 
    break;
    case 800:
     t_config = TWAI_TIMING_CONFIG_800KBITS(); 
    break;
    case 1000:
     t_config = TWAI_TIMING_CONFIG_1MBITS();  
    break;
    default:
    break;
    
  }

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    return;
  }


  if (twai_start() != ESP_OK)
  {
    
    return;
  }


  if (twai_reconfigure_alerts((TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_TX_FAILED), NULL) != ESP_OK)
  {
    
  }


}

void TWAI_Interface :: TXpacketBegin (uint16_t TWAITX_ID,uint8_t TWAITX_EXTD_EN){
    CanTxMessage.identifier = TWAITX_ID;
    CanTxMessage.extd = TWAITX_EXTD_EN;
    CanTxMessage.rtr = 0;   

}

void TWAI_Interface :: TXpacketLoad (uint8_t TWAITX_DATA){  
    CanTxMessage.data[TXdatactrl]= TWAITX_DATA;
    TXdatactrl ++;
    if(TXdatactrl > 7){
        TXdatactrl_OVF = 1;
        TXdatactrl = 7;
    }
 }

uint8_t TWAI_Interface ::    TXpackettransmit (){
    esp_err_t CanTxErr;
    if(TXdatactrl_OVF == 0){
      CanTxMessage.data_length_code = TXdatactrl;
      TXdatactrl = 0;
    }else{
      CanTxMessage.data_length_code = 8;
      TXdatactrl = 0;
      TXdatactrl_OVF = 0;
    }
    
    if ((CanTxErr = twai_transmit(&CanTxMessage, pdMS_TO_TICKS(0))) != ESP_OK)   
    {
      return 0; //Transmission Failed
    }
    else   
      return 1; //Transmission OK

  }

  uint32_t  TWAI_Interface :: RXpacketBegin(){
    
  if (twai_receive(&CanRxMessage, pdMS_TO_TICKS(0)) != ESP_OK)    //ESP_OK: Message successfully received from RX queue
  {
    if(CanRxMessage.extd){
        return CanRxMessage.identifier; //Return 29bit ID Extended
    }else{
        return CanRxMessage.identifier; //Return 11bit ID
    }
  }
  return 0;
  }

uint8_t TWAI_Interface :: RXpacketRead(uint8_t RX_INDEX){
  if(RX_INDEX >= CanRxMessage.data_length_code){
    return CanRxMessage.data[7];
  }
    return CanRxMessage.data[RX_INDEX];
}

uint8_t TWAI_Interface :: RXgetDLC(){
  return CanRxMessage.data_length_code;
}
