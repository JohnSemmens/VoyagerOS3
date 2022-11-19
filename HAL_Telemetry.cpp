// 
// 
// 

#include "HAL_Telemetry.h"
#include "HAL.h"

#include "LoRaManagement.h"
#include "LoRa_E32.h"
#include "TelemetryMessages.h"

extern byte MessageArray[EndMarker+1];


byte EByte_AuxPin = 2;
byte EByte_M0 = 5;
byte EByte_M1 = 6;
LoRa_E32 e32ttl100(&Serial3, EByte_AuxPin, EByte_M0, EByte_M1, UART_BPS_RATE_9600);
ResponseStructContainer c;
LoRaConfiguration configuration;
ResponseStatus rs;

void HALTelemetry::Init()
{
    Init_LoRa();
    Set_LoRa_Mode(LoRa_Mode_Type::RxLowPower);
  //  Set_LoRa_Mode(LoRa_Mode_Type::TxShortRange);
}