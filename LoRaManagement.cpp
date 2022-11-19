// 
// 
// 

#include "LoRaManagement.h"
#include "LoRa_E32.h"

extern LoRa_E32 e32ttl100;
extern ResponseStructContainer c;
extern LoRaConfiguration configuration;
extern ResponseStatus rs;

void Init_LoRa(void)
{
	// Startup all pins and UART
	e32ttl100.begin();

	// load up the Config structure
	c = e32ttl100.getConfiguration();
	configuration = *(LoRaConfiguration*)c.data;

	// set up all the required Config parameters
	configuration.ADDL = 0x01;
	configuration.ADDH = 0x00;
	configuration.CHAN = 23;

	configuration.OPTION.fec = FEC_1_ON;
	configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
	configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
	configuration.OPTION.transmissionPower = POWER_20;
	configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;

	configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
	configuration.SPED.uartBaudRate = UART_BPS_9600;
	configuration.SPED.uartParity = MODE_00_8N1;
	// save config - volatile only
	rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);

	e32ttl100.setMode(MODE_TYPE::MODE_0_NORMAL);
}


void Set_LoRa_Mode(LoRa_Mode_Type mode)
{
	switch (mode)
	{
	case LoRa_Mode_Type::RxNormal:
		// set to Mode 0, Receiver on full time.
		e32ttl100.setMode(MODE_TYPE::MODE_0_NORMAL);
		break;

	case LoRa_Mode_Type::TxLongRange:
		// set to Mode 0, no preamble needed for long range communication
		// power to medium or high
		configuration.OPTION.transmissionPower = POWER_20; // 100mW
		rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE); 

		e32ttl100.setMode(MODE_TYPE::MODE_0_NORMAL);
		break;

	case LoRa_Mode_Type::TxShortRange:
		// set to Mode 1, with short preamble
		// power to minimum.

	//	configuration.OPTION.transmissionPower = POWER_10; // 10mW
		//configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
	//	rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);

		e32ttl100.setMode(MODE_TYPE::MODE_1_WAKE_UP);
		break;

	case LoRa_Mode_Type::RxLowPower:
		// set to Mode 2, sleeping with periodic wakeup

	//	configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
	//	rs = e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_LOSE);

		e32ttl100.setMode(MODE_TYPE::MODE_2_POWER_SAVING);
		break;

	default:;
	}
}