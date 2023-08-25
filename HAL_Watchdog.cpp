// 
// 
// 

#include "HAL_Watchdog.h"
#include "HAL_SDCard.h"

void Watchdog_Init(int timeout)
{
	// the Watchdog_Init should be placed at the end of setup() since the watchdog starts right after this
	Serial.print(F("Initialising watchdog: "));
	Serial.print(timeout);
	Serial.println("s");

	SD_Logging_Event_Messsage("Initialising watchdog: " + String(timeout) + "s");

	WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
	WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
	delayMicroseconds(1); // Need to wait a bit..
	WDOG_STCTRLH = 0x0001; // Enable WDG
	WDOG_TOVALL = timeout * 1000; // These 2 lines set the time-out value in ms. 
	WDOG_TOVALH = 0;
	WDOG_PRESC = 0; // This sets prescale clock so that the watchdog timer ticks at 1kHZ instead of the default 1kHZ/4 = 200 HZ
}

void Watchdog_Pat()
{
	// use the following 4 lines to pat the dog
	noInterrupts();
	WDOG_REFRESH = 0xA602;
	WDOG_REFRESH = 0xB480;
	interrupts()

	//Serial.print("Pat dog."); Serial.print(second()); Serial.println("s.");
}