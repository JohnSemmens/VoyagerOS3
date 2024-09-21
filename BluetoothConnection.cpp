// 
// 
// 

#include "BluetoothConnection.h"
#include "configValues.h"
#include "Wingsail.h"
#include "HAL_SDCard.h"

extern byte BluetoothStatePin;
extern BTStateType BTState;
extern HardwareSerial *Serials[];
extern configValuesType Configuration;
extern int CommandPort;
extern WingSailType WingSail;

void Bluetooth_Init(int BluetoothPort)
{
	// called from setup()
	Serial.println("*** Initialising Bluetooth Serial...");
	Serial.print("Serial Port ");
	Serial.print(BluetoothPort);
	Serial.print(", ");
	Serial.print(Configuration.BTPortBaudRate);
	Serial.println("Baud.");

	(*Serials[BluetoothPort]).begin(Configuration.BTPortBaudRate);
	pinMode(BluetoothStatePin, INPUT_PULLUP);
	Serial.println("*** Bluetooth Serial Initialised.");
}

void BluetoothManageConnection(int BluetoothPort)
{
	// call this connection management procedure at around 5 second intervals

	static BTStateType BTStatePrev;
	String BTStatus;

	if (digitalRead(BluetoothStatePin) == LOW) // LOW is not connected
	{
		// if not connected then proceed with connection process
		switch (BTState)
		{
		case Idle:
		case Initialising:
			BluetoothInitialiseConnection(BluetoothPort);
			break;

		case Initialised:
			(*Serials[BluetoothPort]).print(F("AT+CONA")); // connect to a MAC Addres
			(*Serials[BluetoothPort]).println(Configuration.BT_MAC_Address);
		
			BTState = Connecting;

			BTStatus = "BT:" + GetBTStatus(BTState) + ":" + Configuration.BT_MAC_Address;
			Serial.println(BTStatus);
			SD_Logging_Event_Messsage(BTStatus);
			break;

		case Connecting:
			BTState = Initialised;

			BTStatus = "BT:" + GetBTStatus(BTState);
			Serial.println(BTStatus);
			SD_Logging_Event_Messsage(BTStatus);
			break;

		case Connected:
			// if the current state is "Connected", but now the Connected Status line has dropped
			// then it indicates that we had a connection, but its dropped.
			// hence move the state back to "initialised" to force a reconnection.
			if (digitalRead(BluetoothStatePin) == LOW)
			{
				BTState = Initialised;

				BTStatus = "BT:" + GetBTStatus(BTState);
				Serial.println(BTStatus);
				SD_Logging_Event_Messsage(BTStatus);
			}
			break;
		default:;
		};
	}
	else
	{
		// BluetoothStatePin is high
		BTState = Connected;
	}

	// look for a change in Bluetooth State and then log a message
	if (BTStatePrev != BTState)
	{
		if (BTState == BTStateType::Connected)
		{
			(*Serials[Configuration.BluetoothPort]).println(); // send a line return to clear the buffer

			(*Serials[Configuration.BluetoothPort]).println(F("wake")); // wakeup
			(*Serials[Configuration.BluetoothPort]).println("");
			(*Serials[Configuration.BluetoothPort]).println("");
			(*Serials[Configuration.BluetoothPort]).println(F("flg,3")); // flash green led x 3

			BTStatus = "BT:" + GetBTStatus(BTState);
			Serial.println(BTStatus);
			SD_Logging_Event_Messsage(BTStatus);


			//CheckWingSailVersion();

			CheckWingSailPower();

			//(*Serials[Configuration.BluetoothPort]).println("");
			//(*Serials[Configuration.BluetoothPort]).println(F("ver"));
		}
	}
	BTStatePrev = BTState;
};

void BluetoothInitialiseConnection(int BluetoothPort)
{
	// Initialise the Bluetooth module
	// Call this procedure repeatedly with a minimum of 100ms period between calls

	static int BTInitStep;

	switch (BTInitStep)
	{
	case 0:
		(*Serials[BluetoothPort]).println("AT");
		BTInitStep = 1;
		BTState = Initialising;

		Serial.print(F("BT:"));
		Serial.print(GetBTStatus(BTState));
		Serial.print(",");
		Serial.print(BTInitStep);
		Serial.print(":Serial");
		Serial.print(BluetoothPort);
		Serial.print(":");
		Serial.println("AT");
		break;
	case 1:
		(*Serials[BluetoothPort]).println("AT+DEFAULT");
		BTInitStep = 2;
		BTState = Initialising;

		Serial.print(F("BT:"));
		Serial.print(GetBTStatus(BTState));
		Serial.print(",");
		Serial.print(BTInitStep);
		Serial.print(":Serial");
		Serial.print(BluetoothPort);
		Serial.print(":");
		Serial.println("AT+DEFAULT");
		break;
	case 2:
		(*Serials[BluetoothPort]).println(F("AT+ROLE1"));
		BTInitStep = 3;

		Serial.print(F("BT:"));
		Serial.print(GetBTStatus(BTState));
		Serial.print(",");
		Serial.print(BTInitStep);
		Serial.print(":Serial");
		Serial.print(BluetoothPort);
		Serial.print(":");
		Serial.println(F("AT+ROLE1"));
		break;
	case 3:
		(*Serials[BluetoothPort]).println(F("AT+RESET"));
		BTState = Initialised;

		Serial.print(F("BT:"));
		Serial.print(GetBTStatus(BTState));
		Serial.print(":Serial");
		Serial.print(BluetoothPort);
		Serial.print(":");
		Serial.println(F("AT+RESET"));
		break;
	default:
		BTInitStep = 0;
	}
}

String GetBTStatus(BTStateType BTState)
{
	// return a string representation of the Bluetooth state

	switch ( BTState)
	{
	case Idle:			return "Idle";
	case Initialising:	return "Initialising";
	case Initialised:	return "Initialised";
	case Querying:		return "Querying";
	case Connecting:	return "Connecting";
	case Connected:		return "Connected";
	default:			return "unknown";
	};
}



// Command Line Interpreter - Global Variables
char BT_CLI_Msg[50];
unsigned int BT_CLI_i = 0;

// Collect the characters into a command string until end end of line,
// and then process it.
// V1.0 22/12/2015
// called from a 1 second loop (logging loop)
void BT_CLI_Process_Message(int BluetoothPort)
{
	// Accumulate characters in a command string up to a CR or LF or buffer fills. 
	while ((*Serials[BluetoothPort]).available())
	{
		char received = (*Serials[BluetoothPort]).read();
		if (BT_CLI_i < sizeof(BT_CLI_Msg) - 1) // Ensure there is space for the received character and null terminator
		{
			BT_CLI_Msg[BT_CLI_i++] = received;
		}

		// Process message when new line character is received
		if (received == '\n' || received == '\r' || BT_CLI_i == sizeof(BT_CLI_Msg) - 1)
		{
			BT_CLI_Msg[BT_CLI_i] = '\0';

			// only pass strings to CLI Processor if we are connected.
			// the test is here, because we want pre-connection strings to be reflected
			// through to the serial port for diag.
			if (BTState == BTStateType::Connected)
			{
				BT_CLI_Processor(BluetoothPort);
			}					

			BT_CLI_i = 0;
		}
	}
}


// Process the BT Response String.
// Split into command and parameters separated by commas. 
// V1.0 22/12/2015
// V1.1 13/01/2018 added support for a Vessel Command Parameter. i.e. steer a magnetic heading
// V1.2 1/12/2018 changed strcpy to strncpy to guard against corrupting memory with long strings

void BT_CLI_Processor(int BluetoothPort)
{
	// debug
	//Serial.print("BT_CLI_Msg:");
	//Serial.print(BT_CLI_Msg);
	//Serial.println("/BT_CLI_Msg");

	char cmd[4] = "";
	char param1[30] = "";
	char param2[20] = "";
	char param3[12] = "";
	char param4[12] = "";
	char param5[12] = "";
	char param6[12] = "";

	strcat(BT_CLI_Msg, ",");

	// Split into command and parameters separated by commas. 
	strncpy(cmd, strtok(BT_CLI_Msg, ","), sizeof(cmd)-1);
	strncpy(param1, strtok(NULL, ","), sizeof(param1)-1);
	strncpy(param2, strtok(NULL, ","), sizeof(param2)-1);
	strncpy(param3, strtok(NULL, ","), sizeof(param3)-1);
	strncpy(param4, strtok(NULL, ","), sizeof(param4)-1);
	strncpy(param5, strtok(NULL, ","), sizeof(param5)-1);
	strncpy(param6, strtok(NULL, ","), sizeof(param6)-1);


	// ===============================================
	// Wingsail Command wsv, WingSail Voltage.
	// ===============================================
	// Parameter 1: WingSail Battery Voltage
	// Parameter 2: WingSail Battery currrent
	// 
	if (!strncmp(cmd, "pow", 3))
	{
		WingSail.SolarCell_V = atof(param1);
		WingSail.SolarCell_mA = atof(param2);
		WingSail.Charge_V = atof(param3);
		WingSail.Charge_mA = atof(param4);
		WingSail.Discharge_V = atof(param5);
		WingSail.Discharge_mA = atof(param6);

		WingSail.LastPowerResponseTime = millis();

		SD_Logging_Event_Wingsail_Power();
	}

	// gsv - Get Wingsail Servo response
	if (!strncmp(cmd, "gsv", 3))
	{
		WingSail.Servo_microseconds_reponse = atoi(param1);
		WingSail.LastResponseTime = millis();

		SD_Logging_Event_Wingsail_Monitor("Response");
	}

	if (!strncmp(cmd, "ver", 3))
	{
		// log the wingsail version to the SD Card
		strcat(param1, ",");
		strcat(param1, param2);
		
		// strip the CrLf off the end of the string
		// simply shorten the string, as long as its got some length
		if (strlen(param1) > 4)
		{
			param1[strlen(param1) - 1] = '\0';
		}

		strcpy(WingSail.VersionDate, param1);

		SD_Logging_Event_Messsage(param1);
	}
};