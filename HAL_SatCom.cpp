// 
// 
// 

#include "HAL.h"
#include "HAL_SatCom.h"
#include "Navigation.h"
#include "Mission.h"
#include "HAL_Time.h"
#include "TimeLib.h"
#include "WaveMeasurement.h"
#include "Wingsail.h"
#include "USFSmax.h"
#include "HAL_SDCard.h"
#include "Sd.h"
#include "InternalTemperature.h"
#include <cmath>
#include "CommandState_Processor.h"
#include "astronode.h"
#include "configValues.h"
#include "DisplayStrings.h"

extern HardwareSerial* Serials[];
extern NavigationDataType NavData;
extern uint32_t Minute;
extern MissionValuesStruct MissionValues;
extern File LogFile;
extern HALPowerMeasure PowerSensor;
extern WaveClass Wave;
extern WingSailType WingSail;
extern HALWingAngle WingAngleSensor;
extern HALIMU imu;
extern StateValuesStruct StateValues;
extern configValuesType Configuration;

// Astronode configuration
//#define ASTRONODE_SERIAL Serial4
//#define ASTRONODE_SERIAL_BAUDRATE 9600
#define ASTRONODE_WITH_PLD_ACK true
#define ASTRONODE_WITH_GEO_LOC true
#define ASTRONODE_WITH_EPHEMERIS true
#define ASTRONODE_WITH_DEEP_SLEEP_EN false
#define ASTRONODE_WITH_MSG_ACK_PIN_EN false
#define ASTRONODE_WITH_MSG_RESET_PIN_EN false
#define ASTRONODE_WITH_CMD_EVENT_PIN_EN true
#define ASTRONODE_WITH_TX_PEND_EVENT_PIN_EN false
#define ASTRONODE_PAYLOAD_SIZE 80 //[Bytes]

void HALSatComms::Init()
{
	pinMode(31, INPUT_PULLDOWN);

	if (Configuration.SatCommsEnabled)
	{
		Serial.print("*** Initialising SatComms Port: (S4) ");
		Serial.print(Configuration.SatCommsPort);
		Serial.print(", ");
		Serial.print(Configuration.SatCommsPortBaudRate);
		Serial.println("Baud.");

		//Initialize terminal // (*Serials[Configuration.SatCommsPort])
		(*Serials[Configuration.SatCommsPort]).begin(Configuration.SatCommsPortBaudRate);

		Serial4.begin(9600);
#define TX_SIZE 512
		uint8_t tx_buffer[TX_SIZE];
		(*Serials[Configuration.SatCommsPort]).addMemoryForWrite(tx_buffer, TX_SIZE);

#define RX_SIZE 512
		uint8_t rx_buffer[RX_SIZE];
		(*Serials[Configuration.SatCommsPort]).addMemoryForRead(rx_buffer, RX_SIZE);


		if (astronode.begin(Serial4) == ANS_STATUS_SUCCESS)
		{ // good
			EquipmentStatus = EquipmentStatusType::Found;
		}
		else
		{
			// error
			EquipmentStatus = EquipmentStatusType::NotFound;
		}

	//	Serial.print("read_module_state:");
	//	Serial.println(astronode.read_module_state());
	//	Serial.print("read_module_state:");
	//	Serial.println(astronode.read_module_state());

		astronode.configuration_write(ASTRONODE_WITH_PLD_ACK,
			ASTRONODE_WITH_GEO_LOC,
			ASTRONODE_WITH_EPHEMERIS,
			ASTRONODE_WITH_DEEP_SLEEP_EN,
			ASTRONODE_WITH_MSG_ACK_PIN_EN,
			ASTRONODE_WITH_MSG_RESET_PIN_EN,
			ASTRONODE_WITH_CMD_EVENT_PIN_EN,
			ASTRONODE_WITH_TX_PEND_EVENT_PIN_EN);

	//	astronode.satellite_search_config_write(SAT_SEARCH_2755_MS, true);

		astronode.configuration_save();
		astronode.configuration_read();

		//Clear old messages
		astronode.clear_free_payloads();

		Serial.print(F("Satcomms status: "));
		Serial.println(GetEquipmentStatusString(EquipmentStatus));

		Serial.println(F("*** Initialising Satcomms complete."));
		Serial.println();

		uint8_t data_payload[6] = { "V4-13" };
		uint16_t counter = 0;
		//Try enqueueing a first message in the queue
		if (astronode.enqueue_payload(data_payload, sizeof(data_payload), counter) == ANS_STATUS_SUCCESS) {
			counter++;
			Serial.println(F("enqueue_payload"));
		}
	}
}

void HALSatComms::Read()
{
	if (Configuration.SatCommsEnabled)
	{
		//Serial.print("read_performance_counter:");
		//Serial.println(astronode.read_performance_counter());

		Serial.print("read_module_state:");
		Serial.println(astronode.read_module_state());

		//Serial.print("read_environment_details:");
		//Serial.println(astronode.read_environment_details());

		//Serial.print("read_last_contact_details:");
		//Serial.println(astronode.read_last_contact_details());

		//Query HK and RTC
		uint32_t rtc_time;
		//if (astronode.read_performance_counter() == ANS_STATUS_SUCCESS &&
		//	astronode.read_module_state() == ANS_STATUS_SUCCESS &&
		//	astronode.read_environment_details() == ANS_STATUS_SUCCESS &&
		//	astronode.read_last_contact_details() == ANS_STATUS_SUCCESS &&
		//	astronode.rtc_read(&rtc_time) == ANS_STATUS_SUCCESS)
		//{
		//	// read status flags etc.
		//	RSSI=astronode.end_struct.last_sat_search_peak_rssi;
		//	astronode.read_next_contact_opportunity(&timeToNextSat);

		//	OutboundMsgCount = astronode.mst_struct.msg_in_queue;

		//	//timeValue = rtc_time - 946684800; // difference between unix epoch and y2k epoch <---- Arduino only
		//	timeValue = rtc_time; //  Pico PI. no need for offset
		//	uint32_t TimeZoneOffset = (uint32_t)3600 * Configuration.timezone_offset;
		//	timeValue += TimeZoneOffset;
		//}
		//else
		//{
		//	OutboundMsgCount = -1;
		//}

		//Query and process events
		uint8_t event_type;
		astronode.event_read(&event_type);
		switch (event_type)
		{
		case EVENT_MSG_ACK:
		{
			//If satellite ACK event, read and clear event to be able to queue a new message
			uint16_t counter_read = 0;
			if (astronode.read_satellite_ack(&counter_read) == ANS_STATUS_SUCCESS)
			{
				astronode.clear_satellite_ack();
			}
			break;
		}
		case EVENT_RESET:
		{
			//If reset event, clear event
			astronode.clear_reset_event();
			break;
		}
		case EVENT_CMD_RECEIVED:
		{
			//If command event, read command, save,then clear command
			uint8_t data40[40] = { 0 };
			uint8_t data8[8] = { 0 };
			uint32_t createdDate = 0;

			// check for 40 byte command
			if (astronode.read_command_40B(data40, &createdDate) == ANS_STATUS_SUCCESS &&
				astronode.rtc_read(&rtc_time) == ANS_STATUS_SUCCESS)
			{
				char str[(sizeof data40) + 1];
				memcpy(str, data40, sizeof data40);
				str[sizeof data40] = 0; // Null termination.
				LastCommand = str;
				SD_Logging_Event_Messsage(LastCommand);

				astronode.clear_command();
			}

			// check for 8 byte command
			if (astronode.read_command_8B(data8, &createdDate) == ANS_STATUS_SUCCESS &&
				astronode.rtc_read(&rtc_time) == ANS_STATUS_SUCCESS)
			{
				char str[(sizeof data8) + 1];
				memcpy(str, data8, sizeof data8);
				str[sizeof data8] = 0; // Null termination.
				LastCommand = str;
				SD_Logging_Event_Messsage(LastCommand);

				astronode.clear_command();
			}

			break;
		}
		}
	}
}

void HALSatComms::QueueMessage(uint8_t* OutboundMsg, uint8_t OutboundMsgSize)
{
	//Try enqueueing a message in the queue
	OutboundMsgCount = astronode.mst_struct.msg_in_queue;

	astronode.enqueue_payload(OutboundMsg, OutboundMsgSize, OutboundMsgCount);

	//if (astronode.enqueue_payload(OutboundMsg, OutboundMsgSize, OutboundMsgCount) == ANS_STATUS_SUCCESS) {
	//}
}

void HALSatComms::sendSatComVesselState()
{
	//Set geolocation
	astronode.geolocation_write((int32_t)(NavData.Currentloc.lat * 10000000), (int32_t)(NavData.Currentloc.lng * 10000000));

	byte Message[50];

	Message[0] = byte('V');
	Message[1] = byte(year()-2000);
	Message[2] = byte(month());
	Message[3] = byte(day());
	Message[4] = byte(hour());
	Message[5] = byte(minute());
	Message[6] = byte(second());
	Message[7] = byte(PowerSensor.BatteryOut_V * 25);
	Message[8] = byte(PowerSensor.BatteryOut_I);
	Message[9] = byte(WingSail.Discharge_V * 50);
	Message[10] = byte(NavData.ROLL_Avg + 100);
	Message[11] = byte(NavData.COG / 2);
	Message[12] = byte(NavData.COG_Avg / 2);
	Message[13] = byte(WingSail.Angle / 2);
	Message[14] = byte(NavData.SOG_Avg * 10); // m/s
	Message[15] = byte(NavData.SOG_mps * 10); // m/s
	Message[16] = byte(NavData.HDG_Mag / 2);
	Message[17] = byte(NavData.BTW / 2);
	Message[18] = byte(NavData.DTW / 1000);
	Message[19] = byte(NavData.CTE / 10);
	Message[20] = byte(NavData.MaxCTE / 10);
	Message[21] = byte(NavData.IsBTWSailable) ? 'Y':'N';
	Message[22] = byte(NavData.CourseType);
	Message[23] = byte(NavData.PointOfSail);
	Message[24] = byte(WingAngleSensor.WingSailAngleSensorPort.temperature * 5); //temp *5
	Message[25] = byte(InternalTemperature.readTemperatureC() * 5); //temp *5
	Message[26] = byte(imu.Baro - 900); // baro pressure -900
	Message[27] = byte(WingSail.TimeSinceLastReponse / 100);
	Message[28] = byte(StateValues.mission_index);

	double MantissaLat;
	double IntegerLat;
	double MantissaLng;
	double IntegerLng;

	// current location
	IntegerLat = std::modf(float(NavData.Currentloc.lat) / 10000000UL, &MantissaLat);
	Message[29] = byte(IntegerLat + 90);
	Message[30] = byte(MantissaLat * 100);
	IntegerLng = std::modf(float(NavData.Currentloc.lng) / 10000000UL, &MantissaLng);
	Message[31] = byte(IntegerLng);
	Message[32] = byte(MantissaLng * 100);

// prev waypoint
	IntegerLat = std::modf(float(NavData.prev_WP.lat) / 10000000UL, &MantissaLat);
	Message[33] = byte(IntegerLat + 90);
	Message[34] = byte(MantissaLat * 100);
	IntegerLng = std::modf(float(NavData.prev_WP.lng) / 10000000UL, &MantissaLng);
	Message[35] = byte(IntegerLng);
	Message[36] = byte(MantissaLng * 100);

	// next waypoint
	IntegerLat = std::modf(float(NavData.next_WP.lat) / 10000000UL, &MantissaLat);
	Message[37] = byte(IntegerLat + 90);
	Message[38] = byte(MantissaLat * 100);
	IntegerLng = std::modf(float(NavData.next_WP.lng) / 10000000UL, &MantissaLng);
	Message[39] = byte(IntegerLng);
	Message[40] = byte(MantissaLng * 100);

	Message[41] = byte('z'); // terminator

	QueueMessage(Message, 42);
}

void HALSatComms::sendSatComMissionEvent(int mission_index)
{
	// sat com message - new mission step

	//Set geolocation
	astronode.geolocation_write((int32_t)(NavData.Currentloc.lat * 10000000), (int32_t)(NavData.Currentloc.lng * 10000000));

	byte Message[50];

	Message[0] = byte('M');
	Message[1] = byte(year() - 2000);
	Message[2] = byte(month());
	Message[3] = byte(day());
	Message[4] = byte(hour());
	Message[5] = byte(minute());
	Message[6] = byte(second());

	double MantissaLat;
	double IntegerLat;
	double MantissaLng;
	double IntegerLng;

	// current location
	IntegerLat = std::modf(float(NavData.Currentloc.lat) / 10000000UL, &MantissaLat);
	Message[7] = byte(IntegerLat + 90);
	Message[8] = byte(MantissaLat * 100);

	IntegerLng = std::modf(float(NavData.Currentloc.lng) / 10000000UL, &MantissaLng);
	Message[9] = byte(IntegerLat + 90);
	Message[10] = byte(MantissaLat * 100);

	Message[11] = byte(mission_index);
	Message[12] = byte(MissionValues.MissionList[mission_index].cmd);
	Message[13] = byte(MissionValues.MissionList[mission_index].duration);
	Message[14] = byte(MissionValues.MissionList[mission_index].boundary/10);
	Message[15] = byte(MissionValues.MissionList[mission_index].SteerAWA/2);
	Message[16] = byte(MissionValues.MissionList[mission_index].TrimTabAngle);

	// next waypoint
	IntegerLat = std::modf(float(MissionValues.MissionList[mission_index].waypoint.lat) / 10000000UL, &MantissaLat);
	Message[17] = byte(IntegerLng);
	Message[18] = byte(MantissaLng * 100);

	IntegerLng = std::modf(float(MissionValues.MissionList[mission_index].waypoint.lng) / 10000000UL, &MantissaLng);
	Message[19] = byte(IntegerLng);
	Message[20] = byte(MantissaLng * 100);

	Message[21] = byte('z'); // terminator

	QueueMessage(Message, 22);
}


