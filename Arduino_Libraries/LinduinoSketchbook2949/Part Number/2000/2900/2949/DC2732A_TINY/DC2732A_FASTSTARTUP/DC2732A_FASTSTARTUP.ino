/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_FASTSTARTUP.ino
* Tiny Linduino Sketch for LTC2949 Demo Board - fast init: Go CONT, make fast single shot
*
*
* created: Patrick Wilhelm (PW)
*/

#include <Arduino.h>
#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>

// DC2792A has two isoSPI ports: AUX and MAIN
#define LTCDEF_CS_AUX  9
#define LTCDEF_CS_MAIN 10
// in case of DC2792A select AUX or MAIN port
// default for all other isoSPI boards should be MAIN
#define LTCDEF_CS_SELECT LTCDEF_CS_MAIN


// serial baudrate
#define LTCDEF_BAUDRATE 250000

uint8_t optRstP2ASVClr = 0;

void PrintCSVHeader()
{
	Serial.println(F("TAG0,M0: VREF/-VREF,BAT,TAG1,M1: VNTC / -VNTC,BAT,Tinit,randDelay,dbgCnt,OK/ERR"));
}

void setup()
{
	unsigned long startOfTheDay = millis() + LTC2949_TIMING_BOOTUP;

	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// wait for serial port to connect. Needed for native USB port only
	while (!Serial)
		;
	PrintCSVHeader();

	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	LinduinoSelectSPI(LTCDEF_CS_SELECT);


	while (!LTC_TIMEOUT_CHECK(millis(), startOfTheDay))
		;
	WakeUp();
	LTC2949_OpctlIdle(); // clear OPCTRL
	delay(200);
}

void loop()
{
	boolean inv = false;

	boolean p2asv = bitMaskSetChk(optRstP2ASVClr, 4);
	boolean clr = bitMaskSetChk(optRstP2ASVClr, 2);
	boolean rst = bitMaskSetChk(optRstP2ASVClr, 1);
	optRstP2ASVClr = (optRstP2ASVClr + 1) % 8;

	Serial.print(F("RST:"));
	Serial.println(rst);
	Serial.print(F("p2asv:"));
	Serial.println(p2asv);
	Serial.print(F("clr:"));
	Serial.println(clr);

	// rand sleep used to add some random timing....
	float randSleep = 10.123;
	for (uint8_t i = 0; i < (rst ? 3 : 20); i++)
	{
		uint8_t dbgCnt;
		byte  error;
		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
		bool hsOK;
		unsigned long endTime;

		// start STOP watch
		unsigned long startTime = micros();
		error = LTC2949_WakeupAndAck();

		if (p2asv || clr)
		{
			if (p2asv)
				LTC2949_ADCConfigPasV(LTC2949_BM_ADCCONF_P2ASV);
			else
				LTC2949_ADCConfigPasV(0);
			LTC2949_ADCConfigWrite();
			LTC2949_OpctlAdjUpd(clr);
		}

		LTC2949_OpctlCont(); // write CONT to OPCTRL
		unsigned long timeout = millis() + 200;
		if (inv)
			LTC2949_SlotFastCfg(LTC2949_SLOTMUX_GND, LTC2949_SLOTMUX_VREF2);
		else
			LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2, LTC2949_SLOTMUX_GND);
		inv = !inv;
		while (true)
		{
			// clear HS byte
			//error |= LTC2949_RdFastData();
			// enable fast mode for I2 and AUX
			LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA);
			// trigger fast conversion
			LTC2949_ADxxAddressed();
			// STOP watch ends with 1st conversion ready to be read
			endTime = micros() + 1208; // +8 because of 8us granularity of Linduino!
			LTC2949_READ(LTC2949_REG_DBGCNT, 1, &dbgCnt); // just for debugging
			// wait for worst case conversion and update time
			while (!LTC_TIMEOUT_CHECK(micros(), endTime))
				; //delayMicroseconds(1208);
			// read conversion result
			error |= LTC2949_RdFastData(fastData2949);
			hsOK = LTC2949_FASTSSHT_HS_OK(fastData2949);
			if (LTC_TIMEOUT_CHECK(millis(), timeout) || hsOK)
				break;
		}

		// make 2nd fast conversion (VNTC)
		if (inv)
			LTC2949_SlotFastCfg(LTC2949_SLOTMUX_GND, LTC2949_SLOTMUX_V1);
		else
			LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V1, LTC2949_SLOTMUX_GND);
		LTC2949_ADxxAddressed();

		// calculate total time from STANDBY or SLEEP/POR to first measurement result
		float totalTime = (endTime - startTime)*1e-3;

		// calc end of 2nd measurement
		endTime = micros() + 1208; // +8 because of 8us granularity of Linduino!

		randSleep += totalTime * 1.31254;

		bool invOK = true;

		randSleep += fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX;
		randSleep += fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2;

		invOK = invOK && ((fastData2949[LTC2949_RDFASTDATA_AUX] < 0) != inv);

		// print previous conversion results
		SerialPrintByteArrayHex((byte*)&(fastData2949[LTC2949_RDFASTDATA_HS]), 2, true);
		PrintComma();
		Serial.print(fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX, 4);
		PrintComma();
		Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FBAT, 4);
		PrintComma();

		// wait conv. done
		while (!LTC_TIMEOUT_CHECK(micros(), endTime))
			; //delayMicroseconds(1208);
		// read VNTC conversion result
		error |= LTC2949_RdFastData(fastData2949);

		hsOK = hsOK && LTC2949_FASTSSHT_HS_OK(fastData2949);

		randSleep += fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX;
		randSleep += fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2;

		invOK = invOK && ((fastData2949[LTC2949_RDFASTDATA_AUX] < 0) == inv);

		// print VNTC conversion results
		SerialPrintByteArrayHex((byte*)&(fastData2949[LTC2949_RDFASTDATA_HS]), 2, true);
		PrintComma();
		Serial.print(fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX, 4);
		PrintComma();
		Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FBAT, 4);
		PrintComma();

		if (randSleep < 0)
			randSleep = -randSleep;
		if (randSleep > 100)
			randSleep -= (((unsigned long)randSleep) / 100) * 100;

		Serial.print(totalTime, 3);
		PrintComma();
		Serial.print(randSleep, 3);
		PrintComma();
		Serial.print(dbgCnt);
		PrintComma();
		if (!invOK)
			Serial.print(F("invErr|"));
		if (!hsOK)
			Serial.print(F("hsErr|"));
		PrintOkErr(error);

		if (rst)
		{
			LTC2949_reset();
			delay(2000); // wee need to give enough time for BYP going down!
		}
		else
			// we always start from IDLE for the next round
			LTC2949_OpctlIdle(); // clear OPCTRL

		delay(LTC2949_TIMING_IDLE2CONT2UPDATE + randSleep);
	}
}

void WakeUp()
{
	Serial.println(F("Wakeup LTC2949 and acknowledge:"));
	byte  error = LTC2949_WakeupAndAck();
	Serial.print(F("Done"));
	PrintComma();
	PrintOkErr(error);
}

/*
Fastest init timing:

With P2 making fast BAT measurements the timings are:
From STANDBY <30ms
From SLEEP <90ms
From POR (power-on-reset) <120ms

*/