/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_NTC.ino
* Tiny Linduino Sketch for LTC2949 Demo Board - NTC, current measurement
* Optional fast continuous measurement of I2, samples read from FIFO
* fast channel cycle time can be adjusted by sending
*  F<ms>, where <ms> = milliseconds. E.g. F25 will report average from samples within FIFO every 25ms.
*  F0: disable fast channel readings
* output format:
	I2f,Nf,I1s,Tntc1,Tntc2,Tic,tDut,OK/ERR
	I2f: Average of current samples from FIFOI2 (in uV)
	Nf: Number of current samples from FIFOI2
	following measurements are from slow channel, updated every 100ms:
	I1s:I1 current (in uV)
	Tntc1:Temperature of NTC1 (shunt temperature)
	Tntc2:Temperature of NTC1 (board temperature)
	Tic:Temperature of IC (LTC2949 internal temperature)
	tDut: delta in ms between slow channel readings (nominal 100 +/-5)
	OK/ERR: error code (OK in case of no error)

	In case of F0 (see above) the output will change to
	I1s,Tntc1,Tntc2,Tic,tDut,OK/ERR

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



// define cycle time of reading FIFO I2 and current measurement average reporting
// must be between 10 - 25 ms for this example, set to 0 to disable
#define LTCDEF_FAST_CURRENT_MS 10
#define LTCDEF_FAST_CURRENT_MS_MIN 10
#define LTCDEF_FAST_CURRENT_MS_MAX 25

uint8_t fast_current_ms = LTCDEF_FAST_CURRENT_MS;

unsigned long fast_current_timeout;
// 4 samples headroom
#define LTC_MAXSAMPLESTOBEREAD ((int)(LTCDEF_FAST_CURRENT_MS_MAX / 0.8 + 4))

#define LTCDEF_REPORT_BAT

#define LTCDEF_DEBUG_OUTPUT
// comment to have debug output of all SPI transactions...
#undef LTCDEF_DEBUG_OUTPUT

// serial baudrate
#define LTCDEF_BAUDRATE 250000

// NTC parameters
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3

// buffer for commands received via serial interface
String SerialInputString = "";

void PrintCSVHeader()
{
	Serial.println(F("I2f,Nf,I1s,Tntc1,Tntc2,Tic,tDut,OK/ERR"));
}

void ProcessSerialCommands()
{
	if (!Serial.available())
		return; // no new character, nothing to do

	// Read from serial
	char character = Serial.read(); // Receive a single character from the software serial port
	if (!IsTermChar(character))
	{
		// Add the received character to the receive buffer (only if not \n or \r)
		SerialInputString.concat(character);
		return; // command not yet terminated, nothing to do
	}

	SerialInputString.trim(); // remove any whitespaces

	if (SerialInputString.length() == 0)
		return; // zero length command, nothing to do

	// Echo the Command
	Serial.print(F("> "));
	Serial.println(SerialInputString);
	// 
	//////////////////////////////////////////////////////////////////////
	///////////////////// BEGIN: PROCESS COMMANDS ////////////////////////
	//////////////////////////////////////////////////////////////////////
	//
	/////////////////////////////// COMAND ///////////////////////////////
	if (startsWith(SerialInputString, F("F")))
	{
		int ms;
		unsigned int iStart = 1;
		stringSplitter(SerialInputString, &iStart, &ms);

		if (ms >= LTCDEF_FAST_CURRENT_MS_MIN && ms <= LTCDEF_FAST_CURRENT_MS_MAX)
			fast_current_ms = ms;
		else
			fast_current_ms = 0;

		// reset FIFO:
		LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2); // write FACTRL, clear FACONV
		LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACONV); // write FACTRL, set FACONV
	}
	else if (equals(SerialInputString, F("ER")))
	{
		//LTC2949_EEPROMRead();
		//LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_RESTORE);
		LTC2949_WRITE(LTC2949_REG_EEPROM, LTC2949_BM_EEPROM_RESTORE);
	}
	else
	{
		Serial.println(F("CmdErr"));
	}
	//
	//////////////////////////////////////////////////////////////////////
	///////////////////// END: PROCESS COMMANDS //////////////////////////
	//////////////////////////////////////////////////////////////////////

	// empty SerialInputString buffer
	SerialInputString = "";
}

// calculate average
void ProcessFastData(int16_t* data, byte  len, int32_t* dSum)
{
	*dSum = 0;
	for (byte i = 0; i < len; i++)
		* dSum += data[i];
}

byte ReadFifoI2(uint16_t* samplesRead, int32_t* sum)
{
	samplesRead[0] = LTC_MAXSAMPLESTOBEREAD;
	int16_t buffer[LTC_MAXSAMPLESTOBEREAD];
	// read samples
	byte error = LTC2949_ReadFifo(
		/*byte      addr:             */ LTC2949_REG_FIFOI2,
		/*uint16_t* len:              */ samplesRead,
		/*int16_t*  samples:          */ buffer,
		/*boolean*  fifoFull1stSample:*/ NULL,
		/*int8_t    rdOvrCntDown:     */ LTC2949_RDFIFO_STOP_EMPTY);
	if (samplesRead[0] != 0)
		// sum all samples for average
		ProcessFastData(buffer, samplesRead[0], sum);
	else
		sum = 0;

	return error;
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
#ifdef LTCDEF_DEBUG_OUTPUT
	LTC2949_DebugEnable = true;
#endif
	WakeUp();
	Cont();
	fast_current_timeout = millis() + fast_current_ms;
}

void loop()
{
	byte  error = 0;

	if (fast_current_ms != 0)
	{
		if (LTC_TIMEOUT_CHECK(millis(), fast_current_timeout))
		{
			// next fast channel cycle time
			fast_current_timeout = millis() + fast_current_ms;
			uint16_t samples;
			int32_t  sum;
			// read fast samples from FIFO
			error = ReadFifoI2(&samples, &sum);
			if (samples == 0)
				return; // no samples
			Serial.print(sum / (double)samples * LTC2949_LSB_FI2, 6);
			PrintComma();
			Serial.print(samples);
			PrintComma();
		}
		else
			return;
	}

	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t deltaTint = LTC2949_GetLastTBxInt();
	// LTC2949_ChkUpdate checks for changed in TBx (default is TB4)
	if (!LTC2949_ChkUpdate(&error))
	{
		if (fast_current_ms != 0)
			Serial.println();
		ProcessSerialCommands();
		return;
	}

	// new slow channel, high precision results available
	// as an example we just report current, NTC / IC temperatures, delta TBx

	byte buffer[3];
	// read slow channel current
	error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
	Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1, 7);
	PrintComma();

	// read slow temperature via SLOT1
	error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(true), 1);
	PrintComma();

	// read slow temperature via SLOT2
	error |= LTC2949_READ(LTC2949_VAL_SLOT2, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(false), 1);
	PrintComma();

	// read slow internal temperature
	error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, 1);
	PrintComma();

#ifdef LTCDEF_REPORT_BAT
	// read slow internal temperature
	error |= LTC2949_READ(LTC2949_VAL_BAT, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, 1);
	PrintComma();
	error |= LTC2949_READ(LTC2949_REG_EEPROM, 1, buffer);
	Serial.print(buffer[0]);
	if (buffer[0] == LTC2949_BM_EEPROM_RESTORERSL)
	{
		LTC2949_WRITE(LTC2949_REG_EEPROM, 0);
		//LTC2949_WRITE(LTC2949_REG_OPCTRL, LTC2949_BM_OPCTRL_ADJUPD | LTC2949_BM_OPCTRL_CONT); // ADJUPD not possible in cont mode.
	}
	PrintComma();
	error |= LTC2949_READ(LTC2949_VAL_BATGC, 3, buffer);
	float gc;
	LTC2949_F24BytesToFloat(buffer, &gc);
	Serial.print(gc, 3);
	PrintComma();
#endif

	// calc. difference between last TBx and current TBx, report milliseconds
	Serial.print((unsigned int)((LTC2949_GetLastTBxInt() - deltaTint) * (1.0e3 * LTC2949_LSB_TB1)));
	PrintComma();

	error |= LTC2949_ReadChkStatusFaults(false, true);
	PrintComma();


	PrintOkErr(error);
	if (error)
	{
		PrintOkErr(error);
		Serial.println(F("STOP, error detected! Reset board to restart."));
		while (true)
			;// stop forever in case of error
	}
	if (LTC2949_DebugEnable)
		while (true)
			;// stop forever

	LTC2949_DebugEnable = false;
}

void WakeUp()
{
	Serial.println(F("Wakeup LTC2949 and acknowledge:"));
	byte  error = LTC2949_WakeupAndAck();
	Serial.println(F("Done, check STATUS/FAULTS"));
	error |= LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	PrintOkErr(error);
}

/*!*********************************************************************
\brief Activate / deactivate measurement in LTC2949
- slow continuous mode
- fast continuous CH2 mode
***********************************************************************/
void Cont()
{
	byte error = 0;

	Serial.println(F("Configure SLOT1/2 inputs:"));

	// SLOT1/2 measure temperature via NTC between V1/V2 and GND. 
	LTC2949_SlotsCfg(1, 0, 2, 0);

	Serial.println(F("Configure NTCs:"));

	// enable NTC temperature measurement via SLOT1
	NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);

	// enable NTC temperature measurement via SLOT1
	NtcCfgWrite(2, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);

	Serial.println(F("Enable continuous measurement mode:"));
	// enable measurement
	error |= LTC2949_GoCont(
		/*cfgFast:      */ LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACONV,
		/*byte adcCfg:  */
		LTC2949_BM_ADCCONF_NTC1 |
		LTC2949_BM_ADCCONF_NTC2 |
		/*LTC2949_BM_ADCCONF_P1ASV |*/
		LTC2949_BM_ADCCONF_P2ASV);

	Serial.print(F("Done"));
	PrintComma();
	PrintOkErr(error);
}

void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c)
{
	byte data[3];
	if (LTC2949_DebugEnable)
	{
		Serial.print(F("NTC"));
		Serial.print(ntc1or2);
		Serial.println(':');
	}

	LTC2949_FloatToF24Bytes(rref, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_RREF2 : LTC2949_VAL_RREF1, 3, data);
	LTC2949_FloatToF24Bytes(a, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2A : LTC2949_VAL_NTC1A, 3, data);
	LTC2949_FloatToF24Bytes(b, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2B : LTC2949_VAL_NTC1B, 3, data);
	LTC2949_FloatToF24Bytes(c, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2C : LTC2949_VAL_NTC1C, 3, data);
}

/*
tDut,Tntc1,Tntc2,Tic,OK/ERR
Wakeup LTC2949 and acknowledge:
	-- Write REGSCTRL with default setting 0x80 (RDCVCONF=true, not relevant here....)
DW:FEFF9E0840806426,OK
	-- Read back REGSCTRL
DR:FEFF9E0880806426,OK
	-- Write WKUPACK to acknowledge wakeup and avoid LTC2949 going back to sleep
DW:FE7095DA40002000,OK
Done,OK
Configure SLOT1/2 inputs:
	-- Write 4 bytes starting at SLOT1MUXN: 0,1,0,2
DW:FEEB74B843000100027AC0,OK
Configure NTCs:
NTC1:
	-- got to second page
DW:FEFF9E084081EF14,OK
	-- write RREF, NTC A,B,C for NTC1
DW:FEAA1800464F86A0E876,OK
DW:FED067A046350237BFAA,OK
DW:FED371C446329A2387B0,OK
DW:FED64B6846274AAD932A,OK
NTC2:
	-- write RREF, NTC A,B,C for NTC2
DW:FEADBFFA464F86A0E876,OK
DW:FEE08AD246350237BFAA,OK
DW:FEE39CB646329A2387B0,OK
DW:FEE6A61A46274AAD932A,OK
Enable continuous measurement mode:
	-- check OPCTRL (here in IDLE mode
DR:FEF0D1FC80002000,OK
	-- write ADCCONF (Note: we are still on second page!)
DW:FEDF2854401B8544,OK
	-- Write CLR|ADJUPD (0x22) to OPCTRL (clear and adjust update can be done at the same time)
DW:FEF0D1FC40220B0A,OK
	-- poll OPCTRL to check for CLR and ADJUPD done
DR:FEF0D1FC80220B0A,OK
DR:FEF0D1FC80002000,OK
	-- Write CONT (0x08) to OPCTRL
DW:FEF0D1FC4008C80E,OK
	-- go to first page
DW:FEFF9E0840806426,OK
	-- poll TB1 for first update (takes up to 160ms the first time from IDLE to CONT to update measurement results)
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B2830000000007BE,OK
DR:FE0CC6B283000000FA3C38,OK
	-- configure fast channel (here 0 as we do not fast measurements....)
DW:FEF5EB5040002000,OK
Done,OK
	-- check for next update of TB1 (now every 100 ms TB1 and all slow channel data is updated)
DR:FE0CC6B283000001F470B2,OK
	-- 99ms
99,
	-- read SLOT1 (NTC1)
DR:FEA641908500713796,OK
	-- 22.6 degree Celcius
22.6,
	-- read SLOT2 (NTC1)
DR:FEA8855685007221F2,OK
	-- 22.8 degree Celcius
22.8,
	-- read TEMP (IC temperature)
DR:FEA2F00E850073AAC0,OK
	-- 23.0 degree Celcius
23.0,OK

*/
