/*
* Linear Technology / Analog Devices, Ismaning Design Center
* DC2732A_EXT (Linduino Sketch for LTC2949 Demo Board - Isolated Battery Meter for EV / HEV)
*
* for command description see DC2732A_EXT.docx /.pdf
*
* created:        Patrick Wilhelm (PW)
* last Revision:
* $Revision: 534 $
* $Author: pwilhelm $
* $Date: 2017-07-07 15:09:05 +0200 (Fri, 07 Jul 2017) $
*
*
*/

// DC2792A has two isoSPI ports: AUX and MAIN
#define LTCDEF_CS_AUX  9
#define LTCDEF_CS_MAIN 10
// in case of DC2792A select AUX or MAIN port
// default for all other isoSPI boards should be MAIN
#define LTCDEF_CS_SELECT LTCDEF_CS_MAIN


// serial baudrate
#define LTCDEF_BAUDRATE 250000

#include <Arduino.h>
#include "Linduino.h"
#include <SPI.h>
#include "LTC2949.h"
#include "ltcmuc_tools.h"

#define CFG_INDIRECT_READ      1<<0
#define CFG_BROADCAST_READ     1<<1
#define CFG_AUTO_WRPECCHK      1<<2
#define CFG_RDCV_MUL_6         1<<3

#define PIN_TRIG_OUT  3
#define PIN_TRIG_IN   2
#define LG_DBL_OUTPUT_DIGITS 3

// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8

// NTC: Murata, NTHCG143/Murata (leaded)
#define NTC_STHW_A  8.39126e-4
#define NTC_STHW_B  2.08985e-4
#define NTC_STHW_C  7.13241e-8

// for both:
#define NTC_RREF   100e3

#define LogChannels_FACH1      'e'
#define LogChannels_FACH2      'f'
#define LogChannels_FACHBAT    'g'
#define LogChannels_FACHA      'h'

String SerialInputString = "";

String LogChannels;
double* LogChannelsSum = NULL;
uint16_t LogChannelsMeasCount;
boolean LogChannelsStop;
boolean LogChannelsReportEnable;
uint16_t LogChannelsStopCount;
unsigned long LogTimeout = 0;
byte LogTimeoutInc100Millis = 0; // multiples of 100ms

// Note: cfgFast mirrors LTC2949's register FACTRL. In FACTRL only bits 3..0 are defined, others are don't care. 
//       Here we use bits 7,6 to store additional information on how to process fast data
byte cfgFast;
#define LTCDEF_CFGFAST_BM               ((1 << 7) | (1 << 6))
#define LTCDEF_CFGFAST_CONT_LAST_SAMPLE (1 << 7)
#define LTCDEF_CFGFAST_RESERVED         (1 << 6)
//inline boolean IS_FAST_CONT_FIFO() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV | LTCDEF_CFGFAST_BM)) == (LTC2949_BM_FACTRL_FACONV); }
//inline boolean IS_FAST_CONT_FIFO_AND_LAST_SAMPLE() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV | LTCDEF_CFGFAST_BM)) == (LTC2949_BM_FACTRL_FACONV | LTCDEF_CFGFAST_CONT_LAST_SAMPLE); }
//inline boolean IS_FAST_CONT_LAST_SAMPLE() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV | LTCDEF_CFGFAST_BM)) == (LTC2949_BM_FACTRL_FACONV | LTCDEF_CFGFAST_CONT_LAST_SAMPLE); }
//inline boolean IS_FAST_SSHT() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV | 1 << 7 | 1 << 6)) == (1 << 7 | 1 << 6); }
//inline boolean IS_FAST_SSHT_OR_LAST_SAMPLE() { return (cfgFast & (1 << 7 | 1 << 6)) != 0; }
//inline boolean IS_FAST_CONT() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV)) == LTC2949_BM_FACTRL_FACONV; }
//inline boolean IS_NOT_FAST_CONT() { return (cfgFast & (LTC2949_BM_FACTRL_FACONV)) == 0; }
//inline boolean IS_ANY_FAST() { return (cfgFast & (LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)) != 0; }
//inline boolean IS_NOT_FAST() { return (cfgFast & (LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)) == 0; }

inline boolean DO_RDCV()
{
	if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA))
		return false; // no fast channel active
	if (bitMaskSetChk(cfgFast, LTCDEF_CFGFAST_CONT_LAST_SAMPLE | LTC2949_BM_FACTRL_FACONV))
		return true; // continuous and report last sample
	if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACONV))
		return true; // fast single shot
	return false; // no fast channel active
}


//inline void MAKE_FAST_CONT_FIFO()					{ cfgFast |= (LTC2949_BM_FACTRL_FACONV); }
//inline void MAKE_FAST_CONT_FIFO_AND_LAST_SAMPLE()	{ cfgFast |= (LTC2949_BM_FACTRL_FACONV | 1 << 6); }
//inline void MAKE_FAST_CONT_LAST_SAMPLE()			{ cfgFast |= (LTC2949_BM_FACTRL_FACONV | 1 << 7); }
//inline void MAKE_FAST_SSHT()						{ cfgFast |= (1 << 7 | 1 << 6); }

uint16_t fastFifoCnt;
uint32_t lastUInt32;
double RSNS_NOM;

boolean trigLowHigh;
boolean trig01HighZ;
volatile boolean extTriggerEvent;

void MainInit()
{
	LogChannels = "";
	if (LogChannelsSum)
		delete[] LogChannelsSum;
	LogChannelsSum = NULL;
	LogChannelsMeasCount = 0;
	LogChannelsStop = false;
	LogChannelsStopCount = 1;
	LogChannelsReportEnable = false;
	cfgFast = 0;
	fastFifoCnt = 10;
	lastUInt32 = 0;
	RSNS_NOM = 100e-6;
	extTriggerEvent = false;
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*cellMonitorCount:         */0,
		/*ltc2949onTopOfDaisychain: */false,
		/*debugEnable:              */false
	);

	LTC2949_init_device_state();
	LinduinoSelectSPI(LTCDEF_CS_SELECT);
}

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
	Serial.begin(LTCDEF_BAUDRATE);
	MainInit();
	InitExtTrigger();
}

/*!*********************************************************************
  \brief main loop
  ***********************************************************************/
void loop()
{
	ReportMeasurements();

	ProcessExtTrigger();

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

	EchoCommand();

	// process commands
	if (/*     */equals(SerialInputString, F("T")))           extTriggerEvent = true;
	else if (/**/equals(SerialInputString, F("V")))           LTC2949_GetSiliconRevision(/*revCode: */NULL, /*verbose: */true);
	else if (/**/equals(SerialInputString, F("RESET")))       Reset();
	else if (startsWith(SerialInputString, F("GPO")))         SetPin();
	else if (/**/equals(SerialInputString, F("EW")))          PrintOkErr(LTC2949_EEPROMWrite());
	else if (/**/equals(SerialInputString, F("ER")))          PrintOkErr(LTC2949_EEPROMRead());
	else if (startsWith(SerialInputString, F("WK")))          WakeUpReportStatus(toIntAuto(SerialInputString.substring(2)) != 0);
	else if (startsWith(SerialInputString, F("TBC"))) { LTC2949_WriteTbctl(SerialInputString.substring(3).toFloat(), true); PrintOkErr(0); }
	else if (startsWith(SerialInputString, F("RS")))          GetSetRSNS();
	else if (startsWith(SerialInputString, F("NTC")))         GetSetNTC();
	else if (startsWith(SerialInputString, F("TC")))          ShuntTCComp();
	else if (startsWith(SerialInputString, F("rd")))          SerialRead(true);
	else if (startsWith(SerialInputString, F("wr")))          SerialWrite(true);
	else if (startsWith(SerialInputString, F("RD")))          SerialRead(false);
	else if (startsWith(SerialInputString, F("WR")))          SerialWrite(false);
	else if (startsWith(SerialInputString, F("SPI")))         ConfigSPI();
	else if (startsWith(SerialInputString, F("LG")))          MeasureLG();
	else if (startsWith(SerialInputString, F("lg")))          ConfigFifoReport();
	else if (/**/equals(SerialInputString, F("RA")))          ReportAccus();
	else if (startsWith(SerialInputString, F("IN")))          Init();
	else if (startsWith(SerialInputString, F("CA")))          WriteCalData(false, 2);
	else if (startsWith(SerialInputString, F("ca")))          WriteCalData(true, 2);// write calibration (reduced F24)
	else if (startsWith(SerialInputString, F("RCA")))         ReadCalData(false, 3);
	else if (startsWith(SerialInputString, F("rca")))         ReadCalData(true, 3);// read calibration (reduced F24)
	else if (startsWith(SerialInputString, F("DB")))          DebugEnable(); // debug enable / disable (communication data RAW output)
	else if (startsWith(SerialInputString, F("SC")))          SlotsConfig(); // Slots (fast and slow) config
	else if (startsWith(SerialInputString, F("T")))           ConfigExtTrigger();
	else Serial.println(F("CmdErr")); // unknown command
	// empty SerialInputString buffer
	SerialInputString = "";
}

void Reset()
{
	LTC2949_reset();
	PrintOkErr(0);
}

void DebugEnable()
{
	Serial.println(LTC2949_DebugEnable = (toIntAuto(SerialInputString.substring(2)) != 0));
	PrintOkErr(0);
}

void ConfigFifoReport()
{
	if (SerialInputString.length() > 2)
	{
		Serial.println(fastFifoCnt = SerialInputString.substring(2).toInt());
		PrintOkErr(0);
		return;
	}
	PrintOkErr(LTC2949_ERRCODE_OTHER);
}

void GetSetRSNS()
{
	if (SerialInputString.length() > 2)
		RSNS_NOM = SerialInputString.substring(2).toFloat();
	Serial.println(float2Scientific(RSNS_NOM, 6));
	PrintOkErr(0);
}

void ConfigSPI()
{
	float clock = LTC2949_DEFAULT_SPIFREQU;
	unsigned int iStart = 3;

	if (SerialInputString.length() > 3)
		stringSplitter(SerialInputString, &iStart, &clock);
	LTC2949_SPISettings = SPISettings(clock, MSBFIRST, LTC2949_DEFAULT_SPIMODE);

	Serial.print(clock, 0);
	PrintComma();
	PrintOkErr(0);
}

void ReportMeasurements()
{
	unsigned long updateTimeStamp;
	byte error = 0;

	if ((LogChannels.length() != 0) &&
		(LogTimeoutInc100Millis > 0
			? LTC_TIMEOUT_CHECK(updateTimeStamp = millis(), LogTimeout)
			: LTC2949_ChkUpdate(&error, &updateTimeStamp)))
	{
		LogTimeout = updateTimeStamp + 100UL * LogTimeoutInc100Millis;
		String str = "";
		int16_t* fastData2949 = NULL;
		LogChannelsMeasCount++;

		if ((fastData2949 == NULL) && DO_RDCV())
		{
			// fast single shot or read of last sample requested
			// and fast data not yet read
			fastData2949 = new int16_t[LTC2949_RDFASTDATA_LENGTH];
			// (trigger fast single shot conversion) and read result
			error |= FastSSHTOnDemand(fastData2949, bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACONV));//IS_NOT_FAST_CONT());
		}

		// go through all channels
		for (byte i = 0;;)
		{
			String strChannel = PrintConversion(LogChannels.charAt(i), &error,
				LogChannelsSum
				? LogChannelsSum + i
				: NULL, fastData2949);

			if (LogChannelsReportEnable)
				str += strChannel;

			i++;
			if (i < LogChannels.length())
			{
				if (LogChannelsReportEnable && strChannel.length() > 0)
					str += ',';
			}
			else
			{
				if (LogChannelsReportEnable)
					Serial.println(str);
				break;
			}
		}

		if (fastData2949)
			delete[] fastData2949;

		if (error)
			PrintOkErr(error);
		else
			TrigOutSet(false);
	}
	else
	{
		if (error)
			PrintOkErr(error);
		updateTimeStamp = millis();
	}

	if (LogChannelsStop && (LogChannelsMeasCount >= LogChannelsStopCount))
	{
		LogChannelsStop = false;
		StopConversion();
		Serial.print(updateTimeStamp);
		PrintComma();
		PrintOkErr(error);
	}
}

void StopConversion()
{
	TrigOutSet(false);
	LTC2949_WriteFastCfg(0);
	LTC2949_OpctlIdle();
	if (LogChannelsSum)
	{
		if (LogChannels.length() > 0)
		{
			for (byte i = 0;;)
			{
				// calc and print average
				Serial.print(LogChannelsSum[i] / LogChannelsMeasCount, 7);
				i++;
				PrintComma();
				if (i >= LogChannels.length())
				{
					Serial.println(LogChannelsMeasCount);
					break;
				}
			}
		}
		delete[] LogChannelsSum;
	}
	LogChannelsMeasCount = 0;
	LogChannelsSum = NULL;
	LogChannels = "";
}

byte FastSSHTOnDemand(int16_t* fastData2949, bool sendAdcv)
{
	byte error = 0;
	if (sendAdcv)
	{
		// make fast single shot
		// trigger measurement (broadcast command will trigger cell voltage and current measurement)
		error = LTC2949_ADxx(
			/*byte md = MD_NORMAL     : */MD_FAST,
			/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
			/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
			/*uint8_t pollTimeout = 0 : */0
		);
	}

	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(fastData2949);

	return error;
}

void ReportAccus()
{
#define ACCU_C_PREC 3
#define ACCU_E_PREC 3
#define ACCU_TB_PREC 1
	byte data[16];
	byte error = LTC2949_READ(LTC2949_VAL_C1, 16, data);

	Serial.print(LTC2949_BytesToDouble(data + 0 + 0, 6, true, LTC2949_LSB_C1 / RSNS_NOM), ACCU_C_PREC);  PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 0 + 6, 6, true, LTC2949_LSB_E1 / RSNS_NOM), ACCU_E_PREC);  PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 6 + 6, 4, true, LTC2949_LSB_TB1), ACCU_TB_PREC); PrintComma();

	error |= LTC2949_READ(LTC2949_VAL_C2, 16, data);

	Serial.print(LTC2949_BytesToDouble(data + 0 + 0, 6, true, LTC2949_LSB_C2 / RSNS_NOM), ACCU_C_PREC);  PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 0 + 6, 6, true, LTC2949_LSB_E2 / RSNS_NOM), ACCU_E_PREC);  PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 6 + 6, 4, true, LTC2949_LSB_TB2), ACCU_TB_PREC); PrintComma();

	error |= LTC2949_READ(LTC2949_VAL_C3, 12, data);

	Serial.print(LTC2949_BytesToDouble(data + 0, 8, true, LTC2949_LSB_C3 / RSNS_NOM), ACCU_C_PREC);	PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 8, 4, true, LTC2949_LSB_TB3), ACCU_TB_PREC);	PrintComma();

	error |= LTC2949_READ(LTC2949_VAL_E4, 12, data);

	Serial.print(LTC2949_BytesToDouble(data + 0, 8, true, LTC2949_LSB_E4 / RSNS_NOM), ACCU_E_PREC);	PrintComma();
	Serial.print(LTC2949_BytesToDouble(data + 8, 4, true, LTC2949_LSB_TB4), ACCU_TB_PREC);	Serial.println();

	PrintOkErr(error);
}

void SetPin()
{
	unsigned int iStart = 3;
	int pin = 3, modeLevel = INPUT << 1 | LOW << 0;

	if (stringSplitter(SerialInputString, &iStart, &pin))
		stringSplitter(SerialInputString, &iStart, &modeLevel);

	pin &= 0xF;
	modeLevel &= 3;
	if (pin > 5 || pin < 1)
	{
		PrintOkErr(LTC2949_ERRCODE_OTHER);
		return;
	}
	char m = 'O', l = 'Z';
	LTC2949_GpioConfig(pin, modeLevel);
	if (modeLevel == LTC2949_GPIO_INPUT)
		m = 'I';
	else if (modeLevel == LTC2949_GPIO_OUTPUT_LOW)
		l = 'L';
	else if (modeLevel == LTC2949_GPIO_OUTPUT_TGL)
		l = 'T';
	else
		l = 'H';
	Serial.print(pin);
	Serial.print(m);
	Serial.print(l);
	PrintComma();
	LTC2949_GpioCurrConfigWrite();
	PrintOkErr(0);
}

void GetSetNTC()
{
	unsigned int iStart = 3;
	int ntc1or2;
	byte error = 0;
	boolean additionalParameters = stringSplitter(SerialInputString, &iStart, &ntc1or2);

	if (ntc1or2 == 0)
	{
		// default NTC soldered on demoboard
		NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C, true);
	}
	else if (ntc1or2 == 3)
	{
		// default wired NTC soldered on demoboard and glued to shunt (50u shunt version)
		NtcCfgWrite(1, NTC_RREF, NTC_STHW_A, NTC_STHW_B, NTC_STHW_C, true);
	}
	else if (ntc1or2 == 4)
	{
		// default NTC soldered on demoboard
		NtcCfgWrite(2, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C, true);
	}
	else if (ntc1or2 == 5)
	{
		// default wired NTC soldered on demoboard and glued to shunt (50u shunt version)
		NtcCfgWrite(2, NTC_RREF, NTC_STHW_A, NTC_STHW_B, NTC_STHW_C, true);
	}
	else if (ntc1or2 == 1 || ntc1or2 == 2)
	{
		if (additionalParameters)
		{
			float rref, a, b, c;
			if (stringSplitter(SerialInputString, &iStart, &a)
				&& stringSplitter(SerialInputString, &iStart, &b)
				&& stringSplitter(SerialInputString, &iStart, &c))
			{
				stringSplitter(SerialInputString, &iStart, &rref);
				NtcCfgWrite(ntc1or2, rref, a, b, c, true);
			}
			else
				error = LTC2949_ERRCODE_OTHER;
		}
		else
		{
			error = NtcCfgRead(ntc1or2);
		}
	}
	else
	{
		error = LTC2949_ERRCODE_OTHER;
	}
	PrintComma();
	PrintOkErr(error);
}


#define LTC2949_GOCONT_NTC_NO    0
#define LTC2949_GOCONT_NTC_BOTH  3
#define LTC2949_GOCONT_NTC_1     1
#define LTC2949_GOCONT_NTC_2     2

#define LTC2949_GOCONT_PASV_NO    0
#define LTC2949_GOCONT_PASV_1     1
#define LTC2949_GOCONT_PASV_2     2
#define LTC2949_GOCONT_PASV_BOTH  3

#define FLAG_(XYZ,A) bitMaskSetChk(XYZ,  A)

void MeasureLG()
{
	byte error;
	if (SerialInputString.length() >= 5)
	{
		if (LogChannelsSum)
			delete[] LogChannelsSum;

		LogChannelsMeasCount = 0;
		LogChannelsSum = NULL;

		// defines which fats mode and how to read fast channel data:
		// 0: Continuous FIFO only
		// 1: Continuous FIFO and last sample
		// 2: Continuous Last sample only
		// 3: Fast single shot
		// parameter is ignored if no channel is set for fast conversion (LogChannels_FACH...)
		cfgFast     /**/ = SingleAscii2uint8(SerialInputString.charAt(5 - 3 /*2*/)); // x FLAG
		byte cfgNtc /**/ = SingleAscii2uint8(SerialInputString.charAt(5 - 2 /*3*/)); // y FLAG
		byte cfgPasv/**/ = SingleAscii2uint8(SerialInputString.charAt(5 - 1 /*4*/)); // z FLAG
		boolean tmp = LTC2949_DebugEnable;
		if (FLAG_(cfgPasv, 8))
			LTC2949_DebugEnable = true;
		if (FLAG_(cfgPasv, 4))
			cfgFast |= LTCDEF_CFGFAST_CONT_LAST_SAMPLE;

		unsigned int iStart = 5;
		LogChannelsStop = stringSplitter(SerialInputString, &iStart, &LogChannels);
		if (LogChannelsStop)
		{
			int logChannelsStopCount = 1;
			if (stringSplitter(SerialInputString, &iStart, &logChannelsStopCount))
			{
				int lti;
				stringSplitter(SerialInputString, &iStart, &lti);
				LogTimeoutInc100Millis = lti / 100;
			}
			LogChannelsStopCount = logChannelsStopCount;
			LogChannelsStop = LogChannelsStopCount != 0;
		}
		LogChannelsReportEnable = LogChannels.length() > 0;
		if (LogChannelsReportEnable)
		{
			LogChannelsReportEnable = !FLAG_(cfgNtc, 4);
			boolean enableAverageCalc = FLAG_(cfgNtc, 8);
			if (!LogChannelsReportEnable && !enableAverageCalc)
				enableAverageCalc = true;
			if (enableAverageCalc)
			{
				LogChannelsSum = new double[LogChannels.length()];
				memset(LogChannelsSum, 0, sizeof(double) * LogChannels.length());
			}
		}
		if (LogChannels.indexOf(LogChannels_FACH1) >= 0)
			cfgFast |= LTC2949_BM_FACTRL_FACH1;
		if (LogChannels.indexOf(LogChannels_FACH2) >= 0)
			cfgFast |= LTC2949_BM_FACTRL_FACH2;
		if (LogChannels.indexOf(LogChannels_FACHBAT) >= 0)
			cfgFast |= LTC2949_BM_FACTRL_FACH2; // for BAT fast measurement, either both channels or only channel 2 must be fast!
		if (LogChannels.indexOf(LogChannels_FACHA) >= 0)
			cfgFast |= LTC2949_BM_FACTRL_FACHA;


		if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA))//IS_NOT_FAST())
			cfgFast = 0;
		//else if (!IS_FAST_SSHT())
		//	cfgFast |= LTC2949_BM_FACTRL_FACONV; // if not single shot, we do fast continuous

		lastUInt32 = 0;
		Serial.print(cfgFast);
		PrintComma();
		Serial.print(cfgNtc);
		PrintComma();
		Serial.print(cfgPasv);
		PrintComma();
		Serial.print(LogChannels);
		PrintComma();
		if (LogChannelsStop)
		{
			Serial.print(LogChannelsStopCount);
			PrintComma();
		}
		unsigned long startOfMeas = 0;

		// read, report & clear status
		error = LTC2949_ReadChkStatusFaults(true, true);

		error |= LTC2949_GoCont(cfgFast,
			((cfgNtc & LTC2949_GOCONT_NTC_BOTH) << BM2BITPOS(LTC2949_BM_ADCCONF_NTC1)) |
			((cfgPasv & LTC2949_GOCONT_PASV_BOTH) << BM2BITPOS(LTC2949_BM_ADCCONF_P1ASV)),
			&startOfMeas);
		PrintComma();
		Serial.print(startOfMeas);
		PrintComma();
		LTC2949_DebugEnable = tmp;
		TrigOutSet(true);
		PrintOkErr(error);
	}
	else if (LogChannels.length() == 0)
	{
		StopConversion();
		PrintOkErr(0);
	}
	else
	{
		LogChannelsStop = true;
		LogChannelsStopCount = 1;
	}
}

void ShuntTCComp()
{
	byte error = 0;
	byte dataTC[3], dataT0[3];
	int rs12;
	unsigned int iStart = 2;
	if (stringSplitter(SerialInputString, &iStart, &rs12))
	{
		// collect data
		float tc = 0.0;
		float t0 = 0.0;

		boolean slot1Only;
		if (stringSplitter(SerialInputString, &iStart, &tc)
			&& stringSplitter(SerialInputString, &iStart, &t0))
		{
			int ntcSlot1 = 0;
			stringSplitter(SerialInputString, &iStart, &ntcSlot1);
			slot1Only = (ntcSlot1 != 0);
		}
		else
		{
			PrintOkErr(LTC2949_ERRCODE_OTHER);
			return;
		}

		LTC2949_FloatToF24Bytes(tc, dataTC);
		LTC2949_FloatToF24Bytes(t0, dataT0);

		Serial.print(rs12);
		PrintComma();
		Serial.print(tc, 6);
		PrintComma();
		Serial.print(t0, 1);
		PrintComma();
		for (byte i = 1; i < 3; i++) // check for both channels CH1, CH2
		{
			if (bitMaskClrChk(rs12, i))
				continue;

			boolean isCH1 = (i == 1);
			Serial.print(i);
			LTC2949_WRITE(isCH1 ? LTC2949_VAL_RS1TC : LTC2949_VAL_RS2TC, 3, dataTC);
			LTC2949_WRITE(isCH1 ? LTC2949_VAL_RS1T0 : LTC2949_VAL_RS2T0, 2, dataT0); // LSB of F24 won't be written
			PrintComma();
			SerialPrintByteArrayHex(dataTC, 3, true);
			PrintComma();
			SerialPrintByteArrayHex(dataT0, 2, true);
			PrintComma();
		}
		LTC2949_ADCConfigTC(slot1Only ? LTC2949_BM_ADCCONF_NTCSLOT1 : 0);
		LTC2949_ADCConfigWrite();
		error |= LTC2949_OpctlAdjUpd();
	}
	else
	{
		for (byte i = 1; i < 3; i++) // check for both channels CH1, CH2
		{
			if (bitMaskClrChk(rs12, i))
				continue;

			boolean isCH1 = (i == 1);
			Serial.print(i);
			PrintComma();
			error |= LTC2949_READ(isCH1 ? LTC2949_VAL_RS1TC : LTC2949_VAL_RS2TC, 3, dataTC);
			error |= LTC2949_READ(isCH1 ? LTC2949_VAL_RS1T0 : LTC2949_VAL_RS2T0, 2, dataT0); // LSB of F24 won't be written
			dataT0[2] = 0;
			// TC
			SerialPrintByteArrayHex(dataTC, 3, true);
			PrintComma();
			float fl;
			LTC2949_F24BytesToFloat(dataTC, &fl);
			Serial.print(fl, 6);
			PrintComma();
			// T0
			SerialPrintByteArrayHex(dataT0, 2, true);
			PrintComma();
			LTC2949_F24BytesToFloat(dataT0, &fl);
			Serial.print(fl, 1);
			PrintComma();
		}

		// ADCConf
		error |= LTC2949_ADCConfigRead(dataTC);
		SerialPrintByteArrayHex(dataTC, 1, true);
		PrintComma();

	}

	PrintAdcconf(LTC2949_adcConf);
	PrintComma();
	PrintOkErr(error);
	/*
	TC<N,TC,T0,M>
	Configure shunt temperature coefficient compensation (set TC to zero to disable compensation)
	N: 1 for I2, 2 for I2, 3 to configure both with the same values
	M: 0 to compensate I1 with SLOT1 and I2 with SLOT2, 1 to compensate I1 and I2 with SLOT1
	*/
}

void PrintAdcconf(byte adcConf)
{
	LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_NTCSLOT1), 'N');
	LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_NTC1), '1');
	LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_NTC2), '2');
	//LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_NTCAVCC), 'A');
	LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_P1ASV), 'p');
	LTC_SERIAL_PRINT_BOOLD(bitMaskSetChk(adcConf, LTC2949_BM_ADCCONF_P2ASV), 'P');
}

void AdjustFactrlAdcConf(uint16_t addr, uint16_t len, byte* data)
{
	int16_t i = LTC2949_GetFACTRLDataOffset(addr, len);
	if (i >= 0)
	{
		cfgFast = data[i];
	}
	else if (LTC2949_iAddrRegsCtrl_GetPage())
	{
		i = LTC2949_GetADCCONFDataOffset(addr, len);
		if (i >= 0)
			LTC2949_adcConf = data[i];
	}
}

void SerialRead(boolean noPageAutoAdjust)
{
	unsigned int iStart = 2;
	int addr, len = 1;
	if (stringSplitter(SerialInputString, &iStart, &addr))
		stringSplitter(SerialInputString, &iStart, &len);
	else
	{
		PrintOkErr(LTC2949_ERRCODE_OTHER);
		return;
	}

	if (noPageAutoAdjust)
		bitMaskSetClr(addr, 0x100, LTC2949_iAddrRegsCtrl_GetPage());

	if (!LTC2949_DebugEnable)
	{
		PrintZeroX();
		Serial.print(addr, HEX);
		PrintComma();
	}

	byte* data = new byte[len];
	byte error = LTC2949_READ(addr, len, data);

	SerialPrintByteArrayHex(data, len, true);

	AdjustFactrlAdcConf(addr, len, data);

	PrintComma();
	PrintOkErr(error);

	if (LTC2949_DebugEnable && !LTC2949_iAddrRegsCtrl_GetPage())
		Serial.println(GetMeasurementString(addr, data, len));

	delete[] data;
}

void SerialWrite(boolean noPageAutoAdjust)
{
	unsigned int iStart = 2;
	int addr, len = 1;
	byte* data = NULL;

	if (stringSplitter(SerialInputString, &iStart, &addr) &&
		stringSplitter(SerialInputString, &iStart, &len) &&
		(len < (256 - 16)) && // single page without speacial row is maximum
		(data = new byte[len]) &&
		(StringToBytes(SerialInputString.substring(iStart), len, data) == (unsigned int)len))
	{
		if (noPageAutoAdjust)
			bitMaskSetClr(addr, 0x100, LTC2949_iAddrRegsCtrl_GetPage());

		if (!LTC2949_DebugEnable)
		{
			PrintZeroX();
			Serial.print(addr, HEX);
			PrintComma();
			SerialPrintByteArrayHex(data, len, true);
			PrintComma();
		}
		LTC2949_WRITE(addr, len, data);

		AdjustFactrlAdcConf(addr, len, data);

		delete[] data;
		if (!LTC2949_DebugEnable)
			PrintOkErr(0);
		return;
	}
	PrintOkErr(LTC2949_ERRCODE_OTHER);
}

void EchoCommand()
{
	Serial.print('>');
	Serial.print(' ');
	Serial.println(SerialInputString);
}

byte NtcCfgRead(int ntc1or2)
{
	float rref, a, b, c;
	byte data[3];
	byte error = 0;
	error |= LTC2949_READ(ntc1or2 == 2 ? LTC2949_VAL_RREF2 : LTC2949_VAL_RREF1, 3, data);
	LTC2949_F24BytesToFloat(data, &rref);
	error |= LTC2949_READ(ntc1or2 == 2 ? LTC2949_VAL_NTC2A : LTC2949_VAL_NTC1A, 3, data);
	LTC2949_F24BytesToFloat(data, &a);
	error |= LTC2949_READ(ntc1or2 == 2 ? LTC2949_VAL_NTC2B : LTC2949_VAL_NTC1B, 3, data);
	LTC2949_F24BytesToFloat(data, &b);
	error |= LTC2949_READ(ntc1or2 == 2 ? LTC2949_VAL_NTC2C : LTC2949_VAL_NTC1C, 3, data);
	LTC2949_F24BytesToFloat(data, &c);
	Serial.print(ntc1or2);
	PrintComma();
	Serial.print(float2Scientific(a, 6));
	PrintComma();
	Serial.print(float2Scientific(b, 6));
	PrintComma();
	Serial.print(float2Scientific(c, 6));
	PrintComma();
	Serial.print(float2Scientific(rref, 6));
	return (error);
}

void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c, boolean print)
{
	if (print)
	{
		Serial.print(ntc1or2);
		PrintComma();
		Serial.print(float2Scientific(a, 6));
		PrintComma();
		Serial.print(float2Scientific(b, 6));
		PrintComma();
		Serial.print(float2Scientific(c, 6));
		PrintComma();
		Serial.print(float2Scientific(rref, 6));
	}
	byte data[3];
	LTC2949_FloatToF24Bytes(rref, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_RREF2 : LTC2949_VAL_RREF1, 3, data);
	LTC2949_FloatToF24Bytes(a, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2A : LTC2949_VAL_NTC1A, 3, data);
	LTC2949_FloatToF24Bytes(b, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2B : LTC2949_VAL_NTC1B, 3, data);
	LTC2949_FloatToF24Bytes(c, data);
	LTC2949_WRITE(ntc1or2 == 2 ? LTC2949_VAL_NTC2C : LTC2949_VAL_NTC1C, 3, data);
}

void WriteCalData(boolean reducedF24, unsigned int iStart)
{
	boolean parsLeft;
	do
	{
		byte data[3];
		int addr;
		float cal;
		if (stringSplitter(SerialInputString, &iStart, &addr))
			parsLeft = stringSplitter(SerialInputString, &iStart, &cal);
		else
		{
			PrintOkErr(LTC2949_ERRCODE_OTHER);
			return;
		}

		Serial.print(addr);
		PrintComma();
		Serial.print(float2Scientific(cal, 6));
		PrintComma();
		LTC2949_FloatToF24Bytes(cal, data);
		SerialPrintByteArrayHex(data, reducedF24 ? 2 : 3, true);
		Serial.println();
		LTC2949_WRITE(addr, reducedF24 ? 2 : 3, data);
	} while (parsLeft);
	PrintOkErr(0);
}

void ReadCalData(boolean reducedF24, unsigned int iStart)
{
	byte error = 0;
	byte data[3];
	int addr; float cal;
	boolean parsLeft;
	do
	{
		parsLeft = stringSplitter(SerialInputString, &iStart, &addr);
		data[2] = 0;
		error |= LTC2949_READ(addr, reducedF24 ? 2 : 3, data);
		LTC2949_F24BytesToFloat(data, &cal);
		PrintZeroX();
		Serial.print(addr, HEX);
		PrintComma();
		Serial.print(float2Scientific(cal, 6));
		PrintComma();
		SerialPrintByteArrayHex(data, reducedF24 ? 2 : 3, true);
		Serial.println();
	} while (parsLeft);
	PrintOkErr(error);
}

void ReportInit()
{
	LTC_SERIAL_PRINT_BOOLD(LTC2949_onTopOfDaisychain, 'I');
	PrintComma();
	Serial.print(LTC2949_CellMonitorCount);
	PrintOkErr(0);
}

void Init()
{
	int cfg = 0, cellMonitorCount = LTC2949_CellMonitorCount;
	unsigned int iStart = 2;

	if (stringSplitter(SerialInputString, &iStart, &cfg))
		stringSplitter(SerialInputString, &iStart, &cellMonitorCount);

	LTC2949_init_lib(
		/*cellMonitorCount:         */cellMonitorCount,
		/*ltc2949onTopOfDaisychain: */bitMaskSetChk(cfg, CFG_INDIRECT_READ),
		/*debugEnable:              */LTC2949_DebugEnable
	);
	ReportInit();
}

void SlotsConfig()
{
	byte error;
	byte slot1P, slot1N, slot2P, slot2N, slotFP, slotFN;

	if (SerialInputString.length() > 2)
	{
		unsigned int iStart = 2;
		int inputValue;
		boolean inOk;

		inOk = stringSplitter(SerialInputString, &iStart, &inputValue);
		slot1P = inputValue;
		inOk = inOk && stringSplitter(SerialInputString, &iStart, &inputValue);
		slot1N = inputValue;
		inOk = inOk && stringSplitter(SerialInputString, &iStart, &inputValue);
		slot2P = inputValue;
		inOk = inOk && stringSplitter(SerialInputString, &iStart, &inputValue);
		slot2N = inputValue;
		inOk = inOk && stringSplitter(SerialInputString, &iStart, &inputValue);
		slotFP = inputValue;
		if (inOk)
		{
			error = 0;
			stringSplitter(SerialInputString, &iStart, &inputValue);
			slotFN = inputValue;
			LTC2949_SlotsCfg(slot1P, slot1N, slot2P, slot2N);
			LTC2949_SlotFastCfg(slotFP, slotFN);
		}
		else
			error = LTC2949_ERRCODE_OTHER;
	}
	else
	{
		error = LTC2949_GetSlotsCfg(&slot1P, &slot1N, &slot2P, &slot2N);
		error |= LTC2949_GetSlotFastCfg(&slotFP, &slotFN);
		Serial.print(slot1P);
		PrintComma();
		Serial.print(slot1N);
		PrintComma();
		Serial.print(slot2P);
		PrintComma();
		Serial.print(slot2N);
		PrintComma();
		Serial.print(slotFP);
		PrintComma();
		Serial.print(slotFN);
		PrintComma();
	}

	PrintOkErr(error);
}

byte ChaToAddr(char cha)
{
	switch (cha)
	{
	case LogChannels_FACH1: // fast channel I1
		return LTC2949_REG_FIFOI1;
	case LogChannels_FACH2: // fast channel I2
		return LTC2949_REG_FIFOI2;
	case LogChannels_FACHBAT: // fast channel PasV
		return LTC2949_REG_FIFOBAT;
	case LogChannels_FACHA: // fast channel Aux
		return LTC2949_REG_FIFOAUX;

	case 'i': // slow channel I1
		return LTC2949_VAL_I1;
	case 'I': // slow channel I2
		return LTC2949_VAL_I2;
	case 'p': // slow channel P1
		return LTC2949_VAL_P1;
	case 'P': // slow channel P2
		return LTC2949_VAL_P2;
	case 'a': // slow channel I1AVG
		return LTC2949_VAL_I1AVG;
	case 'A': // slow channel I2AVG
		return LTC2949_VAL_I2AVG;

	case 'V': // slow channel BAT
		return LTC2949_VAL_BAT;
	case 'v': // slow channel VCC
		return LTC2949_VAL_VCC;
	case 'T': // slow channel TEMP
		return LTC2949_VAL_TEMP;
	case '1': // slow channel SLOT1
		return LTC2949_VAL_SLOT1;
	case '2': // slow channel SLOT2
		return LTC2949_VAL_SLOT2;
	case 'R': // slow channel VREF
		return LTC2949_VAL_VREF;
	default:
		return 0;
	}
}

double ChaToLsb(char cha)
{
	switch (cha)
	{
	case LogChannels_FACH1: // fast channel I1
		return LTC2949_LSB_FIFOI1 / RSNS_NOM;
	case LogChannels_FACH2: // fast channel I2
		return LTC2949_LSB_FIFOI2 / RSNS_NOM;
	case LogChannels_FACHBAT: // fast channel PasV
		return LTC2949_LSB_FIFOBAT;
	case LogChannels_FACHA: // fast channel Aux
		return LTC2949_LSB_FIFOAUX;

	case 'i': // slow channel I1
		return LTC2949_LSB_I1 / RSNS_NOM;
	case 'I': // slow channel I2
		return LTC2949_LSB_I2 / RSNS_NOM;
	case 'p': // slow channel P1
		return LTC2949_PLsb(true, RSNS_NOM);
	case 'P': // slow channel P2
		return LTC2949_PLsb(false, RSNS_NOM);
	case 'a': // slow channel I1AVG
		return LTC2949_LSB_I1AVG / RSNS_NOM;
	case 'A': // slow channel I2AVG
		return LTC2949_LSB_I2AVG / RSNS_NOM;

	case 'V': // slow channel BAT
		return LTC2949_LSB_BAT;
	case 'v': // slow channel VCC
		return LTC2949_LSB_VCC;
	case 'T': // slow channel TEMP
		return LTC2949_LSB_TEMP;
	case '1': // slow channel SLOT1
		return LTC2949_SlotLsb(true);
	case '2': // slow channel SLOT2
		return LTC2949_SlotLsb(false);
	case 'R': // slow channel VREF
		return LTC2949_LSB_VREF;

	case 't': // absolute time in sec
		return LTC2949_LSB_TB1;
	case 'd': // delta time in ms
		return LTC2949_LSB_TB1 * 1e3;
	default:
		return 0;
	}
}

#define REPORT_FAST_SAMPLES_PER_SLOW_CONV 128U
#define REPORT_FASTALL_SAMPLES_PER_SLOW_CONV (REPORT_FAST_SAMPLES_PER_SLOW_CONV / 2U)

String PrintConversion(char cha, byte * error, double* sum, int16_t * fastData2949)
{
	double val = 0;
	int32_t iVal = 0;
	String str = "";
	byte buffer[4];

	switch (cha)
	{
	case LogChannels_FACH1:   // fast fifo channel I1
	case LogChannels_FACH2:   // fast fifo channel I2
	case LogChannels_FACHBAT: // fast fifo channel PasV
	case LogChannels_FACHA:   // fast fifo channel Aux
		if (fastData2949 != NULL /*&& IS_FAST_SSHT_OR_LAST_SAMPLE()*/)
		{
			// single shot, use data from fastData2949
			str += String(val = fastData2949[cha - LogChannels_FACH1] * ChaToLsb(cha), LG_DBL_OUTPUT_DIGITS);
			//if (IS_FAST_CONT())
			//	str += ',';
			//else
			//	break;

		}
		else if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACONV))//IS_FAST_CONT())
		{
			// read FIFO data
			int16_t* samples = new int16_t[REPORT_FAST_SAMPLES_PER_SLOW_CONV];
			(*error) |= LTC2949_ReadFifo(
				/*byte addr:             */ ChaToAddr(cha),
				/*uint16_t* len:              */ REPORT_FAST_SAMPLES_PER_SLOW_CONV,
				/*int16_t * samples:          */ samples);
			// analyse and report data
			ProcessFastData(samples, REPORT_FAST_SAMPLES_PER_SLOW_CONV, &iVal);
			delete[] samples;
			// reduced LSB size, as we divide the sum by (REPORT_FAST_SAMPLES_PER_SLOW_CONV / 8U) instead of REPORT_FAST_SAMPLES_PER_SLOW_CONV
			str += String(val = iVal * (ChaToLsb(cha) / (1.0 * REPORT_FAST_SAMPLES_PER_SLOW_CONV)), LG_DBL_OUTPUT_DIGITS);
		}
		break;

	case 'i': // slow channel I1
	case 'I': // slow channel I2
	case 'p': // slow channel P1
	case 'P': // slow channel P2
	case 'a': // slow channel I1AVG
	case 'A': // slow channel I2AVG
		(*error) |= LTC2949_READ(ChaToAddr(cha), 3, buffer);
		str += String(val = LTC_3BytesToInt32(buffer) * ChaToLsb(cha), LG_DBL_OUTPUT_DIGITS);
		break;
	case 'V': // slow channel BAT
	case 'v': // slow channel VCC
	case 'T': // slow channel TEMP
	case '1': // slow channel SLOT1
	case '2': // slow channel SLOT2
	case 'R': // slow channel VREF
		(*error) |= LTC2949_READ(ChaToAddr(cha), 2, buffer);
		str += String(val = LTC_2BytesToInt16(buffer) * ChaToLsb(cha), LG_DBL_OUTPUT_DIGITS);
		break;
	case 't': // absolute time in sec
	case 'd': // delta time in ms
	{
		(*error) |= LTC2949_READ(LTC2949_VAL_TB1, 4, buffer);
		uint32_t tb1 = LTC_4BytesToUInt32(buffer);
		uint32_t treport;
		if (cha == 't')
			treport = tb1;
		else
		{
			treport = tb1 - lastUInt32;
			lastUInt32 = tb1;
		}
		str += String(val = treport * ChaToLsb(cha), (cha == 't' ? 1 : 0));
	}
	break;
	case 'c': // meascounter
		str += String(LogChannelsMeasCount);
		break;
	}
	if (sum)
		* sum += val;
	return str;
}

String GetMeasurementString(uint8_t addr, byte* buffer, int16_t length)
{
	String str = "";

#define GM_IF(A)  if (length < A) return str
#define GM_STR(B) str += '\n' + String(addr, HEX) + ',' + B
#define GM_STRB(C,B) str += '\n' + String(F(C)) + B
#define GM_INC(A) length -= A; addr += A; buffer += A

#define GM_IFSTRINC(A,B) GM_IF(A);GM_STR(B);GM_INC(A)
#define GM_IFSTRBINC(C,A,B) GM_IF(A);GM_STRB(C,B);GM_INC(A)
	//GM_IFSTRBINC("", 3, String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / RSNS_NOM, LG_DBL_OUTPUT_DIGITS));

	while (true)
	{
		if (length <= 0)
			return str;
		switch (addr)
		{
		case LTC2949_VAL_TB1:
		case LTC2949_VAL_TB2:
		case LTC2949_VAL_TB3:
		case LTC2949_VAL_TB4:
			GM_IFSTRINC(4, String(LTC_4BytesToUInt32(buffer) * LTC2949_LSB_TB4, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_C1:
		case LTC2949_VAL_C2:
			GM_IFSTRINC(6, String(LTC2949_BytesToDouble(buffer, 6, true, LTC2949_LSB_C1 / RSNS_NOM), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_C3:
			GM_IFSTRINC(8, String(LTC2949_BytesToDouble(buffer, 8, true, LTC2949_LSB_C3 / RSNS_NOM), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_E1:
		case LTC2949_VAL_E2:
			GM_IFSTRINC(6, String(LTC2949_BytesToDouble(buffer, 6, true, LTC2949_LSB_E1 / RSNS_NOM), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_E4:
			GM_IFSTRINC(8, String(LTC2949_BytesToDouble(buffer, 8, true, LTC2949_LSB_E4 / RSNS_NOM), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_I1MAX:
		case LTC2949_VAL_I2MAX:
		case LTC2949_VAL_I1MIN:
		case LTC2949_VAL_I2MIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_I1MAX / RSNS_NOM, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_P1MAX:
		case LTC2949_VAL_P2MAX:
		case LTC2949_VAL_P1MIN:
		case LTC2949_VAL_P2MIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_PLsb(addr <= LTC2949_VAL_P1MIN) / RSNS_NOM, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_VREF:
		case LTC2949_VAL_BAT:
		case LTC2949_VAL_BATMAX:
		case LTC2949_VAL_BATMIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_SLOT1:
		case LTC2949_VAL_SLOT2:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(addr == LTC2949_VAL_SLOT1), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_SLOT1MAX:
		case LTC2949_VAL_SLOT1MIN:
		case LTC2949_VAL_SLOT2MAX:
		case LTC2949_VAL_SLOT2MIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(addr <= LTC2949_VAL_SLOT1MIN), LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_VCC:
		case LTC2949_VAL_VCCMAX:
		case LTC2949_VAL_VCCMIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_VCC, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_TEMP:
		case LTC2949_VAL_TEMPMAX:
		case LTC2949_VAL_TEMPMIN:
			GM_IFSTRINC(2, String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_I1:
		case LTC2949_VAL_I2:
			GM_IFSTRINC(3, String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / RSNS_NOM, LG_DBL_OUTPUT_DIGITS));
			break;

		case LTC2949_VAL_P1:
		case LTC2949_VAL_P2:
			GM_IFSTRINC(3, String(LTC_3BytesToInt32(buffer) * LTC2949_PLsb(addr == LTC2949_VAL_P1) / RSNS_NOM, LG_DBL_OUTPUT_DIGITS));
			break;

		default:
			addr++;
			length--;
			buffer++;
			break;
		}
	}
	return str;
}

void ProcessFastData(int16_t* data, byte len, int32_t* dSum)
{
	for (byte i = 0; i < len; i++)
		* dSum += data[i];
}

void ProcessFastDataAll(int16_t* data, uint16_t len, int32_t dSum[])
{
	for (byte i = 3; i < len; i += 4)
	{						    // 1st run
		dSum[0] += data[i - 3]; // 0
		dSum[1] += data[i - 2]; // 1
		dSum[2] += data[i - 1]; // 2
		dSum[3] += data[i - 0]; // 3
	}
}

/*!*********************************************************************
\brief Wakeup LTC2949, report and clear all status / alert registers
***********************************************************************/
void WakeUpReportStatus(bool doNotReadClearStatusFaults)
{
	byte error = LTC2949_WakeupAndAck();
	if (!doNotReadClearStatusFaults)
	{
		error |= LTC2949_ReadChkStatusFaults(true, true);
		PrintComma();
	}
	PrintOkErr(error);
}

void InitExtTrigger()
{
	pinMode(PIN_TRIG_IN, INPUT);
	digitalWrite(PIN_TRIG_IN, LOW);

	// We only drive low, so as default (inactive) it is an input
	pinMode(PIN_TRIG_OUT, INPUT);
	// output level when driver becomes active (so when pin becomes OUTPUT it will be driven low)
	digitalWrite(PIN_TRIG_OUT, LOW);

	trigLowHigh = true;
	trig01HighZ = true;
}

void ConfigExtTrigger()
{
	byte error = 0;
	extTriggerEvent = false;

	int mode = SerialInputString.substring(1).toInt();
	if (mode == 0)
		// Disable ExtTrigger
		detachInterrupt(digitalPinToInterrupt(PIN_TRIG_IN));
	else if (mode == 1)
		// trigger on rising edge
		attachInterrupt(digitalPinToInterrupt(PIN_TRIG_IN), ExtTriggerEvent, RISING);
	else if (mode == 2)
		// trigger on falling edge
		attachInterrupt(digitalPinToInterrupt(PIN_TRIG_IN), ExtTriggerEvent, FALLING);
	else if (mode == 3)
		// trigger on any edge
		attachInterrupt(digitalPinToInterrupt(PIN_TRIG_IN), ExtTriggerEvent, CHANGE);
	else
		error = LTC2949_ERRCODE_TIMEOUT;
	PrintOkErr(error);
}

void ProcessExtTrigger()
{
	if (!extTriggerEvent)
		return;

	extTriggerEvent = false;

	if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA))
		return; // nothing to do in case no fast channel is activated

	byte error;

	// FAST SINGLE SHOT
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	error = FastSSHTOnDemand(fastData2949, bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACONV));
	if (error)
	{
		PrintOkErr(error);
		PrintComma();
	}

	boolean ch1OrCh2 = false;
	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH1))
	{
		ch1OrCh2 = true;
		Serial.print(LogChannels_FACH1);
		PrintComma();
		Serial.print(PrintConversion(LogChannels_FACH1, &error, NULL, fastData2949));
		Serial.println();
	}
	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH2))
	{
		ch1OrCh2 = true;
		Serial.print(LogChannels_FACH2);
		PrintComma();
		Serial.print(PrintConversion(LogChannels_FACH2, &error, NULL, fastData2949));
		Serial.println();
	}
	if (ch1OrCh2 && LTC2949_AnyPasV())
	{
		Serial.print(LogChannels_FACHBAT);
		PrintComma();
		Serial.print(PrintConversion(LogChannels_FACHBAT, &error, NULL, fastData2949));
		Serial.println();
	}
	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACHA))
	{
		Serial.print(LogChannels_FACHA);
		PrintComma();
		Serial.print(PrintConversion(LogChannels_FACHA, &error, NULL, fastData2949));
		Serial.println();
	}
}

// always reads 1000 samples and prints the newest len samples
byte FastFifoPrintTail(char cha, uint16_t len)
{
	// samples: 1000,999,....,1,0, where 1000 is the oldest sample which is read first
	// oldest 100 samples go from 1000 to 901
	uint16_t n0 = 1000;
	uint16_t n1 = 901;

	Serial.print(cha);

	// read 10 times 100 samples
	for (byte i = 0; i < 10; i++)
	{
		int16_t buffer[100];
		byte error = LTC2949_ReadFifo(
			/*byte addr:    */ ChaToAddr(cha),
			/*uint16_t* len:     */ 100,
			/*int16_t * samples: */ buffer);

		//  101    101
		if (len >= n1) // we want to read len newest sample, current buffer contains samples n0 downto n1
		{
			byte j = (len < n0)
				? n0 - len
				: 0;
			for (; j < 100; j++)
			{
				PrintComma();
				Serial.print(buffer[j] * ChaToLsb(cha), LG_DBL_OUTPUT_DIGITS);
			}
		}
		n1 -= 100;
		n0 -= 100;

		if (error)
			return error;
	}
	return 0;
}

void ExtTriggerEvent()
{
	extTriggerEvent = true;
}

void TrigOutSet(boolean on)
{
	if (trig01HighZ)
		pinMode(PIN_TRIG_OUT, on ? OUTPUT : INPUT);
	else if (trigLowHigh)
		digitalWrite(PIN_TRIG_OUT, on ? LOW : HIGH);
	else //highLow
		digitalWrite(PIN_TRIG_OUT, on ? HIGH : LOW);
}


#ifdef LTC2949_CONFIG_MOSI_FOR_READ
boolean TestDone()
{
	if (!Serial.available())
		return false; // no new character, nothing to do
	// Read from serial
	char character = Serial.read(); // Receive a single character from the software serial port
	if (character == 'E')
	{
		LTC2949_OpctlIdle();
		return true;
	}
	return false;
}
//	else if (startsWith(SerialInputString, F("TST")))         Test();
void Test()
{
	if (SerialInputString.length() > 3)
		LTC2949_MOSI_FOR_READ = toIntAuto(SerialInputString.substring(3));

#define Test_cycle_Time  32
#define Test_sampleCount  (unsigned long)(Test_cycle_Time/0.8 + 5)
	unsigned long timeSlot = millis() + Test_cycle_Time;
	uint16_t len = Test_sampleCount;
	int16_t samples[Test_sampleCount];

	byte error = LTC2949_GoCont(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA, LTC2949_BM_ADCCONF_P2ASV);

	ReportInit();
	Serial.println(LTC2949_MOSI_FOR_READ, HEX);

	if (LTC2949_MOSI_FOR_READ == 0x55)
	{
		timeSlot = millis() + 300;
		for (uint8_t i = 0; i < 2; i++)
		{
			byte dbgCnt = 0, old_dbgCnt = 1, regsctrl;
			while (true)
			{
				if (TestDone())
					break;
				error |= LTC2949_READ(LTC2949_REG_REGSCTRL, 1, &regsctrl);
				error |= LTC2949_READ(LTC2949_REG_DBGCNT, 1, &dbgCnt);
				if (dbgCnt != old_dbgCnt)
				{
					Serial.print(regsctrl, HEX);
					PrintComma();
					Serial.println(dbgCnt);
				}
				old_dbgCnt = dbgCnt;
				if (LTC_TIMEOUT_CHECK(millis(), timeSlot))
				{
					timeSlot = millis() + 300;
					if (bitMaskSetChk(regsctrl, LTC2949_BM_REGSCTRL_MLK0))
					{
						PrintOkErr(LTC2949_ERRCODE_OTHER);
						bitMaskClr(regsctrl, LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
					}
					else if (bitMaskSetChk(regsctrl, LTC2949_BM_REGSCTRL_MLK1))
					{
						bitMaskClr(regsctrl, LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
					}
					else
					{
						regsctrl |= LTC2949_BM_REGSCTRL_MLK0;
					}
					LTC2949_WRITE(LTC2949_REG_REGSCTRL, regsctrl);
				}
			}
			bitMaskClr(regsctrl, LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0);
			LTC2949_WRITE(LTC2949_REG_REGSCTRL, regsctrl);
			PrintOkErr(error);
			delay(300);
			timeSlot = millis() + 300;
		}
	}

	PrintOkErr(error);

	ReportInit();
	Serial.println(LTC2949_MOSI_FOR_READ, HEX);
	error |= LTC2949_GoCont(
		LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACONV,
		LTC2949_BM_ADCCONF_P2ASV,
		&timeSlot);
	timeSlot += Test_cycle_Time;

	while (true)
	{
		if (TestDone())
		{
			PrintOkErr(error);
			return;
		}
		while (!LTC_TIMEOUT_CHECK(millis(), timeSlot))
			delay(1);

		timeSlot = millis() + Test_cycle_Time;
		byte dbgCnt;
		LTC2949_READ(LTC2949_REG_DBGCNT, 1, &dbgCnt);
		Serial.print(dbgCnt);

		for (uint8_t i = 0; i < 3; i++)
		{
			PrintComma();
			len = Test_sampleCount;
			error |= LTC2949_ReadFifo(LTC2949_REG_FIFOI2 + i, &len, samples, NULL, LTC2949_RDFIFO_STOP_EMPTY);
			Serial.print(len);
}
		Serial.println();
	}
}
#else
#endif
