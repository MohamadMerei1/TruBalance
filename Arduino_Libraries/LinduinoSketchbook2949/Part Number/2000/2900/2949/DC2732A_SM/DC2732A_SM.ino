/*
* Linear Technology / Analog Devices, Ismaning Design Center
* DC2732A_SM (Linduino Sketch for LTC2949 Demo Board - Isolated Battery Meter for EV / HEV)
*
* Basic example for
*  safety measurements
*  timing requirements of above
*
* created: Patrick Wilhelm (PW)
* last Revision:
* $Revision: 534 $
* $Author: pwilhelm $
* $Date: 2017-07-07 15:09:05 +0200 (Fri, 07 Jul 2017) $
*
*/

#include <Arduino.h>
#include <Linduino.h>
#include <SPI.h>
#include <LTC2949.h>
#include <ltcmuc_tools.h>

// DC2792A has two isoSPI ports: AUX and MAIN
#define LTCDEF_CS_AUX  9
#define LTCDEF_CS_MAIN 10
// in case of DC2792A select AUX or MAIN port
// default for all other isoSPI boards should be MAIN
#define LTCDEF_CS_SELECT LTCDEF_CS_MAIN

// serial baudrate
#define LTCDEF_BAUDRATE 250000

// number of digits to be reported for floating point numbers
#define NUMBER_OF_DIGITS 3
// default number of executions of safety measure to calculate average time
#define DEFAULT_SM_LOOP_COUNT 10

#define LTC2949_BM_FACTRL_SSHT_CONF (LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)
#define LTC2949_BM_FACTRL_CONF (LTC2949_BM_FACTRL_SSHT_CONF | LTC2949_BM_FACTRL_FACONV)

// NTC parameters
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3

// buffer for commands received via serial interface
String SerialInputString;

byte WakeUpReportStatus(boolean statusCleared, boolean clr = true);
byte CheckInitialConditionCont(byte cfgFast = LTC2949_BM_FACTRL_FACHA, byte adcCfg = 0);
void NtcCfgWrite(int ntc1or2, float rref = NTC_RREF, float a = NTC_STH_A, float b = NTC_STH_B, float c = NTC_STH_C);

// Initializes hardware and variables
void setup()
{
	SerialInputString = "";
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	// done in LTC2949.cpp
	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
	LinduinoSelectSPI(LTCDEF_CS_SELECT);

	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB port only
	}
	PrintUsage();
}

// Print usage help
void PrintUsage()
{
	Serial.print(F("\
Commands:\n\
RST: Reset\n\
RTMP: Reset, read temp\n\
WK: WakeUp,rd,clr STATUS\n\
GPO<P>,<M>: Set GPO P mode M\n\
INJ: All error injectors\n\
SM<N>: Do SM N (1,2,3,4,9,24)\n\
ATSR3: ~ impl. example\n\
DB1: Enable...\n\
DB0: ...disable debug output\n"));
}

// main loop (executed in an endless loop by arduino)
void loop()
{
	ProcessSerialCommands();
}

// Wake-up LTC2949, report and clear all status / alert registers
byte WakeUpReportStatus(boolean statusCleared, boolean clr)
{
	byte data[10];
	boolean expChkFailed;
	byte error = LTC2949_WakeupAndAck();
	PrintOkErr(error);

	error |= LTC2949_ReadChkStatusFaults(
		/*boolean lockMemAndClr,   :*/ clr,
		/*boolean printResult,	   :*/ true,
		/*byte len,				   :*/ 10,
		/*byte * statFaultsExpAndRd:*/ data,
		/*boolean * expChkFailed,  :*/ &expChkFailed,
		/*byte expDefaultSet)	   :*/ statusCleared
		? LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR | LTC2949_STATFAULTSCHK_IGNORE_STATUPD
		: LTC2949_STATFAULTSCHK_DEFAULT_SETTING);

	if (statusCleared && expChkFailed)
		error |= LTC2949_ERRCODE_FAULTS;
	PrintComma();
	PrintOkErr(error);
	return error;
}

// Check for initial condition of LTC2949 required by a certain
// safety measure. In this case check for continuous operation mode
// being enabled and fast channel prepared for fast single shot of AUX.
// If any condition is not met, this function will set the appropriate
// operating mode.
byte CheckInitialConditionCont(byte cfgFast, byte adcCfg)
{
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(false);

	// check slow continuous
	byte opctrl;
	error |= LTC2949_READ(LTC2949_REG_OPCTRL, 1, &opctrl);
	if (bitMaskClrChk(opctrl, LTC2949_BM_OPCTRL_CONT))
	{
		// read & clear status
		error |= LTC2949_ReadChkStatusFaults(true, false);
		// enable slow cont.
		error |= LTC2949_GoCont(
			/*cfgFast:*/cfgFast,
			/*adcCfg: */adcCfg
		);
	}

	// check fast configuration
	byte factrl;
	error |= LTC2949_READ(LTC2949_REG_FACTRL, 1, &factrl);
	if (bitMaskClrChk(factrl, LTC2949_BM_FACTRL_FACHA) ||
		bitMaskSetChk(factrl, LTC2949_BM_FACTRL_CONF))
	{
		// prepare for fast single shot
		LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACHA);
	}

	return error;
}

// Check for initial condition of LTC2949 required by a certain
// safety measure. In this case check for being in IDLE mode.
// If condition is not met, this function will set the appropriate
// operating mode.
byte CheckInitialConditionIdle()
{
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(false);

	// check slow continuous
	byte opctrl;
	error |= LTC2949_READ(LTC2949_REG_OPCTRL, 1, &opctrl);
	if (bitMaskSetChk(opctrl, LTC2949_BM_OPCTRL_CONT))
	{
		// go IDLE
		{byte e; LTC2949_ChkUpdate(&e); error |= e; }
		LTC2949_OpctlIdle();
		error |= LTC2949_PollUpdate() & ~LTC2949_ERRCODE_TIMEOUT; // we ignore the possible timeout here!
	}
	// just to clear the potential UPDATE bit
	error |= WakeUpReportStatus(true);
	return error;
}

// write NTC configuration
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

// configure GPOs
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

// defines used for AllErrorInjectors
//
// default timeout for polling operations
#define FLTINJ_DEFAULT_TIMEOUT_MS 50
// expected STATUS flags after power-up / reset
#define FLTINJ_STATUS_FLAGS ( \
	LTC2949_BM_STATUS_UVLOA|\
	LTC2949_BM_STATUS_PORA|\
	LTC2949_BM_STATUS_UVLOSTBY|\
	LTC2949_BM_STATUS_UVLOD)
// all fault injectors, besides ESERERRINJ which is not required, 
// can be set at the same time
#define FLTINJ_ISO0_FLAGS ( \
	LTC2949_BM_ISO0_FERR |\
	LTC2949_BM_ISO0_ISERERRINJ |\
	LTC2949_BM_ISO0_FCAERRINJ |\
	LTC2949_BM_ISO0_HWBISTINJ |\
	LTC2949_BM_ISO0_CRCCFGINJ |\
	LTC2949_BM_ISO0_MEMERRINJ)
// expected flags in EXTFAULTS after successful fault injection
#define FLTINJ_EXTFAULTS_FLAGS ( \
	LTC2949_BM_EXTFAULTS_ROMERR|\
	LTC2949_BM_EXTFAULTS_MEMERR|\
	LTC2949_BM_EXTFAULTS_FCAERR|\
	LTC2949_BM_EXTFAULTS_XRAMERR|\
	LTC2949_BM_EXTFAULTS_IRAMERR|\
	LTC2949_BM_EXTFAULTS_HWMBISTEXEC)
// expected flags in FAULTS after successful fault injection
#define FLTINJ_FAULTS_FLAGS ( \
	LTC2949_BM_FAULTS_PROMERR|\
	LTC2949_BM_FAULTS_INTCOMMERR|\
	LTC2949_BM_FAULTS_FAERR|\
	LTC2949_BM_FAULTS_HWBIST|\
	LTC2949_BM_FAULTS_CRCCFG|\
	LTC2949_BM_FAULTS_CRCMEM)
// optional delay at every end of iteration. Default is 0, no delay
// #define FLTINJ_DELAY_LOOP_END_MS 0

// Latent fault checks SM18,SM19,SM21,SM22
// all latent fault checks executed the same time
// Note: SM20 is recommended to be implemented via fault injection on the
// 	  master SPI interface e.g. on the low level SPI driver
void AllErrorInjectors(int cnt)
{
	// it is assumed, the latent fault checks are done
	// directly after power-up / reset
	LTC2949_reset();
	byte error = WakeUpReportStatus(false, false);

	int i = cnt;
	unsigned long maxISO0SOClrTime = 0;

	// start timing measurement
	unsigned long timingMeasure = millis();

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	while (i-- > 0)
	{
		byte data[5];
		/////////////////////////////////////////////////////////////////////////////
		// 1: CHECK DEFAULTS AFTER RESET
		/////////////////////////////////////////////////////////////////////////////
		//
		// read STATUS
		error |= LTC2949_READ(LTC2949_REG_STATUS, 1, data);
		// read EXTFAULTS, FAULTS
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, data + 1);
		// read ISO0
		error |= LTC2949_READ(LTC2949_REG_ISO0, 1, data + 3);
		// check STATUS, EXTFAULTS, FAULTS for default values after power-up / reset
		if (
			/*STATUS:   */ data[0] != FLTINJ_STATUS_FLAGS ||
			/*EXTFAULTS:*/ data[1] != LTC2949_BM_EXTFAULTS_HWMBISTEXEC ||
			/*FAULTS:   */ data[2] != 0x00 ||
			/*ISO0:     */ data[3] != 0x00 ||
			false)
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print(F("not DFLT after RST,"));
			SerialPrintByteArrayHex(data, 4, true, true);
			PrintOkErr(error);
		}
		/////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////


		/////////////////////////////////////////////////////////////////////////////
		// 2: SET ALL INJECTORS IN ISO0
		/////////////////////////////////////////////////////////////////////////////
		// 
		data[0] = FLTINJ_ISO0_FLAGS;
		// set time out
		unsigned long stopWatch = millis() + FLTINJ_DEFAULT_TIMEOUT_MS;
		// write injectors to ISO0
		LTC2949_WRITE(LTC2949_REG_ISO0, 1, data);
		// read-back injectors from ISO0
		error |= LTC2949_READ(LTC2949_REG_ISO0, 1, data + 1);
		if (data[1] == 0x00)
		{
			// this could happen, in case there was a PEC error during write to ISO0
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print(F("ISO0 not written,"));
			PrintOkErr(error);
		}
		/////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////


		/////////////////////////////////////////////////////////////////////////////
		// 3: WAIT FOR FAULT INJECTION EXECUTED
		/////////////////////////////////////////////////////////////////////////////
		// 
		data[2] = 0;
		// read and check ISO0
		// Note: Timeout error will be flagged, if expected results are never read!
		while (
			((error |= LTC2949_READ(LTC2949_REG_ISO0, 1, data + 2)) == 0) &&
			(data[2] != LTC2949_BM_ISO0_ISERERRINJ) &&
			(LTC_TIMEOUT_CHECK(millis(), stopWatch) ? (error |= LTC2949_ERRCODE_TIMEOUT) : (error)) == 0)
			continue;
		// calculate time when ISO0 SO bits were cleared
		unsigned long tmp = millis() - stopWatch + FLTINJ_DEFAULT_TIMEOUT_MS;
		if (tmp > maxISO0SOClrTime)
			maxISO0SOClrTime = tmp;
		// do some sanity checks of the previous reads
		// for debugging only
		if (data[1] != data[0])
		{
			// WARNING ONLY:
			// normally we expect it takes some time
			// until error injectors are processed internally.
			// still, due to async, and if time between write
			// and first read is too high, it can happen
			// that the first read is already different
			Serial.print(F("WRN,1st rd ISO0:0x"));
			Serial.println(data[1], HEX);
		}
		else if (data[1] == data[2])
		{
			// WARNING ONLY:
			// similar to above, we expect some intermediate state 
			// at the 1st reading, before the final expected value
			Serial.print(F("WRN,fast SO CLR ISO0:0x"));
			Serial.println(data[1], HEX);
		}
		/////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////////////
		// 4: CHECK FOR FAULTS BEING FLAGGED
		/////////////////////////////////////////////////////////////////////////////
		// 
		data[3] = 0;
		data[4] = 0;
		// read and check EXTFAULTS, FAULTS
		// Note: Timeout error will be flagged, if expected results are never read!
		while (
			((error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, data + 3)) == 0) &&
			(data[3] != FLTINJ_EXTFAULTS_FLAGS) &&
			(data[4] != FLTINJ_FAULTS_FLAGS) &&
			(LTC_TIMEOUT_CHECK(millis(), stopWatch) ? (error |= LTC2949_ERRCODE_TIMEOUT) : (error)) == 0)
			continue;
		/////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////////////////////
		// 5: LOCK MEMORY TO CLEAR ALL REGS
		/////////////////////////////////////////////////////////////////////////////
		// 
		error |= LTC2949_MemLock(true);
		data[0] = 0x00;
		data[1] = 0x00;
		// clear all
		LTC2949_WRITE(LTC2949_REG_STATUS, 1, data);
		LTC2949_WRITE(LTC2949_REG_EXTFAULTS, 2, data);
		LTC2949_WRITE(LTC2949_REG_ISO0, 1, data);
		// read STATUS
		error |= LTC2949_READ(LTC2949_REG_STATUS, 1, data);
		// read EXTFAULTS, FAULTS
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, data + 1);
		// read ISO0
		error |= LTC2949_READ(LTC2949_REG_ISO0, 1, data + 3);
		if (
			/*STATUS:   */ data[0] != 0 ||
			/*EXTFAULTS:*/ data[1] != 0 ||
			/*FAULTS:   */ data[2] != 0 ||
			/*ISO0:     */ data[3] != 0 ||
			false)
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print(F("clr STAT/FLTS/ISO,"));
			PrintOkErr(error);
		}
		// now we are done and would just 
		// unlock the memory:
		// error |= LTC2949_MemLock(false);
		// still for this example we want to repeat everything
		// ...
		/////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////
		// ...
		// for this iteration test we just write
		// again the default values of
		// STATUS, EXTFAULTS, FAULTS after reset
		// which is the expected state at the very 
		// beginning of this loop
		data[0] = FLTINJ_STATUS_FLAGS;
		data[1] = LTC2949_BM_EXTFAULTS_HWMBISTEXEC;
		data[2] = 0x00;
		LTC2949_WRITE(LTC2949_REG_STATUS, 1, data);
		LTC2949_WRITE(LTC2949_REG_EXTFAULTS, 2, data + 1);
		// Note: no read-back here, as it is done at the
		// beginning of the next iteration
		// unlock the memory
		error |= LTC2949_MemLock(false);
		if (error || (i > 0 && i % 100 == 0))
		{
			Serial.print(F("CNT,"));
			Serial.print(i);
			PrintComma();
			Serial.print(maxISO0SOClrTime);
			PrintComma();
			PrintOkErr(error);
			if (error)
				break;
		}
		//#if(FLTINJ_DELAY_LOOP_END_MS>0)
		//		// optinal delay for debugging only
		//		delay(FLTINJ_DELAY_LOOP_END_MS);
		//#endif
	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	//#if(FLTINJ_DELAY_LOOP_END_MS>0)
	//	// revert delay(FLTINJ_DELAY_LOOP_END_MS) for right timing calculation
	//	timingMeasure -= cnt * FLTINJ_DELAY_LOOP_END_MS;
	//#endif
	float t = (float)timingMeasure / (float)cnt;
	error |= WakeUpReportStatus(false);
	Serial.print(t);
	PrintComma();
	Serial.print(maxISO0SOClrTime);
	PrintComma();
	PrintOkErr(error);
}

// Open Wire Check (SM1)
// 
// Steps:
// - ensure correct initial condition (awake, slow CONT enabled, fast AUX single shot enabled)
// - measure channel								   (meas. 0)
// - measure channel with pull-up current source	   (meas. 1)
// - measure channel								   (meas. 2)
// - measure channel with pull-down current source	   (meas. 3)
// - measure channel								   (meas. 4)
// - measure pull-up current source via 4k resistor   (meas. 5)
// - measure pull-down current source via 4k resistor (meas. 6)
// - measure VREF via 4k resistor                     (meas. 7)
// 
// Note: Not all steps are always necessary. This example shows the maximum number of measurements for this SM
// - measurement of pull-up/down current source via 4k resistor checks for latent faults of the current sources
// - depending on external connections its sufficient to use only one polarity of the current sources
void SafetyMeasure1(int cnt, byte slotP, byte slotN, byte measureFlags, byte slot4kRef)
{
	// reset
	LTC2949_reset();
	byte error = WakeUpReportStatus(false, true);
	error |= WakeUpReportStatus(true, false);

	// constraint for this safety measure:
	// slow continuous
	// AUX fast single shot
	error |= CheckInitialConditionCont();

	// make sure HS byte is cleared
	error |= LTC2949_RdFastData();

	// start timing measurement
	unsigned long timingMeasure = millis();

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	int i = cnt;
	const byte measurements = 8; // 8 measurements, see above
	int16_t meas[measurements];
	memset(meas, 0, sizeof(int16_t) * measurements);
	while (i-- > 0)
	{
		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
		// configure MUX
		LTC2949_SlotFastCfg(slotP, slotN);

		/////////////////////////////////////////////////////////////////////////////////
		// disable current sources (GPIO setting ignored here...)
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(0);
		LTC2949_GpioCurrConfigWrite();
		// DO NOT USE CONT for this test!
		// measure...
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		// Note: part of safety checks is also to evaluate the HS byte being set after measurement and cleared when results have been read successfully, see parameter clrChkHS

		// store result
		meas[0] = fastData2949[LTC2949_RDFASTDATA_AUX];

		/////////////////////////////////////////////////////////////////////////////////
		// enable pull-up current sources on MUXP and MUXN
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(
			LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL |
			LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
			LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL |
			LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
		);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[1] = fastData2949[LTC2949_RDFASTDATA_AUX];

		if (bitMaskSetChk(measureFlags, 2))
		{
			// make some more measurements to show settling time due to RC filter
			for (uint8_t l = 3; l < measurements; l++)
			{
				LTC2949_ADxx();
				error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
				meas[l] = fastData2949[LTC2949_RDFASTDATA_AUX];
			}
		}

		/////////////////////////////////////////////////////////////////////////////////
		// disable current sources (GPIO setting ignored here...)
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(0);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[2] = fastData2949[LTC2949_RDFASTDATA_AUX];

		if (bitMaskSetChk(measureFlags, 1))
			continue;

		// extended measurements, see note at function definition

		/////////////////////////////////////////////////////////////////////////////////
		// enable pull-down current sources on MUXP and MUXN
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(
			LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
			LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
		);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[3] = fastData2949[LTC2949_RDFASTDATA_AUX];
		/////////////////////////////////////////////////////////////////////////////////
		// disable current sources (GPIO setting ignored here...)
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(0);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[4] = fastData2949[LTC2949_RDFASTDATA_AUX];

		/////////////////////////////////////////////////////////////////////////////////
		// check current sources
		/////////////////////////////////////////////////////////////////////////////////
		// configure MUX
		LTC2949_SlotFastCfg(slot4kRef, 0); // measured versus GND
		// enable pull-up current sources on MUXP and MUXN
		LTC2949_CurrSrcConfig(
			LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL |
			LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
			LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL |
			LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
		);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[5] = fastData2949[LTC2949_RDFASTDATA_AUX];
		/////////////////////////////////////////////////////////////////////////////////
		// enable pull-down current sources on MUXP and MUXN
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(
			LTC2949_BM_FCURGPIOCTRL_MUXPCUREN |
			LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
		);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[6] = fastData2949[LTC2949_RDFASTDATA_AUX];
		/////////////////////////////////////////////////////////////////////////////////
		// disable current sources (GPIO setting ignored here...)
		/////////////////////////////////////////////////////////////////////////////////
		LTC2949_CurrSrcConfig(0);
		LTC2949_GpioCurrConfigWrite();
		// measure...
		LTC2949_ADxx();
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		meas[7] = fastData2949[LTC2949_RDFASTDATA_AUX];

	}
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);


	Serial.print(t);
	for (byte i = 0; i < measurements; i++)
	{
		PrintComma();
		Serial.print(meas[i] * LTC2949_LSB_FAUX);
	}
	PrintComma();
	PrintOkErr(error);
}


// 10mV, e.g. for VREF2 - VREF2_250k or ((VREF2_250k-GND) - (VREF2-GND))
#define SM_AUX_250k_MAX_OFFSET (250e3*5*8e-9)
#define SM_AUX_250k_MAX_OFFSET_INT ((int16_t)(SM_AUX_250k_MAX_OFFSET / LTC2949_LSB_FAUX + 0.5))
// 2mV, e.g. for GND-GND or V1-V1
#define SM_AUX_MAX_OFFSET (2e-3)
#define SM_AUX_MAX_OFFSET_INT ((int16_t)(SM_AUX_MAX_OFFSET / LTC2949_LSB_FAUX + 0.5))

// Leakage Current Check via 2nd Pin Pair (SM2)
// for simplification only single ended measurements are performed
void SafetyMeasure2(int cnt, byte mux1, byte mux2, byte muxrefa, byte muxrefb, float Re, float Rs)
{
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	int16_t zero_gnd, zero_1, zero_2, zero_12, zero_21;
	int16_t mux1_meas, muxrefa_meas, muxrefb_meas;
	float ReDividedByRs = Re / Rs;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);

	// will also clear all STATUS , ALERT registers
	// fast aux only (single shot)
	error |= CheckInitialConditionCont();
	// make sure HS byte is cleared at the beginning
	error |= LTC2949_RdFastData();

	// start timing measurement
	unsigned long timingMeasure = millis();

	for (int cnt_ = cnt; (cnt_ > 0) && (error == 0); cnt_--)
	{
		//////////////
		//////////////
		// dummy conversion, sanity check by measuring zero volts (just for debugging, not necessary)
		LTC2949_SlotFastCfg(0, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_gnd = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// dummy conversion, sanity check by measuring zero volts at common mode of input mux1 (just for debugging, not necessary)
		LTC2949_SlotFastCfg(mux1, mux1);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_1 = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// dummy conversion, sanity check by measuring zero volts at common mode of input mux2 (just for debugging, not necessary)
		LTC2949_SlotFastCfg(mux2, mux2);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_2 = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// leakage measurement mux1-mux2
		LTC2949_SlotFastCfg(mux1, mux2);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_12 = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// leakage measurement mux2-mux1
		LTC2949_SlotFastCfg(mux2, mux1);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_21 = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// the normal measurement of the input
		LTC2949_SlotFastCfg(mux1, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		mux1_meas = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// reference measurement of the common point the redundant inputs with their series resistors
		LTC2949_SlotFastCfg(muxrefa, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		muxrefa_meas = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// reference measurement of the unknown input signal (for debugging only!)
		LTC2949_SlotFastCfg(muxrefb, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		muxrefb_meas = fastData2949[LTC2949_RDFASTDATA_AUX];

		// check results
		Serial.print('G');
		PrintComma();
		Serial.print(zero_gnd * LTC2949_LSB_FAUX, 6);
		PrintComma();
		if (abs(zero_gnd) >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results
		Serial.print('Z');
		Serial.print((char)('0' + mux1));
		PrintComma();
		Serial.print(zero_1 * LTC2949_LSB_FAUX, 6);
		PrintComma();
		if (abs(zero_1) >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results
		Serial.print('Z');
		Serial.print((char)('0' + mux2));
		PrintComma();
		Serial.print(zero_2 * LTC2949_LSB_FAUX, 6);
		PrintComma();
		if (abs(zero_2) >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// 30mV, well below the safety target of 50 mV
#define SM_AUX_LEAK_RDIV_ABS_TOL (30e-3/LTC2949_LSB_FAUX)

		// check results
		Serial.print('D');
		Serial.print((char)('0' + mux1));
		Serial.print((char)('0' + mux2));
		PrintComma();
		Serial.print(zero_12 * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print(zero_12 * ReDividedByRs * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print(zero_12);
		PrintComma();
		if (abs(zero_12) >= SM_AUX_LEAK_RDIV_ABS_TOL / ReDividedByRs)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results
		Serial.print('D');
		Serial.print((char)('0' + mux2));
		Serial.print((char)('0' + mux1));
		PrintComma();
		Serial.print(zero_21 * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print(zero_21 * ReDividedByRs * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print(zero_21);
		PrintComma();
		if (abs(zero_21) >= SM_AUX_LEAK_RDIV_ABS_TOL / ReDividedByRs)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check nominal measurement
		Serial.print('R');
		Serial.print((char)('0' + muxrefa));
		PrintComma();
		Serial.print(muxrefa_meas * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print('R');
		Serial.print((char)('0' + muxrefb));
		PrintComma();
		Serial.print(muxrefb_meas * LTC2949_LSB_FAUX, 6);
		PrintComma();
		Serial.print(mux1_meas * LTC2949_LSB_FAUX, 6);
		PrintComma();
		fastData2949[0] = mux1_meas - muxrefb_meas;
		Serial.print(fastData2949[0] * LTC2949_LSB_FAUX, 6);
		PrintComma();
		if (abs(fastData2949[0]) >= SM_AUX_LEAK_RDIV_ABS_TOL)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);
	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
}

// Reverse Polarity Check (SM3)
// Steps:
// - as an example we measure BAT: VBATP-VBAM via P2 ADC and VBATM-VBATP via AUX ADC
// - any other input signal can also be measured with reverse polarity via AUX
//   ADC only via two or three subsequent measurements (3: -,+,- or +,-,+)
void SafetyMeasure3(int cnt, byte muxp, byte muxn, uint16_t muxOffs)
{
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	int16_t plusBAT, minusBAT, minusMUXa, plusMUX, minusMUXb, zero_gnd;
	int32_t minusminusMUXsum;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);
	// will also clear all STATUS , ALERT registers
	// fast aux only (single shot)
	error |= CheckInitialConditionCont(LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACH2, LTC2949_BM_ADCCONF_P2ASV);

	// start timing measurement
	unsigned long timingMeasure = millis();

	for (int cnt_ = cnt; (cnt_ > 0) && (error == 0); cnt_--)
	{
		////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		//////////////
		//////////////
		// dummy conversion, sanity check by measuring zero volts (just for debugging, not necessary)
		LTC2949_SlotFastCfg(0, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		zero_gnd = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// measure BAT via P2 and -BAT via AUX simultaneously
		LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VBATM, LTC2949_SLOTMUX_VBATP);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		plusBAT = fastData2949[LTC2949_RDFASTDATA_BAT];
		minusBAT = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		//////////////
		//////////////
		// -mux measurement
		LTC2949_SlotFastCfg(muxn, muxp);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		minusMUXa = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// +mux measurement
		LTC2949_SlotFastCfg(muxp, muxn);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		plusMUX = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// -mux measurement
		LTC2949_SlotFastCfg(muxn, muxp);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		minusMUXb = fastData2949[LTC2949_RDFASTDATA_AUX];

		// check results GND-GND
		Serial.print('Z');
		PrintComma();
		Serial.print(zero_gnd * LTC2949_LSB_FAUX, 5);
		PrintComma();
		zero_gnd = abs(zero_gnd);
		if (zero_gnd >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results BAT, -BAT
		Serial.print('B');
		PrintComma();
		Serial.print(plusBAT * LTC2949_LSB_FBAT, 3);
		PrintComma();
		Serial.print(-minusBAT * LTC2949_LSB_FAUX, 3);
		PrintComma();
		plusBAT = plusBAT + minusBAT;
		Serial.print(plusBAT * LTC2949_LSB_FAUX, 5);
		PrintComma();
		plusBAT = abs(plusBAT);
		if (plusBAT >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results -AUX, +AUX, -AUX
		minusminusMUXsum = -minusMUXa - minusMUXb;
		Serial.print('X');
		PrintComma();
		Serial.print(-minusMUXa * LTC2949_LSB_FAUX, 3);
		PrintComma();
		Serial.print(plusMUX * LTC2949_LSB_FAUX, 3);
		PrintComma();
		Serial.print(-minusMUXb * LTC2949_LSB_FAUX, 3);
		PrintComma();
		Serial.print(minusminusMUXsum * 0.5 * LTC2949_LSB_FAUX, 3);
		PrintComma();
		minusminusMUXsum = minusminusMUXsum - ((int32_t)plusMUX) * 2;
		minusminusMUXsum = minusminusMUXsum / 2;
		Serial.print(minusminusMUXsum * LTC2949_LSB_FAUX, 5);
		PrintComma();
		minusminusMUXsum = abs(minusminusMUXsum);
		if (minusminusMUXsum >= muxOffs)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
}

// TODO: INL error
#define SM_AUX_INL_MAX_ERR 5e-3

// Internal Leakage Current Check (SM4.1, SM4.2, SM4.3)
void SafetyMeasure4_x(int cnt, byte muxvref)
{
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	int16_t p250k, n250k, auxOffset, vrefvx, vref2;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);
	// will also clear all STATUS , ALERT registers
	// fast aux only (single shot)
	error |= CheckInitialConditionCont();

	// start timing measurement
	unsigned long timingMeasure = millis();

	for (int cnt_ = cnt; (cnt_ > 0) && (error == 0); cnt_--)
	{
		////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		//////////////
		//////////////
		// offset measurement
		LTC2949_SlotFastCfg(0, 0);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		auxOffset = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// check leakage on muxn
		LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2, LTC2949_SLOTMUX_VREF2_250k);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		n250k = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// check leakage on muxp
		LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2_250k, LTC2949_SLOTMUX_VREF2);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		p250k = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// vref2
		LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2, LTC2949_SLOTMUX_GND);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		vref2 = fastData2949[LTC2949_RDFASTDATA_AUX];
		//////////////
		//////////////
		// vref via Vx
		LTC2949_SlotFastCfg(muxvref, LTC2949_SLOTMUX_GND);
		// trigger the measurement
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		vrefvx = fastData2949[LTC2949_RDFASTDATA_AUX];

		// check results GND-GND
		Serial.print('Z');
		PrintComma();
		Serial.print(auxOffset * LTC2949_LSB_FAUX, 5);
		PrintComma();
		auxOffset = abs(auxOffset);
		if (auxOffset >= SM_AUX_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results MUXP leakage
		Serial.print('P');
		PrintComma();
		Serial.print(p250k * LTC2949_LSB_FAUX, 5);
		PrintComma();
		p250k = abs(p250k);
		if (p250k >= SM_AUX_250k_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);

		// check results MUXN leakage
		Serial.print('N');
		PrintComma();
		Serial.print(n250k * LTC2949_LSB_FAUX, 5);
		PrintComma();
		n250k = abs(n250k);
		if (n250k >= SM_AUX_250k_MAX_OFFSET_INT)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);


		// INL:
		// three points: auxOffset, vref2, vrefvx
		// calculate gain error
		float gain = LTC2949_VREF / (vrefvx - auxOffset) / LTC2949_LSB_FAUX;
		// adjust VREF2 measurement for gain and offset error
		float vref2Adj = (vref2 - auxOffset) * gain * LTC2949_LSB_FAUX;
		// adjust VREF2 measurement for gain and offset error
		float inl = vref2Adj - LTC2949_VREF2;
		// check results MUXN leakage
		Serial.print('L');
		PrintComma();
		Serial.print(auxOffset * LTC2949_LSB_FAUX, 5);
		PrintComma();
		Serial.print(vref2 * LTC2949_LSB_FAUX, 5);
		// also SM15 can be done here:
		vref2 = vref2 - LTC2949_VREF2_INT_NOMINAL;
		vref2 = abs(vref2);
		if (vref2 > LTC2949_VREF2_INT_ABS_TOL)
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('|');
			Serial.print('E');
		}
		PrintComma();
		Serial.print(vrefvx * LTC2949_LSB_FAUX, 5);
		PrintComma();
		Serial.print(gain, 5);
		PrintComma();
		Serial.print(inl, 5);
		inl = abs(inl);
		if (inl > SM_AUX_INL_MAX_ERR)
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('|');
			Serial.print('e');
		}
		PrintComma();

		PrintOkErr(error);
	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
}

#define SM9_TEST_REG LTC2949_REG_STATCEOFM

// External Communication Verification (SM9)
// Note:
// - the following shows how to inject errors on LTC2949 side using
//   ISO0 bit ESERERRINJ. It is recommended to use this feature
//   for software debugging only. External communication
//   errors can be injected much more efficient on the SPI master
//   side, via altering a single bit in the data or PEC read from
//   LTC2949
void SafetyMeasure9()
{
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);
	// another time to report the cleared status values
	error |= WakeUpReportStatus(true);
	byte temp;

	boolean ok = false;

	// the actual test starts here.
	// start timing measurement
	unsigned long timingMeasure = millis();
	while (true)
	{
		byte data;
		// initial check of STATUS / FAULTS reg, must be zero apart of the UPDATE bit that is ignored
		// Note: late on we will also lock the memory and check status again!
		// read status
		if (error |= LTC2949_READ(LTC2949_REG_STATUS, 1, &data) || data & ~LTC2949_BM_STATUS_UPDATE)
		{
			Serial.println('a'); break;
		}
		// read FAULTS (must still be zero)
		if ((error = LTC2949_READ(LTC2949_REG_FAULTS, 1, &data)) || data)
		{
			Serial.println('b'); break;
		}

		// we will use LTC2949_REG_FAMUXN as a test register for communication test, which is a good candidate for several reasons:
		//  written value is ignored by LTC2949 as long as LTC2949_REG_FAMUXP is not written
		//  changes to fast mux configuration are only considered when doing fast single shot conversion (running fast continuous is not considered)
		//  before starting the test we read the current value to be able to restore it
		//  its on the special row, so independent from current page select
		// 
		// store current value of FAMUXN
		if ((error = LTC2949_READ(SM9_TEST_REG, 1, &temp)))
		{
			temp = 0;
			{Serial.println('c'); break; }
		}
		// write test data
		LTC2949_WRITE(SM9_TEST_REG, 0xA5U);
		// read back
		if ((error = LTC2949_READ(SM9_TEST_REG, 1, &data)) || (data != 0xA5U))
		{
			Serial.println('d'); SerialPrintByteX2(data); break;
		}
		// write test data
		LTC2949_WRITE(SM9_TEST_REG, 0x5AU);
		// read back
		if ((error = LTC2949_READ(SM9_TEST_REG, 1, &data)) || data != 0x5AU)
		{
			Serial.println('e'); break;
		}
		// lock memory to ensure all memory contend is updated and locked (so for sure if there was any write PEC error before it will be reported in STATUS reg)
		if ((error = LTC2949_MemLock(true)))
		{
			Serial.println('f'); break;
		}
		// read status
		if ((error = LTC2949_READ(LTC2949_REG_STATUS, 1, &data)) || (data & ~LTC2949_BM_STATUS_UPDATE))
		{
			Serial.println('g'); break;
		}
		// read FAULTS (must still be zero)
		if ((error = LTC2949_READ(LTC2949_REG_FAULTS, 1, &data)) || data)
		{
			Serial.println('h'); break;
		}
		// set bit to insert fake PEC errors generated by LTC2949
		LTC2949_WRITE(LTC2949_REG_ISO0, LTC2949_BM_ISO0_ESERERRINJ);
		// read back (note PEC error injector will only become active after memory unlock)
		if ((error = LTC2949_READ(LTC2949_REG_ISO0, 1, &data)) || data != LTC2949_BM_ISO0_ESERERRINJ)
		{
			Serial.println('i'); break;
		}
		// unlocking the memory will enable the PEC error injector
		LTC2949_MemLock(false);
		// now we poll for the error
		// note: For sure polling must end with PEC error before timeout!
		//  the last written value to FAMUXN was 0x5AU which is unequal to 0xA5U (so we keep polling as long as PEC error injector is not yet active)
		//  once the PEC error injector is active, we will receive them and LTC2949_PollReg will return with the PEC error
		byte err = LTC2949_PollReg(SM9_TEST_REG, 0xA5U, 0xFFU, LTC2949_TIMING_CONT_CYCLE * 10, true);
		if (bitMaskSetChk(err, LTC2949_ERRCODE_TIMEOUT) || (err == 0))
		{
			// no error is an error here!
			{Serial.println('j'); SerialPrintByteX2(err); break; }
		}
		// disable injector
		LTC2949_WRITE(LTC2949_REG_ISO0, 0);
		// now we poll FAMUXN with the last written data, but do not stop on PEC errors. At some point (before timeout) we must read
		// the right value without PEC error
		if ((error = LTC2949_PollReg(SM9_TEST_REG, 0x5AU, 0xFFU, LTC2949_TIMING_CONT_CYCLE, false)))
		{
			Serial.println('k'); break;
		}
		// read status (must still be zero, apart UPDATE bit)
		if ((error = LTC2949_READ(LTC2949_REG_STATUS, 1, &data)) || (data & ~LTC2949_BM_STATUS_UPDATE))
		{
			Serial.println('l'); break;
		}
		// read FAULTS (must still be zero)
		if ((error = LTC2949_READ(LTC2949_REG_FAULTS, 1, &data)) || data)
		{
			Serial.println('m'); break;
		}

		// try PEC error detection when writing from Master to slave
		// we can introduce errors at several places, see LTC2949_DCWR_ERRORTYPE_..., e.g. LTC2949_DCWR_ERRORTYPE_CMDADDR
		for (byte errorType = 0; errorType < LTC2949_DCWR_ERRORTYPES_CNT; errorType++)
		{
			LTC2949_DcWR_PECerror(
				/*byte addr:*/SM9_TEST_REG,
				/*byte data:*/errorType,
				/*byte errorType:*/errorType);
		}
		// now we poll for the EXTCOMMERR bit
		if ((error = LTC2949_PollReg(LTC2949_REG_FAULTS, LTC2949_BM_FAULTS_EXTCOMMERR, 0xFFU, LTC2949_TIMING_CONT_CYCLE, true)))
		{
			Serial.println('n'); break;
		}
		// read back again FAMUXN to check that none of the above write commands was successful
		if ((error = LTC2949_READ(SM9_TEST_REG, 1, &data)) || (data != 0x5AU))
		{
			Serial.println('o'); break;
		}

		ok = true;
		break;
	}
	// calc execution time
	timingMeasure = millis() - timingMeasure;

	if (!ok)
	{
		error |= LTC2949_ERRCODE_OTHER;
		// in any case we have to clear the PEC error injector
		LTC2949_WRITE(LTC2949_REG_ISO0, 0);
		delay(LTC2949_TIMING_CONT_CYCLE);
	}

	error |= LTC2949_MemLock(true);
	LTC2949_WRITE(LTC2949_REG_FAULTS, 0);
	LTC2949_MemLock(false);
	delay(LTC2949_TIMING_CONT_CYCLE);

	// clear status faults....
	error |= WakeUpReportStatus(true);


	// restore previous value
	LTC2949_WRITE(SM9_TEST_REG, temp);

	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(timingMeasure);
	PrintComma();
	PrintOkErr(error);

}

#define SM11_TIMEOUT_LONG  (4000*1.05)
#define SM11_TIMEOUT_SHORT (300*1.05)

#define SM11_SM12_OPT_LONG           1
#define SM11_SM12_OPT_VERBOSE        2
#define SM11_SM12_OPT_NOANARB        4
#define SM11_SM12_OPT_DO_SM11_SM12   8
#define SM11_SM12_OPT_ANA_RDMDWR     16

#define SM11_SM12_DFLT_OCC (\
	LTC2949_BM_OCC1CTRL_OCC1DAC0 | LTC2949_BM_OCC1CTRL_OCC1DAC1 | LTC2949_BM_OCC1CTRL_OCC1DAC2 |\
	LTC2949_BM_OCC1CTRL_OCC1DGLT0 | LTC2949_BM_OCC1CTRL_OCC1DGLT1 |\
	LTC2949_BM_OCC1CTRL_OCC1EN)

// this is usually executed directly before SM11, see SM11_SM12_OPT_DO_SM11_SM12
void SafetyMeasure11(int cnt, uint8_t opt)
{
	// this test is typically done directly after
	// power-up / reset
	LTC2949_reset();
	byte error = WakeUpReportStatus(false, false);

	// configure the OCC without enabling it yet
	byte dataRb[2];
	dataRb[0] = SM11_SM12_DFLT_OCC;
	dataRb[1] = dataRb[0];

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	int i = cnt;
	// start timing measurement
	unsigned long timingMeasure = millis();
	while (i-- > 0)
	{
		byte data[2];
		// check initial state
		error |= LTC2949_READ(LTC2949_REG_ISO1, 2, data);
		if (data[0] != 0 || data[1] != 0) // both registers must be zero
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(i);
			Serial.print(F("DFTL,"));
			SerialPrintByteArrayHex(data, 2, true, true);
			PrintOkErr(error);
		}

		// for every iteration we flip two bits
		dataRb[0] ^= LTC2949_BM_OCC1CTRL_OCC1DAC0 | LTC2949_BM_OCC1CTRL_OCC1DGLT0;
		dataRb[1] ^= LTC2949_BM_OCC2CTRL_OCC2DAC0 | LTC2949_BM_OCC2CTRL_OCC2DGLT0;
		// write configuration
		LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, dataRb);
		// read back configuration and ...
		error |= LTC2949_READ(LTC2949_REG_OCC1CTRL, 2, data);
		// ... check
		if (data[0] != dataRb[0] || data[1] != dataRb[1])
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(F("CFG,"));
			SerialPrintByteArrayHex(dataRb, 2, true, true);
			SerialPrintByteArrayHex(data, 2, true, true);
			PrintOkErr(error);
		}
		// Note: the shadow register can't be reset, as it is continuously overwritten by LTC2949!

		// select short OCC
		if (bitMaskClrChk(opt, SM11_SM12_OPT_LONG)) // bit 0 of opt can be set to make the long, 4 seconds, test!
			LTC2949_WRITE(LTC2949_REG_ISO2, LTC2949_BM_ISO2_SHORTOCC);
		// start the test
		LTC2949_WRITE(LTC2949_REG_ISO1, LTC2949_BM_ISO1_AFSR2);
		// read back written data
		error |= LTC2949_READ(LTC2949_REG_ISO1, 2, data);
		// set time out
		unsigned long stopWatch = millis() + (bitMaskClrChk(opt, SM11_SM12_OPT_LONG) ? SM11_TIMEOUT_SHORT : SM11_TIMEOUT_LONG);
		// check read-back
		if (data[0] != LTC2949_BM_ISO1_AFSR2 || data[1] != (bitMaskClrChk(opt, SM11_SM12_OPT_LONG) ? LTC2949_BM_ISO2_SHORTOCC : 0))
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(i);
			Serial.print(F("SET,"));
			SerialPrintByteArrayHex(data, 2, true, true);
			PrintOkErr(error);
		}

		// poll for test done
		data[0] = 0;
		while (
			((error |= LTC2949_READ(LTC2949_REG_ISO1, 1, data)) == 0) &&
			(data[0] != LTC2949_BM_ISO1_AFSR2RSL) &&
			(LTC_TIMEOUT_CHECK(millis(), stopWatch) ? (error |= LTC2949_ERRCODE_TIMEOUT) : (error)) == 0)
			continue;
		if (data[0] != LTC2949_BM_ISO1_AFSR2RSL)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(i);
			Serial.print(F("RSL,"));
			SerialPrintByteArrayHex(data, 1, true, true);
			PrintOkErr(error);
		}

		// poll for shadow reg
		while (
			((error |= LTC2949_READ(LTC2949_REG_OCC1CTRLSHDW, 2, data)) == 0) &&
			((data[0] != dataRb[0]) ||
			(data[1] != dataRb[1])) &&
				(LTC_TIMEOUT_CHECK(millis(), stopWatch) ? (error |= LTC2949_ERRCODE_TIMEOUT) : (error)) == 0)
			continue;
		if ((data[0] != dataRb[0]) || (data[1] != dataRb[1]))
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(i);
			Serial.print(F("SHD,"));
			SerialPrintByteArrayHex(data, 2, true, true);
			SerialPrintByteArrayHex(dataRb, 2, true, true);
			PrintOkErr(error);
		}

		//delay(200);
		// clear results
		data[0] = 0;
		data[1] = 0;
		LTC2949_WRITE(LTC2949_REG_ISO1, 2, data);
		//LTC2949_WRITE(LTC2949_REG_OCC1CTRLSHDW, 2, data);
		if (error || (i > 0 && i % 10 == 0))
		{
			Serial.print(F("CNT,"));
			Serial.print(i);
			PrintComma();
			PrintOkErr(error);
			if (error)
				break;
		}
	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// TODO: only now we would enable the OCC
	dataRb[0] |= LTC2949_BM_OCC1CTRL_OCC1EN;
	dataRb[1] |= LTC2949_BM_OCC2CTRL_OCC2EN;
	// configure the thresholds
	LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, dataRb);
	delay(300); // to be sure OCC bit would be set in case of overcurrent
	// check status of OCC
	error |= WakeUpReportStatus(false);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
	// clear again
	// data[0] = 0;
	// data[1] = 0;
	// LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, data);
}

#define SM12_ANAREG4F_IPIMTOAUX (1UL << 3 | 1UL << 4)
#define SM12_POSTH_INT (0.5/LTC2949_LSB_FAUX + 0.5)
#define SM12_NEGTH_INT (-0.3/LTC2949_LSB_FAUX - 0.5)
#define SM12_I2AUX_ABS 2e-3
// squeeze aux measurements after disabling the current sources 
#define SM12_AUX_SQUEEZE_COUNT 0


// this is usually executed directly after SM11, see SM11_SM12_OPT_DO_SM11_SM12
void SafetyMeasure12(byte cnt, uint8_t opt)
{
	byte error = CheckInitialConditionCont(LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACH2 /*| LTC2949_BM_FACTRL_FACH1*/);
	int16_t ocowpth, ocowzero, ocownth;

	byte dataRb[2];

	// sanity check: OCC must have been already configured before SM11
	error |= LTC2949_READ(LTC2949_REG_OCC1CTRL, 2, dataRb);

	// check to make sure both OCCs are enabled
	if (bitMaskClrChk(dataRb[0], LTC2949_BM_OCC1CTRL_OCC1DAC0 | LTC2949_BM_OCC1CTRL_OCC1DAC1 | LTC2949_BM_OCC1CTRL_OCC1DAC2) ||
		bitMaskClrChk(dataRb[1], LTC2949_BM_OCC2CTRL_OCC2DAC0 | LTC2949_BM_OCC2CTRL_OCC2DAC1 | LTC2949_BM_OCC2CTRL_OCC2DAC2) ||
		bitMaskClrChk(dataRb[0], LTC2949_BM_OCC1CTRL_OCC1EN) ||
		bitMaskClrChk(dataRb[1], LTC2949_BM_OCC2CTRL_OCC2EN)
		)
	{
		Serial.print(F("EN,OCC,"));
		dataRb[0] = SM11_SM12_DFLT_OCC;
		dataRb[1] = dataRb[0];
		// only now we would enable the OCCs
		LTC2949_WRITE(LTC2949_REG_OCC1CTRL, 2, dataRb);
		// read back
		error |= LTC2949_READ(LTC2949_REG_OCC1CTRL, 2, dataRb);
		if (dataRb[0] != SM11_SM12_DFLT_OCC || dataRb[1] != SM11_SM12_DFLT_OCC)
			error |= LTC2949_ERRCODE_OTHER;
		PrintOkErr(error);
	}

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	int i = cnt;
	//boolean pos = true;
	// start timing measurement
	unsigned long timingMeasure = millis();
	while (i-- > 0)
	{
		byte muxp, muxn;
		uint32_t ui24 = 0;
		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

		LTC2949_SlotFastCfg(LTC2949_SLOTMUX_IPT, LTC2949_SLOTMUX_IMT);
		error |= LTC2949_GetSlotFastCfg(&muxp, &muxn);
		if (muxp != LTC2949_SLOTMUX_IPT || muxn != LTC2949_SLOTMUX_IMT)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
		{
			Serial.print(i);
			Serial.print(F("MX,"));
			Serial.print(muxp);
			PrintComma();
			Serial.print(muxn);
			PrintComma();
			PrintOkErr(error);
		}

		LTC2949_EnterDebug();
		if (bitMaskSetChk(opt, SM11_SM12_OPT_ANA_RDMDWR))
		{
			error |= LTC2949_SwBypRead(0x4F, &ui24); // read
			if (ui24)
				error |= LTC2949_ERRCODE_OTHER;
			if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
			{
				Serial.print(i);
				Serial.print(F("aR,"));
				Serial.print(ui24, HEX);
				PrintComma();
				PrintOkErr(error);
			}
		}
		ui24 = SM12_ANAREG4F_IPIMTOAUX;         // modify
		error |= LTC2949_SwBypWrite(0x4F, ui24); // write
		if (bitMaskClrChk(opt, SM11_SM12_OPT_NOANARB))
		{
			// check written bit
			error |= LTC2949_SwBypRead(0x4F, &ui24); // read
			if (ui24 != SM12_ANAREG4F_IPIMTOAUX)
				error |= LTC2949_ERRCODE_OTHER;
			if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || error)
			{
				Serial.print(i);
				Serial.print(F("AR1,"));
				Serial.print(ui24, HEX);
				PrintComma();
				PrintOkErr(error);
			}
		}
		LTC2949_ExitDebug();

		LTC2949_GpioCurrConfigClr();
		LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PUMUXP | LTC2949_CURR_PDMUXN);
		LTC2949_GpioCurrConfigWrite();
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		ocowpth = fastData2949[LTC2949_RDFASTDATA_AUX];
		if (ocowpth < SM12_POSTH_INT)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || ocowpth <= SM12_POSTH_INT)
		{
			Serial.print(i);
			Serial.print(F("PTH,"));
			Serial.print(ocowpth * LTC2949_LSB_FAUX, 6);
			//PrintComma();
			//Serial.print(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FI1, 6);
			PrintComma();
			Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2, 6);
			PrintComma();
			PrintOkErr(error);
		}

		LTC2949_GpioCurrConfigClr();
		LTC2949_GpioCurrConfigWrite();
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		ocowzero = fastData2949[LTC2949_RDFASTDATA_AUX];
#if(SM12_AUX_SQUEEZE_COUNT>0)
		int16_t auxAr[SM12_AUX_SQUEEZE_COUNT];
		for (uint8_t k = 0; k < SM12_AUX_SQUEEZE_COUNT; k++)
		{
			// 2nd conversion
			LTC2949_ADxx();
			// poll LTC2949 for conversion done
			error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
			auxAr[k] = fastData2949[LTC2949_RDFASTDATA_AUX];
		}
#endif
		float diff = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX;
		diff -= fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2;
		diff = abs(diff);
		if (diff > SM12_I2AUX_ABS)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || diff > SM12_I2AUX_ABS)
		{
			Serial.print(i);
			Serial.print(F("I2X,"));
			Serial.print(ocowzero * LTC2949_LSB_FAUX, 6);
			PrintComma();
#if(SM12_AUX_SQUEEZE_COUNT>0)
			for (uint8_t k = 0; k < LTCDEF_AUXCOUNT; k++)
			{
				Serial.print(auxAr[k] * LTC2949_LSB_FAUX, 6);
				PrintComma();
			}
#endif
			//Serial.print(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FI1, 6);
			//PrintComma();
			Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2, 6);
			PrintComma();
			Serial.print(diff, 6);
			PrintComma();
			PrintOkErr(error);
		}

		LTC2949_GpioCurrConfigClr();
		LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PDMUXP | LTC2949_CURR_PUMUXN);
		LTC2949_GpioCurrConfigWrite();
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		ocownth = fastData2949[LTC2949_RDFASTDATA_AUX];
		if (ocownth > SM12_NEGTH_INT)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || ocownth >= SM12_NEGTH_INT)
		{
			Serial.print(i);
			Serial.print(F("NTH,"));
			Serial.print(ocownth * LTC2949_LSB_FAUX, 6);
			PrintComma();
			Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2, 6);
			PrintComma();
			PrintOkErr(error);
		}

		LTC2949_GpioCurrConfigClr();
		LTC2949_GpioCurrConfigWrite();
		LTC2949_ADxx();
		// poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
		ocowzero = fastData2949[LTC2949_RDFASTDATA_AUX];
#if(SM12_AUX_SQUEEZE_COUNT>0)
		for (uint8_t k = 0; k < LTCDEF_AUXCOUNT; k++)
		{
			// 2nd conversion
			LTC2949_ADxx();
			// poll LTC2949 for conversion done
			error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
			auxAr[k] = fastData2949[LTC2949_RDFASTDATA_AUX];
		}
#endif
		diff = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX;
		diff -= fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2;
		diff = abs(diff);
		if (diff > SM12_I2AUX_ABS)
			error |= LTC2949_ERRCODE_OTHER;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_VERBOSE) || diff > SM12_I2AUX_ABS)
		{
			Serial.print(i);
			Serial.print(F("I3X,"));
			Serial.print(ocowzero * LTC2949_LSB_FAUX, 6);
			PrintComma();
#if(SM12_AUX_SQUEEZE_COUNT>0)
			for (uint8_t k = 0; k < LTCDEF_AUXCOUNT; k++)
			{
				Serial.print(auxAr[k] * LTC2949_LSB_FAUX, 6);
				PrintComma();
			}
#endif
			//Serial.print(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FI1, 6);
			//PrintComma();
			Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2, 6);
			PrintComma();
			Serial.print(diff, 6);
			PrintComma();
			PrintOkErr(error);
		}

		LTC2949_EnterDebug();
		ui24 = 0;
		if (bitMaskSetChk(opt, SM11_SM12_OPT_ANA_RDMDWR))
		{
			error |= LTC2949_SwBypRead(0x4F, &ui24); // read
			ui24 &= ~SM12_ANAREG4F_IPIMTOAUX;         // modify
		}
		error |= LTC2949_SwBypWrite(0x4F, ui24); // write
		if (bitMaskClrChk(opt, SM11_SM12_OPT_NOANARB))
		{
			// check written bit
			error |= LTC2949_SwBypRead(0x4F, &ui24); // read
			if (ui24 != 0)
			{
				error |= LTC2949_ERRCODE_OTHER;
				Serial.print(i);
				Serial.print(F("AR2,"));
				SerialPrintByteArrayHex((byte*)ui24, 3, true, true);
				PrintOkErr(error);
			}
		}
		LTC2949_ExitDebug();
	}
	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check status of OCC
	error |= WakeUpReportStatus(false);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
}

// size of the cyclic buffer to store current measurement sets
// at least two is necessary, as one set is always written with new values (e.g. FIFO samples)
// whereas one set is used to make the redundant computation for comparison of the results
// 2: one will be updated continuously, the other is used to check results
#define ATSR3_DATA_BUFFER_SIZE 2 

#define LTCDEF_ATSR3_CYCLE_TIME 15

// max. number of samples to be read per loop
#define ATSR3_FIFOBUFFERSIZE ((uint8_t)(LTCDEF_ATSR3_CYCLE_TIME/0.8+4.5))
//24U /* we read a maximum of 24 samples per loop */

#define ATSR3_FAST_SAMPLES_PER_SLOW 128U /* 128 fast samples per slow sample */

// used to store current measurement results from
// - FIFO
// - Slow channel I
// - Slow channel I-RAW
struct IData
{
	// stores values of registers C1, C2
	// Charge check removed as it is redundant and also because it will fail after the first time the open wire check is performed 
	// (open wire current sources on CF2P, CF2M will cause error on C2)
	//int64_t cSlow[ATSR3_DATA_BUFFER_SIZE];
	// stores values of registers I1, I2
	int32_t iSlow[ATSR3_DATA_BUFFER_SIZE];
	// stores values of registers I1RAW, I2RAW
	// the source is 3 bytes unsigned integer, as we have to remove the offset later we already use 4 bytes signed integer here!
	int32_t iSlowRaw[ATSR3_DATA_BUFFER_SIZE];
	// stores sum of FIFO samples
	int32_t iFastAvg[ATSR3_DATA_BUFFER_SIZE];
	// cyclic buffer position where new fifo samples are stored
	byte iFastPos = 0;

	// buffer for FIFO samples
	int16_t buffer[ATSR3_FIFOBUFFERSIZE];
	// number of samples available
	uint16_t len;

	// stores number of fifo samples that were accumulated into iFastAvg
	// we always have to accumulate 128 fast samples which are linked to the
	// measurements reported by the slow channel (I1, I2, I1RAW, I2RAW)
	byte iFastSampleCount = 0;
	// cyclic buffer position where new slow channel current measurements will be stored
	byte iSlowPos = 0;
	// gain correction of I1 / I2
	float gci = 1.0;
};

// AFSR 3 UserDIA 
// Redundant Computation in Host (ATSR3)
// 
// RAW versus I maximum absolute error in I-LSBs (2uV)
#define ATSR3_MAXABSTOL_RAW 2.0
// RAW versus I maximum relative error (0.05%)
#define ATSR3_MAXRELTOL_RAW 0.05e-2
// FIFO-average versus I maximum absolute error in I-LSBs (~9 LSBs ~ 9uV)
#define ATSR3_MAXABSTOL_FIFO (LTC2949_LSB_FI1 / LTC2949_LSB_I1 + 0.5)
// FIFO-average versus I maximum relative error (0.05%)
#define ATSR3_MAXRELTOL_FIFO 0.05e-2
// I1 versus I2 maximum absolute error in I-LSBs (19uV)
#define ATSR3_MAXABSTOL_I12 20.0
// I1 versus I2 maximum relative error (0.25%)
#define ATSR3_MAXRELTOL_I12 0.25e-2
// I1 versus I2 fast sample maximum absolute error in fast-I-LSBs (50uV)
#define ATSR3_MAXABSTOL_FASTI12 (50e-6/LTC2949_LSB_FI1)


// C1 versus C2 maximum error definitions
// max. cycle time in seconds
#define ATSR3_MAX_CYCTIME_SECONDS (LTC2949_TIMING_CONT_CYCLE*1e-3)
// max. abs. tol. C1 versus C2 using LTC2949_LSB_C1 which depends on LTC2949_TBFAC (LTC2949_LSB_C1=1.21899e-05*LTC2949_TBFAC)
// this can be used for external clock configurations
#define ATSR3_MAXABSTOL_C12 (ATSR3_MAXABSTOL_I12 * LTC2949_LSB_I1 * ATSR3_MAX_CYCTIME_SECONDS / LTC2949_LSB_C1 + 0.5)
// LSB of Cx when using internal clock
#define LTC2949_LSB_C1_INTCLK (1.21899e-05 * LTC2949_INTC)
// max. abs. tol. C1 versus C2 using LTC2949_LSB_C1_INTCLK (internal clock only!)
#define ATSR3_MAXABSTOL_C12_INTCLK (ATSR3_MAXABSTOL_I12 * LTC2949_LSB_I1 * ATSR3_MAX_CYCTIME_SECONDS / LTC2949_LSB_C1_INTCLK + 0.5)
// 1.5% for internal clock
#define ATSR3_MAXRELTOL_C12 1.5e-2

#define ATSR3_RS1GC 1.0 /* OPTIONAL: replace with MEM GC factor (LTC2949_VAL_RS1GC, LTC2949_VAL_RS2GC) */
#define ATSR3_RS2GC 1.0 /* OPTIONAL: replace with MEM GC factor (LTC2949_VAL_RS1GC, LTC2949_VAL_RS2GC) */

// abs. tol. for periodic leakage test 
// as 
// for this reason we increase the abs. tol. by factor 5
#define ATSR3_LEAK_SM_ABSTOL (5*LTC2949_LEAKTEST_ABSTOL)


// optional to make fast single shot measurements every xxx samples to do e.g. open wire checks
#define ATSR3_OPT_DO_ADCV     0x01U
#define ATSR3_OPT_STOP_ON_ERR 0x02U
//#define ATSR3_OPT_VERBOSE0    0x04U
//#define ATSR3_OPT_VERBOSE1    0x08U
//#define ATSR3_OPT_VERBOSE2    0x10U
//#define ATSR3_OPT_VERBOSE3    0x20U
//#define ATSR3_OPT_DLY3 0x08U
//#define ATSR3_OPT_DLY4 0x10U
//#define ATSR3_OPT_DLY5 0x20U
//#define ATSR3_OPT_FD   0x80U
//#define ATSR3_OPT_ADCV_CYC_COUNT 8

byte ATSR3DoFastSSHT(uint8_t adcvCount, float gcaux);
byte ATSR3PrcFast(IData* iData, uint8_t m);
byte ATSR3PrcSlow(IData* iData, uint8_t m);
byte ATSR3DoChecks(IData* iData, uint8_t m, byte iSlowChkPos);
byte ATSR3DoCheckI1vsI2(IData* iData, byte iSlowChkPos);
byte ATSR3DoCheckFastI1vsI2(IData* iData);

byte ATSR3DoFastSSHT(uint8_t adcvCount, float gcaux)
{
	// fast single shot interrupt for NTC temperature measurement
	// and other diagnostic measurements
	LTC2949_DebugEnable = bitMaskSetChk(adcvCount, 0x80);
	bitMaskClr(adcvCount, 0x80);
	byte error = 0;
	int16_t* auxMeasures = new int16_t[adcvCount];

	// array for all possible fast single shot measurement results (I1, I2, BAT, AUX)
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

	// do the fast single shot measurements as fast as possible
	// --> keep the time when not being in fast cont. as short as possible
	// --> keep the time when not doing charge accumulation as short as possible
	LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA); // stop cont
	unsigned long stopWatchMicros = micros();

	// Array for MUX configuration
	byte muxCfg[2];
	for (byte i = 0; i < adcvCount; i++)
	{
		switch (i)
#define LTCDEF_ATSR3_FSSHT_NTC          0
#define LTCDEF_ATSR3_FSSHT_VREF2        1
#define LTCDEF_ATSR3_FSSHT_MVREF2_250K  2
#define LTCDEF_ATSR3_FSSHT_VREF2_250K   3
		{
			// Note: order of the measurements also ensures every 
			// every measurement is different
		case LTCDEF_ATSR3_FSSHT_NTC:
			// NTC voltage drop
			muxCfg[1] = LTC2949_SLOTMUX_V1;  // MUXP
			muxCfg[0] = LTC2949_SLOTMUX_GND; // MUXN
			break;
		case LTCDEF_ATSR3_FSSHT_VREF2:
			// VREF2 (22) - GND (0)
			muxCfg[1] = LTC2949_SLOTMUX_VREF2;  // MUXP
			muxCfg[0] = LTC2949_SLOTMUX_GND;    // MUXN
			break;
		case LTCDEF_ATSR3_FSSHT_MVREF2_250K:
			// GND (0) - VREF2_250k (23)
			muxCfg[1] = LTC2949_SLOTMUX_GND;        // MUXP
			muxCfg[0] = LTC2949_SLOTMUX_VREF2_250k; // MUXN
			break;
		default:
		case LTCDEF_ATSR3_FSSHT_VREF2_250K:
			// VREF2_250k (23) - GND (0)
			muxCfg[1] = LTC2949_SLOTMUX_VREF2_250k;  // MUXP
			muxCfg[0] = LTC2949_SLOTMUX_GND;         // MUXN
			break;
		}
		// write MUX control
		LTC2949_WRITE(LTC2949_REG_FAMUXN, 2, muxCfg);

		// ...trigger measurement 
		LTC2949_ADxx();
		if (LTC2949_DebugEnable)
			delay(3); // for sure after 2 milliseconds the conversion result is ready

		// ...poll and read LTC2949 fast conversion results
		// Note: its not necessary to check for HS byte being cleared
		//       via issuing a 2nd RDCV, because the measurements itself
		//       (vref, -vref) are for sure different, so we would
		//       realize if the conversion result was not updated.
		if (LTC2949_DebugEnable)
			error |= LTC2949_RdFastData(fastData2949);
		else
			error |= LTC2949_PollFastData(fastData2949);

		if (error)
		{
			Serial.print(F("FDP:"));
			SerialPrintByteArrayHex((byte*)fastData2949, LTC2949_RDFASTDATA_LENGTH * 2, false, true);
			PrintOkErr(error);
		}


		// store result for later processing
		auxMeasures[i] = fastData2949[LTC2949_RDFASTDATA_AUX];
	}
	// reset to prev. setting for open wire check
	muxCfg[1] = LTC2949_SLOTMUX_CFI2P;  // MUXP
	muxCfg[0] = LTC2949_SLOTMUX_CFI2M;  // MUXN
		// write MUX control
	LTC2949_WRITE(LTC2949_REG_FAMUXN, 2, muxCfg);

	// enable fast continuous
	LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACONV);
	// stop watch end
	stopWatchMicros = micros() - stopWatchMicros;
	LTC2949_DebugEnable = false;

	// check results
	// remark the programed gain correction factors 1.66, 1.33....
	// that have to be considered here!
	//  VREF2: LTC2949_SM_AA_FP_FAC / gcaux
	//  VREF2_250k: LTC2949_SM_55_FP_FAC / gcaux
	for (byte i = 0; i < adcvCount; i++)
	{
		int16_t diff, absTol, nom;
		char errID;
		diff = auxMeasures[i];
		switch (i)
		{
		case LTCDEF_ATSR3_FSSHT_NTC:
			// NTC voltage drop
			nom = LTC2949_VREF / 2.0 / LTC2949_LSB_FAUX;
			diff -= LTC2949_VREF / 2.0 / LTC2949_LSB_FAUX;
			// here +-100%, the practical range could be reduced further, depending on
			// used NTC and temperature range. For -40 to 125 dC the possible voltage
			// range is very close to 0V - 3.0V
			absTol = LTC2949_VREF / 2.0 / LTC2949_LSB_FAUX;
			errID = 'l';
			break; // todo: change to float and real values instead of LSBs
		case LTCDEF_ATSR3_FSSHT_VREF2:
			// VREF2 (22) - GND (0)
			diff *= gcaux / LTC2949_SM_AA_FP_FAC; // revert the gain correction
			auxMeasures[LTCDEF_ATSR3_FSSHT_VREF2] = diff;
			nom = LTC2949_VREF2_INT_NOMINAL;
			diff -= LTC2949_VREF2_INT_NOMINAL;
			absTol = LTC2949_VREF2_INT_ABS_TOL;
			errID = 'k';
			break;
		case LTCDEF_ATSR3_FSSHT_MVREF2_250K:
			// GND (0) - VREF2_250k (23)
			diff *= gcaux / LTC2949_SM_55_FP_FAC; // revert the gain correction
			auxMeasures[LTCDEF_ATSR3_FSSHT_MVREF2_250K] = diff;
			nom = LTC2949_VREF2_INT_NOMINAL;
			diff = -diff - LTC2949_VREF2_INT_NOMINAL;
			absTol = LTC2949_VREF2_INT_ABS_TOL;
			errID = 'n';
			break;
		default:
		case LTCDEF_ATSR3_FSSHT_VREF2_250K:
			// VREF2_250k (23) - GND (0)
			diff *= gcaux / LTC2949_SM_55_FP_FAC; // revert the gain correction
			auxMeasures[LTCDEF_ATSR3_FSSHT_VREF2_250K] = diff;
			nom = LTC2949_VREF2_INT_NOMINAL;
			diff -= LTC2949_VREF2_INT_NOMINAL;
			absTol = LTC2949_VREF2_INT_ABS_TOL;
			errID = 'm';
			//  leakage check via difference of above VREF2.. measurements.
			AbsRelErrCheck(
				/*float diff   */auxMeasures[LTCDEF_ATSR3_FSSHT_VREF2] - auxMeasures[LTCDEF_ATSR3_FSSHT_VREF2_250K],
				/*float nom    */0,
				/*float abs    */ATSR3_LEAK_SM_ABSTOL,
				/*float rel    */1.0e-2, // not relevant
				/*byte * error*/&error,
				/*char id      */'o');
			AbsRelErrCheck(
				/*float diff   */auxMeasures[LTCDEF_ATSR3_FSSHT_VREF2] + auxMeasures[LTCDEF_ATSR3_FSSHT_MVREF2_250K],
				/*float nom    */0,
				/*float abs    */ATSR3_LEAK_SM_ABSTOL,
				/*float rel    */1.0e-2, // not relevant
				/*byte * error*/&error,
				/*char id      */'p');
			break;
		}
		AbsRelErrCheck(
			/*float diff   */diff,
			/*float nom    */nom,
			/*float abs    */absTol,
			/*float rel    */1.0e-2,
			/*byte * error*/&error,
			/*char id      */errID);
	}

#define ATSR3_PRINT_RESULTS true
#ifdef ATSR3_PRINT_RESULTS
	Serial.println();
	for (byte i = 0; i < adcvCount; i++)
	{
		Serial.print('[');
		Serial.print(i);
		Serial.print(']');
		Serial.print(':');
		Serial.println(auxMeasures[i] * LTC2949_LSB_FAUX, 6);
	}
#endif


	// When going to fast continuous (FACONV-bit in FACTRL is set) the first sample 
	// will be lost (marked as RDOVR), if FIFO is read before the first sample was 
	// stored to the FIFO. RDCV polling method (poll for HS byte = 0x0F) can be used 
	// to know when the first sample is stored into the FIFO(s). Alternatively put 
	// a minimum wait time of 1.2ms.
	//delay(2); // for sure we have some fast samples now
	// print indicator showing we made the fast single shots
	Serial.print('/');
	// print milliseconds
	Serial.print(stopWatchMicros * 1e-3, 1);
	Serial.print('|');
	Serial.print(stopWatchMicros * 1e-3 / adcvCount, 2);
	Serial.println('\\');
	// not necessary, as we read FIFO only every xxx ms, defined by
	// LTCDEF_ATSR3_CYCLE_TIME
	//error |= LTC2949_PollFastData(); // poll for first sample
	delete[] auxMeasures;
	return error;
}

// does the redundant CHECK: compare RAW result with I and fifo average with I
byte ATSR3DoChecks(IData* iData, uint8_t m, byte iSlowChkPos)
{
	// m=0: I1
	// m=1: I2
	byte error = 0;

	// get the average and convert to float
	float diff = iData[m].iFastAvg[iSlowChkPos];
	// scale average
	diff *= LTC2949_LSB_FI1 / LTC2949_LSB_I1 / ATSR3_FAST_SAMPLES_PER_SLOW;
	// store scaled result (as it is printed later and compared to the other channel)
	iData[m].iFastAvg[iSlowChkPos] = diff;
	// calculate the abs. error
	diff -= iData[m].iSlow[iSlowChkPos];
	// check for tolerances
	AbsRelErrCheck(
		/*float diff   */diff,
		/*float nom    */iData[m].iSlow[iSlowChkPos],
		/*float abs    */ATSR3_MAXABSTOL_FIFO,
		/*float rel    */ATSR3_MAXRELTOL_FIFO,
		/*byte * error*/&error,
		/*char id      */'A');

	// get the raw value, remove the offset and convert to float
	diff = iData[m].iSlowRaw[iSlowChkPos] - ((int32_t)1 << 17);
	diff *= iData[m].gci; // scale with trim
	diff *= LTC2949_FAST2SLOW_MAGIC_GC; // scale with 'magic' factor
	diff *= m == 0 ? ATSR3_RS1GC : ATSR3_RS2GC; // OPTIONAL: scale the MEM GC factor
	// store adjusted result (as it is printed later and compared to the other channel)
	iData[m].iSlowRaw[iSlowChkPos] = diff;
	diff -= iData[m].iSlow[iSlowChkPos]; // calculate the abs. error
	AbsRelErrCheck(
		/*float diff   */diff,
		/*float nom    */iData[m].iSlow[iSlowChkPos],
		/*float abs    */ATSR3_MAXABSTOL_RAW,
		/*float rel    */ATSR3_MAXRELTOL_RAW,
		/*byte * error*/&error,
		/*char id      */'R');

	// report measurements for debugging
	Serial.print(iData[m].iSlow[iSlowChkPos]);
	PrintComma();
	Serial.print(iData[m].iSlowRaw[iSlowChkPos]);
	PrintComma();
	Serial.print(iData[m].iFastAvg[iSlowChkPos]);
	PrintComma();
	// optional print also charge
	// Serial.print((float)iData[m].cSlow[iSlowChkPos], 0);
	// PrintComma();
	return error;
}

byte ATSR3DoCheckI1vsI2(IData* iData, byte iSlowChkPos)
{
	byte error = 0;

	// here we compare I1 with I2. It is sufficient to 
	// compare only FIFO-AVG. Still its
	// a good sanity check to compare all (I, RAW, FIFO-AVG)
	//
	// get the average, calc diff of I1 to I2 and convert to float
	float diff = iData[1].iFastAvg[iSlowChkPos] - iData[0].iFastAvg[iSlowChkPos];
	AbsRelErrCheck(
		/*float diff   */diff,
		/*float nom    */iData[0].iFastAvg[iSlowChkPos],
		/*float abs    */ATSR3_MAXABSTOL_I12,
		/*float rel    */ATSR3_MAXRELTOL_I12,
		/*byte * error*/&error,
		/*char id      */'a');

	// get the RAW, calc diff of I1 to I2 and convert to float
	diff = iData[1].iSlowRaw[iSlowChkPos] - iData[0].iSlowRaw[iSlowChkPos];
	AbsRelErrCheck(
		/*float diff   */diff,
		/*float nom    */iData[0].iSlowRaw[iSlowChkPos],
		/*float abs    */ATSR3_MAXABSTOL_I12,
		/*float rel    */ATSR3_MAXRELTOL_I12,
		/*byte * error*/&error,
		/*char id      */'r');

	// get the standard value, calc diff of I1 to I2 and convert to float
	diff = iData[1].iSlow[iSlowChkPos] - iData[0].iSlow[iSlowChkPos];
	AbsRelErrCheck(
		/*float diff   */diff,
		/*float nom    */iData[0].iSlow[iSlowChkPos],
		/*float abs    */ATSR3_MAXABSTOL_I12,
		/*float rel    */ATSR3_MAXRELTOL_I12,
		/*byte * error*/&error,
		/*char id      */'i');

	// calc diff of C1 to C2 and convert to float
	//diff = iData[1].cSlow[iSlowChkPos] - iData[0].cSlow[iSlowChkPos];
	//AbsRelErrCheck(
	//	/*float diff   */diff,
	//	/*float nom    */iData[0].cSlow[iSlowChkPos],
	//	/*float abs    */ATSR3_MAXABSTOL_C12_INTCLK,
	//	/*float rel    */ATSR3_MAXRELTOL_C12,
	//	/*byte * error*/&error,
	//	/*char id      */'c');

	return error;
}

/*!*********************************************************************
\brief check fast samples I1 versus I2
***********************************************************************/
byte ATSR3DoCheckFastI1vsI2(IData* iData)
{
	uint16_t posI1;
	uint16_t posI2;
	//
	// calculate absolute positions of last sample within sample buffers
	posI1 = iData[0].iFastSampleCount + iData[0].iFastPos * ATSR3_FAST_SAMPLES_PER_SLOW;
	posI2 = iData[1].iFastSampleCount;
	if (iData[0].iFastPos > iData[1].iFastPos)
		posI2 += ATSR3_FAST_SAMPLES_PER_SLOW * (iData[1].iFastPos + ATSR3_DATA_BUFFER_SIZE);
	else
		posI2 += ATSR3_FAST_SAMPLES_PER_SLOW * iData[1].iFastPos;
	//
	// actually we don't care about the absolute value
	// we just need to know the sample count offset of
	// I2 versus I1, which we need to be able to match
	// the samples from both channels within the buffers.
	//
	// part A of offset is the difference of the absolute sample position
	int8_t lenI2vsI1 = iData[1].len - iData[0].len;
	// part B of offset is the difference between number of samples within buffer
	int8_t offsI2vsI1 = posI2 - posI1; // part A of
	// 
	// calculate matching index for buffer I2
	// Note: -1 because below j++ executed already beginning of first iteration
	int8_t j = lenI2vsI1 - offsI2vsI1 - 1;
	// examples
	// posI1, posI2, lenI1, lenI2, offsI2vsI1, lenI2vsI1,   j,   j++
	//   133,   142,    26,    22,          9,        -4, -14, -13
	//   135,   141,    26,    22,          6,        -4, -11, -10
	//   127,   110,    25,    22,        -14,        -3,  13,  14
	//   120,   110,    20,    23,        -10,         3,  12,  13
	//
	byte error = 0;
	uint8_t k = 0;
	for (uint8_t i = 0; i < iData[0].len; i++)
	{
		j++;
		// check if matching sample is available within
		// buffer I2
		if (j < 0 || (uint8_t)j >= iData[1].len)
			continue; // no matching sample inside buffer I2
		//
		// Note: For safety its no problem if there are always
		// some samples we do not compare. There will always be enough
		// overlap between the two buffers and the few samples we do
		// not compare are distributed totally random in time (due to
		// asynchronicity when FIFOs are read).
		//
		// now we can compare the matching samples
		// calculate difference
		float diff = iData[0].buffer[i] - iData[1].buffer[j];
		// check for error
		AbsRelErrCheck(
			/*float diff   */diff,
			/*float nom    */iData[0].buffer[i], // we take I1 as the reference
			/*float abs    */ATSR3_MAXABSTOL_FASTI12,
			/*float rel    */ATSR3_MAXRELTOL_I12,
			/*byte * error*/&error,
			/*char id      */'g');
		//
		// count samples that were checked
		k++;
	}

	if (k == 0)
	{
		// LTC2949_ERRCODE_TIMEOUT is used here to indicate
		// if there were no samples compared at all!
		error |= LTC2949_ERRCODE_TIMEOUT;
		Serial.println(F("TIMEOUT:I1vsI2"));
	}

#undef ATSR3_DEBUG_PRINT_FI1I2_CNT

#ifdef ATSR3_DEBUG_PRINT_FI1I2_CNT
	Serial.print('|');
	Serial.print(k);
	Serial.print('|');
#endif

	if (error)
		Serial.println(F(":I1vsI2"));

	return error;
}

byte ATSR3PrcFast(IData* iData, uint8_t m)
{
	// m=0: I1
	// m=1: I2
	byte error;
	byte iFastPos = iData[m].iFastPos;

	// process fifo samples
	//int16_t buffer[ATSR3_FIFOBUFFERSIZE]; // buffer for FIFO samples
	//uint16_t len = ATSR3_FIFOBUFFERSIZE;
	iData[m].len = ATSR3_FIFOBUFFERSIZE;
	boolean fifoFull1stSample;

	// read FIFO
	error = LTC2949_ReadFifo(
		/*byte      addr:             */ m == 0
		? LTC2949_REG_FIFOI1
		: LTC2949_REG_FIFOI2,
		/*uint16_t* len:              */ &iData[m].len,
		/*int16_t*  samples:          */ iData[m].buffer,
		/*boolean*  fifoFull1stSample:*/ &fifoFull1stSample,
		/*int8_t    rdOvrCntDown:     */ LTC2949_RDFIFO_STOP_EMPTY);

	if (fifoFull1stSample)
	{
		// FIFO overflow
		// this must never happen
		error |= LTC2949_ERRCODE_OTHER;
		Serial.print('F');
	}
	if (error)
	{
		Serial.println(F("FIF:"));
		PrintOkErr(error);
	}


	// process all samples (sum-up to calculate average)
	for (byte i = 0; i < iData[m].len; i++)
	{
		iData[m].iFastAvg[iFastPos] += iData[m].buffer[i];
		iData[m].iFastSampleCount++;
		if (iData[m].iFastSampleCount == ATSR3_FAST_SAMPLES_PER_SLOW)
		{
			// calc of sum of fifo sample done. We are ready to compare 
			// to values reported by the slow channel (see ATSR3DoChecks)
			iFastPos++; // increment to next element of the buffer
			iFastPos %= ATSR3_DATA_BUFFER_SIZE;
			iData[m].iFastPos = iFastPos;
			// initialize the next element of the buffer
			iData[m].iFastSampleCount = 0;
			iData[m].iFastAvg[iFastPos] = 0;
		}
	}
	return error;
}

byte ATSR3PrcSlow(IData* iData, uint8_t m)
{
	// m=0: I1
	// m=1: I2
	byte error;
	byte data[3];
	byte iSlowPos = iData[m].iSlowPos;

	// read and store values from slow channel
	//
	// current
	error = LTC2949_READ(m == 0
		? LTC2949_VAL_I1
		: LTC2949_VAL_I2, 3, data);
	iData[m].iSlow[iSlowPos] = LTC_3BytesToInt32(data);

	// // charge
	// error |= LTC2949_READ(m == 0
	// 	? LTC2949_VAL_C1
	// 	: LTC2949_VAL_C2, 6, data);
	// iData[m].cSlow[iSlowPos] = LTC_6BytesToInt64(data);

	// RAW current
	error |= LTC2949_READ(m == 0
		? LTC2949_VAL_I1RAW
		: LTC2949_VAL_I2RAW, 3, data);
	iData[m].iSlowRaw[iSlowPos] = LTC_3BytesToUInt32(data);

	// increment to next
	iSlowPos++;
	iSlowPos %= ATSR3_DATA_BUFFER_SIZE;
	iData[m].iSlowPos = iSlowPos;

	return error;
}

/*
The following example code shows a possible sequence that guarantees data coherency between values reported
in the slow channel every 100 ms and 128 samples each from the fast channel. Also it is shown how the
necessary checks are performed to ensure correct computation within LTC2949.
*/
void ATSR3ImplementationGuide(int cnt, uint8_t opt, uint8_t slowCycCnt, uint8_t adcvCount)
{
	// counts the number of slow channel cycles, before a set of 
	// fast single shots is performed
	uint8_t cyclesCounter = slowCycCnt;
	byte data[6];
	IData iData[2]; // two elements for I1,I2

	// wakeup and report initial status (checks all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////// 1 /////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// PROM value check: This needs to be done only once, as the PROM values are constant.
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// in IDLE read trim values of current and AUX ADCs and compare with the reported values in memory
	// make sure we are in IDLE mode
	error |= CheckInitialConditionIdle(); // will also clear all STATUS , ALERT registers
	float gcaux;
	if (!error)
	{
		// read trim values from analog die (PROM)
		int8_t gcauxAD, gci1AD, gci2AD;
		LTC2949_EnterDebug();
		error |= LTC2949_SwBypGetTrimms(&gcauxAD, &gci1AD, &gci2AD);
		LTC2949_ExitDebug();
		// read aux trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCV, 1, data);
		// read i1 trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCI1, 1, data + 1);
		// read i2 trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCI2, 1, data + 2);
		// compare trim values
		if (gcauxAD != (int8_t)data[0])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('a');
			Serial.print(gcauxAD);
			PrintComma();
			Serial.println((int8_t)data[0]);
		}
		if (gci1AD != (int8_t)data[1])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('i');
			Serial.print(gci1AD);
			PrintComma();
			Serial.println((int8_t)data[1]);
		}
		if (gci2AD != (int8_t)data[2])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('I');
			Serial.print(gci2AD);
			PrintComma();
			Serial.println((int8_t)data[2]);
		}
		// convert trim value to float
		iData[0].gci = gci1AD * LTC2949_SM_GC_LSB + LTC2949_SM_GC_OFFS;
		iData[1].gci = gci2AD * LTC2949_SM_GC_LSB + LTC2949_SM_GC_OFFS;
		Serial.print(iData[0].gci, 6);
		PrintComma();
		Serial.print(iData[1].gci, 6);
		PrintComma();
		Serial.print(F("Trim "));
		PrintOkErr(error);

		///
		// convert trim value to float
		gcaux = gcauxAD * LTC2949_SM_GC_LSB + LTC2949_SM_GC_OFFS;
		// calc user gain correction value leading to internal fixed point representation of 0xAAAA.... for the calculation of userGC * promGC
		// this is done to ensure all bits of the multiplier input will toggle
		LTC2949_FloatToF24Bytes(LTC2949_SM_AA_FP_FAC / gcaux, data);
		// write user gain correction
		LTC2949_WRITE(LTC2949_VAL_MUX1GC, 3, data);
		// same for 2nd gain correction factor
		LTC2949_FloatToF24Bytes(LTC2949_SM_55_FP_FAC / gcaux, data);
		// write user gain correction
		LTC2949_WRITE(LTC2949_VAL_MUX2GC, 3, data);
		// assign the above calculated user GCs to slot configurations
		// Note: polarity is "don't care"!
		data[0] = 0;
		data[1] = LTC2949_SLOTMUX_VREF2; // 2nd VREF
		LTC2949_WRITE(LTC2949_REG_MUXNSET1, 2, data);
		data[1] = LTC2949_SLOTMUX_VREF2_250k; // 2nd VREF via 250k
		LTC2949_WRITE(LTC2949_REG_MUXNSET2, 2, data);
	}
	else
		return;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////// 2 /////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// Continuous redundant computation check: This needs to be periodically during continuous operation
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//
	// initialize the first FIFO averages
	iData[0].iFastAvg[0] = 0;
	iData[1].iFastAvg[0] = 0;

	// start timing measurement
	unsigned long startOfMeas;

	// read & clear status
	error |= LTC2949_ReadChkStatusFaults(true, false);

	// select I2 input channel for AUX (needed for open wire checks, that are performed later)
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_CFI2P, LTC2949_SLOTMUX_CFI2M);
	LTC2949_GpioCurrConfigClr();
	LTC2949_GpioCurrConfigWrite();

	// enable measurement, fast cont. mode both channels and AUX enabled!
	error |= LTC2949_GoCont(LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACONV, 0, &startOfMeas);
	// Note: LTC2949_GoCont returns after slow cont mode is running and fast mode (in this case fast continuous) was enabled.
	// When going to fast continuous (FACONV-bit in FACTRL is set) the first sample 
	// will be lost (marked as RDOVR), if FIFO is read before the first sample was 
	// stored to the FIFO. RDCV polling method (poll for HS byte = 0x0F) can be used 
	// to know when the first sample is stored into the FIFO(s). Alternatively put 
	// a minimum wait time of 1.2ms.
	//delay(2); // wait 2ms, for sure we have some fast samples now

	// not necessary, as we read FIFO only every xxx ms, defined by
	// LTCDEF_ATSR3_CYCLE_TIME
	//error |= LTC2949_PollFastData(); // or alternatively poll for first sample

	if (error)
	{
		Serial.print(F("INIT,"));
		PrintOkErr(error);
		return;
	}


	// whole sequence is executed every xxx ms
	unsigned long timeSlot = millis() + LTCDEF_ATSR3_CYCLE_TIME;

	// cyclic buffer position where next redundant computation is performed
	byte iSlowChkPos = 0;
	// end of measurement
	//unsigned long endOfMeas = startOfMeas + cnt * 100;

	// this is the continuous test where we
	// - read slow channel I1, I2, I1RAW, I2RAW
	// - read fast channel's FIFOs I1, I2
	// - compare fast channel's FIFO samples I1 vs. I2
	// - calculate average of 128 FIFO samples
	// - compare the FIFO averages to the slow values I1, I2, I1RAW, I2RAW
	//boolean currentSourcesEnabled = false;
#define OPENWIRECHECKSTATE_ENTER 5
#define OPENWIRECHECKSTATE_DONE  0
	uint8_t OpenWireCheckState = OPENWIRECHECKSTATE_DONE;
	//boolean skipI1vsI2 = false;
	while (true)
	{
		// stop in case of error if flag ATSR3_OPT_STOP_ON_ERR is set
		if ((error != 0) && bitMaskSetChk(opt, ATSR3_OPT_STOP_ON_ERR))
			break;

		if (!LTC_TIMEOUT_CHECK(millis(), timeSlot))
			continue;

		// next time slot
		timeSlot = millis() + LTCDEF_ATSR3_CYCLE_TIME;

		byte error_;

		// store the last TBx value that was read (to be able to calculate time difference afterwards...)
		uint32_t lastTB1 = LTC2949_GetLastTBxInt();
		if (LTC2949_ChkUpdate(&error_)) // check for slow channel update
		{
			error |= ATSR3PrcSlow(iData, 0);
			error |= ATSR3PrcSlow(iData, 1);

			// SM16 oscillator check
			// check for delta 100ms from TB1
			// calculate difference between last TB1 and current TB1 in seconds
			float deltaT = (LTC2949_GetLastTBxInt() - lastTB1) * LTC2949_LSB_TB1;
			AbsRelErrCheck(
				/*float diff   */deltaT - 0.1,
				/*float nom    */0.1,
				/*float abs    */0, // absolute tolerance not relevant here!
				/*float rel    */5.0e-2,
				/*byte * error*/&error,
				/*char id      */'t');

			// we only stop at multiples of 100 ms
			if (cnt < 0 || (cnt--))
				;
			else
				break;
		}
		error |= error_;

		// process fast channel
		byte iFastPos0 = iData[0].iFastPos;
		byte iFastPos1 = iData[1].iFastPos;
		error |= ATSR3PrcFast(iData, 0);
		error |= ATSR3PrcFast(iData, 1);

		////////////////////////////////////////////////
		////////////////////////////////////////////////
		////////////////////////////////////////////////
		if (OpenWireCheckState)
		{
			int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
			float i1f, i2f, i2Auxf, i2AuxfMinusI1f;

			// read LTC2949 fast conversion results (latest conversion done)
			// we use this conversion to do the open wire check
			error |= LTC2949_RdFastData(fastData2949); // get last conversion result
			// calculate float values in volts
			i1f = fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FI1; // not affected by current sources
			i2f = fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI2; // affected by current sources
			i2Auxf = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX; // affected by current sources (same as I2)
			i2AuxfMinusI1f = i2Auxf - i1f;

			// Tolerances for current sources and internal filter resistors
#define LTCDEF_MIN_R50_CURR_SRC_DROP        (150e-6 * 45)
#define LTCDEF_MAX_R50_CURR_SRC_DROP        (350e-6 * 70)
#define LTCDEF_NOM_R50_CURR_SRC_DROP        (0.5*(LTCDEF_MAX_R50_CURR_SRC_DROP+LTCDEF_MIN_R50_CURR_SRC_DROP))
#define LTCDEF_NOM_R50_CURR_SRC_DROP_ABSTOL (0.5*(LTCDEF_MAX_R50_CURR_SRC_DROP-LTCDEF_MIN_R50_CURR_SRC_DROP))

			// check that the AUX measurement is
			// within the current ADCs full scale range
			// plus the maximum voltage drop across one
			// 50 ohm filter resistors (for the worst case
			// when only one current source is enabled.)
			// Note: If any pin is open, the conversion result will be >= 300mV
			// so for sure a open wire will be caught here
			AbsRelErrCheck(
				/*float diff   */i2Auxf,
				/*float nom    */0,
				/*float abs    */LTC2949_IADC_FULLSCALE + LTCDEF_MAX_R50_CURR_SRC_DROP,
				/*float rel    */1.0e-2, // not relevant
				/*byte * error*/&error,
				/*char id      */'H');

			// check AUX and I2 are identical
			AbsRelErrCheck(
				/*float diff   */i2Auxf - i2f,
				/*float nom    */i2f,
				/*float abs    */5e-3,   // 5mV
				/*float rel    */5.0e-2, // 5%
				/*byte * error*/&error,
				/*char id      */'J');

			switch (OpenWireCheckState)
			{// one step per LTCDEF_ATSR3_CYCLE_TIME=15ms: 15, 30, 45, 60, 75, 90
			case OPENWIRECHECKSTATE_ENTER: // previous setting LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PUMUXP | LTC2949_CURR_PDMUXN); //AUX MEASUREMENT NOM:0.013V,IMOW:0.010V,IPOW:+0.700V
				// this way we also check the current sources work!
				AbsRelErrCheck(
					/*float diff   */i2AuxfMinusI1f - LTCDEF_NOM_R50_CURR_SRC_DROP, // I2/AUX is affetced by the current sources, I1 only by the system current. Thus if we substract I1 from AUX we have the effect of the current sources only
					/*float nom    */LTCDEF_NOM_R50_CURR_SRC_DROP,
					/*float abs    */LTCDEF_NOM_R50_CURR_SRC_DROP_ABSTOL,
					/*float rel    */1e-3, // no relevant
					/*byte * error*/&error,
					/*char id      */'K');
				LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PDMUXP | LTC2949_CURR_PUMUXN); //AUX MEASUREMENT NOM:-0.013V,IMOW:-0.700V,IPOW:-0.010V
				LTC2949_GpioCurrConfigWrite();
				break;

			case OPENWIRECHECKSTATE_ENTER - 1: // previous setting LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PDMUXP | LTC2949_CURR_PUMUXN); //AUX MEASUREMENT NOM:-0.013V,IMOW:-0.700V,IPOW:-0.010V
				AbsRelErrCheck(
					/*float diff   */i2AuxfMinusI1f + LTCDEF_NOM_R50_CURR_SRC_DROP, // I2 is affetced by the current sources, I1 only by the system current. Thus if we substract I1 from I2 we have the effect of the current sources only
					/*float nom    */-LTCDEF_NOM_R50_CURR_SRC_DROP,
					/*float abs    */LTCDEF_NOM_R50_CURR_SRC_DROP_ABSTOL,
					/*float rel    */1e-3, // no relevant
					/*byte * error*/&error,
					/*char id      */'L');
				LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PUMUXP | LTC2949_CURR_PUMUXN); //AUX MEASUREMENT NOM:0.000V,IMOW:-0.700V,IPOW:+0.700V
				LTC2949_GpioCurrConfigWrite();
				break;

			case OPENWIRECHECKSTATE_ENTER - 2: // previous setting LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PUMUXP | LTC2949_CURR_PUMUXN); //AUX MEASUREMENT NOM:0.000V,IMOW:-0.700V,IPOW:+0.700V
				AbsRelErrCheck(
					/*float diff   */i2AuxfMinusI1f, // I2 is affetced by the current sources, I1 only by the system current. Thus if we substract I1 from I2 we have the effect of the current sources only
					/*float nom    */0,
					/*float abs    */LTCDEF_NOM_R50_CURR_SRC_DROP_ABSTOL,
					/*float rel    */1e-3, // no relevant
					/*byte * error*/&error,
					/*char id      */'M');
				LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PDMUXP | LTC2949_CURR_PDMUXN); //AUX MEASUREMENT NOM:0.000V,IMOW:-0.002V,IPOW:0.002V
				LTC2949_GpioCurrConfigWrite();
				break;

			case OPENWIRECHECKSTATE_ENTER - 3: // previous setting LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PDMUXP | LTC2949_CURR_PDMUXN); //AUX MEASUREMENT NOM:0.000V,IMOW:-0.002V,IPOW:0.002V
				AbsRelErrCheck(
					/*float diff   */i2AuxfMinusI1f, // I2 is affetced by the current sources, I1 only by the system current. Thus if we substract I1 from I2 we have the effect of the current sources only
					/*float nom    */0, // Note: pull-down current source not effective for signals close to ground!
					/*float abs    */LTCDEF_NOM_R50_CURR_SRC_DROP_ABSTOL,
					/*float rel    */1e-3, // no relevant
					/*byte * error*/&error,
					/*char id      */'N');
				LTC2949_Gpo5CurrConfigSet(0); //AUX MEASUREMENT NOM:V,IMOW:V,IPOW:V
				LTC2949_GpioCurrConfigWrite();
				break;

			case OPENWIRECHECKSTATE_ENTER - 4:
			{
				float absTol = 5e-2 * i2f;
				if (absTol < 0)
					absTol = -absTol;
				absTol = max(absTol, 5 * LTC2949_LSB_FAUX);
				// one more dummy state to make sure
				// the inputs are not effected anymore 
				// by post-affects of the current sources
				AbsRelErrCheck(
					/*float diff   */i2AuxfMinusI1f, // I2 is affetced by the current sources, I1 only by the system current. Thus if we substract I1 from I2 we have the effect of the current sources only
					/*float nom    */0, // Note: pull-down current source not effective for signals close to ground!
					/*float abs    */absTol,
					/*float rel    */1e-3, // no relevant
					/*byte * error*/&error,
					/*char id      */'O');
			}
			break;

			default:
			case OPENWIRECHECKSTATE_DONE: // IDLE state
				OpenWireCheckState = OPENWIRECHECKSTATE_DONE;
				break;
			}

#ifdef ATSR3_PRINT_RESULTS
			// print data for debug
			//Serial.println();
			Serial.println(F("I12ow:"));
			Serial.println(OpenWireCheckState);
			Serial.println(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FI1, 6);
			Serial.println(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FI1, 6);
			Serial.println(fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX, 4);
			//Serial.println();
#endif
			OpenWireCheckState--;
		}

		// check if we just filled a complete set
		// of 128 samples
		if ((iFastPos0 != iData[0].iFastPos) &&
			(iFastPos1 != iData[1].iFastPos))
		{
			if (bitMaskSetChk(opt, ATSR3_OPT_DO_ADCV))
			{
				switch (cyclesCounter)
				{
				case 1: // the last 100ms cycle before we interrupt for the FSSHTs
					// enable the current sources for the open wire check
					LTC2949_Gpo5CurrConfigSet(LTC2949_CURR_PUMUXP | LTC2949_CURR_PDMUXN); //AUX MEASUREMENT NOM:0.013V,IMOW:0.010V,IPOW:+0.700V
					LTC2949_GpioCurrConfigWrite();
					OpenWireCheckState = OPENWIRECHECKSTATE_ENTER;
					//Serial.println(F("I12PU"));
					// compare also every fast sample of I1 and I2 and not only the average of 128 samples of I1 and I2 done within ATSR3DoCheckI1vsI2
					error |= ATSR3DoCheckFastI1vsI2(iData);
					break;
				case 0:
					// this was the last full set of 128 samples
					// now we can interrupt the fast continuous measurement
					//  to do the fast single shot interruption
					// this is doing the fast single shot interruption (e.g. for open wire check...)
					error |= ATSR3DoFastSSHT(adcvCount, gcaux);
					//delay(2);
					//skipI1vsI2 = true;


					// initialize current FIFO buffer
					// We have to do this here as ATSR3PrcFast might already have read
					// some samples from the interrupted fast cont. cycle
					iData[0].iFastSampleCount = 0;
					iData[1].iFastSampleCount = 0;
					iData[0].iFastAvg[iData[0].iFastPos] = 0;
					iData[1].iFastAvg[iData[1].iFastPos] = 0;
					break;
				default:
					// compare also every fast sample of I1 and I2 and not only the average of 128 samples of I1 and I2 done within ATSR3DoCheckI1vsI2
					error |= ATSR3DoCheckFastI1vsI2(iData);
					break;
				}
			}
			else
				// compare also every fast sample of I1 and I2 and not only the average of 128 samples of I1 and I2 done within ATSR3DoCheckI1vsI2
				error |= ATSR3DoCheckFastI1vsI2(iData);
			if (cyclesCounter == 0)
				// reset counter
				cyclesCounter = slowCycCnt;
		}

		// check if we are ready to compare slow with fast channel
		if (
			iSlowChkPos == iData[0].iSlowPos ||
			iSlowChkPos == iData[1].iSlowPos ||
			iSlowChkPos == iData[0].iFastPos ||
			iSlowChkPos == iData[1].iFastPos
			)
			continue; // no new set to check
		Serial.print(cyclesCounter);
		PrintComma();
		// we have a new set to check
		error_ = ATSR3DoChecks(iData, 0, iSlowChkPos);
		error_ |= ATSR3DoChecks(iData, 1, iSlowChkPos);
		//if (!skipI1vsI2)
		if (cyclesCounter > 0 && cyclesCounter < slowCycCnt)
			error_ |= ATSR3DoCheckI1vsI2(iData, iSlowChkPos);
		//skipI1vsI2 = false;

		PrintOkErr(error_);
		error |= error_;

		// increment to next
		iSlowChkPos++;
		iSlowChkPos %= ATSR3_DATA_BUFFER_SIZE;

		if (cyclesCounter > 0)
			cyclesCounter--;
	}

	// Note: as we can calculate the missing time dT (time we were not accumulating) we could just add
	// dT * Ixavg
	// Ixavg: average current of some long time interval (e.g. 1h)
	// the average current is just I1avg = C1 / TB1
	// see below as an example how to compensate for the charge error 
	// induced by the fast single shot periods.

	LTC2949_OpctlIdle();
	startOfMeas = millis() - startOfMeas;
	// read time to calculate accumulation error
	// We simply measure time with Linduino and compare to TB1-4
	float dutCharge, dutTime, dutAverageCurrent;
	error |= LTC2949_READ(LTC2949_VAL_TB1, 4, data);
	dutTime = LTC_4BytesToUInt32(data) * LTC2949_LSB_TB1;
	Serial.print(dutTime * 1e3, 1);
	Serial.print(',');

	error |= LTC2949_READ(LTC2949_VAL_TB2, 4, data);
	Serial.print(LTC_4BytesToUInt32(data) * LTC2949_LSB_TB2 * 1e3, 1);
	Serial.print(',');

	error |= LTC2949_READ(LTC2949_VAL_TB3, 4, data);
	Serial.print(LTC_4BytesToUInt32(data) * LTC2949_LSB_TB3 * 1e3, 1);
	Serial.print(',');

	error |= LTC2949_READ(LTC2949_VAL_TB4, 4, data);
	Serial.print(LTC_4BytesToUInt32(data) * LTC2949_LSB_TB4 * 1e3, 1);
	Serial.print(',');

	error |= LTC2949_READ(LTC2949_VAL_C1, 6, data);
	dutCharge = LTC_6BytesToInt64(data) * LTC2949_LSB_C1;
	dutAverageCurrent = dutCharge / dutTime;
	Serial.print(dutCharge, 6);
	Serial.print(',');

	// apply correction to the measured charge (as an example we just do it for C1)
	Serial.print(dutCharge + (1.0e-3 * startOfMeas - dutTime) * dutAverageCurrent, 6);
	Serial.print(',');


	error |= LTC2949_READ(LTC2949_VAL_C2, 6, data);
	Serial.print(LTC_6BytesToInt64(data) * LTC2949_LSB_C2, 6);
	Serial.print(',');

	Serial.print(startOfMeas);
	Serial.print(',');
	PrintOkErr(error);

	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	WakeUpReportStatus(true);
	//PrintOkErr(error);
}

void AbsRelErrCheck(float diff, float nom, float abs, float rel, byte* error, char id)
{
	if (abs(diff) > abs)
	{
		// calculate the rel. error
		//diff /= nom;
		if (abs(diff / nom) > rel)
		{
			// mismatch between fifo average and slow current
			*error |= LTC2949_ERRCODE_OTHER;
			Serial.print(id);
			Serial.print(F(":Err:"));
			Serial.print(diff, 5);
			Serial.print('|');
		}
	}
}

void SafetyMeasure24(int cnt)
{
	// reset
	LTC2949_reset();
	byte error = WakeUpReportStatus(false, true);
	error |= WakeUpReportStatus(true, false);

	// write some random data
	uint64_t randomData = (~(uint64_t)millis() - micros());
	uint64_t tmp;
	LTC2949_WRITE(LTC2949_VAL_C1, 8, (byte*)&randomData);
	error |= LTC2949_EEPROMWrite();
	tmp = 0;
	LTC2949_WRITE(LTC2949_VAL_C1, 8, (byte*)&tmp);
	error |= LTC2949_READ(LTC2949_VAL_C1, 8, (byte*)&tmp);
	if (error || tmp != 0)
	{
		SerialPrintByteArrayHex((byte*)&tmp, 8, true, false);
		PrintComma();
		PrintOkErr(error);
		return;
	}
	//LTC2949_WRITE(LTC2949_REG_EEPROM, 0);
	error |= LTC2949_EEPROMIsReady();


	// start timing measurement
	unsigned long timingMeasure = millis();

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	int i = cnt;
	byte result;
	while (i-- > 0)
	{
		error |= LTC2949_EEPROMCommand(LTC2949_BM_EEPROM_RESTORE);
		LTC2949_WRITE(LTC2949_REG_EEPROM, 0); // clear result bit
		// check for random data
		error |= LTC2949_READ(LTC2949_VAL_C1, 8, (byte*)&tmp);
		if (tmp != randomData)
		{
			error |= LTC2949_ERRCODE_OTHER;
		}
		// clear random data
		tmp = 0;
		LTC2949_WRITE(LTC2949_VAL_C1, 8, (byte*)&tmp);
		error |= LTC2949_READ(LTC2949_VAL_C1, 8, (byte*)&tmp);
		if (tmp != 0)
		{
			error |= LTC2949_ERRCODE_OTHER;
		}
	}

	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	SerialPrintByteArrayHex(&result, 1, true);
	PrintComma();
	PrintOkErr(error);
}

// Process commands from serial interface
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
	if (equals(SerialInputString, F("WK")))
	{
		WakeUpReportStatus(false);
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("GPO")))
		SetPin();
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("INJ")))
	{
		int cnt;
		if (SerialInputString.length() > 4) // Note: INJ,100 not INJ100!
		{
			unsigned int iStart = 4;
			stringSplitter(SerialInputString, &iStart, &cnt);
		}
		else
			cnt = DEFAULT_SM_LOOP_COUNT;

		AllErrorInjectors(cnt);
		/* EXAMPLE:
		> INJ,200
		OK
		STAT:0x0F000000000000008000,OK
		CNT,100,41,OK
		OK
		STAT:0x0F000000000000008000,OK
		31.39,41,OK		*/
	}
	else if (startsWith(SerialInputString, F("ATSR3")))
	{
		// ATSR3,100,0x03,9,4
		int cnt;
		unsigned int iStart = 6;
		int opt = 0, slowCycCnt = 8, adcvCount = 6;
		if (SerialInputString.length() > iStart)
		{
			if (stringSplitter(SerialInputString, &iStart, &cnt) &&
				stringSplitter(SerialInputString, &iStart, &opt) &&
				stringSplitter(SerialInputString, &iStart, &slowCycCnt))
				stringSplitter(SerialInputString, &iStart, &adcvCount);
		}
		else
			cnt = DEFAULT_SM_LOOP_COUNT;

		ATSR3ImplementationGuide(cnt, opt, slowCycCnt, adcvCount);
		// ATSR3,100,0x01,10,5
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("SM")))
	{
		int sm, cnt;
		unsigned int iStart = 2;
		boolean additionalParameters = stringSplitter(SerialInputString, &iStart, &sm);

		if (additionalParameters)
			additionalParameters = stringSplitter(SerialInputString, &iStart, &cnt);
		else
			cnt = DEFAULT_SM_LOOP_COUNT;

		switch (sm)
		{
		case 1://///////////////////////////// SM1 ///////////////////////////////
		{
			int p = 4, n = 5, measureFlags = 0, slot4kRef = 1;
			if (additionalParameters &&
				stringSplitter(SerialInputString, &iStart, &p) &&
				stringSplitter(SerialInputString, &iStart, &n) &&
				stringSplitter(SerialInputString, &iStart, &measureFlags))
				stringSplitter(SerialInputString, &iStart, &slot4kRef);
			SafetyMeasure1(cnt, p, n, measureFlags, slot4kRef);
			break;
		}

		case 2://///////////////////////////// SM2 ///////////////////////////////
		{
			int mux1 = 7, mux2 = 5, muxrefa = 9, muxrefb = 3;
			float Re = 30e3, Rs = 3.6e3;
			if (additionalParameters &&
				stringSplitter(SerialInputString, &iStart, &mux1) &&
				stringSplitter(SerialInputString, &iStart, &mux2) &&
				stringSplitter(SerialInputString, &iStart, &muxrefa))
				stringSplitter(SerialInputString, &iStart, &muxrefb);
			SafetyMeasure2(cnt, mux1, mux2, muxrefa, muxrefb, Re, Rs);
			break;
		}

		case 3://///////////////////////////// SM3 ///////////////////////////////
		{
			int muxp = LTC2949_SLOTMUX_V1, muxn = LTC2949_SLOTMUX_GND, muxOffs = 100e-3 / LTC2949_LSB_FAUX;
			if (additionalParameters &&
				stringSplitter(SerialInputString, &iStart, &muxp) &&
				stringSplitter(SerialInputString, &iStart, &muxn))
				stringSplitter(SerialInputString, &iStart, &muxOffs);
			SafetyMeasure3(
				/*cnt,   */cnt,
				/*muxp,  */muxp,
				/*muxn,  */muxn,
				/*muxOffs*/muxOffs
			);
			break;
		}

		case 4://///////////////////////////// SM4_x /////////////////////////////
		{
			int muxvref = LTC2949_SLOTMUX_VBATP;
			if (additionalParameters)
				stringSplitter(SerialInputString, &iStart, &muxvref);
			SafetyMeasure4_x(cnt, muxvref);
			break;
		}

		case 9://///////////////////////////// SM9 ///////////////////////////////
		{
			SafetyMeasure9();
			break;
		}

		case 11:////////////////////////////// SM11 //////////////////////////////
		case 12:////////////////////////////// SM12 //////////////////////////////
		{
			int opt = 0;
			if (additionalParameters)
				stringSplitter(SerialInputString, &iStart, &opt);

			if (bitMaskSetChk(opt, SM11_SM12_OPT_DO_SM11_SM12))
			{
				SafetyMeasure11(cnt, opt);
				SafetyMeasure12(cnt, opt);
			}
			else if (sm == 11)
				SafetyMeasure11(cnt, opt);
			else
				SafetyMeasure12(cnt, opt);

			break;
		}

		case 24://///////////////////////////// SM24 //////////////////////////////
		{
			SafetyMeasure24(cnt);
			/*
			> SM24,50
			OK
			STAT:0x0F000000000000008000,OK
			OK
			STAT:0x00000000000000000000,OK
			OK
			STAT:0x00000000000000000000,OK
			1354.88,0x00,OK
			*/
			break;
		}

		default:
			break;
		}
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("RST")))
	{
		LTC2949_reset();
		//delay(1500); // wait for LTC2949 going to sleep again
		PrintOkErr(0);
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("RTMP")))
	{
		LTC2949_reset();
		byte error = WakeUpReportStatus(false, true);
		error |= WakeUpReportStatus(true, false);
		NtcCfgWrite(1); NtcCfgWrite(2);// configure NTC		
		LTC2949_SlotsCfg(1, 0, 2, 0);// SLOT1/2 measure temperature via NTC between V1/V2 and GND. 
		error |= LTC2949_GoCont(0, LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_NTC2);
		byte err;
		while (
			!LTC2949_ChkUpdate(&err) ||
			err != 0)
			;
		byte buffer[2];
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
		PrintOkErr(error);
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("DB")))
	{
		DebugEnable();
	}
	/////////////////////// UNKNOWN COMAND ///////////////////////////////
	else
	{
		Serial.println(F("CmdErr"));
		PrintUsage();
	}
	//
	//////////////////////////////////////////////////////////////////////
	///////////////////// END: PROCESS COMMANDS //////////////////////////
	//////////////////////////////////////////////////////////////////////

	// empty SerialInputString buffer
	SerialInputString = "";
}

// debug enable / disable (communication data RAW output)
void DebugEnable()
{
	Serial.println(LTC2949_DebugEnable = (toIntAuto(SerialInputString.substring(2)) != 0));
	PrintOkErr(0);
}

/*
// Template for safety measures
void SafetyMeasure_template(int cnt)
{
	// constraint: IDLE (reset will happen if not in IDLE)
	byte error = CheckInitialConditionIdle();

	// start timing measurement
	unsigned long timingMeasure = millis();

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	int i = cnt;
	byte result;
	while (i-- > 0)
	{
		;
	}

	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	SerialPrintByteArrayHex(&result, 1, true);
	PrintComma();
	PrintOkErr(error);
}
*/

// backup
//....RDCV to get last measurement result for open wire check:
//int16_t currentOpenwire[2]-;

// SM17 is included within ATSR3ImplementationGuide
#ifdef LTCDEF_SM17EXAMPLE

// covers also: SM4, SM5, SM, SM, SM, SM, SM, SM, SM
// SM17b is meant to replace SM17 in case were a similar test must be executed periodically during cont. meas. mode
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
#define SAFETYMEASURE17_ALT_ERRINJECT0 0x01U
#define SAFETYMEASURE17_ALT_ERRINJECT1 0x02U
#define SAFETYMEASURE17_ALT_ERRINJECT2 0x04U
#define SAFETYMEASURE17_ALT_ERRINJECT3 0x08U
#define SAFETYMEASURE17_ALT_ERRINJECT4 0x10U
#define SAFETYMEASURE17_ALT_ERRINJECT5 0x20U
#define SAFETYMEASURE17_ALT_ERRINJECT6 0x40U
#define SAFETYMEASURE17_ALT_ERRINJECT7 0x80U
void SafetyMeasure17b(byte errInject, byte cnt)
#else
void SafetyMeasure17b(byte cnt)
#endif
{
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	byte error = WakeUpReportStatus(true);

	/////////////////////////////////////
	////////////////// 1 ////////////////
	/////////////////////////////////////
	// in IDLE read trim values of current and AUX ADCs
	// This needs to be done only once, as the PROM values are constant.
	error |= CheckInitialConditionIdle(); // will also clear all STATUS , ALERT registers
	// read trim values from analog die
	byte data[3];

	byte cnt_ = cnt;
	unsigned long timingMeasure = millis();

	while (cnt_-- > 0)
	{
		// we do this cnt times and measure the timing, as it is quite long lasting process
		int8_t gcauxAD, gci1AD, gci2AD;
		LTC2949_EnterDebug();
		error |= LTC2949_SwBypGetTrimms(&gcauxAD, &gci1AD, &gci2AD);
		LTC2949_ExitDebug();
		// read trim values from memory
		// This needs to be done only once, as the PROM values are constant.
		// read aux trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCV, 1, data);
		// read i1 trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCI1, 1, data + 1);
		// read i2 trim value from memory
		error |= LTC2949_READ(LTC2949_REG_GCI2, 1, data + 2);
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
		// for debugging, some error injectors
		if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT6 | SAFETYMEASURE17_ALT_ERRINJECT7))
			data[0]++;
		else if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT6) && bitMaskClrChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT7))
			data[1]++;
		else if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT7) && bitMaskClrChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT6))
			data[2]++;
#endif
		// compare trim values
		if (gcauxAD != (int8_t)data[0])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('a');
			Serial.print(gcauxAD);
			PrintComma();
			Serial.println((int8_t)data[0]);
		}
		if (gci1AD != (int8_t)data[1])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('i');
			Serial.print(gci1AD);
			PrintComma();
			Serial.println((int8_t)data[1]);
		}
		if (gci2AD != (int8_t)data[2])
		{
			error |= LTC2949_ERRCODE_OTHER;
			Serial.print('I');
			Serial.print(gci2AD);
			PrintComma();
			Serial.println((int8_t)data[2]);
		}
	}
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	Serial.println(t);

	// convert trim value to float
	float gcaux = (int8_t)(data[0]) * LTC2949_SM_GC_LSB + LTC2949_SM_GC_OFFS;
	// sanity check: compare it with the F24 value also reported in memory:
	error |= LTC2949_READ(LTC2949_REG_F24GCVH, 2, data);
	// the 3 bytes are split, so we need two read commands
	error |= LTC2949_READ(LTC2949_REG_F24GCVL, 1, data + 2);
	float usergc;
	LTC2949_F24BytesToFloat(data, &usergc);
	if (abs(usergc - gcaux) > LTC2949_SM_GC_LSB * 0.5)
	{
		// error is more than half a LSB size which is for sure above possible rounding / trimming differences
		// report error
		error |= LTC2949_ERRCODE_OTHER;
		Serial.print('f');
	}
	Serial.print(gcaux, 6);
	PrintComma();
	Serial.print(usergc, 6);
	PrintComma();
	// calc user gain correction value leading to internal fixed point representation of 0xAAAA.... for the calculation of userGC * promGC
	// this is done to ensure all bits of the multiplier input will toggle
	usergc = LTC2949_SM_AA_FP_FAC / gcaux;
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT0))
		usergc = usergc * 0.98;
#endif
	Serial.print(usergc);
	PrintComma();
	LTC2949_FloatToF24Bytes(usergc, data);
	// write user gain correction
	LTC2949_WRITE(LTC2949_VAL_MUX1GC, 3, data);
	// calc user gain correction value leading to internal fixed point representation of 0x5555.... for the calculation of userGC * promGC
	// this is done to ensure all bits of the multiplier input will toggle
	usergc = LTC2949_SM_55_FP_FAC / gcaux;
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT1))
		usergc = usergc * 1.03;
#endif
	Serial.println(usergc);
	LTC2949_FloatToF24Bytes(usergc, data);
	// write user gain correction
	LTC2949_WRITE(LTC2949_VAL_MUX2GC, 3, data);
	// assign the above calculated user GCs to slot configurations
	data[0] = 0;
	data[1] = LTC2949_SLOTMUX_VREF2; // 2nd VREF
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT2))
		data[1] = LTC2949_SLOTMUX_GND; // 2nd VREF
#endif
	LTC2949_WRITE(LTC2949_REG_MUXNSET1, 2, data);
	data[1] = LTC2949_SLOTMUX_VREF2_250k; // 2nd VREF via 250k
#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT3))
		data[1] = LTC2949_SLOTMUX_GND; // 2nd VREF
#endif
	LTC2949_WRITE(LTC2949_REG_MUXNSET2, 2, data);

#ifdef LTCDEF_ERROR_INJECTORS_ENABLE
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT4))
	{
		LTC2949_FloatToF24Bytes(1.01, data);
		LTC2949_WRITE(LTC2949_VAL_RS1GC, 3, data);
	}
	if (bitMaskSetChk(errInject, SAFETYMEASURE17_ALT_ERRINJECT5))
	{
		LTC2949_FloatToF24Bytes(0.99, data);
		LTC2949_WRITE(LTC2949_VAL_RS2GC, 3, data);
	}
#endif

	// Adjust update for above user GC writes
	// commented, as it is done within LTC2949_GoCont
	//error |= LTC2949_OpctlAdjUpd();

	// now we do 4 fast single shot measurements and store results for checks
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	int16_t I1[4];
	int16_t I2[4];
	int16_t VREF2[4];
	int16_t VREF2Abserr[4];
	int16_t I12Abserr[4];

	// read & clear status
	error |= LTC2949_ReadChkStatusFaults(true, false);

	// enable measurement
	error |= LTC2949_GoCont(
		/*cfgFast:      */ LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2);
	// Note: all the steps above need to be done only once, as e.g. the PROM values, memory GC values.... are static

	// make sure HS byte is cleared at the beginning
	error |= LTC2949_RdFastData();

	/////////////////////////////////////
	////////////////// 2 ////////////////
	/////////////////////////////////////
	// this part needs to be done periodically, again we measure the timing by executing cnt times
	cnt_ = cnt;
	timingMeasure = millis();
	while (cnt_-- > 0)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			// configure MUX
			if (i == 0)
				LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2, 0);
			else if (i == 1)
				LTC2949_SlotFastCfg(0, LTC2949_SLOTMUX_VREF2);
			else if (i == 2)
				LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2_250k, 0);
			else
				LTC2949_SlotFastCfg(0, LTC2949_SLOTMUX_VREF2_250k);
			// Note: We measure the positive and negative value of the two reference inputs. 
			// Very likely this will cause all bits of the ADC data path (ADC raw value) to toggle, 
			// at least after the routine was executed several times. 
			// The reason is, the measurements will be different, at least
			// by one LSB. Thus if a bit is stuck and would cause some harmful failure we will
			// detect it very likely. see e.g.
			/*
			>>> X=123;n=16;
			>>> for x in [X,X+1,-X,-X+1]:
			print('{0:{fill}{width}b}'.format((x + 2**n) % 2**n, fill='0', width=n))

			Output:
			0000000001111011
			0000000001111100
			1111111110000101
			1111111110000110
			*/
			// trigger the measurement
			LTC2949_ADxx();
			// poll LTC2949 for conversion done
			error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
			// store results
			I1[i] = fastData2949[LTC2949_RDFASTDATA_I1];
			I2[i] = fastData2949[LTC2949_RDFASTDATA_I2];
			VREF2[i] = fastData2949[LTC2949_RDFASTDATA_AUX];
			// calc VREF2 error
			// Note: here we reverse the gain adjustment, that was done above by writing LTC2949_VAL_MUX1GC, LTC2949_VAL_MUX2GC
			if (i == 0)
				VREF2Abserr[i] = VREF2[i] * gcaux / LTC2949_SM_AA_FP_FAC - LTC2949_VREF2 / LTC2949_LSB_FAUX;
			else if (i == 1)
				VREF2Abserr[i] = -VREF2[i] * gcaux / LTC2949_SM_AA_FP_FAC - LTC2949_VREF2 / LTC2949_LSB_FAUX;
			else if (i == 2)
				VREF2Abserr[i] = VREF2[i] * gcaux / LTC2949_SM_55_FP_FAC - LTC2949_VREF2 / LTC2949_LSB_FAUX;
			else
				VREF2Abserr[i] = -VREF2[i] * gcaux / LTC2949_SM_55_FP_FAC - LTC2949_VREF2 / LTC2949_LSB_FAUX;

			// relative error of VREF2 measurement
			float errRel = VREF2Abserr[i] * LTC2949_LSB_FAUX / LTC2949_VREF2;
			if (abs(errRel) > LTC2949_VREF2_TOL)
			{
				// relative error violation
				error |= LTC2949_ERRCODE_OTHER;
				Serial.print('R');
				Serial.println(errRel);
			}

			// calculate error of I1 vs. I2
			I12Abserr[i] = I2[i] - I1[i];
			if (abs(I12Abserr[i]) > LTC2949_I1I2_FAST_ABS_TOL)
			{
				// abs. error check failed, so we calculate the relative error
				// max of relative to I1 or relative to I2
				errRel = max(abs(I12Abserr[i] / I1[i]), abs(I12Abserr[i] / I2[i]));

				if (errRel > LTC2949_I1I2_TOL)
				{
					// all error checks failed, report error
					error |= LTC2949_ERRCODE_OTHER;
					Serial.print('I');
					Serial.print(I12Abserr[i]);
					PrintComma();
					Serial.println(errRel);
				}
			}
		}
	}
	// report timing
	timingMeasure = millis() - timingMeasure;
	t = (float)timingMeasure / (float)cnt;
	Serial.println(t);
	// report results for debugging
	for (uint8_t i = 0; i < 4; i++)
	{
		Serial.print(I1[i] * LTC2949_LSB_FI1, 6);
		PrintComma();
		Serial.print(I2[i] * LTC2949_LSB_FI2, 6);
		PrintComma();
		Serial.print(I12Abserr[i] * LTC2949_LSB_FI2 * 1e6, 0);
		Serial.print('u');
		PrintComma();
		Serial.print(VREF2[i] * LTC2949_LSB_FAUX, 4);
		PrintComma();
		Serial.print(VREF2Abserr[i] * LTC2949_LSB_FAUX * 1e6, 0);
		Serial.println('u');
	}
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	PrintOkErr(error);

	/*
	Example output:
	> SM17
	10,0
	STAT:0x10000000000000000000,OK
	STAT:0x00000000000000000000,OK
	STAT:0x10000000000000000000,OK
	<<<<<<<< time it takes to read the trim values from analog die in ms >>>>>>>>
	84.60
	<<<<<<<< AUX trim, AUX trim from F24, LTC2949_VAL_MUX1GC, LTC2949_VAL_MUX2GC >>>>>>>>
	1.01,1.01,1.66,1.33
	<<<<<<<< time it takes for the periodic test (Note: as we do 4 fast measurement,
	the theoretical time is 4*1.5ms = 6ms. Here it takes 9.7ms due to Linduino processing time) >>>>>>>>
	9.70
	<<<<<<<< I1, I2, I12 abs. error, gain adjusted VREF2 measures, abs. error VREF2, 4 times >>>>>>>>
	-0.069794,-0.069817,-23u,3.9631,2626u
	-0.069779,-0.069825,-46u,-3.9631,2626u
	-0.069787,-0.069825,-38u,3.1703,1876u
	-0.069787,-0.069817,-30u,-3.1703,1876u
	STAT:0x10000000000000000000,OK
	OK
	*/
}

#ifdef LTC2949_SWBYPGETPROM
uint8_t debug_printPROM()
{
	uint32_t prom[8];
	byte error = LTC2949_SwBypGetPROM(prom, 8);
	for (uint8_t i = 0; i < 8; i++)
	{
		Serial.print(i);
		PrintComma();
		PrintZeroX();
		Serial.println(prom[i], HEX);
	}
	return error;
}
#endif

#endif //LTCDEF_SM17EXAMPLE


/*!*********************************************************************
\brief Memory Checks (SM7)

Steps:
- ensure correct initial condition IDLE, STATUS/ALERT cleared

Note:
-
**********************************************************************
void SafetyMeasure7(byte cnt)
{
	// constraint: IDLE
	byte error = CheckInitialConditionIdle(); // will also clear all STATUS , ALERT registers
	// NOTE: LTC2949_BM_EXTFAULTS_HWMBISTEXEC bit must be checked after Power-up before!

	if (error)
	{
		PrintOkErr(error);
		return;
	}

	// only the following code is typically
	// executed to perform this safety measure
	// we execute it cnt times and calculate the
	// average time afterwards
	byte i = cnt;
	byte errInjectors[] = { LTC2949_BM_ISO0_HWBISTINJ, LTC2949_BM_ISO0_CRCCFGINJ, LTC2949_BM_ISO0_MEMERRINJ };
	byte errReports[] = { LTC2949_BM_FAULTS_HWBIST, LTC2949_BM_FAULTS_CRCCFG, LTC2949_BM_FAULTS_CRCMEM };
	// start timing measurement
	unsigned long timingMeasure = millis();
	while (i-- > 0)
	{
		byte data;

		for (byte m = 0; m < 3; m++)
		{
			/////////////////////////////////////////////////////////////////
			// lock the memory
			if (error |= LTC2949_MemLock(true))
			{
				SerialPrintByteX2(4 | m); break;
			}
			// clear faults
			LTC2949_WRITE(LTC2949_REG_FAULTS, 0);
			// check faults is zero
			if (error |= LTC2949_READ(LTC2949_REG_FAULTS, 1, &data) || data)
			{
				SerialPrintByteX2(5 | m); break;
			}
			// inject error
			LTC2949_WRITE(LTC2949_REG_ISO0, errInjectors[m]);
			// unlocking the memory will enable the fault injector
			LTC2949_MemLock(false);
			// poll for the injected fault
			if (error |= LTC2949_PollReg(
				LTC2949_REG_FAULTS, errReports[m], 0xFFU, LTC2949_TIMING_CONT_CYCLE, true))
			{
				SerialPrintByteX2(6 | m); break;
			}
			// clear error injector
			LTC2949_WRITE(LTC2949_REG_ISO0, 0);
		}
		if (error)
			break;
	}

	// calc average execution time
	timingMeasure = millis() - timingMeasure;
	float t = (float)timingMeasure / (float)cnt;
	// check FAULTS / EXTFAULTS register
	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
	error |= WakeUpReportStatus(true);
	Serial.print(t);
	PrintComma();
	PrintOkErr(error);
}
*/

////// void SafetyMeasure7alt(byte cnt)
////// {
////// 	// constraint: IDLE
////// 	byte error = CheckInitialConditionIdle(); // will also clear all STATUS , ALERT registers
////// 	// NOTE: LTC2949_BM_EXTFAULTS_HWMBISTEXEC bit must be checked after Power-up before!
////// 
////// 	if (error)
////// 	{
////// 		PrintOkErr(error);
////// 		return;
////// 	}
////// 	/*
////// #define LTC2949_BM_ISO0_FERR 0x1
////// // inject parity error into ISER master to slave stream
////// #define LTC2949_BM_ISO0_ISERERRINJ 0x4
////// // inject error into ESER slave to master stream
////// #define LTC2949_BM_ISO0_ESERERRINJ 0x8
////// // inject error in the comparator of the AUX redundant channel
////// #define LTC2949_BM_ISO0_FCAERRINJ 0x10
////// // HW BIST; run the test with some error randomly injected
////// #define LTC2949_BM_ISO0_HWBISTINJ 0x20
////// // Inject error at check of CRC of XRAM versions of CFG Regs.
////// #define LTC2949_BM_ISO0_CRCCFGINJ 0x40
////// // Inject error at check of CRC of MEM
////// #define LTC2949_BM_ISO0_MEMERRINJ 0x80
////// 	*/
////// #define LTCDEF_ISO0_INJALL (LTC2949_BM_ISO0_FERR|LTC2949_BM_ISO0_FCAERRINJ|LTC2949_BM_ISO0_HWBISTINJ|LTC2949_BM_ISO0_CRCCFGINJ|LTC2949_BM_ISO0_MEMERRINJ)
////// 	LTC2949_ReadChkStatusFaults(true, true);
////// 	byte i = cnt;
////// 	// start timing measurement
////// 	unsigned long timingMeasure = millis();
////// 	unsigned long timing = 0;
////// 	while (i-- > 0)
////// 	{
////// 		byte data;
////// 		// inject error
////// 		LTC2949_WRITE(LTC2949_REG_ISO0, errInjectors[m]);
////// 
////// 		if (error |= LTC2949_READ(LTC2949_REG_FAULTS, 1, &data) || data)
////// 		{
////// 			SerialPrintByteX2(5 | m); break;
////// 		}
////// 		// unlocking the memory will enable the fault injector
////// 		LTC2949_MemLock(false);
////// 		// poll for the injected fault
////// 		if (error |= LTC2949_PollReg(
////// 			LTC2949_REG_FAULTS, errReports[m], 0xFFU, LTC2949_TIMING_CONT_CYCLE, true))
////// 		{
////// 			SerialPrintByteX2(6 | m); break;
////// 		}
////// 		// clear error injector
////// 		LTC2949_WRITE(LTC2949_REG_ISO0, 0);
////// 		if (error)
////// 			break;
////// 	}
////// 
////// 	// calc average execution time
////// 	timingMeasure = millis() - timingMeasure;
////// 	float t = (float)timingMeasure / (float)cnt;
////// 	// check FAULTS / EXTFAULTS register
////// 	// wakeup and report initial status (all status regs + FAULTS + EXTFAULTS)
////// 	error |= WakeUpReportStatus(true);
////// 	Serial.print(t);
////// 	PrintComma();
////// 	PrintOkErr(error);
////// }




/* Enable of current source, see timing measured with GUI below
time,FIFOAUX
87.4275999999998,0
87.4283819999998,0
87.4291639999998,0
87.4299459999998,0
87.4307279999998,0.134315551757813
87.4315099999998,0.269756652832031
87.4322919999998,0.384937866210938
87.4330739999998,0.490739501953125
87.4338559999998,0.589412658691406
87.4346379999998,0.677580688476563
87.4354199999998,0.731231872558594
87.4362019999998,0.732357421875
87.4369839999998,0.733107788085938
87.4377659999998,0.733482971191406
87.4385479999998,0.733858154296875
87.4393299999998,0.733858154296875
87.4401119999998,0.733858154296875
87.4408939999998,0.733858154296875
87.4416759999998,0.733858154296875
87.4424579999998,0.733858154296875
87.4432399999998,0.733858154296875
87.4440219999998,0.733858154296875
87.4448039999998,0.733858154296875
87.4455859999998,0.733858154296875
87.4463679999998,0.733858154296875
87.4471499999998,0.733858154296875
87.4479319999998,0.733858154296875
87.4487139999998,0.733858154296875
87.4494959999998,0.733858154296875
87.4502779999998,0.733858154296875
87.4510599999998,0.734233337402344
87.4518419999998,0.734233337402344
// disable:
time,FIFOAUX
110.201004,0.734233337402344
110.201786,0.734233337402344
110.202568,0.734233337402344
110.20335,0.734233337402344
110.204132,0.734233337402344
110.204914,0.734233337402344
110.205696,0.734233337402344
110.206478,0.734233337402344
110.20726,0.734233337402344
110.208042,0.734233337402344
110.208824,0.734233337402344
110.209606,0.734233337402344
110.210388,0.734233337402344
110.21117,0.734233337402344
110.211952,0.734233337402344
110.212734,0.734233337402344
110.213516,0.734233337402344
110.214298,0.734233337402344
110.21508,0.734233337402344
110.215862,0.734233337402344
110.216644,0.734233337402344
110.217426,0.734233337402344
110.218208,0.734233337402344
110.21899,0.734233337402344
110.219772,0.734233337402344
110.220554,0.734608520507813
110.221336,0.734233337402344
110.222118,0.734233337402344
110.2229,0.734233337402344
110.223682,0.734233337402344
110.224464,0.734233337402344
110.225246,0.734233337402344
110.226028,0.734233337402344
110.22681,0.734233337402344
110.227592,0.734233337402344
110.228374,0.734233337402344
110.229156,0.734233337402344
110.229938,0.734233337402344
110.23072,0.734233337402344
110.231502,0.734233337402344
110.232284,0.734608520507813
110.233066,0.734233337402344
110.233848,0.734233337402344
110.23463,0.734233337402344
110.235412,0.617551391601562
110.236194,0.610047729492188
110.236976,0.603669616699219
110.237758,0.598041870117188
110.23854,0.592414123535156
110.239322,0.586786376953125
110.240104,0.576656433105469
110.240886,0.571779052734375
110.241668,0.56727685546875
110.24245,0.562774658203125
110.243232,0.5582724609375
110.244014,0.553770263671875
110.244796,0.549643249511719
110.245578,0.544765869140625
110.24636,0.536511840820313
110.247142,0.532384826660156
110.247924,0.453971557617187
110.248706,0.447968627929688
110.249488,0.444591979980469
110.25027,0.441590515136719
110.251052,0.438589050292969
110.251834,0.435587585449219
110.252616,0.432586120605469
110.253398,0.426583190917969
110.25418,0.423956909179688
110.254962,0.420955444335938
110.255744,0.417578796386719
110.256526,0.414952514648438
110.257308,0.411951049804688
110.25809,0.409324768066406
110.258872,0.406323303222656
110.259654,0.403697021484375
110.260436,0.345543640136719
110.261218,0.343292541503906
110.262,0.341041442871094
110.262782,0.338790344238281
110.263564,0.336539245605469
110.264346,0.334288146972656
110.265128,0.332412231445313
110.26591,0.327910034179688
110.266692,0.325658935546875
110.267474,0.323407836914063
110.268256,0.321531921386719
110.269038,0.319280822753906
110.26982,0.317404907226563
110.270602,0.31515380859375
110.271384,0.313277893066406
110.272166,0.311026794433594
110.272948,0.267505554199219
110.27373,0.263753723144531
110.274512,0.262252990722656
110.275294,0.260377075195313
110.276076,0.258876342773438
110.276858,0.257000427246094
110.27764,0.255499694824219
110.278422,0.253623779296875
110.279204,0.252123046875
110.279986,0.250622314453125
110.280768,0.248746398925781
110.28155,0.245744934082031
110.282332,0.244244201660156
110.283114,0.242368286132813
110.283896,0.240867553710938
110.284678,0.239366821289063
110.28546,0.205975524902344
110.286242,0.203349243164063
110.287024,0.201848510742188
110.287806,0.200722961425781
110.288588,0.199222229003906
110.28937,0.1980966796875
110.290152,0.196595947265625
110.290934,0.195470397949219
110.291716,0.193969665527344
110.292498,0.192844116210938
110.29328,0.191718566894531
110.294062,0.190217834472656
110.294844,0.187966735839844
110.295626,0.186841186523438
110.296408,0.185340454101563
110.29719,0.184214904785156
110.297972,0.157952087402344
110.298754,0.156826538085937
110.299536,0.156076171875
110.300318,0.154950622558594
110.3011,0.153825073242188
110.301882,0.152699523925781
110.302664,0.150823608398438
110.303446,0.1500732421875
110.304228,0.148947692871094
110.30501,0.147822143554688
110.305792,0.14707177734375
110.306574,0.145946228027344
110.307356,0.1440703125
110.308138,0.142944763183594
110.30892,0.142194396972656
110.309702,0.14106884765625
110.310484,0.123435241699219
110.311266,0.122309692382813
110.312048,0.121559326171875
110.31283,0.120808959960938
110.313612,0.12005859375
110.314394,0.119308227539063
110.315176,0.117432312011719
110.315958,0.116681945800781
110.31674,0.115931579589844
110.317522,0.115181213378906
110.318304,0.114430847167969
110.319086,0.113680480957031
110.319868,0.112179748535156
110.32065,0.111429382324219
110.321432,0.110679016113281
110.322214,0.109928649902344
110.322996,0.0934205932617188
110.323778,0.0919198608398438
110.32456,0.091544677734375
110.325342,0.0907943115234375
110.326124,0.0904191284179688
110.326906,0.0896687622070313
110.327688,0.0889183959960938
110.32847,0.088543212890625
110.329252,0.0877928466796875
110.330034,0.0874176635742187
110.330816,0.0866672973632813
110.331598,0.0862921142578125
110.33238,0.0851665649414063
110.333162,0.0844161987304688
110.333944,0.084041015625
110.334726,0.0832906494140625
110.335508,0.0686585083007813
110.33629,0.067532958984375
110.337072,0.0671577758789063
110.337854,0.0667825927734375
110.338636,0.0664074096679688
110.339418,0.0660322265625
110.3402,0.0652818603515625
110.340982,0.0649066772460938
110.341764,0.064531494140625
110.342546,0.0641563110351562
110.343328,0.0637811279296875
110.34411,0.0634059448242188
110.344892,0.0626555786132812
110.345674,0.0622803955078125
110.346456,0.0619052124023438
110.347238,0.061530029296875
110.34802,0.0529008178710938
110.348802,0.052525634765625
110.349584,0.0521504516601563
110.350366,0.0521504516601563
110.351148,0.0517752685546875
110.35193,0.0514000854492188
110.352712,0.0506497192382813
110.353494,0.0502745361328125
110.354276,0.0498993530273438
110.355058,0.0498993530273438
110.35584,0.049524169921875
110.356622,0.0491489868164063
110.357404,0.0483986206054688
110.358186,0.0480234375
110.358968,0.0480234375
110.35975,0.0476482543945312
110.360532,0.0412701416015625
110.361314,0.0408949584960938
110.362096,0.0408949584960938
110.362878,0.040519775390625
110.36366,0.0401445922851563
110.364442,0.0401445922851563
110.365224,0.0397694091796875
110.366006,0.0393942260742188
110.366788,0.03901904296875
110.36757,0.0386438598632813
110.368352,0.0386438598632813
110.369134,0.0382686767578125
110.369916,0.0378934936523437
110.370698,0.037518310546875
110.37148,0.037518310546875
110.372262,0.0371431274414063
110.373044,0.03301611328125
110.373826,0.0326409301757812
110.374608,0.0318905639648438
110.37539,0.0318905639648438
110.376172,0.031515380859375
110.376954,0.031515380859375
110.377736,0.0311401977539063
110.378518,0.0311401977539063
110.3793,0.0307650146484375
110.380082,0.0307650146484375
110.380864,0.0303898315429688
110.381646,0.0300146484375
110.382428,0.0300146484375
110.38321,0.0296394653320313
110.383992,0.0296394653320313
110.384774,0.0292642822265625
110.385556,0.0247620849609375
110.386338,0.0247620849609375
110.38712,0.0243869018554688
110.387902,0.0243869018554688
110.388684,0.0243869018554688
110.389466,0.02401171875
110.390248,0.02401171875
110.39103,0.0236365356445313
110.391812,0.0236365356445313
110.392594,0.0232613525390625
110.393376,0.0232613525390625
110.394158,0.0232613525390625
110.39494,0.0228861694335938
110.395722,0.0228861694335938
110.396504,0.0228861694335938
110.397286,0.022510986328125
110.398068,0.0191343383789063
110.39885,0.0191343383789063
110.399632,0.0187591552734375
110.400414,0.0187591552734375
110.401196,0.0187591552734375
110.401978,0.0187591552734375
110.40276,0.0183839721679688
110.403542,0.0183839721679688
110.404324,0.0183839721679688
110.405106,0.0183839721679688
110.405888,0.0180087890625
110.40667,0.0180087890625
110.407452,0.0176336059570313
110.408234,0.0176336059570313
110.409016,0.0176336059570313
110.409798,0.0176336059570313
110.41058,0.0153825073242188
110.411362,0.0153825073242188
110.412144,0.0153825073242188
110.412926,0.0153825073242188
110.413708,0.01500732421875
110.41449,0.01500732421875
110.415272,0.01500732421875
110.416054,0.0146321411132813
110.416836,0.0146321411132813
110.417618,0.0146321411132813
110.4184,0.0146321411132813
110.419182,0.0146321411132813
110.419964,0.0142569580078125
110.420746,0.0142569580078125
110.421528,0.0142569580078125
110.42231,0.0142569580078125
110.423092,0.0123810424804688
110.423874,0.012005859375
110.424656,0.012005859375
110.425438,0.012005859375
110.42622,0.012005859375
110.427002,0.012005859375
110.427784,0.012005859375
110.428566,0.0116306762695313
110.429348,0.0116306762695313
110.43013,0.0116306762695313
110.430912,0.0116306762695313
110.431694,0.0116306762695313
110.432476,0.0116306762695313
110.433258,0.0112554931640625
110.43404,0.0112554931640625
110.434822,0.0112554931640625
110.435604,0.0101299438476563
110.436386,0.0101299438476563
110.437168,0.0101299438476563
110.43795,0.0101299438476563
110.438732,0.0097547607421875
110.439514,0.0097547607421875
110.440296,0.0097547607421875
110.441078,0.0097547607421875
110.44186,0.0097547607421875
110.442642,0.0097547607421875
110.443424,0.0097547607421875
110.444206,0.0097547607421875
110.444988,0.00937957763671875
110.44577,0.00937957763671875
110.446552,0.00937957763671875
110.447334,0.00937957763671875
110.448116,0.00862921142578125
110.448898,0.00862921142578125
110.44968,0.00862921142578125
110.450462,0.00862921142578125
110.451244,0.0082540283203125
110.452026,0.0082540283203125
110.452808,0.0082540283203125
110.45359,0.0082540283203125
110.454372,0.0082540283203125
110.455154,0.0082540283203125
110.455936,0.0082540283203125
110.456718,0.0082540283203125
110.4575,0.0082540283203125
110.458282,0.0082540283203125
110.459064,0.00787884521484375
110.459846,0.00787884521484375
110.460628,0.00712847900390625
110.46141,0.00712847900390625
110.462192,0.00712847900390625
110.462974,0.00712847900390625
110.463756,0.00712847900390625
110.464538,0.0067532958984375
110.46532,0.0067532958984375
110.466102,0.0067532958984375
110.466884,0.0067532958984375
110.467666,0.0067532958984375
110.468448,0.0067532958984375
110.46923,0.0067532958984375
110.470012,0.0067532958984375
110.470794,0.0067532958984375
110.471576,0.0067532958984375
110.472358,0.00637811279296875
110.47314,0.00637811279296875
110.473922,0.00637811279296875
110.474704,0.00637811279296875
110.475486,0.00637811279296875
110.476268,0.00637811279296875
110.47705,0.0060029296875
110.477832,0.0060029296875
110.478614,0.0060029296875
110.479396,0.0060029296875
110.480178,0.0060029296875
110.48096,0.00562774658203125
110.481742,0.00562774658203125
110.482524,0.00562774658203125
110.483306,0.00562774658203125
110.484088,0.00562774658203125
110.48487,0.00562774658203125
110.485652,0.00562774658203125
110.486434,0.00562774658203125
110.487216,0.00562774658203125
110.487998,0.00562774658203125
110.48878,0.00562774658203125
110.489562,0.0052525634765625
110.490344,0.0052525634765625
110.491126,0.0052525634765625
110.491908,0.0052525634765625
110.49269,0.0052525634765625
110.493472,0.0052525634765625
110.494254,0.0052525634765625
110.495036,0.00487738037109375
110.495818,0.00487738037109375
110.4966,0.00487738037109375
110.497382,0.00487738037109375
110.498164,0.00487738037109375
110.498946,0.00487738037109375
110.499728,0.00487738037109375
110.50051,0.00487738037109375
110.501292,0.00487738037109375
110.502074,0.004502197265625
110.502856,0.004502197265625
110.503638,0.004502197265625
110.50442,0.004502197265625
110.505202,0.004502197265625
110.505984,0.004502197265625
110.506766,0.004502197265625
110.507548,0.004502197265625
110.50833,0.004502197265625
110.509112,0.004502197265625
110.509894,0.004502197265625
110.510676,0.004502197265625
110.511458,0.004502197265625
110.51224,0.004502197265625
110.513022,0.004502197265625
110.513804,0.004502197265625
110.514586,0.00412701416015625
110.515368,0.00412701416015625
110.51615,0.00412701416015625
110.516932,0.00412701416015625
110.517714,0.00412701416015625
110.518496,0.00412701416015625
110.519278,0.00412701416015625
110.52006,0.00412701416015625
110.520842,0.00412701416015625
110.521624,0.00412701416015625
110.522406,0.00412701416015625
110.523188,0.00412701416015625
110.52397,0.0037518310546875
110.524752,0.0037518310546875
110.525534,0.0037518310546875
110.526316,0.0037518310546875
110.527098,0.0037518310546875
110.52788,0.0037518310546875
110.528662,0.0037518310546875
110.529444,0.0037518310546875
110.530226,0.0037518310546875
110.531008,0.0037518310546875
110.53179,0.0037518310546875
110.532572,0.0037518310546875
110.533354,0.0037518310546875
110.534136,0.0037518310546875
110.534918,0.0037518310546875
110.5357,0.00337664794921875
110.536482,0.00337664794921875
110.537264,0.00337664794921875
110.538046,0.0037518310546875
110.538828,0.00337664794921875
110.53961,0.00337664794921875
110.540392,0.00337664794921875
110.541174,0.00337664794921875
110.541956,0.00337664794921875
110.542738,0.00337664794921875
110.54352,0.00337664794921875
110.544302,0.00337664794921875
110.545084,0.00337664794921875
110.545866,0.00337664794921875
110.546648,0.00337664794921875
110.54743,0.00337664794921875
110.548212,0.00337664794921875
110.548994,0.00337664794921875
110.549776,0.00337664794921875
110.550558,0.00337664794921875
110.55134,0.00337664794921875
110.552122,0.00300146484375
110.552904,0.00300146484375
110.553686,0.00300146484375
110.554468,0.00300146484375
110.55525,0.00300146484375
110.556032,0.00300146484375
110.556814,0.00300146484375
110.557596,0.00300146484375
110.558378,0.00300146484375
110.55916,0.00300146484375
110.559942,0.00300146484375
110.560724,0.00300146484375
110.561506,0.00300146484375
110.562288,0.00300146484375
110.56307,0.00300146484375
110.563852,0.00300146484375
110.564634,0.00300146484375
110.565416,0.00300146484375
110.566198,0.00300146484375
110.56698,0.00300146484375
110.567762,0.00300146484375
110.568544,0.00300146484375
110.569326,0.00262628173828125
110.570108,0.00300146484375
110.57089,0.00262628173828125
110.571672,0.00262628173828125
110.572454,0.00262628173828125
110.573236,0.00262628173828125
110.574018,0.00262628173828125
110.5748,0.00262628173828125
110.575582,0.00262628173828125
110.576364,0.00262628173828125
110.577146,0.00262628173828125
110.577928,0.00262628173828125
110.57871,0.00262628173828125
110.579492,0.00262628173828125
110.580274,0.00262628173828125
110.581056,0.00262628173828125
110.581838,0.00262628173828125
110.58262,0.00262628173828125
110.583402,0.00262628173828125
110.584184,0.00262628173828125
110.584966,0.00262628173828125
110.585748,0.00262628173828125
110.58653,0.00262628173828125
110.587312,0.00262628173828125
110.588094,0.00262628173828125
110.588876,0.00262628173828125
110.589658,0.00262628173828125
110.59044,0.00262628173828125
110.591222,0.00262628173828125
110.592004,0.00262628173828125
110.592786,0.00262628173828125
110.593568,0.00262628173828125
110.59435,0.00262628173828125
110.595132,0.00262628173828125
110.595914,0.00262628173828125
110.596696,0.00262628173828125
110.597478,0.00262628173828125
110.59826,0.00262628173828125
110.599042,0.0022510986328125
110.599824,0.00262628173828125
110.600606,0.0022510986328125
110.601388,0.00262628173828125
110.60217,0.0022510986328125
110.602952,0.0022510986328125
110.603734,0.0022510986328125
110.604516,0.0022510986328125
110.605298,0.0022510986328125
110.60608,0.0022510986328125
110.606862,0.0022510986328125
110.607644,0.0022510986328125
110.608426,0.0022510986328125
110.609208,0.0022510986328125
110.60999,0.0022510986328125
110.610772,0.0022510986328125
110.611554,0.0022510986328125
110.612336,0.0022510986328125
110.613118,0.0022510986328125
110.6139,0.0022510986328125
110.614682,0.0022510986328125
110.615464,0.0022510986328125
110.616246,0.0022510986328125
110.617028,0.0022510986328125
110.61781,0.0022510986328125
110.618592,0.0022510986328125
110.619374,0.0022510986328125
110.620156,0.0022510986328125
110.620938,0.0022510986328125
110.62172,0.0022510986328125
110.622502,0.0022510986328125
110.623284,0.0022510986328125
110.624066,0.0022510986328125
110.624848,0.0022510986328125
110.62563,0.0022510986328125
110.626412,0.0022510986328125
110.627194,0.0022510986328125
110.627976,0.0022510986328125
110.628758,0.0022510986328125
110.62954,0.0022510986328125
110.630322,0.0022510986328125
110.631104,0.0022510986328125
110.631886,0.0022510986328125
110.632668,0.0022510986328125
110.63345,0.0022510986328125
110.634232,0.0022510986328125
110.635014,0.0022510986328125
110.635796,0.0022510986328125
110.636578,0.0022510986328125
110.63736,0.0022510986328125
110.638142,0.0022510986328125
110.638924,0.0022510986328125
110.639706,0.00187591552734375
110.640488,0.00187591552734375
110.64127,0.00187591552734375
110.642052,0.00187591552734375
110.642834,0.00187591552734375
110.643616,0.00187591552734375
110.644398,0.00187591552734375
110.64518,0.00187591552734375
110.645962,0.00187591552734375
110.646744,0.00187591552734375
110.647526,0.00187591552734375
110.648308,0.00187591552734375
110.64909,0.00187591552734375
110.649872,0.00187591552734375
110.650654,0.00187591552734375
110.651436,0.00187591552734375
110.652218,0.00187591552734375
110.653,0.00187591552734375
110.653782,0.00187591552734375
110.654564,0.00187591552734375
110.655346,0.00187591552734375
110.656128,0.00187591552734375
110.65691,0.00187591552734375
110.657692,0.00187591552734375
110.658474,0.00187591552734375
110.659256,0.00187591552734375
110.660038,0.00187591552734375
110.66082,0.00187591552734375
110.661602,0.00187591552734375
110.662384,0.00187591552734375
110.663166,0.00187591552734375
110.663948,0.00187591552734375
*/
/*
[master 5f66061] ++++++++++++++++++++++++++++++++++++++++++ LAST VERSION BEFORE DC2732A_SM.ino rework! ++++++++++++++++++++++++++++++++++++++++++
push: 8e86e0e..5f66061  master -> master
*/
