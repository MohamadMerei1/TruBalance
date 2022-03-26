#include <Arduino.h>
/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_BASIC.ino
* Linduino Sketch for LTC2949 Demo Board - Isolated Battery Meter for EV / HEV
*
* Basic example for
*  fast continuous CH2 + P2ASV --> continuous fast measurements of I2, BAT
*  NTC1: SLOT1 measures temp via NTC (NTC connected between V1 and GND)
*  continuous reading and reporting of delta-TB1, I1, P1, BAT, NTC1 / IC temp. from slow channel.
*  Reporting last fast I2, BAT conversion result
*  Reporting count and average of samples from FIFOs I2, BAT
*  supports SPI / isoSPI

*  Example output:
   tDut,I1,P1,BAT,Tntc,TIC,fI2,fBAT,fifoCnt,fifoI2Avg,fifoBATAvg,OK/ERR
   STAT:0x10000000000000000000,OK
   STAT:0x10000000000000000000,OK
   99,0.049831,0.084803,1.702,29.2,33.4,-0.10011,1.701,130,-0.100125,1.701,OK
   100,0.049831,0.084809,1.702,29.4,33.4,-0.10016,1.701,128,-0.100126,1.701,OK
   99,0.049831,0.084803,1.702,29.4,33.4,-0.10016,1.701,127,-0.100125,1.701,OK
*
   tDut,I1,P1,BAT,Tntc,TIC,fI2,fBAT,fifoCnt,fifoI2Avg,fifoBATAvg,OK/ERR
   STAT:0x10000000000000000000,OK
   STAT:0x10000000000000000000,OK
   100,0.000000,0.000000,-0.051,25.4,25.8,0.00000,-0.051,131,-0.000000,-0.051,OK
   99,-0.000001,0.000000,-0.051,25.4,25.8,0.00000,-0.051,126,-0.000001,-0.051,OK
   99,-0.000001,0.000000,-0.051,25.4,25.8,0.00000,-0.051,128,-0.000000,-0.051,OK
   100,-0.000001,0.000000,-0.051,25.4,25.8,0.00000,-0.051,126,-0.000000,-0.051,OK



*  Notes:
*   tDut: delta TB1 in ms (typically 100, as values are reported for every slow conversion cycle which lasts 100 ms)
*   I1: slow / high precision I1 current measurement in V (x 1/Rsns for current in A)
*   P1: slow / high precision P1 power measurement in V*V (x 1/Rsns for power in W)
*   BAT: slow AUX channel BAT measurement in V
*   Tntc: slow AUX channel SLOT1 measurement. SLOT1 is configured for temperature measurement via NTC in degree Celsius
*   TIC:  slow AUX channel IC temperature measurement in degree Celsius
*   fI2: latest CH2 current fast measurement in V (x 1/Rsns for current in A)
*   fBAT: latest CH2 voltage (P2 as voltage) fast BAT measurement in V
*   fifoCnt: number of samples in FIFO before read
*   fifoI2Avg: average of CH2 current samples read from FIFO
*   fifoBATAvg: average of CH2 voltage (P2 as voltage) samples read from FIFO
*   OK/ERR: Tag indicating all measurements / communication was successful (OK) or some failure occurred (ERR)
*
* created: Patrick Wilhelm (PW)
*/


#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>

// DC2792A has two isoSPI ports: AUX and MAIN
#define LTCDEF_CS_AUX  9
#define LTCDEF_CS_MAIN 10
// in case of DC2792A select AUX or MAIN port
// default for all other isoSPI boards should be MAIN
#define LTCDEF_CS_SELECT LTCDEF_CS_MAIN


// enable measurement of Vx-Vy via SLOT2
#define ADD_SLOT2_MEASUREMENT
  // disable measurement of Vx-Vy via SLOT2
#undef ADD_SLOT2_MEASUREMENT
#define SLOT2_MEAS_POS 3
#define SLOT2_MEAS_NEG 4

// serial baudrate
#define LTCDEF_BAUDRATE 250000
// define this for wired NTC glued
#define LTCDEF_NTC_WIRED
#undef LTCDEF_NTC_WIRED

#ifdef LTCDEF_NTC_WIRED
// NTC: Murata, NTHCG143/Murata (leaded)
#define NTC_STH_A  8.39126e-4
#define NTC_STH_B  2.08985e-4
#define NTC_STH_C  7.13241e-8
#else
// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#endif // LTCDEF_NTC_WIRED

#define NTC_RREF   100e3

// fast channel configuration
#define LTCDEF_FACTRL_CONFIG (LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACONV)
// ADC configuration
#define LTCDEF_ADCCFG_CONFIG (LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_P2ASV)

// significant digits
#define LTCDEF_DIGITS_BATFAST     3
#define LTCDEF_DIGITS_I2FAST      5
#define LTCDEF_DIGITS_BATFIFO_AVG 3
#define LTCDEF_DIGITS_I2FIFO_AVG  6
#define LTCDEF_DIGITS_I1SLOW      6
#define LTCDEF_DIGITS_P1SLOW      6
#define LTCDEF_DIGITS_BATSLOW     3
#define LTCDEF_DIGITS_TEMPSLOW    1

/*!*********************************************************************
\brief prints the CSV header of the measurement output
***********************************************************************/
void PrintCSVHeader()
{
#ifdef ADD_SLOT2_MEASUREMENT
	Serial.println(F("tDut,I1,P1,BAT,Tntc,S2,TIC,fI2,fBAT,fifoCnt,fifoI2Avg,fifoBATAvg,OK/ERR"));
#else
	Serial.println(F("tDut,I1,P1,BAT,Tntc,TIC,fI2,fBAT,fifoCnt,fifoI2Avg,fifoBATAvg,OK/ERR"));
#endif
}


void setup()
{
	unsigned long startOfTheDay = millis() + LTC2949_TIMING_BOOTUP;
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	// configure SPI, also done in LTC2949.cpp:
	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
	LinduinoSelectSPI(LTCDEF_CS_SELECT);


	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB port only
	}
	PrintCSVHeader();
	while (!LTC_TIMEOUT_CHECK(millis(), startOfTheDay))
	{
		;
	}
	byte error = WakeUpReportStatus();
	error |= Cont(false); // Cont mode will be enabled later
	delay(LTC2949_TIMING_CONT_CYCLE);
}

void loop()
{
	if (ChkDeviceReady())
		return;

	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t deltaTint = LTC2949_GetLastTBxInt();

	// LTC2949_ChkUpdate checks for changed in TBx (default is TB4)
	byte  error;
	if (!LTC2949_ChkUpdate(&error))
		return;

	if (error)
	{
		PrintOkErr(error);
		delay(LTC2949_TIMING_CONT_CYCLE);
		return;
	}


	// new slow channel, high precission results available
	// as an example we just report delta TBx and slow currents
	// calc difference between last TBx and current TBx, report milliseconds
	Serial.print((unsigned int)((LTC2949_GetLastTBxInt() - deltaTint) * (1.0e3 * LTC2949_LSB_TB1)));
	PrintComma();

	// read slow current I1
	byte buffer[3];
	error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
	Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1, LTCDEF_DIGITS_I1SLOW);
	PrintComma();

	// read slow current P1
	error |= LTC2949_READ(LTC2949_VAL_P1, 3, buffer);
	Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1, LTCDEF_DIGITS_P1SLOW);
	PrintComma();

	// read slow current BAT
	error |= LTC2949_READ(LTC2949_VAL_BAT, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LTCDEF_DIGITS_BATSLOW);
	PrintComma();

	// read slow temperature via SLOT1
	error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(true), 1);
	PrintComma();

#ifdef ADD_SLOT2_MEASUREMENT
	// read slow temperature via SLOT2
	error |= LTC2949_READ(LTC2949_VAL_SLOT2, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_SlotLsb(false), 4);
	PrintComma();
#endif


	// read slow internal temperature
	error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, buffer);
	Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
	PrintComma();

	// read last fast conversion results via RDCV command
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

	// Note: as LTC2949 is running fast continuous, so we do not need to send ADCV command
	// still, if ADCV is send it doesn't hurt, as it is ignored.
	// to allow cell voltage measurement in case LTC68xx is connected to the
	// same isoSPI bus we can send the ADCV command.
	//if (false) // uncomment to disable sending of adcv commamd
	error |= LTC2949_ADxx();

	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(fastData2949);

	// print fast I2
	Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2, LTCDEF_DIGITS_I2FAST);
	PrintComma();
	// print fast BAT
	Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST);

	// additionally to the last sample reported above, we also read 
	// samples from the FIFOs I2 and BAT
	ReportFifos();

	PrintComma();

	PrintOkErr(error);

}

/*!*********************************************************************
\brief The simplest processing we can do on the fast FIFO samples:
Sum of all samples
***********************************************************************/
void ProcessFastData(int16_t * data, byte  len, int32_t* dSum)
{
	*dSum = 0;
	for (byte i = 0; i < len; i++)
		*dSum += data[i];
}


void ReportFifos()
{
	uint16_t samples;
	int32_t  sumI = 0, sumBAT = 0;

	// check for number of samples available:
	// Note: Its enough to check number of samples of BAT as I2
	// will have the same or +1 sample (+1 very unlikely, but can happen,
	// as new samples are written to the FIFO in sequence I1, I2, BAT, AUX
	// within some ~us)
	byte error = LTC2949_RdFifoSampleCount(LTC2949_REG_FIFOBAT, &samples);
	if ((samples > 0) && (error == 0))
	{
		// function is called once per LTC2949 CONT tick which is roughly 100ms, 
		// which equals roughly 128 fast samples. Still due to asynchronism 
		// and potential longer delay from measurement start to the first time
		// FIFOs are read it can happen, we have to read slightly more samples
		// To limit the memory allocation we limit the read to max. 140 samples
		const uint8_t maxSamplesToBeRead = 140;
		uint16_t len = min(maxSamplesToBeRead, samples);
		int16_t * buffer = new int16_t[len];

		// read fast I2 samples
		error = LTC2949_ReadFifo(
			/*byte  addr:        */ LTC2949_REG_FIFOI2,
			/*uint16_t *len:     */ &len,
			/*int16_t * samples: */ buffer);
		// sum all samples for average
		ProcessFastData(buffer, len, &sumI);
		if (len != samples)
		{
			// if we read more or less samples than available, something went wrong!
			error |= LTC2949_ERRCODE_OTHER;
			// reset len to original value
			len = min(maxSamplesToBeRead, samples);
		}

		// read fast BAT samples
		error |= LTC2949_ReadFifo(
			/*byte  addr:                 */ LTC2949_REG_FIFOBAT,
			/*uint16_t *len:              */ &len,
			/*int16_t * samples:          */ buffer);
		// sum all samples for average
		ProcessFastData(buffer, len, &sumBAT);
		if (len != samples)
			error |= LTC2949_ERRCODE_OTHER;

		delete[] buffer;
	}
	// print number of samples available
	PrintComma();
	Serial.print(samples);
	// print average of FIFOs
	PrintComma();
	Serial.print(sumI / (double)samples * LTC2949_LSB_FIFOI2, LTCDEF_DIGITS_I2FIFO_AVG);
	PrintComma();
	Serial.print(sumBAT / (double)samples * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFIFO_AVG);
}

/*!*********************************************************************
\brief Checks if the device as awake and in slow and fast continuous mode
This will restart the whole measurement e.g. in case the device was
powered off or there is a severe comminucation issue.
This can be demonstrated by just powering off the device or unplugging
the isoSPI interface while measurement is running.
***********************************************************************/
byte  ChkDeviceReady()
{
	byte data[10];
	byte error;
	boolean expChkFailed;

	if ((error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, data) != 0) || // PEC error (e.g. rebooting)
		(data[0] != LTC2949_BM_OPCTRL_CONT) ||              // not in continuous mode
		(error = LTC2949_READ(LTC2949_REG_FACTRL, 1, data) != 0) || // PEC error
		(data[0] != LTCDEF_FACTRL_CONFIG) ||               // not or wrong fast mode
		(error = LTC2949_ADCConfigRead(data) != 0) ||				// PEC error
		(data[0] != LTCDEF_ADCCFG_CONFIG) ||				// wrong ADC config
		(error = LTC2949_ReadChkStatusFaults(
			/*boolean lockMemAndClr:    */ false,
			/*boolean printResult:      */ false,
			/*byte len:                 */ 10,
			/*byte * statFaultsExpAndRd:*/ data,
			/*boolean * expChkFailed:   */ &expChkFailed,
			/*byte expDefaultSet):      */ LTC2949_STATFAULTSCHK_IGNORE_STATUPD | LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR) != 0) ||
		expChkFailed
		)
	{
		error |= WakeUpReportStatus();
		error |= Cont(false); // go IDLE
		delay(LTC2949_TIMING_CONT_CYCLE);
		error |= Cont(true);
		return error;
	}
	return 0;
}

/*!*********************************************************************
\brief Wakeup LTC2949, report and clear all status / alert registers
***********************************************************************/
byte WakeUpReportStatus()
{
	byte  error = LTC2949_WakeupAndAck();
	error |= LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	PrintOkErr(error);
	return error;
}


/*!*********************************************************************
\brief Activate / deactivate measurement in LTC2949
- slow continuous mode
- fast continuous CH2 mode
***********************************************************************/
byte  Cont(boolean enable)
{
	if (enable)
	{
		byte error = 0;
#ifdef LTCDEF_READ_FROM_EEPROM
		error = LTC2949_EEPROMRead();
#else
		// fast slot not used, still we configure something
		LTC2949_SlotFastCfg(3, 2);
		// SLOT1 measures temperature via NTC between V1 and GND. 
		// SLOT2 used optionally, see ADD_SLOT2_MEASUREMENT
		LTC2949_SlotsCfg(1, 0, SLOT2_MEAS_POS, SLOT2_MEAS_NEG);
		// enable NTC temperature measurement via SLOT1
		NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
#endif

#ifdef LTCDEF_WRITE_TO_EEPROM
		error = LTC2949_EEPROMWrite();
#endif
		uint8_t  revCode;
		error |= LTC2949_GetSiliconRevision(&revCode);
		Serial.print(revCode, HEX);
		// read & clear status
		error |= LTC2949_ReadChkStatusFaults(true, false);
		// enable measurement
		return error | LTC2949_GoCont(
			/*cfgFast:      */ LTCDEF_FACTRL_CONFIG,
			/*byte adcCfg:  */ LTCDEF_ADCCFG_CONFIG);
	}
	LTC2949_WriteFastCfg(0);
	LTC2949_OpctlIdle();
	return 0;
}

void NtcCfgWrite(int ntc1or2, float rref, float a, float b, float c)
{
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
