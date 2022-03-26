/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_BCI.ino
* Tiny Linduino Sketch for LTC2949 Demo Board:
*  implementation of fast continuous measurement cycles with changing AUX MUX setting
*  mitigate any kind of possible error conditions, repeat states where possible, recover where necessary
*
* EXPECTS EXTERNAL CONNECITONS:
*  current inputs shorted (or uOhm shunt without current)
*  VREF connected to V3 and BATP
*  GND connected to BATM
*
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

// some more error codes, additionally to 
// the internal error codes from the LTC2949 library
// (e.g. LTC2949_ERRCODE_COMMERR... see LTC2949.h for
// error codes reported from library functions)
#define LTCDEF_ERRCODE_MUX_READBACK 0x0100U
#define LTCDEF_ERRCODE_ABSRELCHK    0x0200U
#define LTCDEF_ERRCODE_ADCCONF_RB   0x0400U
#define LTCDEF_ERRCODE_HS_SET_CHK   0x0800U
#define LTCDEF_ERRCODE_HS_CLR_CHK   0x1000U
#define LTCDEF_ERRCODE_STAT_FAULTS  0x2000U

// increased tolerances, to check for catastrophic errors only
// relative tolerance used for conversion result checks
#define LTCDEF_REL_TOL 5e-2
// absolute tolerance used for voltage conversion result checks
#define LTCDEF_ABS_TOL_V 100e-3
// absolute tolerance for current
#define LTCDEF_ABS_TOL_I 2e-3
// Note: Above values are equivalent to the limits of LTC2949's safety targets.
// Note: For the EFT/BCI we look for big, catastrophic failures, e.g. MUX not
// switching, ADC not triggering. 
// We are not interested in real noise at the analog pins. 
// It may be, that the transients of the EFT / BCI cause some real and too big
// noise at the IC-pins that gets converted to violating conversion results. 
// Above limits can be increased in this case.

// nominal current for the current measurement checks (no current, shorted input)
#define LTCDEF_NOM_I 0.0
// nominal value of the BAT input, see required external connections!
#define LTCDEF_BAT_NOM LTC2949_VREF

// fixed number of AXU MUX configurations, see also required external connections!
#define LTCDEF_SLOTCONFCOUNT 6

#define LTCDEF_SLOTCONFALLFLAGS ((1<<LTCDEF_SLOTCONFCOUNT)-1)

#define LTCDEF_FACTRL_CONT  (LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACONV)

unsigned long checkStatusFaults;

void setup()
{
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// wait for serial port to connect. Needed for native USB port only
	while (!Serial)
		;

	Serial.print(F("INIT,"));
	PrintOkErr(Init());
	//example output line:0,0x0F0F,8,0,2.3937,3.0041,OK
	Serial.println(F("N,HS,I1[uV],I2[uV],AUX,BAT,OK/ERR"));
}

// sets the AUX mux, reads-back configuration and returns the expected voltage
// the mux settings and external connections are chosen to have a clear change
// between subsequent settings (2.4V,-2.4V,0,3.0V,-3.0V,0)
float SlotConf(byte sel, uint16_t* error)
{
	byte data[2];
	byte dataRb[2];
	float nom;

	switch (sel)
	{
	case 0:
		data[1] = LTC2949_SLOTMUX_VREF2; //MUXP
		data[0] = LTC2949_SLOTMUX_GND;   //MUXN
		nom = LTC2949_VREF2; // nominal value
		break;
	case 1:
		data[1] = LTC2949_SLOTMUX_GND;
		data[0] = LTC2949_SLOTMUX_VREF2;
		nom = -LTC2949_VREF2;
		break;
	case 2:
		data[1] = LTC2949_SLOTMUX_VREF2_250k;
		data[0] = LTC2949_SLOTMUX_VREF2;
		nom = 0;
		break;
	case 3:
		data[1] = LTC2949_SLOTMUX_V3;
		data[0] = LTC2949_SLOTMUX_GND;
		nom = LTC2949_VREF;
		break;
	case 4:
		data[1] = LTC2949_SLOTMUX_GND;
		data[0] = LTC2949_SLOTMUX_V3;
		nom = -LTC2949_VREF;
		break;
	default:
	case 5:
		data[1] = LTC2949_SLOTMUX_GND;
		data[0] = LTC2949_SLOTMUX_GND;
		nom = 0;
		break;
	}

	// write configuration
	LTC2949_WRITE(LTC2949_REG_FAMUXN, 2, data);
	// read back configuration
	error[0] |= LTC2949_READ(LTC2949_REG_FAMUXN, 2, dataRb);
	// check configuration
	if (data[0] != dataRb[0] || data[1] != dataRb[1])
		error[0] |= LTCDEF_ERRCODE_MUX_READBACK;
	return nom; // return nominal value
}

// checks for relative / absolute errors of measurements
void AbsRelErrCheck(float diff, float nom, float abs, float rel, uint16_t* error, char id)
{
	// check absolute error
	if (abs(diff) > abs)
	{
		// check relative error
		if (abs(diff / nom) > rel)
		{
			// check failed
			*error |= LTCDEF_ERRCODE_ABSRELCHK;
			// print debug information about the failed check
			Serial.print(id);
			Serial.print(F(":Err:"));
			Serial.print(diff, 5);
			Serial.print('|');
		}
	}
}

// initialize LTC2949
uint16_t Init()
{
	uint16_t error = 0;
	byte data;
	unsigned long timeout;
	bool hsOK;
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	// Initialize LTC2949 library's internal variables that reflect
	// LTC2949's device state
	LTC2949_init_device_state();
	LinduinoSelectSPI(LTCDEF_CS_SELECT);

	error |= LTC2949_WakeupAndAck(); // wakeup LTC2949
	// in case LTC2949 was already in MEASURE state
	// this is the worst case timing to go to STANDBY first
	timeout = millis() + 200;
	LTC2949_OpctlIdle(); // clear OPCTRL, go to STANDBY state
	while (!LTC_TIMEOUT_CHECK(millis(), timeout))
		;
	error |= LTC2949_ReadChkStatusFaults(true, true); // read all status/faults register, report and clear them
	Serial.println();
	LTC2949_ADCConfigPasV(LTC2949_BM_ADCCONF_P2ASV); // enable P2ASV
	LTC2949_ADCConfigWrite(); // write ADCCONF
	error |= LTC2949_OpctlAdjUpd(true); // trigger adjust update (for activate changes within ADCCONF)
	error |= LTC2949_ADCConfigRead(&data); // read back ADCCONF
	if (data != LTC2949_BM_ADCCONF_P2ASV) // check ADCCONF value
		error |= LTCDEF_ERRCODE_ADCCONF_RB; // error when reading back ADCCONF

	LTC2949_OpctlCont(); // write CONT to OPCTRL (enable slow cont. mode)
	timeout = millis() + LTC2949_TIMING_IDLE2CONT2UPDATE; // set timeout, here: worst case time LTC2949 successfully operating in slow cont. mode.

	float nom = SlotConf(1, &error); // write and check AUX MUX configuration

	// the following loop is the fastest way to enable all ADCs:
	// continuously try to make fast single shot conversion. Once the
	// first conversion was triggered and read successfully (checked via
	// HS-Byte) we know everything is up and running.
	while (true)
	{
		unsigned long fastConversionEndTime;
		// clear HS byte
		error |= LTC2949_RdFastData();
		// enable fast mode for I2 and AUX
		LTC2949_WriteFastCfg(LTCDEF_FACTRL_CONT);
		// trigger fast conversion
		LTC2949_ADxxAddressed(); // trigger measurement
		// STOP watch ends with 1st conversion ready to be read
		fastConversionEndTime = micros() + 1208; // +8 because of 8us granularity of Linduino!
		// wait for worst case conversion and update time
		while (!LTC_TIMEOUT_CHECK(micros(), fastConversionEndTime))
			;
		// read conversion result
		error |= LTC2949_RdFastData(fastData2949);
		// check HS-Byte
		hsOK = LTC2949_FASTSSHT_HS_OK(fastData2949);
		// break at timeout or first successfull conversion
		if (LTC_TIMEOUT_CHECK(millis(), timeout) || hsOK)
			break;
	}
	// check if there was a successfull fast conversion
	if (!hsOK)
		error |= LTCDEF_ERRCODE_HS_SET_CHK; // failed to make fast single shot
	// check the conversion result. Here we only check the AUX channel.
	// more checks will be done in the main loop.
	AbsRelErrCheck(
		/*float diff   */nom - fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX,
		/*float nom    */nom,
		/*float abs    */LTCDEF_ABS_TOL_V,
		/*float rel    */LTCDEF_REL_TOL,
		/*byte * error */&error,
		/*char id      */'v');
	checkStatusFaults = millis() + 200; // check status / faults registers first time after 200 ms.	
	error |= LTC2949_ReadChkStatusFaults(false, true); // read all status/faults register, report and clear them
	Serial.println();
	FifoLoopInit();
	return error;
}

/*!*********************************************************************
\brief The simplest processing we can do on the fast FIFO samples:
Sum of all samples
***********************************************************************/
void ProcessFastData(int16_t* data, byte  len, int32_t* dSum)
{
	*dSum = 0;
	for (byte i = 0; i < len; i++)
		* dSum += data[i];
}

enum states : byte
{
	stopFifo = 0x01,	// 1
	readFifo,			// 2
	stopFifoChk,		// 3
	changeMux,			// 4
	startFifo,			// 5
	chkSlowChannel,		// 6
	clrStatFlt,	     	// 7
	waitFifo,			// 8
	checkStatFlt,		// 9
	lockMem,	     	// 10
	unLockMem,	     	// 11
} state;

unsigned long stateTimeStamp;
uint16_t errCount;
int16_t nomAux;
byte selAux;
byte auxFlags;
unsigned long auxFlagsTimeout;

// 2 wait fifo cycles lead to approx 8 fast samples
#define LTCDEF_WAITFIFOCNT_INIT 2
byte waitFifoCnt = LTCDEF_WAITFIFOCNT_INIT;
// state machine cycle time is ~1.2ms
// +8 because of Linduino granularity
#define LTCDEF_CYCLE_TIME_US 1208
// first fifo read after 10 ms
#define LTCDEF_INIT_TIME_US 10000

// max. number of PEC errors between successful reads of all 6 AUX channels
#define LTCDEF_ERR_COUNT_MAX 2000

// define analog measurement tolerances
#define LTCDEF_CURRENT_ABS_MAX (LTCDEF_ABS_TOL_I/LTC2949_LSB_FI2)
#define LTCDEF_BAT_MAX ((LTCDEF_BAT_NOM+LTCDEF_ABS_TOL_V)/LTC2949_LSB_FBAT)
#define LTCDEF_BAT_MIN ((LTCDEF_BAT_NOM-LTCDEF_ABS_TOL_V)/LTC2949_LSB_FBAT)
#define LTCDEF_AUX_DELTA (LTCDEF_ABS_TOL_V/LTC2949_LSB_FAUX)

#define SM_DEBUG true
#define FIFO_DEBUG true
#define FIFO_PEC_DEBUG true

#define LTCDEF_AUX_FLAGS_TIMEOUT 2000
#define LTCDEF_MEMLOCK_TIMEOUT 1000

void FifoLoopInit()
{
	stateTimeStamp = micros() + LTCDEF_INIT_TIME_US;
	state = stopFifo;
	errCount = 0;
	nomAux = -LTC2949_VREF2 / LTC2949_LSB_FAUX;
	selAux = 1;
	auxFlags = 0;
	// timeout ofter 2 seconds: Meaning if we are not able to 
	auxFlagsTimeout = millis() + LTCDEF_AUX_FLAGS_TIMEOUT;

	byte error = 0;
	LTC2949_ChkUpdate(&error);
	LTC2949_autoForceWrRegsCtrl = false;
	Serial.print(F("UPD,"));
	Serial.println(LTC2949_GetLastTBxDbl());
}

// READ FIFO
// - read burst must end with two PEC bytes
// - ignore all samples in case of
//  - PEC errors
//  - TAG mismatch
//  - (see below: there is even a very unlike case of TAG error and correct PEC due to single bit flip and one bit shift)
// - explicitly check TAGs and only accept samples with TAG 0x00 or 0xAA (meaning do not just check for != 0x55)
boolean ProcessFifo(byte fa, int16_t mi, int16_t ma, boolean* fatalError, float lsb)
{
	boolean measOK = true;
	byte error;
#define LTCDEF_MAXSAMPLESTOBEREAD 16
#define LTCDEF_FIFO_RD_BYTES (LTCDEF_MAXSAMPLESTOBEREAD*3)
#define LTCDEF_FIFO_RD_WORDS ((int)(LTCDEF_FIFO_RD_BYTES>>1 + LTCDEF_FIFO_RD_BYTES%2))
	int16_t buffer[LTCDEF_FIFO_RD_WORDS];
	uint8_t samplesRead = 0;
	error = LTC2949_READ(fa, LTCDEF_FIFO_RD_BYTES, (byte*)buffer);
	if (error == 0)
	{
		// no PEC error
		for (uint8_t i = 0; i < LTCDEF_FIFO_RD_BYTES; i += 3)
		{
			byte tag = ((byte*)buffer)[i + 2];
			int16_t sample = (((byte*)buffer)[i + 0]) << 8 | ((byte*)buffer)[i + 1];
			switch (tag)
			{
			case LTC2949_FIFO_TAG_OK:
			case LTC2949_FIFO_TAG_WROVR:
				// good sample
				buffer[samplesRead++] = sample;
				break;

			case LTC2949_FIFO_TAG_RDOVR:
				// sample already read
				break;

				//case LTC2949_FIFO_TAG_WROVR >> 1 : // this is 0x55=RDOVR
				//case LTC2949_FIFO_TAG_RDOVR << 1 : // this is 0xAA=WROVR
			case (LTC2949_FIFO_TAG_WROVR << 1U) & 0xFFU:
			case (LTC2949_FIFO_TAG_RDOVR >> 1U) & 0xFFU:
				// some bit shifted wrong tags:
				Serial.println(F("ERR,TAG,SHIFT"));
				// see example of one bit shift and one bit flip that shows wrong tag but correct PEC:
				// Summary:                                                                              |<--------disturbed data---------->|
				// real data : 0x00000000000000000000000000010000E0680000000000000000000000000000000000400030000000002A80002A80002A80002AFEDC
				// corrected : 0x00000000000000000000000000010000E06800000000000000000000000000000000004000000000000055000055000055000055FDB8
				// (correction done, by shift and bit flip)
				//measOK = false;
				//break;

			default:
				// any other tag: fatal error
				Serial.print(F("ERR,TAG,"));
				Serial.print(i);
				Serial.print(',');
				Serial.print(sample);
				Serial.print(',');
				SerialPrintByteX2(tag);
				Serial.println();
				//fatalError[0] = true;
				measOK = false;
				break;
			}
		}
		if (!measOK)
		{
			// for debugging only: print RAW data
			SerialPrintByteArrayHex((byte*)buffer, LTCDEF_FIFO_RD_BYTES, true);
			Serial.print(F(",ERR,TAG,"));
			PrintOkErr(error);
		}
		if (samplesRead > 0)
		{
			// check samples
			int32_t sum;
			ProcessFastData(buffer, samplesRead, &sum);
			sum = sum / samplesRead;
			boolean report = false;
			if (sum < mi || sum > ma)
			{
				report = true;
				for (uint8_t i = 0; i < samplesRead; i++)
				{
					PrintZeroX();
					Serial.print(buffer[i], HEX);
					Serial.print(',');
					Serial.print(buffer[i] * lsb);
					Serial.print(',');
				}
				Serial.println();
				Serial.print(F("ERR,"));
				if (measOK)
				{
					fatalError[0] = true;
					measOK = false;
				}
			}
			else if (FIFO_DEBUG)
			{
				report = true;
				Serial.print(F("SA,"));
			}
			if (report)
			{
				Serial.print(fa, HEX);
				Serial.print(',');
				Serial.print(samplesRead);
				Serial.print(',');
				Serial.println(sum * lsb);
			}
		}
		else //samplesRead == 0
		{
			// no samples
			Serial.print(F("ERR,ZERO,"));
			Serial.println(fa, HEX);
			fatalError[0] = true;
			measOK = false;
		}

#if defined(LTC2949_DCRW_RAW_DATA_DEBUG) && defined(ARDUINO)
		if (!measOK)
		{
			// for debugging only: print RAW data
			Serial.print(F("ERR,RAW:"));
			byte* rawbytes;
			uint16_t bl = LTC2949_GetDcRwRawData(&rawbytes);
			SerialPrintByteArrayHex(rawbytes, bl, true);
			Serial.println();
		}
#endif

	}
	else
	{
		// for debugging only: print info about PEC errors
		if (FIFO_PEC_DEBUG)
		{
			Serial.print(F("ERR,PEC,"));
			Serial.print(fa, HEX);
			Serial.print(',');
			Serial.println(samplesRead);
		}
		errCount++; // ERROR
		measOK = false;
	}
	return measOK;
}

// FIFO loop with state machine:
//  fast continuous measurements for different AUX inputs
void loop()
{
	boolean fatalError = false;

	if (auxFlags == LTCDEF_SLOTCONFALLFLAGS)
	{
		unsigned long nextTimeout = millis() + LTCDEF_AUX_FLAGS_TIMEOUT;
		unsigned long auxTime = nextTimeout - auxFlagsTimeout;
		Serial.print(F("AXOK,"));
		Serial.print(auxTime);
		Serial.print(',');
		Serial.println(errCount);
		errCount = 0;
		auxFlags = 0;
		// timeout for measuring all AUX channels
		// in case of too many errors without any successful 
		// measurements, this will stop the execution
		auxFlagsTimeout = nextTimeout;
	}
	else if (LTC_TIMEOUT_CHECK(millis(), auxFlagsTimeout))
	{
		// timeout for measuring all AUX channels
		Serial.print(F("ERR,AXFLAGS,"));
		Serial.println(auxFlags, HEX);
		fatalError = true;
		stateTimeStamp = micros(); // to make sure following timeout is triggered
	}

	if (!LTC_TIMEOUT_CHECK(micros(), stateTimeStamp))
		return;
	stateTimeStamp = micros() + LTCDEF_CYCLE_TIME_US; // next time stamp

	switch (state)
	{
	case stopFifo: // stop fast continuous operation: preparation to change AUX MUX setting
	{
		if (SM_DEBUG) Serial.println(F("stopFifo"));

		stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay

		byte data;

		LTC2949_WRITE(LTC2949_REG_FACTRL, 0x00); // disable fast continuous conversion
		if (LTC2949_READ(LTC2949_REG_FACTRL, 1, &data) || // read back
			data != 0x00) // check data
		{
			errCount++; // ERROR
			break;
		}
		state = readFifo; // OK, go to next state
		break;
	}
	case readFifo: // read and check all FIFOs
	{
		if (SM_DEBUG) Serial.println(F("readFifo"));

		Serial.print(selAux);
		Serial.println(':');
		// read and check all FIFOs
		boolean iOk = ProcessFifo(LTC2949_REG_FIFOI2, -LTCDEF_CURRENT_ABS_MAX, LTCDEF_CURRENT_ABS_MAX, &fatalError, LTC2949_LSB_FI2);
		boolean bOk = ProcessFifo(LTC2949_REG_FIFOBAT, LTCDEF_BAT_MIN, LTCDEF_BAT_MAX, &fatalError, LTC2949_LSB_FBAT);
		boolean xOk = ProcessFifo(LTC2949_REG_FIFOAUX, nomAux - LTCDEF_AUX_DELTA, nomAux + LTCDEF_AUX_DELTA, &fatalError, LTC2949_LSB_FAUX);
		if (iOk && bOk && xOk)
			auxFlags |= 1 << selAux;

		// this step is never repeated as we can't repeat FIFO reads with PEC errors!
		state = stopFifoChk; // OK, go to next state
		break;
	}
	case stopFifoChk: // sanity check: RDCV is used to read the HS-Byte to check fast continuous mode has stopped
	{
		// state stopFifoChk is a sanity check that can be omitted

		if (SM_DEBUG) Serial.println(F("stopFifoChk"));

		stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay

		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
		byte error;

		// RDCV to check handshake byte (HS-byte)
		if ((error = LTC2949_RdFastData(fastData2949)))
		{
			errCount++;
		}
		else if (LTC2949_FASTSSHT_HS_OK(fastData2949) || LTC2949_FASTSSHT_HS_CLR(fastData2949)) // HS byte either 0x00 or 0x0F
		{
			; // OK, nothing to do.
		}
		else
		{
			Serial.println(F("ERR,HS_sn"));
			fatalError = true; // no HS byte
			break;
		}
		if (LTC2949_RdFastData(fastData2949))
		{
			errCount++;
		}
		else if ((error == 0) && !LTC2949_FASTSSHT_HS_CLR(fastData2949))
		{
			// HS byte must be cleared but only if first RDCV was OK!
			Serial.println(F("ERR,HS_s"));
			fatalError = true; // HS byte not cleared
			break;
		}
		state = changeMux; // always go to next state
		break;
	}
	case changeMux: // change and check the AUX MUX setting
	{
		if (SM_DEBUG) Serial.println(F("changeMux"));

		stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay

		uint16_t error = 0;
		byte nextSelAux = (selAux + 1) % LTCDEF_SLOTCONFCOUNT;

		nomAux = SlotConf(nextSelAux, &error) / LTC2949_LSB_FAUX; // configure AUX MUX and read back
		if (error)
		{
			errCount++; // ERROR
			break; // repeat!
		}
		selAux = nextSelAux;
		state = startFifo; // OK, go to next state
		break;
	}
	case startFifo: // enable fast continuous mode with new MUX setting
	{
		if (SM_DEBUG) Serial.println(F("startFifo"));

		byte data;

		LTC2949_WRITE(LTC2949_REG_FACTRL, LTCDEF_FACTRL_CONT); // enable fast continuous conversion
		if (LTC2949_READ(LTC2949_REG_FACTRL, 1, &data) || // read back, check PEC error
			data != LTCDEF_FACTRL_CONT) // check data
		{
			errCount++; // ERROR
			stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // repeat without delay
			break; // repeat
		}
		waitFifoCnt = LTCDEF_WAITFIFOCNT_INIT;
		state = chkSlowChannel; // OK, go to next state
		break;
	}
	default:
	case chkSlowChannel: // check the slow channel: update of TB1 and STATUS/FAULTS/ALERT registers
	{
		if (SM_DEBUG) Serial.println(F("chkSlowChannel"));

		byte error = 0;

		// slow channel is updated only every 100 ms, so we do not have to read more often than every 100 ms
		if (LTC_TIMEOUT_CHECK(millis(), checkStatusFaults))
		{
			// first check memory lock status
			byte mlk = LTC2949_MemLockStatusInt();
			if (mlk != LTC2949_REGSCTRL_MEMLOCK_UNLOCKED)
				error = LTC2949_MemLockStatus(&mlk); // update status
			if (error)
			{
				// PEC error
				stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay
				state = chkSlowChannel; // repeat
				errCount++;
			}
			else if (mlk == LTC2949_REGSCTRL_MEMLOCK_REQUEST)
			{
				// memory lock requested
				unsigned long memLockTimeout = checkStatusFaults + LTCDEF_MEMLOCK_TIMEOUT;
				if (LTC_TIMEOUT_CHECK(millis(), memLockTimeout))
				{
					// timeout of memory lock
					Serial.print(F("ERR,MLR,"));
					Serial.print(mlk, HEX);
					Serial.print(',');
					Serial.println(LTC2949_GetLastTBxDbl());
					fatalError = true;
					break;
				}
				// nothing to do at this time, we still wait for the memory being locked.
				state = waitFifo; // OK, go to next state
			}
			else if (mlk == LTC2949_REGSCTRL_MEMLOCK_ACK)
			{
				// memory locked successful: we are ready to clear STATUS, (EXT)FAULTS
				stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay
				state = clrStatFlt; // next state: clear STATUS, (EXT)FAULTS
			}
			else if (mlk == LTC2949_REGSCTRL_MEMLOCK_ERR)
			{
				// not allowed combination of memory lock bits MLK0, MLK1
				Serial.println(F("ERR,UPD,MLK"));
				fatalError = true;
			}
			else //if (mlk == LTC2949_REGSCTRL_MEMLOCK_UNLOCKED)
			{
				// memory is unlocked, which is the default case
				// check for changes of TB1
				if (LTC2949_ChkUpdate(&error))
				{
					checkStatusFaults = millis() + 100; // status / faults registers must be updated every 100 ms.
					stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay
					state = checkStatFlt; // OK, go to next state
					Serial.print(F("UPD,"));
					Serial.println(LTC2949_GetLastTBxDbl());
				}
				else if (error == 0)
				{
					unsigned long checkStatusFaultsTimeout = checkStatusFaults + 300;
					if (LTC_TIMEOUT_CHECK(millis(), checkStatusFaultsTimeout))
					{
						Serial.print(F("ERR,UPD,"));
						Serial.println(LTC2949_GetLastTBxDbl());
						fatalError = true;
					}
				}
				else
				{
					// PEC error
					stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay
					state = chkSlowChannel; // repeat
					errCount++;
				}
			}
		}
		else
		{
			state = waitFifo; // OK, go to next state
		}
		break;
	}
	case clrStatFlt: // clear STATUS, (EXT)FAULTS registers
	{
		if (SM_DEBUG) Serial.println(F("clrStatFlt"));

		stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // go to next step without delay
		state = unLockMem; // default next state

		byte data[10];
		byte error;
		// always read and report current values of STATUS, (EXT)FAULTS registers
		error = LTC2949_READ(LTC2949_REG_STATUS, 8, data); // STATUS and alerts
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, &(data[8])); // (EXT)FAULTS registers
		if (error)
		{
			errCount++;
			state = clrStatFlt; // repeat
			break; // repeat
		}
		// report current values
		LTC2949_ReportFaults(data, 10);
		Serial.println();

		// clear and read back all STATUS, ALTERT, (EXT)FAULTS regs
		memset(data, 0, 8);
		LTC2949_WRITE(LTC2949_REG_STATUS, 8, data);
		memset(data, 0, 2);
		LTC2949_WRITE(LTC2949_REG_EXTFAULTS, 2, data);
		// check if registers were cleared correctly
		error = LTC2949_READ(LTC2949_REG_STATUS, 8, data);
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, &(data[8]));
		if (error)
		{
			errCount++;
			state = clrStatFlt; // repeat
			break; // repeat
		}
		else
		{
			// no PEC error
			// check if all regs were read as 0x00
			for (uint8_t i = 0; i < 10; i++)
			{
				if (data[i] != 0)
				{
					errCount++;
					state = clrStatFlt; // repeat and clear again
					break;
				}
			}
		}
		break;
	}
	case waitFifo: // RDCV command is used to read HS-bytes to check if fast continuous mode is running again
	{
		if (SM_DEBUG) Serial.println(F("waitFifo"));

		byte error;
		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
		int16_t fastData2949_2[LTC2949_RDFASTDATA_LENGTH];

		// there is a very rare condition when an incomplete transaction (e.g. missing +long isoSPI pulse)
		// causes the HS byte to be cleared only with the next transaction (even if there were
		// tens of milliseconds between the commands)
		// always issuing two RDCVs helps to avoid this condition:
		error = LTC2949_RdFastData(fastData2949);	 // 1st RDCV
		error |= LTC2949_RdFastData(fastData2949_2); // 2nd RDCV
		if (error) // check both RDCVs were without PEC errors
		{
			// PEC error
			stateTimeStamp = micros() + LTCDEF_CYCLE_TIME_US; // repeat only >1.2ms later
			errCount++;
			break; // repeat current state
		}
		else if (LTC2949_FASTSSHT_HS_OK(fastData2949) || LTC2949_FASTSSHT_HS_OK(fastData2949_2))
		{
			; // OK: no PEC error for both RDCVs and at least one HS-byte was 0x0F
			// this means the fast continuous mode is running again
		}
		else
		{
			// FATAL ERROR. fast continuous mode not running after > 1.2ms
			// Execution will stop after some debug report
			Serial.print(F("ERR,HS1,"));
			SerialPrintByteArrayHex((byte*)fastData2949, LTC2949_RDFASTDATA_LENGTH * 2);
			Serial.print(',');
			SerialPrintByteArrayHex((byte*)fastData2949_2, LTC2949_RDFASTDATA_LENGTH * 2);
			Serial.println();
			uint32_t hscnt = 0;
			unsigned long startTime = micros();
			while ((LTC2949_RdFastData(fastData2949) != 0) || (hscnt < 10))
			{
				SerialPrintByteArrayHex((byte*)fastData2949, LTC2949_RDFASTDATA_LENGTH * 2);
				Serial.print(',');
				Serial.print(micros() - startTime);
				Serial.println();
				if (LTC2949_FASTSSHT_HS_OK(fastData2949))
					hscnt++;
			}
			Serial.print(F("HS2,"));
			Serial.print(hscnt);
			Serial.print(',');
			SerialPrintByteArrayHex((byte*)fastData2949, LTC2949_RDFASTDATA_LENGTH * 2);
			Serial.println();
			fatalError = true;
			break;
		}

		// counter for waiting FIFO to fill-up
		// while waiting we will periodically check for updates of the slow channel
		waitFifoCnt--;
		if (waitFifoCnt)
			state = chkSlowChannel; // counter not zero, continue waiting for FIFO but check update of slow channel first
		else
			state = stopFifo; // OK, go to next state to stop the FIFO
		break;
	}
	case checkStatFlt: // check STATUS, ALERT, (EXT)FAULTS registers
	{
		if (SM_DEBUG) Serial.println(F("checkStatFlt"));

		byte error;
		byte data[10];

		// read all STATUS, ALTERT, (EXT)FAULTS regs
		error = LTC2949_READ(LTC2949_REG_STATUS, 8, data); // STATUS and alerts
		error |= LTC2949_READ(LTC2949_REG_EXTFAULTS, 2, &(data[8])); // (EXT)FAULTS registers
		LTC2949_ReportFaults(data, 10); // report current register values
		Serial.println();
		if (error)
		{
			stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // repeat without delay
			errCount++; // ERROR
			break;
		}
		// check all error flags and report them in case of error
		// check STATUS
		if (bitMaskClrChk(data[0], LTC2949_BM_STATUS_UPDATE)) // UPDATE bit must always be set! ...
		{
			Serial.println(F("ERR,ST,UPD"));
			fatalError = true;
		}
		if ((data[0] & ~LTC2949_BM_STATUS_UPDATE) != 0) // ... other bits must be cleared
		{
			Serial.println(F("ERR,ST,MSC"));
			fatalError = true;
		}
		// check EXTFAULTS
		if (data[8] != 0)
		{
			Serial.println(F("ERR,ST,EXF"));
			fatalError = true;
		}
		if ((data[9] & ~LTC2949_BM_FAULTS_EXTCOMMERR) != 0) // any other bit besides EXTCOMMERR must never be set
		{
			Serial.println(F("ERR,ST,FLT"));
			fatalError = true;
		}
		// check all alerts
		for (uint8_t i = 1; i < 9; i++)
		{
			if (data[i] != 0)
			{
				Serial.print(F("ERR,ST,ALT"));
				Serial.println(i);
				fatalError = true;
				break;
			}
		}
		if (fatalError)
			break;
		// check FAULTS: EXTCOMMERR may be set in case of PEC error from MASTER to SLAVE
		if (bitMaskSetChk(data[9], LTC2949_BM_FAULTS_EXTCOMMERR))
		{
			stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // next state without delay
			Serial.println(F("ERR,ST,WRE"));
			state = lockMem; // lock the memory to allow clear of all STATUS, ALTERT, (EXT)FAULTS registers
			break;
		}

		state = waitFifo; // OK, go to next state
		break;
	}
	case lockMem: // lock the memory: Preparation to clear all STATUS, ALTERT, (EXT)FAULTS registers
	{
		if (SM_DEBUG) Serial.println(F("lockMem"));

		byte error, mlk;

		// LTC2949_MemLockStatusInt will report the last status of memory lock that was successfully read.
		if (LTC2949_MemLockStatusInt() != LTC2949_REGSCTRL_MEMLOCK_REQUEST)
			LTC2949_MemLockRequest(); // only write again if not yet requested
		error = LTC2949_MemLockStatus(&mlk); // always update current memory lock status

		if (error)
		{
			stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // repeat without delay
			errCount++; // ERROR
			break;
		}
		else if (mlk == LTC2949_REGSCTRL_MEMLOCK_REQUEST || mlk == LTC2949_REGSCTRL_MEMLOCK_ACK)
			; // OK, nothing to do.
		else if (mlk == LTC2949_REGSCTRL_MEMLOCK_UNLOCKED || (fatalError = (mlk == LTC2949_REGSCTRL_MEMLOCK_ERR)))
		{
			// in case of LTC2949_REGSCTRL_MEMLOCK_ERR: this will be a fatal error
			//
			// in case of LTC2949_REGSCTRL_MEMLOCK_UNLOCKED:
			//  note that LTC2949_MemLockStatusInt() will now also report LTC2949_REGSCTRL_MEMLOCK_UNLOCKED
			//  so we'll just repeat the state and try to lock again.
			errCount++; // ERROR (e.g. bit error in the previous CMD bytes)
			stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // repeat without delay
			Serial.print(F("ERR,MLK,"));
			Serial.print(mlk, HEX);
			Serial.print(',');
			Serial.println(LTC2949_iAddrRegsCtrl[LTC2949_REGSCTRL_IX] & ~(LTC2949_BM_REGSCTRL_MLK1 | LTC2949_BM_REGSCTRL_MLK0), HEX);
			break; // repeat state
		}
		checkStatusFaults = millis() + 200; // timeout for memory lock set to 200 ms
		state = waitFifo; // OK, go to next state
		break;
	}
	case unLockMem: // unlock the memory: last action after clearing of all STATUS, ALTERT, (EXT)FAULTS registers
	{
		if (SM_DEBUG) Serial.println(F("unLockMem"));

		stateTimeStamp -= LTCDEF_CYCLE_TIME_US; // without delay

		byte mlk;

		LTC2949_MemLockRelease(); // release memory lock

		// check memory lock status
		if (LTC2949_MemLockStatus(&mlk) || (mlk != LTC2949_REGSCTRL_MEMLOCK_UNLOCKED))
		{
			errCount++;
			break; // repeat
		}
		checkStatusFaults = millis() + 200; // worst case time to check again for slow channel update
		state = waitFifo; // OK, go to next state
		break;
	}
	}

	if (fatalError || errCount > LTCDEF_ERR_COUNT_MAX)
	{
		// fatal ERROR. following code is for debugging purposes only
		Serial.print(state);
		Serial.print(',');
		Serial.println(errCount);

		regDump(0x000, 0x1FF);
		delay(1);
		regDump(0x000, 0x1FF);
		delay(110);
		regDump(0x000, 0x1FF);


		while (true)
		{
			if (!Serial.available())
				continue;

			switch ((char)Serial.read())
			{
			case '0':
			{
				LTC2949_DebugEnable = !LTC2949_DebugEnable;
				break;
			}
			case 'r':
			{
				int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
				byte error = LTC2949_RdFastData(fastData2949);
				Serial.print(F("RDCV,"));
				SerialPrintByteArrayHex((byte*)fastData2949, LTC2949_RDFASTDATA_LENGTH * 2);
				Serial.print(',');
				PrintOkErr(error);
				break;
			}
			case 'a':
			{
				Serial.println(F("adcv"));
				LTC2949_ADxx();
				break;
			}
			case 'A':
			{
				Serial.println(F("ADCV"));
				LTC2949_ADxxAddressed();
				break;
			}

			case 'd':
				regDump(0x000, 0x0FF);
				break;
			case 'D':
				regDump(0x100, 0x1FF);
				break;
			case 'e':
			{
				uint16_t a = Serial.readStringUntil('\n').toInt();
				regDump(a, a + 15);
				break;
			}
			case 'w':
			{
				Serial.print(F("WRITE,"));
				String s = Serial.readStringUntil('\n');
				unsigned int iStart = 0;
				int a, d;
				if (stringSplitter(s, &iStart, &a))
				{
					stringSplitter(s, &iStart, &d);
					Serial.print(a, HEX);
					Serial.print(',');
					Serial.println(d, HEX);
					LTC2949_WRITE(a, d);
					regDump(a, a);
				}
				else
					Serial.println(F("ERR"));

				break;
			}
			default:
				break;
			}
		}
	}
}

// dump all registers of LTC2949
void regDump(uint16_t start, uint16_t end)
{

	Serial.println(F("DUMP"));
	for (uint16_t addr = start; addr <= end; addr += 16)
	{
		byte data[16];
		byte error = LTC2949_READ(addr, 16, data);
		SerialPrintByteX2(error);
		Serial.print(',');
		SerialPrintByteX2(addr);
		Serial.print(',');
		SerialPrintByteArrayHex(data, 16);
		Serial.println();
	}
}
