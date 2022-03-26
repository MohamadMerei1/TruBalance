/*
* Linear Technology / Analog Devices, Ismaning Design Center
* DC2732A_FIFO12 (Linduino Sketch for LTC2949 Demo Board - Isolated Battery Meter for EV / HEV)
*
* Basic example for
*  fast I1+I2
*  continuously reading FIFO samples every xx ms (configurable e.g. 50ms)
*  reading last fast samples every  xx ms (e.g. 200ms, to show arbitrary timed reads via RDCV command without affecting FIFO read)
*  SPI / isoSPI possible
*
*
*
* created: Patrick Wilhelm (PW)
* last Revision:
* $Revision: 534 $
* $Author: pwilhelm $
* $Date: 2017-07-07 15:09:05 +0200 (Fri, 07 Jul 2017) $
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

// default values
// number of FIFO samples to be read per cycle
#define MAX_SAMPLES_PER_FIFO_READ 100
// default fifo read cycle time (ms)
#define TIME_DIFF_FIFO_RD 50
// default last fast sample read cycle time (ms)
#define TIME_DIFF_LAST_SAMPLE_RD  4000
// min allowed cycle time (ms) (for last sample and fifo read)
// for practical reasons and limited speed of Linduino we restrict the minimum cycle time to 30 ms. This is not a limitation of LTC2949!
// (30 ms is more than the worst case processing time of Linduino)
#define TIME_DIFF_MIN 30
// LTC2949 has maximum of 1000 samples. To avoid WROVR (fifo full) we have to read fast enough (minus some margin for reading / processing of the data on the Linduino)
#define TIME_DIFF_FIFO_RD_MAX (uint16_t)(0.8 * 1000 - TIME_DIFF_MIN)

#include <Arduino.h>
#include "Linduino.h"
#include <SPI.h>
#include "LTC2949.h"
#include "ltcmuc_tools.h"

#include <stdint.h>


//operation modes
#define LTC2949_BM_FACTRL_SSHT_CONF (LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)
#define LTC2949_BM_FACTRL_CONF (LTC2949_BM_FACTRL_SSHT_CONF | LTC2949_BM_FACTRL_FACONV)


// time stamps for time slots (cycles)
unsigned long timeSlotFifoRd;
unsigned long timeSlotLastSampleRd;
// used to report uC time
unsigned long timingReport;
// time difference in ms between events (cycle times)
uint16_t timeDiffFifoRd;
uint16_t timeDiffLastSampleRd;
// maximum number of samples to be read in the FIFO read time slot
uint16_t maxSamplesFifoRead;

// true in case we are ready to read measurement results
boolean ready;
// buffer for commands received via serial interface
String SerialInputString;

/*!**********************************************************************
 \brief Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
	ready = false;
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

	// init time slots
	timeSlotFifoRd = 0;
	timeSlotLastSampleRd = 0;
	timeDiffFifoRd = TIME_DIFF_FIFO_RD;
	timeDiffLastSampleRd = TIME_DIFF_LAST_SAMPLE_RD;
	maxSamplesFifoRead = MAX_SAMPLES_PER_FIFO_READ;

	timingReport = 0;

	while (!Serial)
	{
		; // wait for serial port to connect. Needed for native USB port only
	}
	PrintUsage();
}

/*!**********************************************************************
\brief Print usage help
***********************************************************************/
void PrintUsage()
{
	Serial.print(F("\
Commands for LTC2949:\n\
WK     : Wakeup, read, clear status\n\
CNT    : Enble slow & fast continuous\n\
GO*    : Read measurements\n\
CNTGO* : Enable and read\n\
IDL    : Stop measurement\n\
RST    : Reset\n\
SPI*   : Change SPI frequency\n\
DB*    : Enable debug output\n\
*: optional add comma separated parameters, see docu.\n\
"));
}

/*!*********************************************************************
\brief prints the CSV header of the measurement output
***********************************************************************/
void PrintCSVHeader()
{
	Serial.println(F("tuC,tDut,I1,I2,..."));
}

/*!*********************************************************************
\brief main loop (executed in an endless loop by arduino)
***********************************************************************/
void loop()
{
	boolean doFifoRd = false;
	boolean doLastSample = false;

	// check for events from time slots
	if (LTC_TIMEOUT_CHECK(millis(), timeSlotFifoRd))
	{
		timeSlotFifoRd = millis() + timeDiffFifoRd;
		doFifoRd = true;
	}
	// we force doLastSample to happen in a doFifoRd slot!
	if (doFifoRd && LTC_TIMEOUT_CHECK(millis(), timeSlotLastSampleRd))
	{
		timeSlotLastSampleRd = millis() + timeDiffLastSampleRd;
		doLastSample = true;
	}

	// process any serial interface commands
	ProcessSerialCommands();

	// check if there is some time slot hit
	if (!ready || (!doFifoRd && !doLastSample))
		return; // not ready, or no time slot

	// ready to read measurement results

	String str;
	// report uC time
	str = String(millis() - timingReport);
	timingReport = millis();
	str += ',';

	ChkDeviceReady();

	// report slow, high precission results
	str += ReportHighPrecResults();

	if (doLastSample)
	{
		// report last fast samples via RDCV command
		str += ReportLastSample();
	}
	else
	{
		str += ',';
		str += ',';
	}

	// fifo is always reported
	str += ReportFifo(LTC2949_REG_FIFOI1); // report FIFOs
	str += ReportFifo(LTC2949_REG_FIFOI2); // report FIFOs

// report uC processing time
	str += String(millis() - timingReport);

	// output measurements via serial interface
	Serial.println(str);
}

/*!*********************************************************************
\brief Checks if the device as awake and in slow and fast continuous mode
This will restart the whole measurement e.g. in case the device was
powered off or there is a severe comminucation issue.
This can be demonstrated by just powering off the device or unplugging
the isoSPI interface while measurement is running.
***********************************************************************/
void ChkDeviceReady()
{
	byte data[10];
	byte  error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, data);

	// CRC error or device not yet ready?
	boolean initRequired =
		//((error & LTC2949_ERRCODE_PECERR_MASK) != 0) ||	   // PEC error (e.g. rebooting)
		(error != 0) ||	   // PEC error (e.g. rebooting)
		bitMaskSetChk(data[0], LTC2949_BM_OPCTRL_SLEEP) || // not ready after reboot
		bitMaskClrChk(data[0], LTC2949_BM_OPCTRL_CONT);	   // not in continuous mode

	if (!initRequired)
	{
		// also check if fast continuous mode is activated
		error = LTC2949_READ(LTC2949_REG_FACTRL, 1, data);
		initRequired =
			(error != 0) ||	   // PEC error (e.g. rebooting)
			!bitMaskSetChk(data[0], LTC2949_BM_FACTRL_CONF); // not in fast cont mode
	}

	if (initRequired)
	{
		if (error)
			PrintOkErr(error);
		Serial.println(F("WK,CONTGO"));
		delay(100);
		WakeUpReportStatus();
		Cont(true);
		Go(false, 5);
	}
	else
	{
		boolean expChkFailed;
		error = LTC2949_ReadChkStatusFaults(false, false, 10, data, &expChkFailed);
		if (expChkFailed || error != 0)
		{
			LTC2949_ReportFaults(data, 10);
			PrintComma();
			PrintOkErr(error);
			WakeUpReportStatus();
		}
	}
}

/*!*********************************************************************
\brief Reports high precission results if available
***********************************************************************/
String ReportHighPrecResults()
{
	String str;
	byte  error;
	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t deltaTint = LTC2949_GetLastTBxInt();

	// LTC2949_ChkUpdate checks for changed in TBx (default is TB4)
	if (LTC2949_ChkUpdate(&error))
	{
		// new slow channel, high precission results available
		// as an example we just report delta TBx and slow currents
		// calc difference between last TBx and current TBx, report milliseconds
		str += String((unsigned int)((LTC2949_GetLastTBxInt() - deltaTint) * (1.0e3 * LTC2949_LSB_TB1)));
		str += ',';
		byte buffer[3];
		// read slow current I1
		error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1, 6);
		str += ',';
		// read slow current I2
		error |= LTC2949_READ(LTC2949_VAL_I2, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I2, 6);
		if (error)
		{
			Serial.print('G');
			PrintOkErr(error);
		}
	}
	else
	{
		// no new values to be reported.
		str += ',';
		str += ',';
	}
	str += ',';
	return str;
}

/*!*********************************************************************
\brief Read and report the latest fast samples (fast I1/2)
***********************************************************************/
String ReportLastSample()
{
	String str = "";
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

	// read last conversion results via RDCV command
	byte error = LTC2949_ADxx();
	error |= LTC2949_PollFastData(fastData2949);
	if (error)
	{
		// error != 0 means some error occured
		Serial.print('B');
		PrintOkErr(error);
	}

	// generate output string
	// fast I1
	str += String(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FIFOI1, 6);
	str += ',';
	// fast I2
	str += String(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2, 6);
	str += ',';
	return str;
}

String ReportFifo(byte  addr)
{
	byte  error = 0;
	uint16_t len = maxSamplesFifoRead;
	String str = "";
	for (uint16_t samples;;)
	{
		// check for number of samples available:
		error |= LTC2949_RdFifoSampleCount(addr, &samples);
		str += samples;
		str += ',';
		if (samples == 0 || len == 0 || error)
		{
			PrintOkErr(LTC2949_ERRCODE_OTHER);
			return str;
		}
		break;
	}
	int16_t * buffer = new int16_t[len];
	boolean fifoFull1stSample;
	// read FIFO data
	error |= LTC2949_ReadFifo(
		/*byte  addr:                 */ addr,
		/*uint16_t *len:              */ &len,
		/*int16_t * samples:          */ buffer,
		/*boolean * fifoFull1stSample:*/ &fifoFull1stSample,
		/*int8_t rdOvrCntDown:        */LTC2949_RDFIFO_STOP_EMPTY);

	// no FIFO full allowed here!
	if (fifoFull1stSample)
	{
		// something went wrong
		Serial.print('C');
		Serial.print(addr);
		PrintComma();
		LTC_SERIAL_PRINT_BOOLD(fifoFull1stSample, 'F');
		PrintOkErr(LTC2949_ERRCODE_OTHER);
	}
	// data post processing (here just averaging the samples)
	int32_t  sumI;
	ProcessFastData(buffer, len, &sumI);

	delete[] buffer;

	if (error)
	{
		Serial.print('A');
		// error != 0 means some error occured
		PrintOkErr(error);
	}
	else if (len == 0)
	{
		// we read fifos every M milliseconds, so if we do not read any samples here, this is an error
		Serial.print('H');
		PrintOkErr(LTC2949_ERRCODE_OTHER);
	}

	// generate output string
	str += String(len);
	str += ',';
	str += String(sumI / (double)len * LTC2949_LSB_FIFOI1, 0);
	str += ',';
	return str;
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

/*!*********************************************************************
\brief Activate / deactivate measurement in LTC2949
- slow continuous mode
- fast continuous CH1&2 mode
***********************************************************************/
void Cont(boolean enable)
{
	byte  error = 0;
	if (enable)
	{
		// read, report & clear status
		error |= LTC2949_ReadChkStatusFaults(true, true);
		error |= LTC2949_GoCont(
			/*cfgFast:      */ LTC2949_BM_FACTRL_CONF,
			/*byte adcCfg:  */ LTC2949_BM_ADCCONF_P2ASV);
		PrintComma();
	}
	else
	{
		// go IDLE
		ready = false;
		LTC2949_WriteFastCfg(0);
		LTC2949_OpctlIdle();
	}
	if (error)
		Serial.print('D');
	PrintOkErr(error);
}

/*!*********************************************************************
\brief Enable read of measurement results
only if LTC2949 is in...
...slow continuous mode
...fast continuous CH1&2 mode
***********************************************************************/
void Go(boolean chkStatus, unsigned int iStart)
{
	// read configuration
	//   number of FIFO samples
	//   time between FIFO read
	//   time between last fast sample read
	if (SerialInputString.length() > iStart)
	{
		int _maxSamplesFifoRead = MAX_SAMPLES_PER_FIFO_READ;
		int _timeDiffFifoRd = TIME_DIFF_FIFO_RD;
		int _timeDiffLastSampleRd = TIME_DIFF_LAST_SAMPLE_RD;

		if (stringSplitter(SerialInputString, &iStart, &_maxSamplesFifoRead))
			if (stringSplitter(SerialInputString, &iStart, &_timeDiffFifoRd))
				stringSplitter(SerialInputString, &iStart, &_timeDiffLastSampleRd);

		maxSamplesFifoRead = _maxSamplesFifoRead;
		timeDiffFifoRd = _timeDiffFifoRd;
		timeDiffLastSampleRd = _timeDiffLastSampleRd;
	}
	if (timeDiffFifoRd < TIME_DIFF_MIN)
		timeDiffFifoRd = TIME_DIFF_MIN;
	if (timeDiffFifoRd > TIME_DIFF_FIFO_RD_MAX)
		timeDiffFifoRd = TIME_DIFF_FIFO_RD_MAX;
	if (timeDiffLastSampleRd < TIME_DIFF_MIN)
		timeDiffLastSampleRd = TIME_DIFF_MIN;
	// Note: For timeDiffLastSampleRd we do not need to restict the maximum time

	// init time stamps
	timeSlotLastSampleRd = millis() + timeDiffLastSampleRd;
	timeSlotFifoRd = millis() + timeDiffFifoRd;

	if (!chkStatus)
	{
		// ready for reading slow and fast results
		ready = true;
	}
	else
	{
		// check if LTC2949 is in slow & fast continuous mode

		byte 	error;
		byte opctrl, factrl;
		boolean tmp = LTC2949_DebugEnable;
		LTC2949_DebugEnable = true;
		error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &opctrl);
		error |= LTC2949_READ(LTC2949_REG_FACTRL, 1, &factrl);
		LTC2949_DebugEnable = tmp;

		if (bitMaskSetChk(opctrl, LTC2949_BM_OPCTRL_CONT) &&
			bitMaskSetChk(factrl, LTC2949_BM_FACTRL_FACONV | LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2))
		{
			// ready for reading slow and fast results
			ready = true;
		}
		else
		{
			// NOT ready for reading slow and fast results
			// issue CNT command first!
			ready = false;
			error |= LTC2949_ERRCODE_OTHER;
		}
		if (error)
			Serial.print('E');
		PrintOkErr(error);
	}

	Serial.print(maxSamplesFifoRead);
	PrintComma();
	Serial.print(timeDiffFifoRd);
	PrintComma();
	Serial.print(timeDiffLastSampleRd);
	PrintCSVHeader();
}

/*!*********************************************************************
\brief Process commands from serial interface
***********************************************************************/
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
		WakeUpReportStatus();
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("CNT")))
	{
		Cont(true); // continuous measurement
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("GO")))
	{
		Go(true, 2); // continuous measurement
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("CNTGO")))
	{
		Cont(true); // continuous measurement
		Go(false, 5);
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("IDL")))
	{
		Cont(false); // IDLE mode
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("RST")))
	{
		LTC2949_reset();
		delay(1500); // wait for LTC2949 going to sleep again
		PrintOkErr(0);
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("DB")))
	{
		DebugEnable();
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("SPI")))
	{
		ConfigSPI();
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

/*!*********************************************************************
\brief Configure SPI frequency
***********************************************************************/
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

/*!*********************************************************************
\brief debug enable / disable (communication data RAW output)
***********************************************************************/
void DebugEnable()
{
	Serial.println(LTC2949_DebugEnable = (toIntAuto(SerialInputString.substring(2)) != 0));
	PrintOkErr(0);
}

/*!*********************************************************************
\brief Wakeup LTC2949, report and clear all status / alert registers
***********************************************************************/
void WakeUpReportStatus()
{
	byte  error = LTC2949_WakeupAndAck();
	error |= LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	if (error)
		Serial.print('F');
	PrintOkErr(error);
}

