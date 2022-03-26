/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_FSSHT_RR.ino
* Tiny Linduino Sketch for LTC2949 Demo Board - fast init and fast single shot RR with minimum cycle time (1.2ms)
*
* created: Patrick Wilhelm (PW)
*/

#include <Arduino.h>
#include <SPI.h>
#include <ltcmuc_tools.h>
#include <LTC2949.h>

// DC2792A has two isoSPI ports: AUX and MAIN
#define LTCDEF_CS_AUX  9
#define LTCDEF_CS_MAIN 10
// in case of DC2792A select AUX or MAIN port
// default for all other isoSPI boards should be MAIN
#define LTCDEF_CS_SELECT LTCDEF_CS_MAIN


// serial baudrate
#define LTCDEF_BAUDRATE 250000

#define LOOP1 loop
#define LOOP2 loop_


// some more error codes, additionally to 
// the internal error codes from the LTC2949 library
// (e.g. LTC2949_ERRCODE_COMMERR... see LTC2949.h for
// error codes reported from library functions)
#define LTCDEF_ERRCODE_ADCCONF_RB   0x0100U
#define LTCDEF_ERRCODE_ABS0CHK      0x0200U
#define LTCDEF_ERRCODE_ABS1CHK      0x0400U
#define LTCDEF_ERRCODE_HS_SET_CHK   0x0800U
//#define LTCDEF_ERRCODE_HS_CLR_CHK   0x1000U
//#define LTCDEF_ERRCODE_STAT_FAULTS  0x2000U


// absolute tolerance used for voltage conversion result checks
#define LTCDEF_ABS_TOL_V 100e-3
// VREF2 limits for this test
#define LTCDEF_VREF2_LOW  (LTC2949_VREF2*0.9)
#define LTCDEF_VREF2_HIGH (LTC2949_VREF2*1.1)
#define LTCDEF_VREF2_INT_LOW  (LTCDEF_VREF2_LOW/LTC2949_LSB_FAUX)
#define LTCDEF_VREF2_INT_HIGH (LTCDEF_VREF2_HIGH/LTC2949_LSB_FAUX)
#define LTCDEF_VREF2_INT_NOM  (LTC2949_VREF2/LTC2949_LSB_FAUX)

boolean ChkAux(int16_t meas, int16_t low, int16_t high)
{
	if (meas > high)
		return false;
	if (meas < low)
		return false;
	return true;
}

void setup()
{
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// wait for serial port to connect. Needed for native USB port only
	while (!Serial)
		;
	Serial.print(F("INIT,"));
	PrintOkErr(Init());
	//example output line:
	Serial.println(F("TBD"));
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
	LTC2949_ADCConfigPasV(LTC2949_BM_ADCCONF_P2ASV); // enable P2ASV
	LTC2949_ADCConfigWrite(); // write ADCCONF
	error |= LTC2949_OpctlAdjUpd(false); // trigger adjust update (for activate changes within ADCCONF)
	error |= LTC2949_ADCConfigRead(&data); // read back ADCCONF
	if (data != LTC2949_BM_ADCCONF_P2ASV) // check ADCCONF value
		error |= LTCDEF_ERRCODE_ADCCONF_RB; // error when reading back ADCCONF

	LTC2949_OpctlCont(); // write CONT to OPCTRL (enable slow cont. mode)
	timeout = millis() + LTC2949_TIMING_IDLE2CONT2UPDATE; // set timeout, here: worst case time LTC2949 successfully operating in slow cont. mode.

	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);

	byte buffer[_cmd_max_len];
	LTC2949_FastSPI_wrfamux_vref2vsgnd_a(buffer);

	// the following loop is the fastest way to enable all ADCs:
	// continuously try to make fast single shot conversion. Once the
	// first conversion was triggered and read successfully (checked via
	// HS-Byte) we know everything is up and running.
	while (true)
	{
		unsigned long fastConversionEndTime;
		// clear HS byte
		//error |= LTC2949_RdFastData();
		// enable fast mode for I2 and AUX
		LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA);
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
	if (false == ChkAux(fastData2949[LTC2949_RDFASTDATA_AUX], LTCDEF_VREF2_INT_LOW, LTCDEF_VREF2_INT_HIGH))
		error |= LTCDEF_ERRCODE_ABS0CHK;
	return error;
}

#define LTCDEF_FSSHT_COUNT 1000
#define LTCDEF_FSSHT_TIME 1208
// true: read till 2nd PEC for RDCV commands; false: read only till first HS byte for RDCV commands
#define LTCDEF_RDCV_RD2PECS true
// true: read results after triggering new conversion. This way the time the host 
// waits for next conversion results can be used to read previous conversion results.
// drawback is that the HS-byte information is lost, as it is cleared with the ADCV
// false: read conversion results before triggering the next one. This way the 
// HS-byte can be checked.
#define LTCDEF_RDCV_POST_ADCV true

// define wait time (us) between conversions
// Those numbers were tweaked to incorporate SPI transaction times
// and to achieve minimum cycle time with Arduino Due
// Nominal value of LTC2949 between ADCV and conversion results ready is 1.2ms
#if (LTCDEF_RDCV_POST_ADCV == true)
#define LTCDEF_FSSHT_TIME 1150
#define LTCDEF_FSSHT_TIME 1300
#else
#define LTCDEF_FSSHT_TIME 1090
#endif

#if (LOOP2 == loop)
#define LTCDEF_FSSHT_COUNT 2000
#define LTCDEF_FSSHT_TIME 1204
#endif

uint16_t fsshtTime = LTCDEF_FSSHT_TIME;

boolean UpdateFsshtTime()
{
	if (!Serial.available())
		return false; // no new character, nothing to do

	String SerialInputString = "";
	while (true)
	{
		// Read from serial
		char character = Serial.read(); // Receive a single character from the software serial port
		if (!IsTermChar(character))
		{
			// Add the received character to the receive buffer (only if not \n or \r)
			SerialInputString.concat(character);
			continue; // command not yet terminated, nothing to do
		}

		SerialInputString.trim(); // remove any whitespaces

		if (SerialInputString.length() == 0)
			return false; // zero length command, nothing to do
		break;
	}

	fsshtTime = toIntAuto(SerialInputString);
	Serial.println(fsshtTime);
	return true;
}

void LOOP1()
{
	UpdateFsshtTime();
	uint16_t error = 0; // error codes variable
	byte bufferRdcv[_cmd_max_len]; // buffer for RDCV read command
	byte bufferOther[_cmd_wrfamux_vref2vsgnd_a_len]; // buffer for write and ADCV command
	unsigned long fastConversionEndTime; // holds time stamps for conversion results ready
	int16_t rawAdcVref2 = LTCDEF_VREF2_INT_NOM, rawAdcVref2Neg = -LTCDEF_VREF2_INT_NOM; // holds conversion results
	byte vref2hs0 = 0x0F, vref2neghs0 = 0x0F; // holds handshake byte

	LTC2949_FastSPI_wrfamux_gndvsvref2_250k_a(bufferOther); // select -VREF2 for first acquisition.

	unsigned long stopwatch = millis(); // start stop watch
	for (uint16_t i = 0; i < LTCDEF_FSSHT_COUNT; i++)
	{
		// +++++++++++++++++++++++++++++++++++++++++++++++++++
		// ++++++++++++++++++++++++ ADCV 1 +++++++++++++++++++
		// +++++++++++++++++++++++++++++++++++++++++++++++++++
		if (!LTCDEF_RDCV_POST_ADCV || i == 0)
		{
			LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion for -VREF2
			fastConversionEndTime = micros() + fsshtTime; // set time stamp for conversion results ready
		}
		if (i > 0) // skip for first iteration! See below
		{
			// those checks are for the previous conversion result.
			// checks / data processing is always done during the time the host
			// needs to wait for conversion results
			rawAdcVref2 = LTC2949_FastSPIDecode_RdcvGetAux(bufferRdcv); // get +VREF conversion result
			vref2hs0 = LTC2949_FastSPIDecode_RdcvGetHS0(bufferRdcv); // get handshake byte of +VREF conversion
			if (error |= ChkAux(rawAdcVref2, LTCDEF_VREF2_INT_LOW, LTCDEF_VREF2_INT_HIGH) ? 0 : LTCDEF_ERRCODE_ABS0CHK)
				break; // onversion result check failed
			if (error |= vref2hs0 == (LTCDEF_RDCV_POST_ADCV ? 0x00 : 0x0F) ? 0 : LTCDEF_ERRCODE_HS_SET_CHK)
				break; // Hs byte check failed (in case of LTCDEF_RDCV_POST_ADCV HS-byte must be cleared!)
			if (error |= LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4, 6) ? 0 : LTC2949_ERRCODE_PECERR_1) // check the first PEC
				break; // PEC check failed
			if (LTCDEF_RDCV_RD2PECS) // only check 2nd PEC if it was read
				if (error |= LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4 + 8, 6) ? 0 : LTC2949_ERRCODE_PECERR_1) // check the 2nd PEC
					break; // PEC check failed
		}
		LTC2949_FastSPI_wrfamux_vref2vsgnd_a(bufferOther); // select +VREF2 for the next conversion
		while (!LTC_TIMEOUT_CHECK(micros(), fastConversionEndTime))
			; // wait for conversion ready
		if (LTCDEF_RDCV_POST_ADCV)
		{
			LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion for +VREF2
			fastConversionEndTime = micros() + fsshtTime; // set time stamp for conversion results ready
		}
		if (LTCDEF_RDCV_RD2PECS)
			LTC2949_FastSPI_rdcvhs_a(bufferRdcv); // get conversion result, read until last PEC
		else
			LTC2949_FastSPI_rdcv1hs_a(bufferRdcv); // get conversion result, only read until 1st HS-byte
		// +++++++++++++++++++++++++++++++++++++++++++++++++++
		// ++++++++++++++++++++++++ ADCV 2 +++++++++++++++++++
		// +++++++++++++++++++++++++++++++++++++++++++++++++++
		if (!LTCDEF_RDCV_POST_ADCV)
		{
			LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion for +VREF2
			fastConversionEndTime = micros() + fsshtTime; // set time stamp for conversion results ready
		}
		rawAdcVref2Neg = LTC2949_FastSPIDecode_RdcvGetAux(bufferRdcv); // get -VREF conversion result
		vref2neghs0 = LTC2949_FastSPIDecode_RdcvGetHS0(bufferRdcv); // get handshake byte of -VREF conversion
		if (error |= ChkAux(rawAdcVref2Neg, -LTCDEF_VREF2_INT_HIGH, -LTCDEF_VREF2_INT_LOW) ? 0 : LTCDEF_ERRCODE_ABS1CHK)
			break; // onversion result check failed
		if (error |= vref2neghs0 == (LTCDEF_RDCV_POST_ADCV ? 0x00 : 0x0F) ? 0 : LTCDEF_ERRCODE_HS_SET_CHK)
			break; // Hs byte check failed (in case of LTCDEF_RDCV_POST_ADCV HS-byte must be cleared!)
		if (error |= LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4, 6) ? 0 : LTC2949_ERRCODE_PECERR_0) // check the first PEC
			break; // PEC check failed
		if (LTCDEF_RDCV_RD2PECS) // only check 2nd PEC if it was read
			if (error |= LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4 + 8, 6) ? 0 : LTC2949_ERRCODE_PECERR_1) // check the 2nd PEC
				break; // PEC check failed
		LTC2949_FastSPI_wrfamux_gndvsvref2_250k_a(bufferOther); // select +VREF2 for the next conversion
		while (!LTC_TIMEOUT_CHECK(micros(), fastConversionEndTime))
			; // wait for conversion ready
		if (LTCDEF_RDCV_POST_ADCV)
		{
			LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion for -VREF2
			fastConversionEndTime = micros() + fsshtTime; // set time stamp for conversion results ready
		}
		if (LTCDEF_RDCV_RD2PECS)
			LTC2949_FastSPI_rdcvhs_a(bufferRdcv); // get conversion result, read until last PEC
		else
			LTC2949_FastSPI_rdcv1hs_a(bufferRdcv); // get conversion result, only read until 1st HS-byte
	}
	stopwatch = millis() - stopwatch;

	Serial.print(stopwatch / (2.0e-3 * LTCDEF_FSSHT_COUNT));
	Serial.print(',');
	Serial.print(rawAdcVref2 * LTC2949_LSB_FAUX);
	Serial.print(',');
	Serial.print(rawAdcVref2Neg * LTC2949_LSB_FAUX);
	Serial.print(',');
	Serial.print(vref2hs0, HEX);
	Serial.print(',');
	Serial.print(vref2neghs0, HEX);
	Serial.print(',');
	PrintOkErr(error);
	if (error)
		while (!UpdateFsshtTime())
			;
}


void LOOP2()
{
	//Serial.println(_cmd_max_len);
	//Serial.println(_cmd_wrfamux_vref2vsgnd_a_len);

	UpdateFsshtTime();
	
	uint16_t error = 0; // error codes variable
	byte bufferRdcv[_cmd_max_len]; // buffer for RDCV read command
	byte bufferOther[_cmd_wrfamux_vref2vsgnd_a_len]; // buffer for write and ADCV command
	unsigned long fastConversionEndTime; // holds time stamps for conversion results ready
	int16_t rawAdcVref2Neg = 0x7FFF, rawAdcVref2 = 0x7FFF;
	bool vrefPolarity = false;
	unsigned long stopwatch;
	
	delay(350);
	
	Serial.print((millis()/1000)%100);
	Serial.print(',');
	
	LTC2949_READ(LTC2949_VAL_TEMP,2,bufferOther);
	Serial.print(LTC_2BytesToInt16(bufferOther) * LTC2949_LSB_TEMP, 2);
	Serial.print(',');
	

	LTC2949_FastSPI_wrfamux_gndvsvref2_250k_a(bufferOther); // select -VREF2 for first acquisition.
	LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion for -VREF2
	fastConversionEndTime = micros() + fsshtTime;
	//LTC2949_FastSPI_wrfamux_vref2vsgnd_a(bufferOther); // select +VREF2 for the next conversion

	// this is the initial condition for making the right timing measurements. ADCV was send and data is ready to be read
	stopwatch = micros(); // start stop watch
	for (uint16_t i = 0; i < LTCDEF_FSSHT_COUNT; i++)
	{
		// ++++++++++++++++++++ MUXCFG +++++++++++++++++++
		if (vrefPolarity)
			LTC2949_FastSPI_wrfamux_gndvsvref2_250k_a(bufferOther); // select -VREF2 for next acquisition.
		else
			LTC2949_FastSPI_wrfamux_vref2vsgnd_a(bufferOther); // select +VREF2 for the next conversion

		while (!LTC_TIMEOUT_CHECK(micros(), fastConversionEndTime))
			; // wait for conversion ready

		// ++++++++++++++++++++ ADCV +++++++++++++++++++++
		LTC2949_FastSPI_adcv_a(bufferOther); // trigger conversion
		fastConversionEndTime = micros() + fsshtTime; // set time stamp for conversion results ready
		// ++++++++++++++++++++ RDCV +++++++++++++++++++++
		LTC2949_FastSPI_rdcvhs_a(bufferRdcv); // get previous conversion result, read until last PEC
		// ++++++++++++++++++++ DATA processing ++++++++++++
		// check PEC
		if (!LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4, 6))
			error |= LTC2949_ERRCODE_PECERR_0;
		if (!LTC2949_FastSPIDecode_ChkPec(bufferRdcv + 4 + 8, 6))
			error |= LTC2949_ERRCODE_PECERR_1;
		// check conversion result
		if (vrefPolarity)
		{
			rawAdcVref2 = LTC2949_FastSPIDecode_RdcvGetAux(bufferRdcv);
			if (!ChkAux(rawAdcVref2, LTCDEF_VREF2_INT_LOW, LTCDEF_VREF2_INT_HIGH))
				error |= LTCDEF_ERRCODE_ABS0CHK;
		}
		else
		{
			rawAdcVref2Neg = LTC2949_FastSPIDecode_RdcvGetAux(bufferRdcv);
			if (!ChkAux(rawAdcVref2Neg, -LTCDEF_VREF2_INT_HIGH, -LTCDEF_VREF2_INT_LOW))
				error |= LTCDEF_ERRCODE_ABS0CHK;
		}
		if (error)
			break;
		vrefPolarity = !vrefPolarity; // flip polarity
	}
	stopwatch = micros() - stopwatch;

	Serial.print(stopwatch / (1.0*LTCDEF_FSSHT_COUNT));
	Serial.print(',');
	Serial.print(rawAdcVref2 * LTC2949_LSB_FAUX);
	Serial.print(',');
	Serial.print(rawAdcVref2Neg * LTC2949_LSB_FAUX);
	Serial.print(',');
	PrintOkErr(error);
	if (error)
		while (!UpdateFsshtTime())
			;
}

/*
Some other way to have BAT / Current guaranteed every 1.2 ms, but RR over several AUX
channels longer (due to dummy AUX measurements, see below)

Make the RDCV always direct after the ADCV. This way the ADCV are always at distances of 1.2ms.
Drawback is that the HS-Byte information is lost (HS-Byte is always cleared with the ADCV, the
conversion results remain and are only updated at the end of a conversion).

The problem is, without the HS-Byte you don�t know if the ADCV was effective.
E.g. due to disturbance in the communication it could happen, that the ADCV is ignored.
To overcome this you could add dummy measurements with AUX set to GND-VREF2 or better GND-VREF
or some other signal, so some input signal that is for sure always different to any other measurement.

Sequence would look like this

1.	0ms:
	a.	ADCV
	b.	RDCV (Aux result = dummy channel, as this is the default MUX config, see below)
	c.	WRITE next MUX setting
	d.	READ next MUX setting
2.	1.2ms:
	a.	ADCV
	b.	RDCV (Aux result = �real� channel)
	c.	WRITE dummy MUX setting
	d.	READ dummy MUX setting �
3.	2.4ms:
	a.	ADCV
	b.	RDCV (Aux result = dummy channel)
	c.	WRITE next MUX setting
	d.	READ next MUX setting
4.	3.6ms:
	a.	ADCV
	b.	RDCV (Aux result = �real� channel)
	c.	WRITE dummy MUX setting
	d.	READ dummy MUX setting �
5.	�
6.	�
7.	xxx ms (last of a sequence / any unused AUX slot):
	a.	ADCV
	b.	RDCV (�.)
	c.	WRITE dummy MUX setting
	d.	READ dummy MUX setting

No delays (or the minimum CS idle time which is 1 us) between a-d.


*/
