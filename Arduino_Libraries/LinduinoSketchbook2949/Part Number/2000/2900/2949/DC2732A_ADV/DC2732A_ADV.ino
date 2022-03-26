/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_ADV.ino
* Linduino Sketch for LTC2949 Demo Board - Isolated Battery Meter for EV / HEV
*
* Advanced example for complex measurement cycles
*  round-robin over several inputs via fast single shot measurement, possibility to configure GPIOs and current sources per SLOT
*  short fast continuous measurement windows to increase resolution of fast current measurements
*  All AUX measurements done in fast mode only at update rate <100ms, thus slow channel is never updated and ignored
*   This means, following values from slow channel will never be updated:
*     BAT, TEMP, VCC, SLOT1, SLOT2, VREF
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
// define this for wired NTC glued
#define LTCDEF_NTC_WIRED
#undef LTCDEF_NTC_WIRED
#define LTCDEF_NTC_SLOT_DFLT
//#undef LTCDEF_NTC_SLOT_DFLT

#ifdef LTCDEF_NTC_WIRED
// NTC: Murata, NTHCG143/Murata (leaded)
#define NTC_STH_A  8.39126e-4
#define NTC_STH_B  2.08985e-4
#define NTC_STH_C  7.13241e-8
#elif defined(LTCDEF_NTC_SLOT_DFLT)
// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#else
#define NTC_STH_A	8.29036E-04
#define NTC_STH_B	2.09968E-04
#define NTC_STH_C	7.18289E-08
#endif // LTCDEF_NTC_WIRED

#define NTC_RREF   100e3

// number of digits to be printed via serial port for certain measurements
#define LTCDEF_DIGITS_I1SLOW      6
#define LTCDEF_DIGITS_P1SLOW      6
#define LTCDEF_DIGITS_BATFIFO_AVG 4
#define LTCDEF_DIGITS_AUXFIFO_AVG 4
#define LTCDEF_DIGITS_I2FIFO_AVG  6
#define LTCDEF_DIGITS_BATFAST     4
#define LTCDEF_DIGITS_AUXFAST     4
#define LTCDEF_DIGITS_I2FAST      5

// Cycle time for measurements
#define LTCDEF_CYCLE_TIME_SLOT_FAST 50U
// delay after 2nd slot
#define LTCDEF_DELAY_AFTER_SLOT 2
// delay 300 ms after 2nd slot (switch off) to have enough settling time
#define LTCDEF_DELAY_AFTER_SLOT_MS 300U
#undef LTCDEF_DELAY_AFTER_SLOT

unsigned long timeSlotFast = 0;
int64_t lastC1 = 0;


// Slot and GPIO, current source configuration:
// CURR_GPO5_CTRL: any combination of (see LTC2949.h)
//                  LTC2949_BM_FCURGPIOCTRL_GPO5CTRL0
//                  LTC2949_BM_FCURGPIOCTRL_GPO5CTRL1
//                  LTC2949_BM_FCURGPIOCTRL_MUXPCURPOL
//                  LTC2949_BM_FCURGPIOCTRL_MUXPCUREN
//                  LTC2949_BM_FCURGPIOCTRL_MUXNCURPOL
//                  LTC2949_BM_FCURGPIOCTRL_MUXNCUREN
// or any out of
// LTC2949_GPO5_HI,LTC2949_GPO5_LO,LTC2949_GPO5_TGL,LTC2949_GPO5_IN
// or'ed with any out of
// LTC2949_CURR_PUMUXP,LTC2949_CURR_PDMUXP,LTC2949_CURR_DISMUXP
// or'ed with any out of
// LTC2949_CURR_PUMUXN,LTC2949_CURR_PDMUXN,LTC2949_CURR_DISMUXN

// GPO1TO4_CTRL: any combination of (see LTC2949.h)
//                LTC2949_BM_FGPIOCTRL_GPO1CTRL0
//                LTC2949_BM_FGPIOCTRL_GPO1CTRL1
//                LTC2949_BM_FGPIOCTRL_GPO2CTRL0
//                LTC2949_BM_FGPIOCTRL_GPO2CTRL1
//                LTC2949_BM_FGPIOCTRL_GPO3CTRL0
//                LTC2949_BM_FGPIOCTRL_GPO3CTRL1
//                LTC2949_BM_FGPIOCTRL_GPO4CTRL0
//                LTC2949_BM_FGPIOCTRL_GPO4CTRL1
// or any out of
// LTC2949_GPO1_HI,LTC2949_GPO1_LO,LTC2949_GPO1_TGL,LTC2949_GPO1_IN
// or'ed with any out of
// LTC2949_GPO2_HI,LTC2949_GPO2_LO,LTC2949_GPO2_TGL,LTC2949_GPO2_IN
// or'ed with any out of
// LTC2949_GPO3_HI,LTC2949_GPO3_LO,LTC2949_GPO3_TGL,LTC2949_GPO3_IN
// or'ed with any out of
// LTC2949_GPO4_HI,LTC2949_GPO4_LO,LTC2949_GPO4_TGL,LTC2949_GPO4_IN
// 
// MUXP/MUXN: any out of
//   LTC2949_SLOTMUX_V1, LTC2949_SLOTMUX_V2, ... LTC2949_SLOTMUX_GND (see LTC2949.h)
#define LTCDEF_SLOT_GPIO_CURR_SRC_CFG(MUXP,MUXN,CURR_GPO5_CTRL,GPO1TO4_CTRL) CURR_GPO5_CTRL,GPO1TO4_CTRL,MUXN,MUXP

#define LTCDEF_FASTSLOTS_BYTES_PER_SLOT 4
#define LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_CURR_GPO5_CTRL 0
#define LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_GPO1TO4_CTRL   1
#define LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_MUXN            2
#define LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_MUXP            3

#define LTCDEF_FSLOTS_TERM_BYTE 0xFFU
#define LTCDEF_FSLOTS_TERM_SYMBOL LTCDEF_SLOT_GPIO_CURR_SRC_CFG(LTCDEF_FSLOTS_TERM_BYTE,LTCDEF_FSLOTS_TERM_BYTE,LTCDEF_FSLOTS_TERM_BYTE,LTCDEF_FSLOTS_TERM_BYTE)

// array of slot definitions
// define all channels to be measured in fast mode here:
// Format is: Pa,Na,FCURGPIOCTRLa,FGPIOCTRLa,Pb,Nb,FCURGPIOCTRLb,FGPIOCTRLb,....,0xFF
// Pa: positive input of slot A
// Na: negative input of slot A
// FCURGPIOCTRLa: value of register LTC2949_REG_FCURGPIOCTRL for slot A
// FGPIOCTRLa:  value of register LTC2949_REG_FGPIOCTRL for slot A
// ...
// 0xFF: termination symbol (must be the last element!)
// every slot is measure via a fast single shot measurement that takes a maximum of 1.2 ms.
// The total time it takes to make all measurements must not exceed the cycle time LTCDEF_CYCLE_TIME_SLOT_FAST defined above!
#ifdef LTCDEF_NTC_SLOT_DFLT
static const byte fastSlots[] PROGMEM = { // SLOT: CONFIG
	// SLOT0:	
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		// MUX: V1 - GND (NTC1)
		LTC2949_SLOTMUX_V1,	  // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT1:	
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		// MUX: V2 - GND (NTC2)
		LTC2949_SLOTMUX_V2,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT2:	
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		// MUX:  V3 - GND (connect VREF to V3 externally! used for NTC temperature measurement)
		LTC2949_SLOTMUX_V3,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT3:	
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		// MUX: VREF2 - GND
		LTC2949_SLOTMUX_VREF2,   // MUXP
		LTC2949_SLOTMUX_GND,     // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT4:	
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		// MUX: VBATP - VBATM
		LTC2949_SLOTMUX_VBATP,   // MUXP
		LTC2949_SLOTMUX_VBATM,   // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance
	LTCDEF_FSLOTS_TERM_SYMBOL };
#else
static const byte fastSlots[] PROGMEM = { // SLOT: CONFIG
	// SLOT 0 (NEW!): Vchassis, switch (V10/GPO3) closed
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V3,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_HI | LTC2949_GPO4_IN), // GPO3 output HIGH, others high impedance

	// SLOT 1:
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V6,  // MUXP
		LTC2949_SLOTMUX_V7,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 2:
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V8,  // MUXP
		LTC2949_SLOTMUX_V9,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 3:
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V1,  // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 4:
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V2,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 5: Vchassis, switch open (input should have been settled here. To be verified. Add some dummy measurements, if not settled)
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V3,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 6: Vchassis, switch open (repeated to see if it is already settled)
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V3,   // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_IN | LTC2949_GPO4_IN), // GPO1-4 high impedance

	// SLOT 7: Note: here we already close the switch by setting GPO3 HIGH, to have the input settled with 1st measurement of next cycle
	LTCDEF_SLOT_GPIO_CURR_SRC_CFG(
		LTC2949_SLOTMUX_V4,  // MUXP
		LTC2949_SLOTMUX_GND,  // MUXN
		// GPIO / current sources:
		LTC2949_GPO5_IN | LTC2949_CURR_DISMUXN | LTC2949_CURR_DISMUXP, // GPO5 high impedance, all current sources disabled
		LTC2949_GPO1_IN | LTC2949_GPO2_IN | LTC2949_GPO3_HI | LTC2949_GPO4_IN), // GPO3 output HIGH, others high impedance

	LTCDEF_FSLOTS_TERM_SYMBOL };
#endif
// TODO: maybe change code to have one cycle with switch enabled and one cycle with switch disables....
// 1 second switch enabled
// 1 second switch disabled


#ifdef LTCDEF_NTC_SLOT_DFLT
// NTC configuration
// define the fast slot that measures NTC1
#define LTCDEF_FAST_SLOT_VNTC1 0
// define the fast slot that measures NTC2
#define LTCDEF_FAST_SLOT_VNTC2 1
// define the fast slot that measures VREF
#define LTCDEF_FAST_SLOT_VREF  2
#else
// NTC configuration
// define the fast slot that measures NTC1
#define LTCDEF_FAST_SLOT_VNTC1 3
// define the fast slot that measures NTC2
#define LTCDEF_FAST_SLOT_VNTC2 4
// define the fast slot that measures VREF
#define LTCDEF_FAST_SLOT_VREF  7
#endif

#if defined LTCDEF_FAST_SLOT_VREF && defined LTCDEF_FAST_SLOT_VNTC1
#define LTCDEF_NTC1_CALC_ENABLED
#else
#undef LTCDEF_NTC1_CALC_ENABLED
#endif
#if defined LTCDEF_FAST_SLOT_VREF && defined LTCDEF_FAST_SLOT_VNTC2
#define LTCDEF_NTC2_CALC_ENABLED
#else
#undef LTCDEF_NTC2_CALC_ENABLED
#endif

// enable/disable NTC calculation in host MCU
//#undef LTCDEF_NTC1_CALC_ENABLED
//#undef LTCDEF_NTC2_CALC_ENABLED

// define default AUX slot, that is measured during fast continuous periods
#define LTCDEF_FASTAUX_DEFAULT_P LTC2949_SLOTMUX_VREF2
#define LTCDEF_FASTAUX_DEFAULT_N LTC2949_SLOTMUX_GND

// Note: getAt is needed on Linduino to access PROGMEM. Depending on used controller it could be replaced by e.g. #define getAt(A,I) (A)[I]
byte getAt(const byte * arr, byte i)
{
	PGM_P p = reinterpret_cast<PGM_P>(arr);
	byte c = pgm_read_byte(p + i);
	return c;
}
void copyTo(const byte * arr, byte i, byte * dest, uint8_t len)
{
	PGM_P p = reinterpret_cast<PGM_P>(arr);
	for (uint8_t l = 0; l < len; l++)
	{
		dest[l] = pgm_read_byte(p + i);
		i++;
	}
}

/*!*********************************************************************
\brief prints the CSV header of the measurement output
***********************************************************************/
void PrintCSVHeader()
{
	/*
	dT: delta time in milliseconds between two subsequent updates of TB1
		must be between 99-101 ms, its also a diagnostic check showing ADCs run at the right clock speed
	dC: delta charge in uVs between two subsequent updates of C1
	I1: 100ms slow channel current measurement
	P1: 100ms slow channel power measurement
	nI2: number of I2 fast samples read from FIFO used to calculate average current avgI2
	avgI2: average current from FIFO
	nBAT: number of BAT fast samples read from FIFO used to calculate average voltage avgBAT
	avgBAT: average voltage BAT from FIFO
	nAUX: number of AUX fast samples read from FIFO used to calculate average voltage avgAUX
	avgAUX: average voltage AUX from FIFO (channel defined by LTCDEF_FASTAUX_DEFAULT_P/_N)
	...
	Following columns are in the format Vp-Vn, e.g. V1-V0 for V1 input versus GND.
	Those are the user defined slots.
	Note: for every user defined slot a fast single shot measurement is performed
		  that converts Vp-Vn (this is the user defined slot), I2 and BAT
	...
	xI2: Average of I2 current measurements from all fast single measurements of all user defined slots
	xBAT: Average of BAT voltage measurements from all fast single measurements of all user defined slots
	tNTC1: NTC1 temperature measurement (optional, see LTCDEF_NTC1_CALC_ENABLED)
	tNTC2: NTC2 temperature measurement (optional, see LTCDEF_NTC2_CALC_ENABLED)
	ERR/OK: report of error code
	*/
	Serial.print(F("dT,dC,I1,P1,nI2,avgI2,nBAT,avgBAT,nAUX,avgAUX,"));

	for (uint8_t i = 0; ; i += LTCDEF_FASTSLOTS_BYTES_PER_SLOT)
	{
		byte muxp = getAt(fastSlots, i + LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_MUXP);
		if (muxp == LTCDEF_FSLOTS_TERM_BYTE)
			break;
		byte muxn = getAt(fastSlots, i + LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_MUXN);
		uint16_t currGpio =
			getAt(fastSlots, i + LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_CURR_GPO5_CTRL) |
			getAt(fastSlots, i + LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_GPO1TO4_CTRL) << 8;
		if (currGpio)
			Serial.print('g');
		Serial.print('V');
		Serial.print(muxp, DEC);
		Serial.print('-');
		Serial.print('V');
		Serial.print(muxn, DEC);
		Serial.print(',');
	}
	Serial.print(F("xI2,xBAT,"));
#ifdef LTCDEF_NTC1_CALC_ENABLED
	Serial.print(F("tNTC1,"));
#endif
#ifdef LTCDEF_NTC2_CALC_ENABLED
	Serial.print(F("tNTC2,"));
#endif
	Serial.println(F("ERR/OK"));
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
		; // wait for serial port to connect. Needed for native USB port only
	PrintCSVHeader();
	while (!LTC_TIMEOUT_CHECK(millis(), startOfTheDay))
		;
	byte error = WakeUpReportStatus();
	error |= Cont(false); // Cont mode will be enabled later
	delay(LTC2949_TIMING_CONT_CYCLE);
}

byte ReportSlowData(uint32_t lastTB1, String * reportStr)
{
	byte data[12];
	// new slow channel, high precision results available
	// report I1, P1, time and charge delta

	// calc difference between last TB1 and current TB1, report milliseconds
	reportStr[0] += (unsigned int)((LTC2949_GetLastTBxInt() - lastTB1) * (1.0e3 * LTC2949_LSB_TB1));
	reportStr[0] += ',';
	// charge
	byte error = LTC2949_READ(LTC2949_VAL_C1, 6, data);
	int64_t currentC1 = LTC_6BytesToInt64(data);
	// calc difference between last C1 and current C1 and report mVms / uVs
	reportStr[0] += (int)((currentC1 - lastC1) * (1.0e6 * LTC2949_LSB_C1));
	reportStr[0] += ',';
	lastC1 = currentC1;

	// read slow current I1
	error |= LTC2949_READ(LTC2949_VAL_I1, 3, data);
	reportStr[0] += String(LTC_3BytesToInt32(data) * LTC2949_LSB_I1, LTCDEF_DIGITS_I1SLOW);
	reportStr[0] += ',';

	// read slow current P1
	error |= LTC2949_READ(LTC2949_VAL_P1, 3, data);
	reportStr[0] += String(LTC_3BytesToInt32(data) * LTC2949_LSB_P1, LTCDEF_DIGITS_P1SLOW);
	reportStr[0] += ',';
	return error;
}

void loop()
{
	byte  error;
	String reportStr = "";

	// check if device is awake and configured correctly
	if ((error = ChkDeviceReady()) != 0)
	{
		PrintOkErr(error);
		delay(LTC2949_TIMING_CONT_CYCLE);
		return;
	}

	// check fast slot cycle
	// Force fast cycle in case there was slow update!
	if (!LTC_TIMEOUT_CHECK(millis(), timeSlotFast) /*&& !slowChannelDataPrinted*/)
		return; // net yet
	// next fast update time-stamp
	timeSlotFast = millis() + LTCDEF_CYCLE_TIME_SLOT_FAST;

	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t lastTB1 = LTC2949_GetLastTBxInt();

	// LTC2949_ChkUpdate checks for changed in TB1
	if (LTC2949_ChkUpdate(&error, NULL, LTC2949_VAL_TB1) /*&& !slowChannelDataPrinted*/)
	{
		error |= ReportSlowData(lastTB1, &reportStr);
	}
	// suspend fast continuous mode
	LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA);

	//////////////////////////////////
	// read all samples from FIFOs  //
	//////////////////////////////////
	uint16_t samples;
	int32_t  sum;

	// I2
	error |= ReportFifo(LTC2949_REG_FIFOI2, &samples, &sum);
	if (reportStr.length() == 0)
	{
		// placeholder as no slow channel results were printed
		reportStr += F(",,,,");
	}
	// print number of samples available
	reportStr += samples;
	reportStr += ',';
	// print average of FIFOs
	reportStr += String(sum / (double)samples * LTC2949_LSB_FI2, LTCDEF_DIGITS_I2FIFO_AVG);
	reportStr += ',';

	// BAT
	error |= ReportFifo(LTC2949_REG_FIFOBAT, &samples, &sum);
	reportStr += samples;
	reportStr += ',';
	reportStr += String(sum / (double)samples * LTC2949_LSB_FBAT, LTCDEF_DIGITS_BATFIFO_AVG);
	reportStr += ',';

	// AUX
	error |= ReportFifo(LTC2949_REG_FIFOAUX, &samples, &sum);
	reportStr += samples;
	reportStr += ',';
	reportStr += String(sum / (double)samples * LTC2949_LSB_FAUX, LTCDEF_DIGITS_AUXFIFO_AVG);
	reportStr += ',';

	//////////////////////////////////
	// fast single shots            //
	//////////////////////////////////

	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	// with every single shot AUX we also get BAT and I2
	// as an example we just sum up those measurements
	int32_t sumBat = 0;
	sum = 0;

	// make sure HS byte is cleared
	// Note: we could even use the data reported here, but we already reported the average of all FIFOs above
	// so we ignore this set of fast conversion results
	error |= LTC2949_RdFastData();

	// cycle through all defined fast slots
	// = software defined fast round-robin sequence
	byte muxi = 0;
#ifdef LTCDEF_NTC1_CALC_ENABLED
	int16_t ntc1 = 0x8000;
#endif
#ifdef LTCDEF_NTC2_CALC_ENABLED
	int16_t ntc2 = 0x8000;
#endif
#if defined LTCDEF_NTC1_CALC_ENABLED || defined LTCDEF_NTC2_CALC_ENABLED
	int16_t vref = 0x8000;
#endif

	// cycle through all fast slots
	while (true)
	{
		byte gpioCurrMuxCfg[LTCDEF_FASTSLOTS_BYTES_PER_SLOT];

		copyTo(fastSlots, muxi, gpioCurrMuxCfg, LTCDEF_FASTSLOTS_BYTES_PER_SLOT);

		if (gpioCurrMuxCfg[LTCDEF_SLOT_GPIO_CURR_SRC_CFG_I_MUXN] == LTCDEF_FSLOTS_TERM_BYTE)
			break;

		muxi += LTCDEF_FASTSLOTS_BYTES_PER_SLOT;

		// write MUX and GPIO and current source control
		LTC2949_WRITE(LTC2949_REG_FCURGPIOCTRL, 4, gpioCurrMuxCfg);

		// ...trigger measurement
		error |= LTC2949_ADxx();

#if defined(LTCDEF_DELAY_AFTER_SLOT_MS) & defined(LTCDEF_DELAY_AFTER_SLOT)
		if (muxi == LTCDEF_DELAY_AFTER_SLOT * LTCDEF_FASTSLOTS_BYTES_PER_SLOT)
		{
			delay(LTCDEF_DELAY_AFTER_SLOT_MS);
			reportStr += ' ';
		}
#endif

		// ...poll LTC2949 for conversion done
		error |= LTC2949_PollFastData(fastData2949);
		sum += fastData2949[LTC2949_RDFASTDATA_I2];
		sumBat += fastData2949[LTC2949_RDFASTDATA_BAT];
		// ...print conversion result
		reportStr += String(fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX, LTCDEF_DIGITS_AUXFAST);
		reportStr += ',';
		// store NTC related measurements for later usage
		if (false)
			;
#ifdef LTCDEF_NTC1_CALC_ENABLED
		else if (muxi == (LTCDEF_FASTSLOTS_BYTES_PER_SLOT * (1 + LTCDEF_FAST_SLOT_VNTC1)))
			ntc1 = fastData2949[LTC2949_RDFASTDATA_AUX];
#endif
#ifdef LTCDEF_NTC2_CALC_ENABLED
		else if (muxi == (LTCDEF_FASTSLOTS_BYTES_PER_SLOT * (1 + LTCDEF_FAST_SLOT_VNTC2)))
			ntc2 = fastData2949[LTC2949_RDFASTDATA_AUX];
#endif
#if defined LTCDEF_NTC1_CALC_ENABLED || defined LTCDEF_NTC2_CALC_ENABLED
		else if (muxi == (LTCDEF_FASTSLOTS_BYTES_PER_SLOT * (1 + LTCDEF_FAST_SLOT_VREF)))
			vref = fastData2949[LTC2949_RDFASTDATA_AUX];
#endif
	}
	// select default channel
	LTC2949_SlotFastCfg(LTCDEF_FASTAUX_DEFAULT_P, LTCDEF_FASTAUX_DEFAULT_N);

	// enable fast continuous mode for next cycle
	LTC2949_WriteFastCfg(LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA | LTC2949_BM_FACTRL_FACONV);

	// calc number of fast measurements, that were performed
	muxi /= LTCDEF_FASTSLOTS_BYTES_PER_SLOT;

	// print sum of I2, BAT
	reportStr += String(sum * LTC2949_LSB_FI2 / muxi, LTCDEF_DIGITS_I2FAST);
	reportStr += ',';
	reportStr += String(sumBat * LTC2949_LSB_FBAT / muxi, LTCDEF_DIGITS_BATFAST);
	reportStr += ',';

#ifdef LTCDEF_NTC1_CALC_ENABLED
	reportStr += CalcNTCTemp(vref, ntc1);
	reportStr += ',';
#endif
#ifdef LTCDEF_NTC2_CALC_ENABLED
	reportStr += CalcNTCTemp(vref, ntc2);
	reportStr += ',';
#endif

	Serial.print(reportStr);

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

byte ReportFifo(uint8_t addr, uint16_t * samplesRead, int32_t * sum)
{
	byte error = 0;
#ifdef debug_samplesavailable
	uint16_t samplesAvailable;
	error = LTC2949_RdFifoSampleCount(addr, &samplesAvailable);
	if ((samplesAvailable == 0) || (error != 0))
		return error;
#endif

	// we reserve double the amount of samples we typically have
	const uint8_t maxSamplesToBeRead = min(LTCDEF_CYCLE_TIME_SLOT_FAST / 0.8 + 4, 32);// 4 samples headroom
#ifdef debug_samplesavailable
	samplesRead[0] = min(maxSamplesToBeRead, samplesAvailable);
#else
	samplesRead[0] = maxSamplesToBeRead;
#endif
	int16_t buffer[maxSamplesToBeRead];
	// read samples
	error |= LTC2949_ReadFifo(
		/*byte      addr:             */ addr,
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

byte ChkDeviceReady()
{
	byte data[10];
	byte error = 0;
	boolean expChkFailed;

	if (
		// read OPCTRL and check for PEC error
		((error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, data)) != 0) ||

		// check if continuous mode is enabled
		(data[0] != LTC2949_BM_OPCTRL_CONT) ||

		// read FACTRL and check for PEC error
		((error = LTC2949_READ(LTC2949_REG_FACTRL, 1, data)) != 0) ||

		// check if correct fast mode is enabled
		((data[0] & ~LTC2949_BM_FACTRL_FACONV) != (LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)) ||

		// read ADCCONF and check for PEC error
		((error = LTC2949_ADCConfigRead(data)) != 0) ||

		// check if ADCCONF is set correctly
		(data[0] != LTC2949_BM_ADCCONF_P2ASV) ||

		// check status and faults registers for any failure
		((error = LTC2949_ReadChkStatusFaults(
			/*boolean lockMemAndClr:    */ false,
			/*boolean printResult:      */ false,
			/*byte len:                 */ 10,
			/*byte * statFaultsExpAndRd:*/ data,
			/*boolean * expChkFailed:   */ &expChkFailed,
			/*byte expDefaultSet):      */ LTC2949_STATFAULTSCHK_IGNORE_STATUPD | LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR)) != 0) || expChkFailed
		)
	{
		Serial.print('R');
		PrintComma();
		error |= WakeUpReportStatus();
		error |= Cont(false); // go IDLE
		delay(LTC2949_TIMING_CONT_CYCLE);
		error |= Cont(true);
		timeSlotFast = millis() + LTCDEF_CYCLE_TIME_SLOT_FAST;
	}
	return error;
}

byte WakeUpReportStatus()
{
	byte  error = LTC2949_WakeupAndAck();
	error |= LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	PrintOkErr(error);
	return error;
}

byte Cont(boolean enable)
{
	// disable all GPIOs and current sources
	// this sets the default configuration in case
	// something different was configured before
	// Note: This function does not yet write the configuration. 
	// See LTC2949_GpioCurrConfigWrite, which does the write.
	LTC2949_GpioCurrConfigClr();
	// only following function will write the actual GPIO configuration to LTC2949!
	LTC2949_GpioCurrConfigWrite();

	if (enable)
	{
		byte error = 0;
		// fast slot, some default setting, will be overwritten later...
		LTC2949_SlotFastCfg(LTCDEF_FASTAUX_DEFAULT_P, LTCDEF_FASTAUX_DEFAULT_N);

		uint8_t  revCode;
		error |= LTC2949_GetSiliconRevision(&revCode);
		Serial.print(F("v0x"));
		Serial.println(revCode, HEX);

		// read & clear status
		error |= LTC2949_ReadChkStatusFaults(true, false);

		// enable measurement
		return error | LTC2949_GoCont(
			/*cfgFast:      */ LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA,
			/*byte adcCfg:  */ LTC2949_BM_ADCCONF_P2ASV);
	}

	LTC2949_WriteFastCfg(0);
	LTC2949_OpctlIdle();
	return 0;
}

// calculate temperature of NTC via Steinhart–Hart equation
float CalcNTCTemp(
	int16_t vref, //!< Integer of AUX fast VREF measurement (the nominal value is 3.0*
	int16_t vntc  //!< 
)
{
	return LTC2949_CalcNTCTemp(
		/*float vref:*/vref,
		/*float vntc:*/vntc,
		/*float rRef:*/NTC_RREF,
		/*float ntcA:*/NTC_STH_A,
		/*float ntcB:*/NTC_STH_B,
		/*float ntcC:*/NTC_STH_C
	);
}







///////// byte gpioConfig = (muxp & 0xE0U) >> 2 | (muxn & 0xE0U) >> 5;
///////// for (byte iGpio = 1; iGpio <= 5; iGpio++)
///////// {
///////// 	LTC2949_GpioConfig(
///////// 		/*byte gpio:*/ iGpio,
///////// 		/*byte mode:*/ (gpioConfig & 0x01) ? LTC2949_BM_GPIOCFG_OUPUT_HIGH : LTC2949_BM_GPIOCFG_INPUT);
///////// 	gpioConfig >>= 1;
///////// }
////////// // Slot configuration:
////////// // GPIOA, GPIOB: up to two GPIOs that should be 
////////// //             driven high before measurement
////////// //			   of the related slot.
////////// //             set to 0 if not used
////////// // Note: Only the GPIO defined will be driven high
////////// //       all others will be high impedance
////////// // MUXP,MUXN: positive and negative mux input
////////// //            of the related slot
////////// #define LTCDEF_SLOT_GPIO_CFG(GPIOA,GPIOB,MUXP,MUXN) ((GPIOA)<<5)|(MUXP),((GPIOB)<<5)|(MUXN)
////////// #define LTCDEF_SLOT_GPIO_GET(SLOTCFG) ((SLOTCFG)>>5)
////////// #define LTCDEF_SLOT_CURR_SRC_EN 0x80U
////////// #define LTCDEF_SLOT_CURR_SRC_CFG(POS,NEG,MUXP,MUXN) ((POS) ? (LTCDEF_SLOT_CURR_SRC_EN | (((POS)<<1) & 0x60)) : 0)|(MUXP),((GPIOB)<<5)|(MUXN)

//// // array of slot definitions
//// // define all channels to be measured in fast mode here:
//// // Format is: Pa,Na,Pb,Nb,....,0xFF
//// // Pa: positive input of slot a	(bits 7..5 used for GPIO configuration, see above)
//// // Na: negative input of slot a	(bits 7..5 used for GPIO configuration, see above)
//// // ...
//// // 0xFF: termination symbol (must be the last element!)
//// // every slot is measure via a fast single shot measurement that takes a maximum of 1.2 ms.
//// // The total time it takes to make all measurements must not exceed the cycle time LTCDEF_CYCLE_TIME_SLOT_FAST defined above!
//// static const byte fastSlots[] PROGMEM = {                                   // SLOT: CONFIG
//// 	LTCDEF_SLOT_GPIO_CFG(0, 0, LTC2949_SLOTMUX_V1,    LTC2949_SLOTMUX_GND),      // 0:    V1    - GND (NTC1)
//// 	LTCDEF_SLOT_GPIO_CFG(0, 0, LTC2949_SLOTMUX_V2,    LTC2949_SLOTMUX_GND),      // 1:    V2    - GND (NTC2)
//// 	LTCDEF_SLOT_GPIO_CFG(0, 0, LTC2949_SLOTMUX_V3,    LTC2949_SLOTMUX_GND),      // 2:    V3    - GND (connect VREF to V3 externally!)
//// 	LTCDEF_SLOT_GPIO_CFG(0, 0, LTC2949_SLOTMUX_VREF2, LTC2949_SLOTMUX_GND),      // 3:    VREF2 - GND
//// 	LTCDEF_SLOT_GPIO_CFG(0, 0, LTC2949_SLOTMUX_VBATP, LTC2949_SLOTMUX_VBATM),    // 4:    VBATP - GND
//// 	LTCDEF_FSLOTS_TERM_SYMBOL };


		/////   // as an example we enable output drivers on GPO1 (V8) and GPO2 (V9)
		/////   // see \LTSketchbook\libraries\LTC2949\LTC2949.h for possible pin modes (LTC2949_BM_GPIOCFG_...)
		/////   LTC2949_GpioConfig(
		/////   	/*byte gpio:*/ 1,
		/////   	/*byte mode:*/ enable ? LTC2949_BM_GPIOCFG_OUPUT_HIGH : LTC2949_BM_GPIOCFG_INPUT);
		/////   LTC2949_GpioConfig(
		/////   	/*byte gpio:*/ 2,
		/////   	/*byte mode:*/ enable ? LTC2949_BM_GPIOCFG_OUPUT_HIGH : LTC2949_BM_GPIOCFG_INPUT);

		// Note: LTC2949_GpioConfig + LTC2949_GpioCurrConfigWrite can be 
		//       called at any time to change GPIO setting!



