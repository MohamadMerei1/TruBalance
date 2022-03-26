/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_TEST.ino
* Linduino Sketch for LTC2949 Demo Board test
*
* Basic example for
*  fast measurements
*  GPIO control
*  ...
*
* Hardware requirement:
* . 10A current source
* . Linduino programmed to this sketch
* . DC1941 (LTC6820 demoboard)
* . 14-pin flat ribbon cable to connect DC1941 to Linduino
* . cables with hooks to connect supply (VCC, LGND) and HB4, HB5 of DC2732A
* . testadapter with voltage divider 9x 1k:
*   . V10 - 1k - V8 - 1k - V6 - 1k - V4 - 1k - VBATM - 1k - VBATP - 1k - V3 - 1k - V5 - 1k - V7 - 1k - V9
* . RJ45 ethernet patch cable <= 1m (20 cm cable is fine)
*
* Hardware test setup:
*  Set all jumpers on DC2732A to default (see schematic)
*  Set jumpers of DC1941 to upper position, except jumper SLOW which should be set to 0
*  Connect VCC, GND of DC1941 to VCC, LGND of DC2732A
*  Connect Linduino A0 to DC2732A D4
*  Connect Linduino A1 to DC2732A D5
*     The crossed connection is also allowed: A0-D5, A1-D4
*  Connect RJ45 ethernet cable between DC2732A and DC1941
*    (any of the two RJ45 on DC2732A is fine)
*  Connect Linduino to DC1941 via 14-pin flat ribbon cable
*  Connect testadapter to DC2732A J3 (bottom)
*  Connect Linduino via USB to PC
*  Connect current source to DC2732A's shunt (Note to + and - of the board)
*    As we test with 10A only, its fine to use some big hooks that are hooked to the cable lugs installed on the shunt
*
* Software setup:
*  In the Arduino IDE open the Serial monitor, set to "Both NL & CR", 1000000 baud
*  you will see following output:
DC2732A (LTC2949) test:
1   : Do test
2   : Loop test
RST : Reset
DB1 : Enable debug output
DB0 : Disable ~
*
*
* Run the test:
*   Enable current source
*   Enter 1 and hit enter in Arduino's Serial monitor
*   Check the output, final statement should be
TEST OF DC2732A2-A PASSED!
* or
TEST OF DC2732A2-B PASSED!
* or
TEST OF DC2732A2-C PASSED!
* depending on which board you have.
*
* Most critical errors will already cause the first outputs to fail. Typical output is: (DC2732A2-A with 100uOhms)
> 1
STAT:0x0F000000000000008000,OK
OK
EEPROM:OK
VCC:5.524,OK
VREF:3.000,OK
ntc1:1.398,OK
ntc2:1.387,OK
vcc:5.583,OK

I1:0.000999,A,OK
I2:0.001000,A,OK
BAT:0.5583,OK
AUX:2.3922,OK
...
* Example error (wrong current measurement)

I1:0.099981,X,ERR:0x80
I2:0.100034,X,ERR:0x80
BAT:0.5583,OK
AUX:2.3922,OK

*
*
*
*
*
* created: Patrick Wilhelm (PW)
*/


//#include <QuikEval_EEPROM_Wire.h>
#include <SPI.h>
#include <LTC2949.h>
#include <ltcmuc_tools.h>


// serial baudrate
#define LTCDEF_BAUDRATE 1000000

// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3

#define LTCDEF_ABSTOL_TEMP 10.0

#define LTCDEF_MIN_TEMP 10.0
#define LTCDEF_MAX_RNTC 217900
#define LTCDEF_MAX_TEMP 40.0
#define LTCDEF_MIN_RNTC 48820
//
#define LTCDEF_NTC_RATIO0 (LTCDEF_MAX_RNTC/(LTCDEF_MAX_RNTC+NTC_RREF))
#define LTCDEF_NTC_RATIO1 (LTCDEF_MIN_RNTC/(LTCDEF_MIN_RNTC+NTC_RREF))
#define LTCDEF_NTC_RATIO_MAX max(LTCDEF_NTC_RATIO0,LTCDEF_NTC_RATIO1)
#define LTCDEF_NTC_RATIO_MIN min(LTCDEF_NTC_RATIO0,LTCDEF_NTC_RATIO1)
/*
Temp,R
-40,5089000
-35,3549000
-30,2503000
-25,1785000
-20,1286000
-15,935300
-10,686900
-5,509000
0,380500
5,286800
10,217900
15,166900
20,128700
25,100000
30,78230
35,61600
40,48820
45,38930
50,31230
55,25200
60,20440
65,16670
70,13670
75,11260
80,9325
85,7757
90,6482
95,5440
100,4584
105,3879
110,3295
115,2810
120,2405
125,2066
*/

// fast channel configuration
#define LTCDEF_FACTRL_CONFIG (LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)
// ADC configuration
#define LTCDEF_ADCCFG_CONFIG (LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_NTC2 | LTC2949_BM_ADCCONF_P2ASV)
// reference current in amps
#define LTCDEF_REF_CURRENT 10.0
// reference current for 100uOhms board
#define LTCDEF_100U_REF_CURRENT (100e-6*LTCDEF_REF_CURRENT)
// reference current for 200uOhms board
#define LTCDEF_200U_REF_CURRENT (200e-6*LTCDEF_REF_CURRENT)
// reference current for 50uOhms board
#define LTCDEF_50U_REF_CURRENT  (50e-6*LTCDEF_REF_CURRENT)

//#define LTCDEF_TEST_DEBUG
#undef LTCDEF_TEST_DEBUG

#ifndef LTCDEF_TEST_DEBUG
// reference current DC2732A2-A
#define LTCDEF_REF_CURRENT_DC2732A_A  LTCDEF_100U_REF_CURRENT
// reference current DC2732A2-B
#define LTCDEF_REF_CURRENT_DC2732A_B  LTCDEF_50U_REF_CURRENT
// reference current DC2732A2-C
#define LTCDEF_REF_CURRENT_DC2732A_C  LTCDEF_200U_REF_CURRENT
#else
// reference current DC2732A2-A
#define LTCDEF_REF_CURRENT_DC2732A_A  (0.1)
// reference current DC2732A2-B
#define LTCDEF_REF_CURRENT_DC2732A_B  (0.7)
// reference current DC2732A2-C
#define LTCDEF_REF_CURRENT_DC2732A_C  (0.04)
#endif
// reference current tolerance
#define LTCDEF_REF_CURRENT_TOL  0.06
// reference aux tolerance
#define LTCDEF_REF_AUX_TOL  0.05
// reference BAT tolerance
#define LTCDEF_REF_BAT_TOL  0.05
// reference aux tolerance
#define LTCDEF_REF_VCC_TOL  0.05

// GPIO 
#define LTCDEF_GPIO_UPPER_DRIVE  2
// GPIO 
#define LTCDEF_GPIO_LOWER_DRIVE  3
#define LTCDEF_AUX_DIV  9.0


String SerialInputString;
char DC2732A_x = 0;
boolean contMeasEnabled = false;

#define LTCDEF_PRINTBOARD()   Serial.print(  DC2732A_x ? DC2732A_x : 'X')
#define LTCDEF_PRINTLNBOARD() Serial.println(DC2732A_x ? DC2732A_x : 'X')

void EnableDividerDrive(bool pos)
{
	LTC2949_GpioConfig(LTCDEF_GPIO_UPPER_DRIVE, pos ? LTC2949_BM_GPIOCFG_OUPUT_HIGH : LTC2949_BM_GPIOCFG_OUPUT_LOW);
	LTC2949_GpioConfig(LTCDEF_GPIO_LOWER_DRIVE, !pos ? LTC2949_BM_GPIOCFG_OUPUT_HIGH : LTC2949_BM_GPIOCFG_OUPUT_LOW);
	LTC2949_GpioConfig(4, pos ? LTC2949_BM_GPIOCFG_OUPUT_TOGGLE : LTC2949_BM_GPIOCFG_INPUT);
	LTC2949_GpioConfig(5, !pos ? LTC2949_BM_GPIOCFG_OUPUT_TOGGLE : LTC2949_BM_GPIOCFG_INPUT);
	LTC2949_GpioCurrConfigWrite();
}

void Go()
{
	DC2732A_x = 0;
	byte error;
	byte error_;
	byte data[2];
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];

	error = LTC2949_EEPROMInitCheck();
	Serial.print(F("EEPROM:"));
	PrintOkErr(error);

	error |= Cont(true);
	//delay(LTC2949_TIMING_CONT_CYCLE);
	//error |= (error_ = CheckBoardIDString());
	//PrintComma();
	//PrintOkErr(error_);


	// make sure HS byte is cleared at the beginning
	error |= LTC2949_RdFastData();

	EnableDividerDrive(/*bool pos:*/true);

	//////////////////////////////
	// CHECK IC TEMP
	//////////////////////////////
	error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, data);
	float icTemp = LTC_2BytesToInt16(data) * LTC2949_LSB_TEMP;
	error |= (error_ = (icTemp > LTCDEF_MIN_TEMP && icTemp < LTCDEF_MAX_TEMP) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("ICT:"));
	Serial.print(icTemp, 1);
	PrintComma();
	PrintOkErr(error_);
	//////////////////////////////
	// CHECK NTC1 TEMP
	//////////////////////////////
	error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, data);
	float ntc1 = LTC_2BytesToInt16(data) * LTC2949_LSB_TEMP;
	error |= (error_ = (abs(ntc1 - icTemp) < LTCDEF_ABSTOL_TEMP) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("NTC1:"));
	Serial.print(ntc1, 1);
	PrintComma();
	PrintOkErr(error_);
	//////////////////////////////
	// CHECK NTC2 TEMP
	//////////////////////////////
	error |= LTC2949_READ(LTC2949_VAL_SLOT2, 2, data);
	float ntc2 = LTC_2BytesToInt16(data) * LTC2949_LSB_TEMP;
	error |= (error_ = (abs(ntc2 - icTemp) < LTCDEF_ABSTOL_TEMP) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("NTC2:"));
	Serial.print(ntc2, 1);
	PrintComma();
	PrintOkErr(error_);


	// 1 dummy measurement
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);

	//////////////////////////////
	// CHECK VCC
	//////////////////////////////
	error |= LTC2949_READ(LTC2949_VAL_VCC, 2, data);
	float advcc = LTC_2BytesToInt16(data) * LTC2949_LSB_VCC;
	error |= (error_ = (advcc > 4.9 && advcc < 5.9) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("VCC:"));
	Serial.print(advcc, 3);
	PrintComma();
	PrintOkErr(error_);

	//////////////////////////////
	// CHECK VREF
	//////////////////////////////
	error |= LTC2949_READ(LTC2949_VAL_VREF, 2, data);
	float vref = LTC_2BytesToInt16(data) * LTC2949_LSB_VREF;
	error |= (error_ = (vref > 2.97 && vref < 3.03) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("VREF:"));
	Serial.print(vref, 3);
	PrintComma();
	PrintOkErr(error_);


	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V1, 0);
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	float v1 = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FIFOAUX;

	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V2, 0);
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	float v2 = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FIFOAUX;

	Serial.print(F("ntc1:"));
	Serial.print(v1, 3);
	PrintComma();
	error |= (error_ = (v1 > LTCDEF_NTC_RATIO_MIN * vref && v1 < LTCDEF_NTC_RATIO_MAX * vref) ? 0 : LTC2949_ERRCODE_OTHER);
	PrintOkErr(error_);

	Serial.print(F("ntc2:"));
	Serial.print(v2, 3);
	PrintComma();
	error |= (error_ = (v2 > LTCDEF_NTC_RATIO_MIN * vref && v2 < LTCDEF_NTC_RATIO_MAX * vref) ? 0 : LTC2949_ERRCODE_OTHER);
	PrintOkErr(error_);




	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VREF2, 0);
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);

	//////////////////////////////
	// CHECK VCC from BAT
	//////////////////////////////
	float bat = fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT;
	float advcc_fromBat = bat * LTCDEF_AUX_DIV;
	error |= (error_ = (advcc_fromBat > 4.5 && advcc_fromBat < advcc * (1 + LTCDEF_REF_VCC_TOL)) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("vcc:"));
	Serial.print(advcc_fromBat, 3);
	PrintComma();
	PrintOkErr(error_);
	error |= ReportCheclMeasurements(fastData2949, bat, LTC2949_VREF2);

	//////////////////////////////
	// CHECK V8 - V9
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V8, LTC2949_SLOTMUX_V9);
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	float advcc_v8v9 = -fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FAUX / (LTCDEF_AUX_DIV - 1) * LTCDEF_AUX_DIV;
	error |= (error_ = (advcc_v8v9 > 4.5 && advcc_v8v9 < advcc * (1 + LTCDEF_REF_VCC_TOL)) ? 0 : LTC2949_ERRCODE_OTHER);
	Serial.print(F("vcc_V8V9:"));
	Serial.print(advcc_v8v9, 3);
	PrintComma();
	PrintOkErr(error_);
	// check report
	error |= ReportCheclMeasurements(fastData2949, /*expBat*/advcc_v8v9 / LTCDEF_AUX_DIV, /*expAux*/-advcc_fromBat / LTCDEF_AUX_DIV * (LTCDEF_AUX_DIV - 1));

	//////////////////////////////
	// CHECK R5
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V9, LTC2949_SLOTMUX_V7);
	// trigger the measurement
	error |= LTC2949_ADxx();
	// poll LTC2949 for conversion done
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	// check report
	error |= ReportCheclMeasurements(fastData2949, bat, bat);

	//////////////////////////////
	// CHECK R6
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V5, LTC2949_SLOTMUX_V7);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, -bat);

	//////////////////////////////
	// CHECK R7
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V5, LTC2949_SLOTMUX_V3);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, bat);

	//////////////////////////////
	// CHECK R8
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VBATP, LTC2949_SLOTMUX_V3);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, -bat);

	//////////////////////////////
	// CHECK R9
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_VBATP, LTC2949_SLOTMUX_VBATM);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, bat);

	//////////////////////////////
	// CHECK R4
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V4, LTC2949_SLOTMUX_VBATM);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, -bat);

	//////////////////////////////
	// CHECK R3
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V4, LTC2949_SLOTMUX_V6);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, bat);

	//////////////////////////////
	// CHECK R2
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V8, LTC2949_SLOTMUX_V6);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, -bat);

	//////////////////////////////
	// CHECK R1
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V8, LTC2949_SLOTMUX_V10);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, bat, bat);

	uint16_t an0 = analogRead(0);
	uint16_t an1 = analogRead(1);

	EnableDividerDrive(/*bool pos:*/false);
	delay(100);
	//////////////////////////////
	// CHECK R3
	//////////////////////////////
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V4, LTC2949_SLOTMUX_V6);
	error |= LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	error |= ReportCheclMeasurements(fastData2949, -bat, -bat);

	uint16_t an0_ = analogRead(0);
	uint16_t an1_ = analogRead(1);

	float anRef, anRef_;

	if (an0 < 512)
	{
		anRef = 0.0;
		anRef_ = 5.0;
	}
	else
	{
		anRef_ = 0.0;
		anRef = 5.0;
	}
	Serial.println();
	error |= CheckHB(F(" A:"), an0, anRef);
	error |= CheckHB(F(" B:"), an1, anRef_);
	error |= CheckHB(F("!A:"), an0_, anRef_);
	error |= CheckHB(F("!B:"), an1_, anRef);

	error |= WakeUpReportStatus(false);


	Serial.print(F("TEST OF DC2732A2-"));
	LTCDEF_PRINTBOARD();
	if (error)
	{
		Serial.println(F(" FAILED!"));
		PrintOkErr(error);
		return;
	}

	Serial.println(F(" PASSED!"));
}

byte CheckHB(const __FlashStringHelper* ifsh, uint16_t an, float exp)
{
	float val;
	Serial.print(ifsh);
	Serial.print(val = an / 1023. * 5.0, 2);
	PrintComma();
	byte error = (abs(val - exp) < 0.5) ? 0 : LTC2949_ERRCODE_OTHER;
	PrintOkErr(error);
	return error;
}


char DetectBoard(float current)
{
	if (abs(current / LTCDEF_REF_CURRENT_DC2732A_A - 1.) < LTCDEF_REF_CURRENT_TOL)
		return 'A';
	if (abs(current / LTCDEF_REF_CURRENT_DC2732A_B - 1.) < LTCDEF_REF_CURRENT_TOL)
		return 'B';
	if (abs(current / LTCDEF_REF_CURRENT_DC2732A_C - 1.) < LTCDEF_REF_CURRENT_TOL)
		return 'C';
	return 0;
}

byte CheckBoard(float val)
{
	if (DC2732A_x)
	{
		if (DC2732A_x == DetectBoard(val))
			return(0);
		else
			return(LTC2949_ERRCODE_OTHER);
	}
	else
	{
		DC2732A_x = DetectBoard(val);
		return(DC2732A_x ? 0 : LTC2949_ERRCODE_OTHER);
	}
}

byte ReportCheclMeasurements(int16_t* fastData2949, float expBat, float expAux)
{
	byte error = 0;
	byte error_;
	float val;
	Serial.println();

	Serial.print(F("I1:"));
	Serial.print(val = fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FIFOI1, 6);
	PrintComma();
	error |= (error_ = CheckBoard(val));
	LTCDEF_PRINTBOARD();
	PrintComma();
	PrintOkErr(error_);

	Serial.print(F("I2:"));
	Serial.print(val = fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2, 6);
	PrintComma();
	error |= (error_ = CheckBoard(val));
	LTCDEF_PRINTBOARD();
	PrintComma();
	PrintOkErr(error_);


	Serial.print(F("BAT:"));
	Serial.print(val = fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, 4);
	PrintComma();
	error |= (error_ = ((abs(val / expBat - 1.) < LTCDEF_REF_BAT_TOL) ? 0 : LTC2949_ERRCODE_OTHER));
	PrintOkErr(error_);

	Serial.print(F("AUX:"));
	Serial.print(val = fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FIFOAUX, 4);
	PrintComma();
	error |= (error_ = ((abs(val / expAux - 1.) < LTCDEF_REF_AUX_TOL) ? 0 : LTC2949_ERRCODE_OTHER));
	PrintOkErr(error_);
	return error;
}

void setup()
{
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// configure SPI, also done in LTC2949.cpp:
	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);

	while (!Serial)
		; // wait for serial port to connect. Needed for native USB port only
	SerialInputString = "";
	PrintMenu();
}

void Init()
{
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);

	LTC2949_reset(); //LTC2949_init_device_state();
	delay(LTC2949_TIMING_BOOTUP);
	byte error = WakeUpReportStatus(true);
	PrintOkErr(error);
}

void PrintMenu()
{
	Serial.print(F("\n\n\
DC2732A (LTC2949) test:\n\
1   : Do test\n\
2   : Loop test\n\
RST : Reset\n\
DB1 : Enable debug output\n\
DB0 : Disable ~\n\
"));
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


	//////////////////////////////////////////////////////////////////////
	///////////////////// BEGIN: PROCESS COMMANDS ////////////////////////
	//////////////////////////////////////////////////////////////////////
	//
	/////////////////////////////// COMAND ///////////////////////////////
	if (equals(SerialInputString, F("1")))
	{
		contMeasEnabled = false;
		Init();
		Go();
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("2")))
	{
		Init();
		PrintOkErr(Cont(true));
		//delay(LTC2949_TIMING_CONT_CYCLE);
		contMeasEnabled = !contMeasEnabled;
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (equals(SerialInputString, F("RST")))
	{
		contMeasEnabled = false;
		LTC2949_reset(); //LTC2949_init_device_state();
	}
	/////////////////////////////// COMAND ///////////////////////////////
	else if (startsWith(SerialInputString, F("DB")))
	{
		contMeasEnabled = false;
		DebugEnable();
	}
	/////////////////////// UNKNOWN COMAND ///////////////////////////////
	else
	{
		contMeasEnabled = false;
		Serial.println(F("CmdErr"));
	}
	//
	//////////////////////////////////////////////////////////////////////
	///////////////////// END: PROCESS COMMANDS //////////////////////////
	//////////////////////////////////////////////////////////////////////

	// empty SerialInputString buffer
	SerialInputString = "";
	PrintMenu();
}
/*!*********************************************************************
\brief debug enable / disable (communication data RAW output)
***********************************************************************/
void DebugEnable()
{
	Serial.println(LTC2949_DebugEnable = (toIntAuto(SerialInputString.substring(2)) != 0));
	PrintOkErr(0);
}

boolean measPos = true;

/*!*********************************************************************
\brief main loop (executed in an endless loop by arduino)
***********************************************************************/
void loop()
{
	// process any serial interface commands
	ProcessSerialCommands();


	if (!contMeasEnabled)
		return;

	measPos = !measPos;
	EnableDividerDrive(/*bool pos:*/measPos);
	delay(100);

	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	LTC2949_SlotFastCfg(LTC2949_SLOTMUX_V5, LTC2949_SLOTMUX_V3);

	byte error = LTC2949_ADxx();
	error |= LTC2949_PollFastData(/*int16_t * data:*/fastData2949, /*boolean clrChkHS:*/true);
	//error |= ReportCheclMeasurements(fastData2949, fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT);
	Serial.print(fastData2949[LTC2949_RDFASTDATA_I1] * LTC2949_LSB_FIFOI1, 6);
	PrintComma();
	Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2, 6);
	PrintComma();
	Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT * (measPos ? 1 : -1), 4);
	PrintComma();
	Serial.print(fastData2949[LTC2949_RDFASTDATA_AUX] * LTC2949_LSB_FIFOAUX * (measPos ? 1 : -1), 4);
	PrintComma();
	//uint16_t an0 = analogRead(0);
	//uint16_t an1 = analogRead(1);
	Serial.print(analogRead(0) / 1023. * 5, 1);
	PrintComma();
	Serial.print(analogRead(1) / 1023. * 5, 1);
	PrintComma();

	PrintOkErr(error);
}


/*!*********************************************************************
\brief Wakeup LTC2949, report and clear all status / alert registers
***********************************************************************/
byte WakeUpReportStatus(bool afterReset)
{
	byte  error = LTC2949_WakeupAndAck();
	if (afterReset)
		error |= LTC2949_ReadChkStatusFaults(true, true, 10, NULL, NULL, LTC2949_STATFAULTSCHK_DFLT_AFTER_RESET);
	else
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
		LTC2949_SlotFastCfg(1, 0);
		// SLOT1 measures temperature via NTC between V1 and GND. SLOT2 not used, still we configure something
		LTC2949_SlotsCfg(1, 0, 2, 0);
		// enable NTC temperature measurement via SLOT1
		NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
		// enable NTC temperature measurement via SLOT1
		NtcCfgWrite(2, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
		// read & clear status
		byte error = LTC2949_ReadChkStatusFaults(true, false);
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



/*

DC2732A (LTC2949) test:
1   : Do test
2   : Loop test
RST : Reset
DB1 : Enable debug output
DB0 : Disable ~
> 1
STAT:0x0F000000000000008000,OK
OK
EEPROM:OK
VCC:5.526,OK
VREF:3.000,OK
ntc1:1.381,OK
ntc2:1.368,OK
vcc:5.579,OK

I1:0.099981,A,OK
I2:0.100042,A,OK
BAT:0.5579,OK
AUX:2.3925,OK

I1:0.099981,A,OK
I2:0.100042,A,OK
BAT:0.5583,OK
AUX:0.5613,OK

I1:0.099974,A,OK
I2:0.100034,A,OK
BAT:0.5583,OK
AUX:-0.5586,OK

I1:0.099981,A,OK
I2:0.100042,A,OK
BAT:0.5583,OK
AUX:0.5583,OK

I1:0.099974,A,OK
I2:0.100042,A,OK
BAT:0.5583,OK
AUX:-0.5583,OK

I1:0.099981,A,OK
I2:0.100034,A,OK
BAT:0.5583,OK
AUX:0.5579,OK

I1:0.099981,A,OK
I2:0.100034,A,OK
BAT:0.5583,OK
AUX:-0.5579,OK

I1:0.099981,A,OK
I2:0.100034,A,OK
BAT:0.5583,OK
AUX:0.5583,OK

I1:0.099981,A,OK
I2:0.100042,A,OK
BAT:0.5583,OK
AUX:-0.5583,OK

I1:0.099981,A,OK
I2:0.100034,A,OK
BAT:0.5583,OK
AUX:0.5583,OK

I1:0.099981,A,OK
I2:0.100034,A,OK
BAT:-0.5575,OK
AUX:-0.5579,OK

A:4.88,OK
B:0.04,OK
!A:0.04,OK
!B:4.87,OK
STAT:0x10000000000000000000,OK
TEST OF DC2732A2-A PASSED!


*/




/*
byte CheckBoardIDString()
{
String idStr = GetBoardIDString();
String idStrRef = String(F("LTC2949,Cls,D2949,01,01,DC,DC2732A,-------------"));
Serial.print(F("ID:"));
if (idStr.equals(idStrRef))
{
Serial.println(idStr);
return 0;
}
const char * c = idStrRef.c_str();

uint8_t buffer_count = eeprom_write_buffer(EEPROM_I2C_ADDRESS, (char *)c, 0);

idStr = GetBoardIDString();
if (idStr.equals(idStrRef))
{
Serial.println(idStr);
return 0;
}
Serial.println(idStr);
return LTC2949_ERRCODE_OTHER;
}

String GetBoardIDString()
{
//// <DEMO>
//// <DCNUM>DC2732A</DCNUM>
//// <IDSTR type="1">LTC2949,Cls,D2949,01,01,DC,DC2732A,-------------</IDSTR>
//// </DEMO>
char ui_buffer[QUIKEVAL_ID_SIZE];
// get controller id string
uint8_t byte_count = read_quikeval_id_string(ui_buffer);
if (byte_count != 0)
return String(ui_buffer);
return "";
}

*/


