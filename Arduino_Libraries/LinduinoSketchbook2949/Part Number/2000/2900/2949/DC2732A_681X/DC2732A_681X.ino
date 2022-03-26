/*
* Linear Technology / Analog Devices, Ismaning Design Center
*
* DC2732A_681X.ino
*  Linduino Sketch for LTC2949 Demo Board (DC2732A) - Isolated Battery Meter for EV / HEV
*  and LTC681x-1 Demo Board (DC2259A, DC2350A...) - Multicell Battery Monitor
*
* Hardware required:
*  LTC6820 demoboard (DC1941D or DC2617A or DC2792)
*  Optional: Two LTC6820 boards or one DC2792 (dual LTC6820 demoboard)
*              The two LTC6820 are selected via individual chip selects via Arduino pins 9 and 10. See notes below
*              The software will also work with only one LTC6820
*              Two LTC6820 are necessary to showcase reversible isoSPI fault recovery functionality
*  One or more LTC681x Demo Board (DC2259A, DC2350A), default (see define LTCDEF_CELL_MONITOR_COUNT in code below) is 2 boards in the daisychain
*  the number of cells to be measured can be set via LTCDEF_CELLS_PER_CELL_MONITOR_COUNT, default is 12 (LTC6811)
*  Linduino (DC2026)
*
* Hardware setup (e.g. PARALLEL TO DAISYCHAIN):
*  DC2732A: Enable isoSPI mode (JP3-JP6), disable isoSPI termination (JP1)
*  DC1941D: Set IBIAS, VCMP jumpers to VTH2
*  DC2259A: Set ISOMD jumpers to 1 to enable isoSPI
*  :
*  - Connect Linduino to LTC6820 demoboard
*  - Connect LTC6820 demoboard via RJ45 cable to LTC2949 demoboard (J1)
*  - Connect LTC2949 demoboard (J2) via RJ45 cable to first LTC6811-1 demoboard (J3 / port A)
*  - Connect LTC6811-1 demoboard (J4 / port B) to 2nd LTC6811-1 demoboard (J3 / port A)
*  - Connect as many LTC6811-1 demoboards in the daisychain as required
*  - Connect power supply to LTC2949 demoboard (turrets VCC, LGND)
*  - Connect power supply / batteries to LTC6811-1 demoboard(s) (+ CELL12, - CELL0)
*
* equivalent setups are possible e.g. with LTC6813 (DC2350A)
*
* Software setup:
*  - Set LTCDEF_CELL_MONITOR_COUNT to the number of LTC681x connected in the daisychain (default 2)
*  - Set LTCDEF_CELLS_PER_CELL_MONITOR_COUNT to the number of cells per cell monitor
*  -
* Hardware setup (ON TOP OF DAISYCHAIN):
*  - same as above, but:
*    - DC2732A: Enable isoSPI termination (JP1)
*    - Connect LTC2949 demoboard (J2) via RJ45 cable to last LTC6811-1 demoboard (J4 / port B)
*    - Connect LTC6820 demoboard via RJ45 cable to first LTC6811-1 demoboard (J3 / port A)
*
* See demo manuals of the related demoboards for more details on hardware setup.
*
* Description:
*  LTC2949 will do fast single shot measurements of I2 and BAT (CH2 + P2ASV) synchronous to cell
*  voltage measurements done by the cell monitor(s) LTC6811-1 (and compatible cell monitors of the LTC68xx family)
*  LTC2949 is connected in parallel to the daisychain of cell monitors (meaning at the bottom of the daisychain
*  parallel to the LTC6820) as default.
*  The DC2732A_681X sketch also allows to connect LTC2949 on top of the daisychain (after the last cell monitor). It
*  will automatically detect which configuration is setup and switch between "PARALLEL TO DAISYCHAIN" and
*  "ON TOP OF DAISYCHAIN". This also demonstrates how LTC2949 can benefit from a redundant communication path when used
*  with reversible isoSPI ready cell monitors like the LTC6813.
*    The reversible isoSPI fault recovery feature can be demonstrated when having two LTC6820 connected to the Linduino
*    Any of the two LTC6820 can be disconnected and the software will automatically find a new communication path.

*  Additionally LTC2949 does following measurements with its slow channel (100ms update time)
*   NTC1: SLOT1 measures temp via NTC (NTC connected between V1 and GND)
*   continuous reading and reporting of delta-TB1, I1, P1, BAT, NTC1 / IC temp. from slow channel.
*  Reporting of fast I2, BAT together with cell voltage conversion results

*  Example output: (see the INIT string, that always shows the configuration used, CS tells the used chip select = one for up to two LTC6820)
INIT,PARALLEL TO DAISYCHAIN,CS:10,STAT:0x10000000000000000000,OK
0.3,0.1,0.2,0.3,0.1,0.2,0x000000000000000000000000,0x040000000000040000000000,6.6,6.6,6.6,6.6,6.6,6.6,0.3,0.1,0.2,0.3,0.1,0.2,OK
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
0,-69785,-13063,0.19,-37.6,25.0,0.3,0.1,0.2,0.3,0.1,0.2,0.4,0.1,1.3,0.3,0.5,1.4,-69817,0.19,1064,OK
15,,,,,,0.3,0.1,0.1,0.3,0.1,0.2,0.0,0.1,1.3,0.3,0.5,1.4,-69817,0.19,1072,OK
10,,,,,,0.3,0.1,0.1,0.3,0.1,0.2,0.4,0.1,1.3,0.3,0.5,1.4,-69817,0.19,1068,OK
...
9,,,,,,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,-8,-0.00,832,ERR:0x83
<<<<<<=====================================================================>>>>>>>
<<<<<<========= error injected by unplugging one isoSPI cable =============>>>>>>>
<<<<<<=====================================================================>>>>>>>
ERR:0x1
INIT,PARALLEL TO DAISYCHAIN,CS:10,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
6.6,6.6,6.6,6.6,6.6,6.6,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,ERR:0xB
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
ERR:0x1
INIT,PARALLEL TO DAISYCHAIN,CS:9,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
0.3,0.1,0.2,0.3,0.1,0.2,0x040000000000040000000000,0x040000000000040000000000,6.6,6.6,6.6,6.6,6.6,6.6,0.3,0.1,0.2,0.3,0.1,0.2,ERR:0x9
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
ERR:0x1
INIT,PARALLEL TO DAISYCHAIN,CS:9,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
0.3,0.1,0.2,0.3,0.1,0.2,0x040000000000040000000000,0x040000000000040000000000,6.6,6.6,6.6,6.6,6.6,6.6,0.3,0.1,0.2,0.3,0.1,0.2,ERR:0x9
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
ERR:0x1
INIT,ON TOP OF DAISYCHAIN,CS:10,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0xB
6.6,6.6,6.6,6.6,6.6,6.6,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,ERR:0xB
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
ERR:0x3
INIT,ON TOP OF DAISYCHAIN,CS:10,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0xB
6.6,6.6,6.6,6.6,6.6,6.6,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,6.6,ERR:0xB
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
ERR:0x3
<<<<<<=====================================================================>>>>>>>
<<<<<<========= new functional communication path found ===================>>>>>>>
<<<<<<=====================================================================>>>>>>>
INIT,ON TOP OF DAISYCHAIN,CS:9,STAT:0x10000000000000000000,OK
0.3,0.1,0.2,0.3,0.1,0.2,0x040000000000040000000000,0x040000000000040000000000,6.6,6.6,6.6,6.6,6.6,6.6,0.3,0.1,0.2,0.3,0.1,0.1,OK
tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C7V,C8V,C9V,C4V,C5V,C6V,C10V,C11V,C12V,fI2,fBAT,fT,OK/ERR
0,-69786,-13045,0.19,-37.6,25.0,0.3,0.1,0.2,0.3,0.1,0.1,0.3,0.5,1.4,0.1,0.1,1.3,-69825,0.19,1460,OK
27,,,,,,0.3,0.1,0.2,0.3,0.1,0.1,0.3,0.5,1.4,0.4,0.1,1.3,-69817,0.19,1464,OK
...
*
*  Notes:
*   tDut: delta TB1 in ms (typically 100, as values are reported for every slow conversion cycle which lasts 100 ms)
*   I1: slow / high precision I1 current measurement in V (x 1/Rsns for current in A)
*   P1: slow / high precision P1 power measurement in V*V (x 1/Rsns for power in W)
*   BAT: slow AUX channel BAT measurement in V
*   Tntc: slow AUX channel SLOT1 measurement. SLOT1 is configured for temperature measurement via NTC in degree Celsius
*   TIC:  slow AUX channel IC temperature measurement in degree Celsius
*   CxV: Voltage of cell x
*   fI2: latest CH2 current fast measurement in V (x 1/Rsns for current in A)
*   fBAT: latest CH2 voltage (P2 as voltage) fast BAT measurement in V
*   fT: cell monitor fast measurement cycle time in milliseconds
*   OK/ERR: Tag indicating all measurements / communication was successful (OK) or some failure occurred (ERR)
*
****************************************************************************************************************
****************************************************************************************************************
****************************************************************************************************************
****************************************************************************************************************
* NEWLY ADDED:
* - supports two LTC6820 isoSPI masters, selection is done with chip select on Linduino pin 10 (SS) and 9
* - The DC2792 dual LTC6820 can be used for that purpose without any s/w changes. It has option for ribbon or
*  ‘shield’ connection and an LED for each CS line so that traffic & direction can be visualized.
* - Alternatively two DC1641 can be used:
*  - the connection of two DC1641 demoboards to one Linduino can be done with a custom flat ribbon
*      Linduino’s QuikEval connector (14pin connector J1) has a GPIO on pin 14 that can be used as a 2nd
*      chip select to the second DC1941. A simple way of making the connection is to use a 14pin flat
*      ribbon cable, crimp a third connector to it and swap wires 6 and 14 between the two connectors
*      going to the two DC1941 boards. The Linduino sketch switches then between the two possible chip
*      selects (Arduino pin 10 and 9) to switch between the two communication path.
*
* created: Patrick Wilhelm (PW)
*/

#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>

// number of cell monitors
#define LTCDEF_CELL_MONITOR_COUNT 1
// number of cells per cell monitor (e.g. LTC6811: 12, LTC6813: 18)
// !!!must be multiple of 3!!!
// Its always possible to set less, if not all voltages are of interest 
// for debugging/evaluation. So here we set 6 only.
#define LTCDEF_CELLS_PER_CELL_MONITOR_COUNT 12

// define this for usage of LTC681X only, without LTC2949
#define LTCDEF_LTC681X_ONLY
#undef  LTCDEF_LTC681X_ONLY
#define LTCDEF_TIMING_DEBUG
#undef LTCDEF_TIMING_DEBUG

// serial baudrate
//Note: on Arduino Due, 250000 baud seems to be the maximum
#define LTCDEF_BAUDRATE 250000

// default chip select of the LTC6820
// Note: SS is mapped differently on Arduino Zero / DUE
//       thus direct pin number (10) should be used instead
#define LTCDEF__CS 10
// chip select of the optional 2nd LTC6820
#define LTCDEF_GPO 9

// set sense resistor here
#define LTCDEF_SENSE_RESISTOR 1e-6
// significant digits
// voltages
#define LTCDEF_DIGITS_BATSLOW     2
#define LTCDEF_DIGITS_BATFAST     2
#define LTCDEF_DIGITS_BATFIFO_AVG 2
#define LTCDEF_DIGITS_CELL        3
// currents
#define LTCDEF_DIGITS_I1SLOW      0
#define LTCDEF_DIGITS_I2FAST      0
#define LTCDEF_DIGITS_I2FIFO_AVG  0
// power
#define LTCDEF_DIGITS_P1SLOW      0
// temperature
#define LTCDEF_DIGITS_TEMPSLOW    1

#ifdef LTCDEF_LTC681X_ONLY
// polling not supported in case of LTCDEF_LTC681X_ONLY!!!
#define LTCDEF_POLL_EOC false
#else
// true for making ADCV command and poll EOC
// false for making ADCV command and wait until EOC
#define LTCDEF_POLL_EOC false
#endif

// define LTCDEF_IGNORE_PEC_ERRORS 
// quick work around for new 6815 style devices with PEC+CMDcnt 
// that is currently not supported by this source
#define LTCDEF_IGNORE_PEC_ERRORS
#undef LTCDEF_IGNORE_PEC_ERRORS
//
#ifdef LTCDEF_IGNORE_PEC_ERRORS
// ignore all PEC errors
inline bool err_detected(byte error) { return error > 0x3; }
// or just ignore single PEC errors
//inline bool err_detected(byte error) { return (ALLOW_ONE_PEC_ERROR ? ((error) > 1) : ((error) != 0)); }
#else
inline bool err_detected(byte error) { return error != 0; }
#endif

// NTC: NTCALUG01A104F/TDK (SMD)
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_RREF   100e3

// fast channel configuration: fast single shot, channel 2: I1, BAT (via P2 as voltage see below)
#define LTCDEF_FACTRL_CONFIG LTC2949_BM_FACTRL_FACH2
// ADC configuration (SLOT1 measures temperature via NTC, P2 measures voltage)
#define LTCDEF_ADCCFG_CONFIG (LTC2949_BM_ADCCONF_NTC1 | LTC2949_BM_ADCCONF_P2ASV)

#define LTCDEF_ERR_RETRIES 1

// define to do a reset every xxx milliseconds
//#define LTCDEF_DO_RESET_TEST 1000

unsigned long mcuTime;
uint8_t retries = LTCDEF_ERR_RETRIES;

/*!*********************************************************************
\brief prints the CSV header of the measurement output
***********************************************************************/
void PrintCSVHeader()
{
	// the order the cell voltages are reported is little bit awkward,
	// its not just 1,2,3,4,5,6.... so we can't print the header
	// like this:
	//Serial.print(F("tDut,I1,P1,BAT,Tntc,TIC,C1V..,C"));
	//Serial.print(LTCDEF_CELL_MONITOR_COUNT * 3 * 4);
	//Serial.println(F("V,fI2,fBAT,OK/ERR"));
#ifndef LTCDEF_LTC681X_ONLY
	Serial.print(F("tDut,I1,P1,BAT,Tntc,TIC,"));
#else
	Serial.print(F("tDut,"));
#endif
	for (uint8_t j = 0; j < LTCDEF_CELL_MONITOR_COUNT; j++) // number of cell monitors
	{
		for (uint8_t n = 0; n < LTCDEF_CELLS_PER_CELL_MONITOR_COUNT; n++)// += 3) // 3 per RDCV
		{
			Serial.print((char)('a' + j));
			Serial.print('C');
			Serial.print(n);
			PrintComma();
		}
	}
#ifdef LTCDEF_LTC681X_ONLY
	Serial.println(F("eErr,OK/ERR"));
#else
	Serial.println(F("eErr,fI2,fBAT,fT,OK/ERR"));
#endif

	/*
	!!!!
	!!!! Initially we assume to be parallel to the daisychain, but here we started on top of daisychain
	!!!!
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
	0.0,0.9,0.9,0.9,0.9,0.9,0xDA00000000009A0000000000,0x1E0000000000960000000000,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,ERR:0x9
	INIT,PARALLEL TO DAISYCHAIN,0xFFFFFFFFFFFFFFFFFFFF,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
	ERR:0x9
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0x10000000000000000000,OK
	0.0,0.9,0.9,0.9,0.9,0.9,0x9E0000000000DE0000000000,0x9E00000000009E0000000000,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,OK
	INIT,ON TOP OF DAISYCHAIN,0x10000000000000000000,STAT:0x10000000000000000000,OK
	100,0,0,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1220,OK
	100,-1,6,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1216,OK
	100,0,0,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1220,OK
	100,0,6,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1216,OK
	...
	...
	!!!! normal operation
	...
	...
	100,0,0,-0.00,25.8,28.2,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1216,OK
	100,0,0,-0.00,25.8,28.2,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1228,OK
	100,-1,0,-0.00,25.8,28.2,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1224,OK
	!!!!
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	!!!! We interrupt the isoSPI interface here!!!!!
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	!!!!
	!!!! First we try again with the current configuration (ON TOP OF DAISYCHAIN)
	!!!!
	INIT,ON TOP OF DAISYCHAIN,0x10000000000000000000,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0xB
	ERR:0xB
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
	-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,ERR:0xB
	!!!!
	!!!! Didn't work so we try with the opposite configuration (PARALLEL TO DAISYCHAIN)
	!!!!
	INIT,PARALLEL TO DAISYCHAIN,0xFFFFFFFFFFFFFFFFFFFF,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
	ERR:0x9
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0xB
	-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,ERR:0xB
	!!!!
	!!!! Didn't work so we try with the opposite configuration (ON TOP OF DAISYCHAIN)
	!!!!
	!!!! (we keep doing this, until we find a functional configuration)
	!!!!
	INIT,ON TOP OF DAISYCHAIN,0xFFFFFFFFFFFFFFFFFFFF,STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0xB
	ERR:0xB
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0xFFFFFFFFFFFFFFFFFFFF,ERR:0x9
	-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0xFFFFFFFFFFFFFFFFFFFFFFFF,0xFFFFFFFFFFFFFFFFFFFFFFFF,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,ERR:0xB
	...
	...
	!!!!
	!!!! we keep doing this, until we find a functional configuration
	!!!!
	...
	...
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0x00000000000000000000,ERR:0xA
	0.0,0.0,0.0,0.0,0.0,0.0,0x000000000000000000000000,0x000000000000000000000000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,ERR:0xA
	INIT,ON TOP OF DAISYCHAIN,0xCFB7FF03000001000007,STAT:0x00000000000000000000,ERR:0xA
	ERR:0xB
	...
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	!!!!!We connect LTC2949 parallel to daisychain !!!
	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	...
	tDut,I1,P1,BAT,Tntc,TIC,C1V,C2V,C3V,C13V,C14V,C15V,C4V,C5V,C6V,C16V,C17V,C18V,C7V,C8V,C9V,C19V,C20V,C21V,C10V,C11V,C12V,C22V,C23V,C24V,fI2,fBAT,OK/ERR
	STAT:0x10000000000000000000,OK
	0.0,0.9,0.9,0.9,0.9,0.9,0x060000000000060000000000,0x060000000000060000000000,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,-0.0,OK
	INIT,PARALLEL TO DAISYCHAIN,0x10000000000000000000,STAT:0x10000000000000000000,OK
	!!!!
	!!!! found new functional configuration (PARALLEL TO DAISYCHAIN)
	!!!!
	100,0,0,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1228,OK
	100,1,6,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1224,OK
	99,0,0,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,2476,OK
	100,0,6,-0.00,25.8,28.0,0.0,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.0,0,-0.00,1220,OK
	...
	...
	!!!! normal operation
	...
	...
	*/
}

void setup()
{
	//Initialize serial and wait for port to open:
	Serial.begin(LTCDEF_BAUDRATE);
	// wait for serial port to connect. Needed for native USB port only
	while (!Serial);
	// disable drivers on all SPI pins
	// we do this, as to allow compatibility of Arduino Due
	// with existing Arduino shields that expect SPI signals 
	// on those pins, by routing the SPI header of DUE to
	// those pins.
	pinMode(11, INPUT);
	pinMode(12, INPUT);
	pinMode(13, INPUT);

	digitalWrite(LTCDEF_GPO, HIGH);
	pinMode(LTCDEF_GPO, OUTPUT);

	// configure SPI, also done in LTC2949.cpp:
	// also used for LTC681x
	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);

	Init(LTCDEF__CS, false);
}

void Init(uint8_t selCS, boolean ltc2949onTopOfDaisychain)
{
#ifdef LTCDEF_LTC681X_ONLY
	ltc2949onTopOfDaisychain = true;
#endif

	LTC2949_CS = selCS;
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,			*/LTCDEF_CELL_MONITOR_COUNT,
		/*boolean ltc2949onTopOfDaisychain, */ltc2949onTopOfDaisychain,
		/*boolean debugEnable				*/false
	);
	LTC2949_init_device_state();
	// 
	Serial.print(F("INIT,"));
#ifndef LTCDEF_LTC681X_ONLY
	if (LTC2949_onTopOfDaisychain)
		Serial.print(F("ON TOP OF"));
	else
		Serial.print(F("PARALLEL TO"));
	Serial.print(F(" DAISYCHAIN,"));
#endif

	Serial.print(F("CS:"));
	Serial.print(LTC2949_CS);
	PrintComma();
	//
	delay(LTC2949_TIMING_BOOTUP);
	byte error = 0;
#ifdef LTCDEF_LTC681X_ONLY
	mcuTime = millis();
	error |= CellMonitorInit();
#else
	error = WakeUpReportStatus();
	error |= Cont(false);
	mcuTime = millis();
	delay(LTC2949_TIMING_CONT_CYCLE);
	error |= CellMonitorInit();
	error |= Cont(true);
#endif

	PrintOkErr(error);
	PrintCSVHeader();
}

void PrintCellVoltages(uint16_t * cellMonDat, boolean init)
{
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT * 3; i++)
	{
		if (cellMonDat[i] != 0xFFFFU) // don't print non-existing cells
			Serial.print(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
		if (init)
			cellMonDat[i] = 0xFFFFU; // this allows to check if next cell voltages were read correctly
		PrintComma();
	}
}

String CellVoltagesToString(uint16_t * cellMonDat, uint8_t nic)
{
	String str = "";
	uint8_t i = nic * 3;
	uint8_t lasti = i + 3;
	for (; i < lasti; i++)
	{
		if (cellMonDat[i] != 0xFFFFU)
			str += String(cellMonDat[i] * 100e-6, LTCDEF_DIGITS_CELL);
		str += ',';
	}
	return str;
}

void loop()
{
	String str = "";
#ifndef LTCDEF_LTC681X_ONLY
	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
#endif

	uint16_t cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 3]; // = LTCDEF_CELL_MONITOR_COUNT * 6 bytes
	byte * buffer = (byte *)cellMonDat; // buffer and cellMonDat can share the same memory
	byte  error = 0;
	unsigned long timeBuffer = millis();
	unsigned long timerus = micros();

#ifndef LTCDEF_LTC681X_ONLY
	// store the last TBx value that was read (to be able to calc time difference later...)
	uint32_t deltaT = LTC2949_GetLastTBxInt();
#endif

#ifdef LTCDEF_DO_RESET_TEST
	if (deltaT > LTCDEF_DO_RESET_TEST / 1.0e3 / LTC2949_LSB_TB1)
	{
		LTC2949_reset();
		delay(LTC2949_TIMING_AUTO_SLEEP_MAX*1.5);
		Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
		return;
	}
#endif // LTCDEF_DO_RESET_TEST

	// LTC2949_ChkUpdate checks for changed in TBx (default is to check TB4)
	// this tells if we have updated values available in the slow channel registers
#ifdef LTCDEF_LTC681X_ONLY
	const boolean slowChannelReady = false;
#else
	boolean slowChannelReady = LTC2949_ChkUpdate(&error);
	if (slowChannelReady || LTC_TIMEOUT_CHECK(timeBuffer, mcuTime + LTC2949_TIMING_CONT_CYCLE))
	{
		// in case of any error below we will also enter here! (see last delay(LTC2949_TIMING_IDLE2CONT2UPDATE))
		error |= ChkDeviceStatCfg();
		slowChannelReady = true;
	}
#endif

	// new high precision results available
	// calc difference between last TBx and current TBx, report milliseconds
#ifndef LTCDEF_LTC681X_ONLY
	if (slowChannelReady)
		str += String((unsigned int)((LTC2949_GetLastTBxInt() - deltaT) * (1.0e3 * LTC2949_LSB_TB1))); //Serial.print((unsigned int)((LTC2949_GetLastTBxInt() - deltaT) * (1.0e3 * LTC2949_LSB_TB1)));
	else
#endif
		str += String(timeBuffer - mcuTime); //Serial.print(timeBuffer - mcuTime);
	mcuTime = timeBuffer;
	str += ',';//PrintComma();

#ifndef LTCDEF_LTC681X_ONLY
	// read high precision current I1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_I1, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_I1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I1SLOW);
	}
	str += ',';//PrintComma();

	// read high precision power P1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_P1, 3, buffer);
		str += String(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_P1SLOW); //Serial.print(LTC_3BytesToInt32(buffer) * LTC2949_LSB_P1 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_P1SLOW);
	}
	str += ',';//PrintComma();

	// read voltage BAT
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_BAT, 2, buffer);
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LTCDEF_DIGITS_BATSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_BAT, LTCDEF_DIGITS_BATSLOW);
	}
	str += ',';//PrintComma();

	// read temperature via SLOT1
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, buffer);
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
	}
	str += ',';//PrintComma();

	// read internal temperature
	if (slowChannelReady)
	{
		error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, buffer);
		str += String(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW); //Serial.print(LTC_2BytesToInt16(buffer) * LTC2949_LSB_TEMP, LTCDEF_DIGITS_TEMPSLOW);
	}
	str += ',';//PrintComma();
#endif

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif

	////////////////////////////////////////////////////////////////////
	// fast synchronous cell voltage and current measurement
	////////////////////////////////////////////////////////////////////
	// clear old cell voltage conversion results
	error |= LTC2949_68XX_ClrCells();

	error |= CellMonitorCFGA((byte*)cellMonDat, false);

	if (err_detected(error))
	{
		str = "";
		// not yet initialized or communication error or.... (e.g. ON TOP OF instead of PARALLEL TO DAISYCHAIN)
		PrintOkErr(error);
		delay(100); // we wait 0.1 second, just avoid too many trials in case of error.

		if (retries--)
		{
			Init(LTC2949_CS, LTC2949_onTopOfDaisychain);
		}
		else
		{
			// lets try to toggle isoSPI bus configuration and LTC6820 master (four possible combinations)
			// Note: here we treat all possible combinations to be equally likely. This way the boards
			// can be connected together in any way.
			// In a real system only two combinations would make sense.
			switch (LTC2949_CS + (LTC2949_onTopOfDaisychain ? (uint8_t)128 : (uint8_t)0))
			{
			case (LTCDEF__CS + 0):
				Init(LTCDEF_GPO, false);
				break;
			case (LTCDEF_GPO + 0):
				Init(LTCDEF__CS, true);
				break;
			case (LTCDEF__CS + 128):
				Init(LTCDEF_GPO, true);
				break;
			case (LTCDEF_GPO + 128):
			default:
				Init(LTCDEF__CS, false);
				break;
			}
			retries = LTCDEF_ERR_RETRIES;
		}
		return;
	}

	if (LTCDEF_POLL_EOC)
		timeBuffer = micros();

	// trigger measurement (broadcast command will trigger cell voltage and current measurement)
	error |= LTC2949_ADxx(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */LTCDEF_POLL_EOC ? LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_68XX_T6C_27KHZ_US) : 0
	);


#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif


	cellMonDat[0] = 0xFFFFU; // will be used later

#ifdef LTCDEF_LTC681X_ONLY
	timeBuffer = micros() + LTC2949_68XX_T6C_27KHZ_US;
	// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
	while (!LTC_TIMEOUT_CHECK(micros(), timeBuffer))
		; // wait for all cell voltage measurements to be completed.
	error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVA, cellMonDat);

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif

#else
	if (!LTCDEF_POLL_EOC)
	{
		timeBuffer = micros();
		// for sure we poll for HS of LTC2949's rdcv later, so we do not have to wait here!
		while (false && !LTC_TIMEOUT_CHECK(micros(), timeBuffer + LTC2949_68XX_T6C_27KHZ_US))
			; // wait for all cell voltage measurements to be completed.
	}
	fastData2949[LTC2949_RDFASTDATA_HS] = 0; // clear the HS bytes
	// poll LTC2949 for conversion done
	error |= LTC2949_RdFastData(
		fastData2949,
		cellMonDat,
		LTC2949_68XX_CMD_RDCVA,
		LTC2949_68XX_GETADCVTIMEOUT16US(LTC2949_FASTSSHT_RDY_TIME_US));
	// calc cycle time (ADCV to "results ready")
	deltaT = micros() - timeBuffer;
	// check if we already read valid data from LTC2949
	if (LTC2949_FASTSSHT_HS_OK(fastData2949))
	{
		; // all fine, nothing to do.
	}
	else if (LTC2949_FASTSSHT_HS_CLR(fastData2949))
	{
		// we polled for HS==0x0F before, so it cannot be all HS are zero! something went wrong (e.g. timeout)
		error |= LTC2949_ERRCODE_OTHER;
	}
	else if (LTC2949_FASTSSHT_HS_LAST_OK(fastData2949)) // first HS != 0x0F, last HS == 0x0F
	{
		// we have to read data from LTC2949 again, as only the next RDCVx will report the final conversion results
		// also cell voltages have to be read again, as also those most probably were not updated
		// note: here we must not poll HS! (it must be zero now!)
		error |= LTC2949_RdFastData(
			fastData2949,
			cellMonDat,
			LTC2949_68XX_CMD_RDCVA);

		if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // HS must be cleared now
			error |= LTC2949_ERRCODE_OTHER; // this must never happen in case of fast single shot events
	}
	else
	{
		// Unexpected HS bytes, something went wrong
		error |= LTC2949_ERRCODE_OTHER;
	}
#endif

	char errorExt = error > 1 ? 'X' : '_';
	if (!LTC2949_onTopOfDaisychain || (cellMonDat[0] == 0xFFFFU))
	{
		if (errorExt == '_') errorExt = 'Y';

		// we have to read cell voltages group A (again)
		// for sure we have to read in case LTC2949 is not on top of daisychain!
		error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVA, cellMonDat);
	}
	if (true) // print cell voltages
	{
		String cvs[LTCDEF_CELL_MONITOR_COUNT];

		for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
			cvs[i] = "";

		for (uint8_t rdcvi = 0; ; rdcvi++)
		{
			for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
				cvs[i] += CellVoltagesToString(cellMonDat, i);

			if ((rdcvi > 4) || (rdcvi >= (LTCDEF_CELLS_PER_CELL_MONITOR_COUNT / 3 - 1)))
				break;

			switch (rdcvi)
			{
			case 0:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVB, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'b';
				break;
			case 1:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVC, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'c';
				break;
			case 2:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVD, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'd';
				break;
			case 3:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVE, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'e';
				break;
			default:
			case 4:
				error |= LTC2949_68XX_RdCells(LTC2949_68XX_CMD_RDCVF, cellMonDat);
				if (error > 1 && errorExt == '_') errorExt = 'f';
				break;
			}
		}
		for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
		{
			str += cvs[i]; //Serial.print(cvs[i]);
			cvs[i] = "";
		}
		str += errorExt; //Serial.print(errorExt);
		str += ',';//PrintComma();
	}

#ifdef LTCDEF_TIMING_DEBUG
	str += String(micros() - timerus);
	str += ',';
#endif


#ifndef LTCDEF_LTC681X_ONLY
	// print fast I2
	str += String(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_I2] * LTC2949_LSB_FIFOI2 / LTCDEF_SENSE_RESISTOR, LTCDEF_DIGITS_I2FAST);
	str += ',';//PrintComma();
	// print fast BAT
	str += String(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST); //Serial.print(fastData2949[LTC2949_RDFASTDATA_BAT] * LTC2949_LSB_FIFOBAT, LTCDEF_DIGITS_BATFAST);
	// clear the EOC by reading again if necessary:
	if (!LTC2949_FASTSSHT_HS_CLR(fastData2949))
		error |= LTC2949_RdFastData(fastData2949);
	if (!LTC2949_FASTSSHT_HS_CLR(fastData2949)) // for sure HS bytes must be cleared now
		error |= LTC2949_ERRCODE_OTHER;
	str += ',';//PrintComma();
	str += String(deltaT); //Serial.print(deltaT);
	str += ',';//PrintComma();
#endif
	Serial.print(str);
	str = "";
	PrintOkErr(error);
	if (err_detected(error)) // in case of error we sleep to avoid too many error reports and also to make sure we call ChkDeviceStatCfg in the next loop 
		delay(LTC2949_TIMING_IDLE2CONT2UPDATE);
}

byte ReadPrintCellVoltages(uint16_t rdcv, uint16_t * cellMonDat)
{
	// in case LTC2949 is parallel to the daisychain, we now read only the cell voltages
	byte error = LTC2949_68XX_RdCells(rdcv, cellMonDat);
	PrintCellVoltages(cellMonDat, true);
	return error;
}

#ifndef LTCDEF_LTC681X_ONLY
/*!*********************************************************************
\brief Checks if the device as awake and in slow and fast continuous mode
This will restart the whole measurement e.g. in case the device was
powered off or there is a severe communication issue.
This can be demonstrated by just powering off the device or unplugging
the isoSPI interface while measurement is running.
***********************************************************************/
byte ChkDeviceStatCfg()
{
	byte error;
	byte data[10];
	byte dataOthers;
	boolean expChkFailed;

	// check STATUS (EXT)FAULTS, ALERT registers
	error = LTC2949_ReadChkStatusFaults(
		/*boolean lockMemAndClr:    */ false,
		/*boolean printResult:      */ false,
		/*byte len:                 */ 10,
		/*byte * statFaultsExpAndRd:*/ data,
		/*boolean * expChkFailed:   */ &expChkFailed,
		/*byte expDefaultSet):      */ LTC2949_STATFAULTSCHK_IGNORE_STATUPD | LTC2949_STATFAULTSCHK_DFLT_AFTER_CLR);

	if (err_detected(error))
		return error;

	if (expChkFailed)
	{
		error = LTC2949_ERRCODE_OTHER; // STATUS (EXT)FAULTS, ALERT check failed
		SerialPrintByteArrayHex(data, 10, true); // report the status
		PrintComma();
		return LTC2949_ERRCODE_OTHER;
	}

	// check BRCEN bit
	if (err_detected(error = LTC2949_READ(LTC2949_REG_REGSCTRL, 1, &dataOthers)))
		return error; // PEC error
	if (bitMaskSetClrChk(dataOthers, LTC2949_BM_REGSCTRL_BCREN, !LTC2949_onTopOfDaisychain))
		return LTC2949_ERRCODE_OTHER; // BRCEN != LTC2949_onTopOfDaisychain

	if (err_detected(error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &dataOthers)))
		return error; // PEC error
	if (dataOthers != LTC2949_BM_OPCTRL_CONT)
		return LTC2949_ERRCODE_OTHER; // not in continuous mode

	if (err_detected(error = LTC2949_READ(LTC2949_REG_FACTRL, 1, &dataOthers)))
		return error; // PEC error
	if (dataOthers != LTCDEF_FACTRL_CONFIG)
		return LTC2949_ERRCODE_OTHER;  // not or wrong fast mode

	if (err_detected(error = LTC2949_ADCConfigRead(&dataOthers)))
		return error; // PEC error
	if (dataOthers != LTCDEF_ADCCFG_CONFIG)
		return LTC2949_ERRCODE_OTHER; // wrong ADC configuration

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
byte Cont(boolean enable)
{
	if (enable)
	{
		byte error = 0;
#ifdef LTCDEF_READ_FROM_EEPROM
		error = LTC2949_EEPROMRead();
#else
		// fast slot not used, still we configure something
		LTC2949_SlotFastCfg(3, 2);
		// SLOT1 measures temperature via NTC between V1 and GND. SLOT2 not used, still we configure something
		LTC2949_SlotsCfg(1, 0, 4, 5);
		// enable NTC temperature measurement via SLOT1
		NtcCfgWrite(1, NTC_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C);
#endif

#ifdef LTCDEF_WRITE_TO_EEPROM
		error = LTC2949_EEPROMWrite();
#endif

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
#endif




byte CellMonitorCFGA(byte * cellMonDat, bool verbose)
{
	// read configuration and print
	byte error = LTC2949_68XX_RdCfg(cellMonDat);
	if (verbose)
	{
		SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
		PrintComma();
	}
	// set REFON in configuration registers
	for (uint8_t i = 0; i < LTCDEF_CELL_MONITOR_COUNT; i++)
	{
		cellMonDat[i * 6 + 0] = 1 << 2; //REFON, all others zero
		cellMonDat[i * 6 + 4] = 0; //clear all DCC
		cellMonDat[i * 6 + 5] = 0; //clear all DCC
	}
	// write configuration registers
	error |= LTC2949_68XX_WrCfg(cellMonDat);
	return error;
}

byte CellMonitorInit()
{
	byte cellMonDat[LTCDEF_CELL_MONITOR_COUNT * 6];
	LTC2949_68XX_RdCfg(cellMonDat); // dummy read
	// dummy read of cell voltage group A
	byte error = ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
	// clear all cell voltage groups
	error |= LTC2949_68XX_ClrCells();

	error |= CellMonitorCFGA(cellMonDat, true);

	// read configuration and print
	error |= LTC2949_68XX_RdCfg(cellMonDat);
	SerialPrintByteArrayHex(cellMonDat, LTCDEF_CELL_MONITOR_COUNT * 6, true);
	PrintComma();
	error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
	// trigger cell voltage measurement of all cells in fast mode
	error |= LTC2949_ADxx(
		/*byte md = MD_NORMAL     : */MD_FAST,
		/*byte ch = CELL_CH_ALL   : */CELL_CH_ALL,
		/*byte dcp = DCP_DISABLED : */DCP_DISABLED,
		/*uint8_t pollTimeout = 0 : */0
	);
	delay(2); // wait for conversion results ready to be read
	// print all conversion results
	error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVA, (uint16_t*)cellMonDat);
	// this is just for debugging anyway, so we don't print all values here. See main loop for the actual cyclic measurements
	//error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVB, (uint16_t*)cellMonDat);
	//error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVC, (uint16_t*)cellMonDat);
	//error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVD, (uint16_t*)cellMonDat);
	//error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVE, (uint16_t*)cellMonDat);
	//error |= ReadPrintCellVoltages(LTC2949_68XX_CMD_RDCVF, (uint16_t*)cellMonDat);
	return error;
}

