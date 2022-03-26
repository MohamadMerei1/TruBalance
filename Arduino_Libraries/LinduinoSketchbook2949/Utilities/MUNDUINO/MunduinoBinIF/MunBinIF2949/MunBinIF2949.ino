/*!
MunBinIF2949
LTC2949: Battery monitor GUI communication interface

@verbatim

NOTES
Setup:
Set the terminal baud rate to 115200
Its a binary format interface for maximum throughput on the serial port

GENERAL:
  Command send from master to slave (PC to Linduino):
	Byte0:
	  high nipple: Command (e.g. read/write)
	  low  nipple: Command argument (e.g. bits 11..8 of length / number of bytes)
	Byte1: Optional argument0: e.g. bits 7..0 of length / number of bytes
	Byte2: Optional argument1: e.g. address
	Byte3: Optional argument2: e.g. for write the 1st data byte
	...
	ByteN: Optional argumentN-1: e.g. for write the last data byte

  Reply structure from slave to master (Linduino to PC)
	Byte0:
	  high nipple: Echo of the command
	  low  nipple: Error code (0 if OK)
	Byte1: Optional reply 0: e.g. for read the 1st data byte
	Byte2: Optional reply 1: e.g. for read the 2nd data byte
	...
	ByteM: Optional reply N-1: e.g. for read the last data byte



COMMANDS:
  RAW RD/WR COMMAND (0x00):
	Byte0:
	  high nipple: 0x0
	  low  nipple: bits 11..8 of length (number of bytes)
	Byte1: bits 7..0 of length (number of bytes)
	Byte2: write byte 0
	Byte3: write byte 1
	...
	ByteN: write byte last

  REPLY:
	Byte0:
	  high nipple: 0
	  low  nipple: error code (0 if ok, here it is always 0!)
	Byte1: read byte 0
	Byte2: read byte 1
	...
	ByteM: read byte last

  ECHO COMMAND (0xE0):
	same as RAW RD/WR COMMAND (0x00) with following differences:
	high nipple: 0xE
	The reply sends back the written bytes

  SPI CFG COMMAND (0x50):
	Byte0:
		high nipple: 0x5
		low  nipple: CSpol / SPI mode: bit[3]=CSpol, bit[2]=MSBfirst, bits[1:0]=mode
		mode: see https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus#Clock_polarity_and_phase
			SPI Mode Clock Polarity (CPOL/CKP) Clock Phase (CPHA) Clock Edge (CKE/NCPHA)
			0        0                         0                  1
			1        0                         1                  0
			2        1                         0                  1
			3        1                         1                  0
		CSpol: 1 high active
		MSBfirst: 1 MSB first
	Byte1: UINT32,SPICLK[31:24]
	Byte2: UINT32,SPICLK[23:16]
	Byte3: UINT32,SPICLK[15:8]
	Byte4: UINT32,SPICLK[7:0]
	SPICLK in Hz

	REPLY:, just the command echo 0x50

*/

#include <Arduino.h>
#include <ltcmuc_tools.h>
#include <SPI.h>
#include <LTC2949.h>
#include <Linduino.h>
//#include "stdlib.h"
//#include "pec15_utils.h"
//#include "LTC2949.h"
//#include "ltcmuc_tools.h"

/*!**********************************************************************
\brief  global defines
***********************************************************************/
// COMMANDS
#define MUNBINIF2949_CMD_RAWRW   0x00
#define MUNBINIF2949_CMD_WRITE   0x10
#define MUNBINIF2949_CMD_READ    0x20
#define MUNBINIF2949_CMD_WKUP    0x30
#define MUNBINIF2949_CMD_4       0x40
#define MUNBINIF2949_CMD_SPICFG  0x50
#define MUNBINIF2949_CMD_COMCFG  0x60
#define MUNBINIF2949_CMD_RDCELL  0x70
#define MUNBINIF2949_CMD_RDFAST  0x80
#define MUNBINIF2949_CMD_ADCV    0x90
#define MUNBINIF2949_CMD_RDFIFO  0xA0
#define MUNBINIF2949_CMD_B       0xB0
#define MUNBINIF2949_CMD_C       0xC0
#define MUNBINIF2949_CMD_SPIRW   0xD0
#define MUNBINIF2949_CMD_ECHO    0xE0
#define MUNBINIF2949_CMD_ERR     0xF0
// COMMAND ARGUMENT FLAGS / MASKS / definitions
#define MUNBINIF2949_CMDARG_FLAG_ERR       0x01
// CMD_SPICFG CMDARGs
#define MUNBINIF2949_CMDARG_FLAG_SPI_CS_POL   0x08
#define MUNBINIF2949_CMDARG_FLAG_SPI_MSBFIRST 0x04
#define MUNBINIF2949_CMDARG_MASK_SPI_MODE     0x03
#define MUNBINIF2949_CMDARG_SPI_MODE0         0x00
#define MUNBINIF2949_CMDARG_SPI_MODE1         0x01
#define MUNBINIF2949_CMDARG_SPI_MODE2         0x02
#define MUNBINIF2949_CMDARG_SPI_MODE3         0x03
// CMD_COMCFG CMDARGs
#define MUNBINIF2949_CMDARG_FLAG_COM_BRDCSTRD        0x01
#define MUNBINIF2949_CMDARG_FLAG_COM_INDIRECTRD      0x02
#define MUNBINIF2949_CMDARG_FLAG_COM_AUTOWRPECERRCHK 0x04
#define MUNBINIF2949_CMDARG_FLAG_COM_RES             0x08



// MASKS
#define MUNBINIF2949_CMD_MASK    0xF0
#define MUNBINIF2949_CMDARG_MASK 0x0F

/*!**********************************************************************
\brief  global variables
***********************************************************************/
#define MUNBINIF2949_BUFFER_SIZE 1024
byte dataiobuff[MUNBINIF2949_BUFFER_SIZE];
byte *cellMonDat = NULL;

boolean csPolHigh = false;

/*!**********************************************************************
 \brief  Inititializes hardware and variables
 ***********************************************************************/
void setup()
{
	Serial.begin(115200);
	//LTC2949_init_spi(); // This calls LTC2949_init_lib and LTC2949_init_device_state for isoSPI default settings
	//LTC2949_init_isoSpi();
	// LTC2949_initialize(
	// 	/*daisyChainLength:*/1,//<! 0: stdSpi, 1: isoSPI LTC2949 only, >=2: daisychain, LTC2949 on top
	// 	/*brdcstRd        :*/ false,
	// 	/*indirectRd      :*/ false,
	// 	/*autoWrPecErrChk :*/ false, // true is not compatible with MunBinIF2949 as it would always change to page 0 (to read faults) to check for write PEC errors! Thus it would be impossible for the GUI to switch to page 1
	// 	/*debugEnable     :*/ false);
	// LTC2949_init_device_state();
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	// configure SPI, also done in LTC2949.cpp:
	LTC2949_SPISettings = SPISettings(LTC2949_MAX_SPIFREQU, MSBFIRST, LTC2949_DEFAULT_SPIMODE);
}

/*!*********************************************************************
  \brief main loop

  ***********************************************************************/
void loop()
{
	uint8_t command;
	uint8_t cmdArgs;

	if (!Serial.available())           // Check for user input
		return;

	// 1st byte is always the command
	command = Serial.read();
	cmdArgs = command & MUNBINIF2949_CMDARG_MASK;
	command &= MUNBINIF2949_CMD_MASK;

	// decode command:
	switch (command)
	{
	case MUNBINIF2949_CMD_RAWRW:
		CmdRawRW(cmdArgs);
		break;
	case MUNBINIF2949_CMD_WKUP:
		CmdWkUp();
		break;

	case MUNBINIF2949_CMD_ECHO:
		CmdEcho(cmdArgs);
		break;

	case MUNBINIF2949_CMD_READ:
		CmdRead(cmdArgs);
		break;

	case MUNBINIF2949_CMD_WRITE:
		CmdWrite(cmdArgs);
		break;

	case MUNBINIF2949_CMD_SPICFG:
		CmdConfigSPI(cmdArgs);
		break;

	case MUNBINIF2949_CMD_COMCFG:
		CmdConfigCom(cmdArgs);
		break;

	case MUNBINIF2949_CMD_RDCELL:
		CmdGetCellMonData();
		break;

	case MUNBINIF2949_CMD_RDFAST:
		CmdRdFastData();
		break;

	case MUNBINIF2949_CMD_RDFIFO:
		CmdRdFIFO(cmdArgs);
		break;

	case MUNBINIF2949_CMD_ADCV:
		CmdAdcv();
		break;

	default: // UNKNOWN COMMAND
		Serial.write((uint8_t)(MUNBINIF2949_CMD_ERR));
		break;
	}
}

void spiStartCSHigh()
{
	SPI.beginTransaction(LTC2949_SPISettings);
	digitalWrite(LTC2949_CS, HIGH);
	delayMicroseconds(1);
}

void spiStopCSLow()
{
	delayMicroseconds(1);
	digitalWrite(LTC2949_CS, LOW);
	SPI.endTransaction();
}

static inline void CmdRawRW(uint8_t cmdArgs)
{
	uint16_t len = GetLength(cmdArgs);
	SerialWaitAndReadBytes(dataiobuff, len); // get data to be written to 2949
	LTC2949_wakeup_idle(); //This will guarantee that the LTC2949 isoSPI port is awake.
	if (csPolHigh)
		spiStartCSHigh();
	else
		spiStart();
	for (uint16_t i = 0; i < len; i++)
		dataiobuff[i] = spi_read(dataiobuff[i]);
	if (csPolHigh)
		spiStopCSLow();
	else
		spiStop();
	Serial.write((uint8_t)(MUNBINIF2949_CMD_RAWRW | 0)); // echo command no error
	SerialWaitAndWriteBytes(dataiobuff, len);
}


static inline void CmdConfigCom(uint8_t cmdArgs)
{
	uint8_t daisyChainLength = SerialWaitAndReadByte();
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */daisyChainLength,
		/*boolean ltc2949onTopOfDaisychain,*/bitMaskSetChk(cmdArgs, MUNBINIF2949_CMDARG_FLAG_COM_BRDCSTRD | MUNBINIF2949_CMDARG_FLAG_COM_INDIRECTRD),
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	// COMMAND SUCCESSFULL
	Serial.write((uint8_t)(MUNBINIF2949_CMD_COMCFG)); // echo command
}

static inline void CmdWkUp()
{
	if (LTC2949_WakeupAndAck())
		Serial.write((uint8_t)(MUNBINIF2949_CMD_WKUP | MUNBINIF2949_CMDARG_FLAG_ERR)); // Error
	else
		Serial.write((uint8_t)(MUNBINIF2949_CMD_WKUP)); // echo command
}

static inline void CmdEcho(uint8_t cmdArgs)
{
	uint16_t len;
	// get length
	len = GetLength(cmdArgs);
	// get data to be echoed
	SerialWaitAndReadBytes(dataiobuff, len);
	Serial.write((uint8_t)(MUNBINIF2949_CMD_ECHO)); // echo command
	SerialWaitAndWriteBytes(dataiobuff, len); // echo data
}

static inline void CmdRdFIFO(uint8_t cmdArgs)
{
	uint16_t len = GetLength(cmdArgs);
	uint8_t fifoAddr = SerialWaitAndReadByte();
	int16_t *buffer = (int16_t *)dataiobuff;
	uint16_t samplesRead = 0;
	while (samplesRead < len)
	{
		uint16_t n = len - samplesRead;
		if (n > MUNBINIF2949_BUFFER_SIZE / 2)
			n = MUNBINIF2949_BUFFER_SIZE / 2;
		uint8_t error = LTC2949_ReadFifo(fifoAddr, &n, buffer);

		if (error)
			Serial.write((uint8_t)(MUNBINIF2949_CMD_RDFIFO | MUNBINIF2949_CMDARG_FLAG_ERR)); // echo command
		else
			Serial.write((uint8_t)(MUNBINIF2949_CMD_RDFIFO)); // echo command
		Serial.write(n >> 8);
		Serial.write(n); // print number of samples = 2*n-bytes
		samplesRead += n;
		SerialWaitAndWriteBytes(dataiobuff, n * 2); // send data (Arduino is little endian!!! So LSB first)
		if (error)
			return;
	}
}

static inline void CmdAdcv()
{
	// make fast single shot
	LTC2949_ADxx();
	// READ SUCCESSFULL
	Serial.write((uint8_t)(MUNBINIF2949_CMD_ADCV)); // echo command
}

static inline void CmdRdFastData()
{
	int16_t * fastData2949 = (int16_t*)dataiobuff;

	//if ((LTC2949_isoSpiPartsInDaisyChain > 1) &&
	//	(LTC2949_RDFASTDATA_LENGTH * 2 + (LTC2949_isoSpiPartsInDaisyChain - 1) * 6 <= MUNBINIF2949_BUFFER_SIZE)
	//	)
	//{
	//	cellMonDat = dataiobuff + LTC2949_RDFASTDATA_LENGTH * 2;
	//}
	//else
	//{
	//	cellMonDat = NULL;
	//}
	uint8_t error = LTC2949_RdFastData(fastData2949);// , cellMonDat);
	// check for correct HS bytes
	if (error || !LTC2949_FASTSSHT_HS_LAST_OK(fastData2949))
		// ERRORS
		Serial.write((uint8_t)(MUNBINIF2949_CMD_RDFAST | MUNBINIF2949_CMDARG_FLAG_ERR)); // echo command with error flag set
	else
		// READ SUCCESSFULL
		Serial.write((uint8_t)(MUNBINIF2949_CMD_RDFAST)); // echo command
	SerialWaitAndWriteBytes(dataiobuff, LTC2949_RDFASTDATA_LENGTH * 2); // send data (Arduino is little endian!!! So LSB first)
}

static inline void CmdRead(uint8_t cmdArgs)
{
	uint16_t len;
	uint16_t address;
	// get length
	len = GetLength(cmdArgs);
	// wait for and read address
	address = SerialWaitAndReadByte();
	// adjust address with current page
	if (LTC2949_GetPageShadowBit())
		address |= 0x100; // this makes sure LTC2949_READ won't change page

	uint8_t error;
	//if ((LTC2949_CellMonitorCount > 0) &&
	//	(len + LTC2949_CellMonitorCount * 6 <= MUNBINIF2949_BUFFER_SIZE)
	//	)
	//{
	//	cellMonDat = dataiobuff + len;
	//	_LTC2949_WR_PAGE_ADDR_(address);
	//	error = _LTC2949_IndirectRead_(len, dataiobuff, cellMonDat); // store also data from cell monitors
	//}
	//else
	//{
	//	cellMonDat = NULL;
	error = LTC2949_READ(address, len, dataiobuff);
	//}
	if (error) // read from 2949
		// PEC ERRORS
		Serial.write((uint8_t)(MUNBINIF2949_CMD_READ | MUNBINIF2949_CMDARG_FLAG_ERR)); // echo command with error flag set
	else
		// READ SUCCESSFULL
		Serial.write((uint8_t)(MUNBINIF2949_CMD_READ)); // echo command
	SerialWaitAndWriteBytes(dataiobuff, len);
}


static inline void CmdGetCellMonData()
{
	if (cellMonDat)
	{
		// SUCCESSFULL
		Serial.write((uint8_t)(MUNBINIF2949_CMD_RDCELL)); // echo command
		SerialWaitAndWriteBytes(cellMonDat, (LTC2949_CellMonitorCount) * 6);
	}
	else
	{
		// no data
		Serial.write((uint8_t)(MUNBINIF2949_CMD_RDCELL | MUNBINIF2949_CMDARG_FLAG_ERR)); // echo command
	}
}

static inline void CmdWrite(uint8_t cmdArgs)
{
	uint16_t len;
	uint16_t address;
	// get length
	len = GetLength(cmdArgs);
	// wait for and read address
	address = SerialWaitAndReadByte();
	// adjust address with current page
	if (LTC2949_GetPageShadowBit())
		address |= 0x100; // this makes sure LTC2949_WRITE won't change page

	SerialWaitAndReadBytes(dataiobuff, len); // get data to be written
	// write to 2949
	//if (
	LTC2949_WRITE(address, len, dataiobuff);
	//)
// PEC ERRORS
//Serial.write((uint8_t)(MUNBINIF2949_CMD_WRITE | MUNBINIF2949_CMDARG_FLAG_ERR)); // echo command with error flag set
//else
	// WRITE SUCCESSFULL
	Serial.write((uint8_t)(MUNBINIF2949_CMD_WRITE)); // echo command
}

static inline void CmdConfigSPI(uint8_t cmdArgs)
{
	// get SPI mode and bit order from command low nipple
	boolean msbFirst = bitMaskSetChk(cmdArgs, MUNBINIF2949_CMDARG_FLAG_SPI_MSBFIRST);
	csPolHigh = bitMaskSetChk(cmdArgs, MUNBINIF2949_CMDARG_FLAG_SPI_CS_POL);
	uint8_t spiMode = cmdArgs & MUNBINIF2949_CMDARG_MASK_SPI_MODE;
	uint32_t spiClock = 0;

	for (uint8_t i = 0; i < 4; i++)
		spiClock = spiClock << 8 | SerialWaitAndReadByte();

	LTC2949_SPISettings = SPISettings(
		spiClock,
		msbFirst ? MSBFIRST : LSBFIRST,
		spiMode == 0 ? SPI_MODE0 :
		spiMode == 1 ? SPI_MODE1 :
		spiMode == 2 ? SPI_MODE2 :
		SPI_MODE3);

	digitalWrite(LTC2949_CS, csPolHigh ? LOW : HIGH);

	// LTC2949_SetSPISettings(
	// 	spiClock,
	// 	//msbFirst ? MSBFIRST : LSBFIRST,
	// 	spiMode == 0 ? SPI_MODE0 :
	// 	spiMode == 1 ? SPI_MODE1 :
	// 	spiMode == 2 ? SPI_MODE2 :
	// 	SPI_MODE3
	// );
	// COMMAND SUCCESSFULL
	Serial.write((uint8_t)(MUNBINIF2949_CMD_SPICFG)); // echo command
}


static inline uint8_t SerialWaitAndReadByte()
{
	while (!Serial.available())
		;
	return Serial.read();
}

static inline uint16_t GetLength(uint8_t msb)
{
	// get MSB part of number of bytes
	uint16_t len = msb;
	// wait for LSB of number fo bytes
	len = (len << 8) | SerialWaitAndReadByte();
	return len;
}

static inline void SerialWaitAndReadBytes(byte* buffer, uint16_t len)
{
	// get data to be echoed
	for (uint16_t i = 0; i < len; i++)
		buffer[i] = SerialWaitAndReadByte();
}

static inline void SerialWaitAndWriteBytes(byte* buffer, uint16_t len)
{
	if (len == 0)
		return;
	uint16_t bytesWritten;
	do
	{
		bytesWritten = Serial.write(buffer, len);
		if (bytesWritten == len)
			break;
		buffer += bytesWritten;
		len -= bytesWritten;
	} while (true);
}
