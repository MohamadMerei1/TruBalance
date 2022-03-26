/*
  Example sketch for MCP2515, DC2617A, LTC2949.....
 */

#include "Linduino.h"
#include <SPI.h>
#include "LTC2949.h"
#include "MCP2515.h"
#include "ltcmuc_tools.h"

 // LTC2949_REG_FACTRL only uses bits 3..0
 // we use bit 7 to store the information if ADCV was send
#define CFGFAST_ADCVSEND_FLAG (1U<<7)
byte  cfgFast = 0;
boolean measEnable = false;
boolean measACC = false;
boolean measI1 = false;
boolean measI2 = false;
boolean measP1 = false;
boolean measP2 = false;
boolean measSLOT1 = false;
boolean measSLOT2 = false;
boolean measBAT = false;
boolean measTEMP = false;
boolean measVCC = false;
boolean measVREF = false;

unsigned long sendCANSlowTimeout;
uint8_t  canSendMsgSlowCycleTime100ms = 0xFFU; //0xFFU: disabled, 20 = 20*100ms = 2 seconds
unsigned long sendCANFastTimeoutMicros;
uint8_t  canSendMsgFastCycleTime1ms = 0; // default is disable
int16_t fastSamplesCnt = 0;

String SerialInputString = "";

byte* rawRWData = NULL;
byte  rawRWDataLen = 0;

struct MAStruct
{
	uint8_t   len;
	uint8_t   i;
	int32_t   sum;
	int16_t* samples;
};
MAStruct* movingAverage = NULL;

struct can_frame canMsg;
MCP2515* mcp2515 = NULL;


#define FIFO_NORMAL_READ

// serial baudrate
#define LTCDEF_BAUDRATE 250000

// for DC2617A use pin 9, for CANdiy-shield use pin 10
#define CAN_CS_PIN_NUMBER 9
//#define CAN_BAUDRATE 500000
#define CAN_BAUDRATE CAN_500KBPS
//#define CAN_BAUDRATE CAN_1000KBPS

////////////////////////////////////////////////////
// Definitions for can masseges                   //
//												  //
/////////////////////////////////////////
// CAN MSG IDs                         //
/////////////////////////////////////////
// CAN MSG ID: Current1, Power1, BAT
#define CAN_ID_IP1V      0x110
// CAN MSG ID: Current2, Power2, TEMP
#define CAN_ID_IP2T      0x111
// CAN MSG ID: Slot1, Slot2, VREF, VCC (Slots as voltage)
#define CAN_ID_SVR       0x112
// CAN MSG ID: Slot1, Slot2 (Slots as temperature via NTC)
#define CAN_ID_SLOTSNTC  0x113
// CAN MSG ID: Power1, Power2 as voltage at BAT
#define CAN_ID_PASV      0x114
//#define CAN_ID_XXX     0x115
//#define CAN_ID_XXX     0x116
// CAN MSG ID: RAW read/write registers
#define CAN_ID_RAWRW     0x117
// offset for fast measurement fifos CAN MSG IDs
// ALL FIFO MESSAGE IDs must stay in the following subsequent order without gabs in between
#define CAN_ID_FIFO     0x118
// CAN MSG ID: fifo Current1
#define CAN_ID_FIFOI1   (CAN_ID_FIFO+0)
// CAN MSG ID: fifo Current2
#define CAN_ID_FIFOI2   (CAN_ID_FIFO+1)
// CAN MSG ID: fifo BAT
#define CAN_ID_FIFOBAT  (CAN_ID_FIFO+2)
// CAN MSG ID: fifo AUX
#define CAN_ID_FIFOAUX  (CAN_ID_FIFO+3)
// CAN MSG ID: fast single shot Current1, Current2, BAT, AUX
#define CAN_ID_FSSHT     0x11C
// CAN MSG ID: Request fast measurements (either fifo or fast single shot)
#define CAN_ID_FASTRQ    0x11D
// CAN MSG ID: Error message with error code
#define CAN_ID_ERR       0x11E
// CAN MSG ID: Configure LTC2949 cyclic measurements
#define CAN_ID_CFG2949   0x11F

// FIFO averages
#define CAN_ID_FIFOAVG     0x120
// CAN MSG ID: fifo Current1
#define CAN_ID_FIFOAVGI1   (CAN_ID_FIFOAVG+0)
// CAN MSG ID: fifo Current2
#define CAN_ID_FIFOAVGI2   (CAN_ID_FIFOAVG+1)
// CAN MSG ID: fifo BAT
#define CAN_ID_FIFOAVGBAT  (CAN_ID_FIFOAVG+2)
// CAN MSG ID: fifo AUX
#define CAN_ID_FIFOAVGAUX  (CAN_ID_FIFOAVG+3)
// CAN MSG ID: fast single shot Current2 moving average, BAT, AUX
#define CAN_ID_FSSHTMA     0x124
// CAN MSG ID: Accumulators C1, E1....
#define CAN_ID_ACC_C1      0x125
// CAN MSG ID: Accumulators C1, E1....
#define CAN_ID_ACC_E1      0x126
// CAN MSG ID: Accumulators C1, E1....
#define CAN_ID_ACC_TB1     0x127

// optional messages for additional comparators.
#define CAN_ID_ACC_C2      0x128
#define CAN_ID_ACC_E2      0x129
#define CAN_ID_ACC_TB2     0x12A
#define CAN_ID_ACC_C3      0x12B
#define CAN_ID_ACC_E4      0x12C
#define CAN_ID_ACC_TB3     0x12D
#define CAN_ID_ACC_TB4     0x12E

#undef ENABLE_CAN_ID_ACC_C2_TO_TB4


/////////////////////////////////////////
// CAN MSG CAN_ID_RAWRW                //
/////////////////////////////////////////
// byte index of WRITE flag
#define CAN_MSG_RAWRW_WRITE_BYTE    0
// byte index of COUNTDOWN value
#define CAN_MSG_RAWRW_CNTDWN_BYTE   0
// byte index of PAGE flag (=MSbit of 9-bit-Address)
#define CAN_MSG_RAWRW_PAGE_BYTE     0
// byte index of address (lower 8-bit of 9-bit-Address)
#define CAN_MSG_RAWRW_ADDR_BYTE     1
// maximum number of LTC2949's data bytes per CAN message (data written / read to/from LTC2949)
#define CAN_MSG_RAWRW_DATA_BYTES    6
// byte index of LTC2949's data byte 0
#define CAN_MSG_RAWRW_D0_BYTE       2
// byte index of LTC2949's data byte 1
#define CAN_MSG_RAWRW_D1_BYTE       3
// byte index of LTC2949's data byte 2
#define CAN_MSG_RAWRW_D2_BYTE       4
// byte index of LTC2949's data byte 3
#define CAN_MSG_RAWRW_D3_BYTE       5
// byte index of LTC2949's data byte 4
#define CAN_MSG_RAWRW_D4_BYTE       6
// byte index of LTC2949's data byte 5
#define CAN_MSG_RAWRW_D5_BYTE       7
// bit number of PAGE flag
#define CAN_MSG_RAWRW_PAGE_BIT     0
// bit offset of COUNTDOWN value
#define CAN_MSG_RAWRW_CNTDWN_BIT   1
// length of COUNTDOWN value
#define CAN_MSG_RAWRW_CNTDWN_LEN   6
// mask for COUNTDOWN value
#define CAN_MSG_RAWRW_CNTDWN_MASK  ((1<<CAN_MSG_RAWRW_CNTDWN_LEN)-1)
// bit number of WRITE flag
#define CAN_MSG_RAWRW_WRITE_BIT    7

/////////////////////////////////////////
// CAN MSG CAN_ID_CFG2949              //
/////////////////////////////////////////
// byte 0
// byte index of CH1FAST flag
#define CAN_MSG_CFG2949_CH1FAST_BYTE       0
#define CAN_MSG_CFG2949_CH1FAST_BIT        0
// byte index of CH2FAST flag					  
#define CAN_MSG_CFG2949_CH2FAST_BYTE       0
#define CAN_MSG_CFG2949_CH2FAST_BIT        1
// byte index of  flag					  
#define CAN_MSG_CFG2949_AUXFAST_BYTE       0
#define CAN_MSG_CFG2949_AUXFAST_BIT        2
// byte index of  flag					  
#define CAN_MSG_CFG2949_FCONT_BYTE         0
#define CAN_MSG_CFG2949_FCONT_BIT          3
// byte index of  flag					  
#define CAN_MSG_CFG2949_SLOT1NTC_BYTE      0
#define CAN_MSG_CFG2949_SLOT1NTC_BIT       4
// byte index of  flag					  
#define CAN_MSG_CFG2949_SLOT2NTC_BYTE      0
#define CAN_MSG_CFG2949_SLOT2NTC_BIT       5
// byte index of  flag					  
#define CAN_MSG_CFG2949_P1ASV_BYTE         0
#define CAN_MSG_CFG2949_P1ASV_BIT          6
// byte index of  flag					  
#define CAN_MSG_CFG2949_P2ASV_BYTE         0
#define CAN_MSG_CFG2949_P2ASV_BIT          7
// byte 1
// byte index of  flag
#define CAN_MSG_CFG2949_SLOT1P_BYTE        1
#define CAN_MSG_CFG2949_SLOT1P_BIT	       0
// byte index of  flag				     
#define CAN_MSG_CFG2949_SLOT1N_BYTE        1
#define CAN_MSG_CFG2949_SLOT1N_BIT	       4
// byte 2							     
// byte index of  flag				     
#define CAN_MSG_CFG2949_SLOT2P_BYTE        2
#define CAN_MSG_CFG2949_SLOT2P_BIT	       0
// byte index of  flag				     
#define CAN_MSG_CFG2949_SLOT2N_BYTE        2
#define CAN_MSG_CFG2949_SLOT2N_BIT	       4
// byte 3							     
// byte index of  flag				     
#define CAN_MSG_CFG2949_SLOTFP_BYTE        3
#define CAN_MSG_CFG2949_SLOTFP_BIT	       0
// byte index of  flag				     
#define CAN_MSG_CFG2949_SLOTFN_BYTE        3
#define CAN_MSG_CFG2949_SLOTFN_BIT	       4
// byte 4
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_I1_BYTE       4
#define CAN_MSG_CFG2949_MEAS_I1_BIT	       0
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_I2_BYTE       4
#define CAN_MSG_CFG2949_MEAS_I2_BIT	       1
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_P1_BYTE       4
#define CAN_MSG_CFG2949_MEAS_P1_BIT	       2
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_P2_BYTE       4
#define CAN_MSG_CFG2949_MEAS_P2_BIT	       3
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_SLOT1_BYTE    4
#define CAN_MSG_CFG2949_MEAS_SLOT1_BIT     4
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_SLOT2_BYTE    4
#define CAN_MSG_CFG2949_MEAS_SLOT2_BIT     5
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_BAT_BYTE      4
#define CAN_MSG_CFG2949_MEAS_BAT_BIT       6
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_TEMP_BYTE     4
#define CAN_MSG_CFG2949_MEAS_TEMP_BIT      7
// byte 5
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_VCC_BYTE      5
#define CAN_MSG_CFG2949_MEAS_VCC_BIT       0
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_VREF_BYTE     5
#define CAN_MSG_CFG2949_MEAS_VREF_BIT      1
// byte index of  flag
#define CAN_MSG_CFG2949_NTC1_TYPE_BYTE     5
#define CAN_MSG_CFG2949_NTC1_TYPE_BIT      2
#define CAN_MSG_CFG2949_NTC1_TYPE_BIT1     3
// byte index of  flag
#define CAN_MSG_CFG2949_NTC2_TYPE_BYTE     5
#define CAN_MSG_CFG2949_NTC2_TYPE_BIT      4
#define CAN_MSG_CFG2949_NTC2_TYPE_BIT1     5
// byte index of  flag
#define CAN_MSG_CFG2949_MEAS_ENABLE_BYTE   5
#define CAN_MSG_CFG2949_MEAS_ENABLE_BIT	   7
// byte index of  flag
#define CAN_MSG_CFG2949_ACC_BYTE   5
#define CAN_MSG_CFG2949_ACC_BIT	   6
// byte 6
#define CAN_MSG_CFG2949_MEAS_PERIOD_BYTE   6
// byte 7
#define CAN_MSG_CFG2949_FAST_MEAS_PERIOD_BYTE   7

//												  //
//												  //
////////////////////////////////////////////////////

// SMD type
#define NTC_STH_A  9.85013754e-4
#define NTC_STH_B  1.95569870e-4
#define NTC_STH_C  7.69918797e-8
#define NTC_STH_RREF    100e3

//wired type
#define NTC_STHW_A  8.39126e-4
#define NTC_STHW_B  2.08985e-4
#define NTC_STHW_C  7.13241e-8
#define NTC_STHW_RREF   100e3

// OPTIONAL other A,B,C,REF
#define NTC_STH1_A  8.39126e-4
#define NTC_STH1_B  2.08985e-4
#define NTC_STH1_C  7.13241e-8
#define NTC_STH1_RREF   100e3

void setup()
{
	Serial.begin(LTCDEF_BAUDRATE);
	while (!Serial);
	//Initialize LTC2949 library
	LTC2949_init_lib(
		/*byte cellMonitorCount,           */0,
		/*boolean ltc2949onTopOfDaisychain,*/false,
		/*boolean debugEnable              */false
	);
	LTC2949_init_device_state();
	CanInit(CAN_CS_PIN_NUMBER, CAN_BAUDRATE);
}

void loop()
{
	DoCan();
	DoSerial();

	if (!measEnable)
		return;

	DoFastMeas();
	DoSlowMeas();
}

void DoSerial()
{
	if (!Serial.available())
		return;

	// Read from serial
	char character = Serial.read(); // Receive a single character from the software serial port
	if (!IsTermChar(character))
	{
		// Add the received character to the receive buffer (only if not \n or \r)
		SerialInputString.concat(character);
		return;
	}

	SerialInputString.trim(); // remove any whitespaces

	if (SerialInputString.length() == 0)
		return;

	EchoCommand();

	if (/*     */equals(SerialInputString, F("RST")))
	{
		LTC2949_reset(); PrintOkErr(0);
	}
	else if (startsWith(SerialInputString, F("INT")))
		CanInit();
	else if (startsWith(SerialInputString, F("T")))
		canSendMsgSlowCycleTime100ms = SerialInputString.substring(1).toFloat();
	else if (/**/equals(SerialInputString, F("WK")))
		ChkDeviceReadyAndWakeUp();
	else if (startsWith(SerialInputString, F("DB")))
		DebugEnable();// debug enable / disable
	else
		Serial.println(F("CmdErr"));

	SerialInputString = ""; // empty SerialInputString buffer
}

void DebugEnable()// debug enable / disable
{
	LTC2949_DebugEnable = toIntAuto(SerialInputString.substring(2)) != 0;
	Serial.println(LTC2949_DebugEnable ? F("ON") : F("OFF"));
	PrintOkErr(0);
}

void EchoCommand()
{
	Serial.print('>');
	Serial.print(' ');
	Serial.println(SerialInputString);
}

void CanInit()
{
	unsigned int iStart = 3;
	int cs = CAN_CS_PIN_NUMBER;
	int br = CAN_BAUDRATE;
	if (stringSplitter(SerialInputString, &iStart, &cs))
		stringSplitter(SerialInputString, &iStart, &br);

	CanInit((unsigned char)cs, (CAN_SPEED)br);
}

void CanInit(unsigned char cs, CAN_SPEED canSpeed)
{
	Serial.print(F("INT:"));
	Serial.print((int)cs);
	PrintComma();
	Serial.println(canSpeed);

	if (mcp2515)
		delete mcp2515;

	mcp2515 = new MCP2515(cs);
	mcp2515->reset();

	if (mcp2515->setBitrate(canSpeed) == MCP2515::ERROR_OK)
	{
		Serial.println(F("CAN: setBitrate OK"));
	}

	if (mcp2515->setNormalMode() == MCP2515::ERROR_OK)
	{
		Serial.println(F("CAN: setNormalMode OK"));
	}
}

void PrintCanMsg()
{
	Serial.print(millis());
	PrintComma();
	Serial.print(F("Rx"));
	PrintComma();
	PrintZeroX();
	Serial.print(canMsg.can_id, HEX);
	PrintComma();
	SerialPrintByteArrayHex((byte*)canMsg.data, canMsg.can_dlc, true);
	PrintComma();
	Serial.println(canMsg.can_dlc);
}

void CanMsg_RawRW()
{
	if (canMsg.can_dlc < 3) // at least 3 bytes ((Write,Countdown,Page);(Addr);(data/length))
	{
		PrintErrCan(LTC2949_ERRCODE_OTHER, false);
		return;
	}

	// get register address
	uint16_t addr = canMsg.data[CAN_MSG_RAWRW_ADDR_BYTE];
	// adjust address for requested page
	if (bitMaskSetChk(canMsg.data[CAN_MSG_RAWRW_PAGE_BYTE], 1 << CAN_MSG_RAWRW_PAGE_BIT))
		addr |= 0x100;

	// check if WRITE or READ is requested
	if (bitMaskSetChk(canMsg.data[CAN_MSG_RAWRW_WRITE_BYTE], 1 << CAN_MSG_RAWRW_WRITE_BIT))
	{
		//////////////////// WRITE /////////////////////
		// get countDown used for multi message bursts (which is necesary to write more than CAN_MSG_RAWRW_DATA_BYTES (6) bytes in a single burst)
		byte  countDown = (canMsg.data[CAN_MSG_RAWRW_CNTDWN_BYTE] >> CAN_MSG_RAWRW_CNTDWN_BIT) & CAN_MSG_RAWRW_CNTDWN_MASK;
		// check if this is the first write message
		// rawRWData will only be created with the first message
		if (!rawRWData)
		{
			// data bytes in this message
			rawRWDataLen = canMsg.can_dlc - CAN_MSG_RAWRW_D0_BYTE;
			// all following messages must have CAN_MSG_RAWRW_DATA_BYTES (6) bytes
			rawRWDataLen += countDown * CAN_MSG_RAWRW_DATA_BYTES;
			rawRWData = new byte[rawRWDataLen];
			if (!rawRWData)
			{
				PrintErrCan(LTC2949_ERRCODE_OTHER, false);
				return;
			}
			// copy received data to rawRWData
			memcpy(rawRWData, &(canMsg.data[CAN_MSG_RAWRW_D0_BYTE]), canMsg.can_dlc - CAN_MSG_RAWRW_D0_BYTE);
		}
		else
		{
			// this is a subsequent message of a multi message burst
			// calculate index to store data within rawRWData
			byte  i = rawRWDataLen - (countDown + 1) * CAN_MSG_RAWRW_DATA_BYTES;
			// copy received data to rawRWData
			memcpy(&(rawRWData[i]), &(canMsg.data[CAN_MSG_RAWRW_D0_BYTE]), CAN_MSG_RAWRW_DATA_BYTES);
		}
		// check if this is the last message (--> countDown=0 is the last message)
		if (countDown != 0)
			return;

		byte  error = ChkDeviceReadyAndWakeUp();
		// write data to device
		if (!error)
			LTC2949_WRITE(addr, rawRWDataLen, rawRWData);
		// free memory
		delete[] rawRWData;
		rawRWData = NULL;
		// in case of error send can error message
		if (PrintErrCan(error, false))
			return;
		// prepare acknowledge message
		canMsg.can_dlc = CAN_MSG_RAWRW_D0_BYTE + 2;
		canMsg.data[CAN_MSG_RAWRW_D0_BYTE] = rawRWDataLen;
		canMsg.data[CAN_MSG_RAWRW_D1_BYTE] = error;
		bitMaskClr(canMsg.data[CAN_MSG_RAWRW_WRITE_BYTE], 1 << CAN_MSG_RAWRW_WRITE_BIT); // clear write bit in acknowledge message
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		{
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
			return;
		}
	}
	else
	{
		//////////////////// READ /////////////////////
		if (rawRWData != NULL)
		{
			// this should never happen, but is possible if some previous multi message write was not processed till its end
			delete[] rawRWData;
			PrintErrCan(LTC2949_ERRCODE_OTHER, false);
			return;
		}
		rawRWDataLen = canMsg.data[CAN_MSG_RAWRW_D0_BYTE];
		rawRWData = new byte[rawRWDataLen];
		if (!rawRWData)
		{
			PrintErrCan(LTC2949_ERRCODE_OTHER, false);
			return;
		}

		byte  error = ChkDeviceReadyAndWakeUp();
		// read data and ...
		if (!error)
			error = LTC2949_READ(addr, rawRWDataLen, rawRWData);
		// ... in case of error send can error message
		if (error)
		{
			delete[] rawRWData;
			rawRWData = NULL;
			PrintErrCan(error, false);
			return;
		}

		// prepare to send CAN message with requested bytes
		byte  countDown;
		// calculate countDown and canMsg.can_dlc of first message
		if (rawRWDataLen <= CAN_MSG_RAWRW_DATA_BYTES)
		{
			// single message
			canMsg.can_dlc = CAN_MSG_RAWRW_D0_BYTE + rawRWDataLen;
			countDown = 0;
		}
		else
		{
			// multiple messages
			canMsg.can_dlc = rawRWDataLen % CAN_MSG_RAWRW_DATA_BYTES;
			countDown = rawRWDataLen / CAN_MSG_RAWRW_DATA_BYTES;
			if (canMsg.can_dlc == 0)
			{
				// all messages have CAN_MSG_RAWRW_DATA_BYTES bytes
				canMsg.can_dlc = CAN_MSG_RAWRW_DATA_BYTES;
				countDown--;
			}
			canMsg.can_dlc += CAN_MSG_RAWRW_D0_BYTE;
		}
		// send messages
		byte  i = 0;
		do
		{
			bitMaskClr(canMsg.data[CAN_MSG_RAWRW_CNTDWN_BYTE], CAN_MSG_RAWRW_CNTDWN_MASK << CAN_MSG_RAWRW_CNTDWN_BIT);
			bitMaskSet(canMsg.data[CAN_MSG_RAWRW_CNTDWN_BYTE], countDown << CAN_MSG_RAWRW_CNTDWN_BIT);
			memcpy(&(canMsg.data[CAN_MSG_RAWRW_D0_BYTE]), &(rawRWData[i]), canMsg.can_dlc - CAN_MSG_RAWRW_D0_BYTE);
			if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			{
				PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
				break;
			}
			if (countDown == 0)
				break;
			canMsg.can_dlc = CAN_MSG_RAWRW_D0_BYTE + CAN_MSG_RAWRW_DATA_BYTES;
			i = rawRWDataLen - countDown * CAN_MSG_RAWRW_DATA_BYTES;
			countDown--;
		} while (true);
		delete[] rawRWData;
		rawRWData = NULL;
	}
}

void CanMsg_FastRequest()
{
	if (movingAverage)
	{
		delete[] movingAverage->samples;
		delete movingAverage;
		movingAverage = NULL;
	}

	if ((cfgFast & (LTC2949_BM_FACTRL_FACH1 | LTC2949_BM_FACTRL_FACH2 | LTC2949_BM_FACTRL_FACHA)) == 0)
	{
		canSendMsgFastCycleTime1ms = 0;
		// no fast channel activated, nothing to do.
		PrintErrCan(LTC2949_ERRCODE_OTHER, false);
		return;
	}

	// get number of samples (FIFO / continuous only, ignored for SSHT)
	if (canMsg.can_dlc > 1) // number of samples or must be send
		fastSamplesCnt = canMsg.data[0] << 8 | canMsg.data[1];
	else
		fastSamplesCnt = 0;

	// get fast cycle time
	if (canMsg.can_dlc > 2)
		canSendMsgFastCycleTime1ms = canMsg.data[2];
	else
		canSendMsgFastCycleTime1ms = 0;

	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACONV) && (canSendMsgFastCycleTime1ms == 1))
		// if cycle time is 1ms it does not make sense to read from the FIFO
		// Instead we use RDCV to get the latest measured sample
		fastSamplesCnt = 1;

	byte  error = 0;

	if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACONV) || (fastSamplesCnt <= 1))
	{
		// FAST SINGLE SHOT or FAST CONT but report only last sample
		int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
		if (bitMaskClrChk(cfgFast, CFGFAST_ADCVSEND_FLAG))
			LTC2949_ADxx(); // ignored for fast cont.

		if ((fastSamplesCnt > 1) && (fastSamplesCnt < 129))
		{
			// this will enable the moving average filter
			movingAverage = new MAStruct;
			movingAverage->samples = new int16_t[fastSamplesCnt];
			memset(movingAverage->samples, 0, fastSamplesCnt * sizeof(int16_t));
			movingAverage->i = 0;
			movingAverage->len = fastSamplesCnt;
			movingAverage->sum = 0;
		}

		error |= LTC2949_PollFastData(fastData2949);
		SendFastData(fastData2949);
	}
	else // if in FCONT and at least two samples are requested we read the fifo  if (fastSamplesCnt > 1) 
	{
		// FAST CONT: FIFO

		// pause FIFO write
		LTC2949_WriteFastCfg(cfgFast);

		// debug output
		Serial.print('F');
		PrintComma();
		Serial.println(fastSamplesCnt);

		// this ensures we read all available samples but report the newest fastSamplesCnt samples

#ifdef FIFO_TAIL_READ
		fastSamplesCnt = -fastSamplesCnt;
		error |= ReportFifos(true);
		fastSamplesCnt = -fastSamplesCnt;
#else
		error |= ReportFifos(true);
#endif

		// repeat FIFO write
		LTC2949_WriteFastCfg(cfgFast);	// cont / not SSHT
	}

	// store time for next ADCV/fast event
	sendCANFastTimeoutMicros = micros() + canSendMsgFastCycleTime1ms * 1000;
	bitMaskClr(cfgFast, CFGFAST_ADCVSEND_FLAG); // clear flag

	PrintErrCan(error, false);
}

#ifdef FIFO_TAIL_READ
// read len or all samples available (whichever is smaller)
// reports the newest len samples (or the average)
byte  RdFifoSendCAN(byte  ch, boolean reportAllSamples)
{
	int16_t len = fastSamplesCnt;
	// samples read are: 1000,999,....,1 where 1000 is the oldest sample which is read first
	// oldest 100 samples go from 1000 to 901
	uint32_t average = 0;
	uint16_t nUpper; // holds sample number to be read (N..1)
	uint16_t nLower; // holds sample number to be read (N..1)

	// get number of samples available in FIFO
	byte  error = LTC2949_RdFifoSampleCount(ch + LTC2949_REG_FIFOI1, &nUpper);
	if (error)
		return error;

	// nUpper now holds number of samples available
	if (len < 0)
	{
		// read all available samples and report abs(len) newest samples
		len = -len;
	}
	else
	{
		// read and report len or available samples, whichever is smaller
		nUpper = min((uint16_t)len, nUpper);
	}
	// nUpper now holds number of samples available or len, depending if we 
	// want to report len newest samples or len oldest samples

	// adjust len to make sure we never report more samples than available
	len = min((uint16_t)len, nUpper);
	// calculate last sample number of the first sample read burst
	nLower = nUpper > 99 ? nUpper - 99 : 0;

	// ID of the FIFO channel
	canMsg.can_id = CAN_ID_FIFO + ch;
	canMsg.can_dlc = 8;

	// read max 10 times 100 samples
	byte  i = (nUpper - 1) / 100 + 1;
	byte  k = 0;
	for (; i > 0; i--)
	{
		int16_t buffer[100]; // max. 100 samples will be read
		uint16_t samplesRead = nUpper - nLower + 1; // can never be more than 100!
		// read FIFO data
		error |= LTC2949_ReadFifo(
			/*byte  addr:      */ ch + LTC2949_REG_FIFOI1,
			/*uint16_t *len:     */ &samplesRead,
			/*int16_t * samples: */ buffer);

		// we want to read len newest sample, current buffer contains samples nUpper downto nLower
		if ((uint16_t)len >= nLower)
		{
			byte  j = (uint16_t)len < nUpper
				? nUpper - (uint16_t)len
				: 0;
			for (; j < samplesRead; j++)
			{
				// sum up all samples
				average += buffer[j];
				if (!reportAllSamples)
					continue;

				// also send individual samples
				// buffer is int16_t which is stored little-endian on arduino and most controllers,
				// so canMsg.data is also little-endian
				memcpy(&(canMsg.data[k]), &(buffer[j]), 2);
				k += 2;
				if (k == 8)
				{
					k = 0;
					if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
						PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
				}
			}
		}
		nLower -= samplesRead;
		nUpper -= samplesRead;
	}
	if (reportAllSamples && k != 0)
	{
		// send remaining samples
		canMsg.can_dlc = k;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	}
	// always send the average
	canMsg.can_id = ch + CAN_ID_FIFOAVG;
	canMsg.can_dlc = 6; // 32bit fixed point integer (10 fractional bits)!
	((int32_t*)(canMsg.data))[0] = average / (float)len * 1024.0;
	((uint16_t*)(canMsg.data))[2] = len;
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	return error;
}
#endif

void CanMsg_CFG2949()
{
	cfgFast = 0;
	byte  adcCfg = 0;

	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_CH1FAST_BYTE/* */], 1 << CAN_MSG_CFG2949_CH1FAST_BIT/* */)) cfgFast /**/ |= LTC2949_BM_FACTRL_FACH1;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_CH2FAST_BYTE/* */], 1 << CAN_MSG_CFG2949_CH2FAST_BIT/* */)) cfgFast /**/ |= LTC2949_BM_FACTRL_FACH2;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_AUXFAST_BYTE/* */], 1 << CAN_MSG_CFG2949_AUXFAST_BIT/* */)) cfgFast /**/ |= LTC2949_BM_FACTRL_FACHA;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_FCONT_BYTE/*   */], 1 << CAN_MSG_CFG2949_FCONT_BIT/*   */)) cfgFast /**/ |= LTC2949_BM_FACTRL_FACONV;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_SLOT1NTC_BYTE/**/], 1 << CAN_MSG_CFG2949_SLOT1NTC_BIT/**/)) adcCfg /* */ |= LTC2949_BM_ADCCONF_NTC1;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_SLOT2NTC_BYTE/**/], 1 << CAN_MSG_CFG2949_SLOT2NTC_BIT/**/)) adcCfg /* */ |= LTC2949_BM_ADCCONF_NTC2;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_P1ASV_BYTE/*   */], 1 << CAN_MSG_CFG2949_P1ASV_BIT/*   */)) adcCfg  /**/ |= LTC2949_BM_ADCCONF_P1ASV;
	if (bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_P2ASV_BYTE/*   */], 1 << CAN_MSG_CFG2949_P2ASV_BIT/*   */)) adcCfg  /**/ |= LTC2949_BM_ADCCONF_P2ASV;

	measEnable = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_ENABLE_BYTE], 1 << CAN_MSG_CFG2949_MEAS_ENABLE_BIT);

	byte  ntcType[2];
	ntcType[0] = (canMsg.data[CAN_MSG_CFG2949_NTC1_TYPE_BYTE/**/] >> CAN_MSG_CFG2949_NTC1_TYPE_BIT) & 0x3;
	ntcType[1] = (canMsg.data[CAN_MSG_CFG2949_NTC2_TYPE_BYTE/**/] >> CAN_MSG_CFG2949_NTC2_TYPE_BIT) & 0x3;

	byte  slot1P = (canMsg.data[CAN_MSG_CFG2949_SLOT1P_BYTE/**/] >> CAN_MSG_CFG2949_SLOT1P_BIT) & 0xF;
	byte  slot1N = (canMsg.data[CAN_MSG_CFG2949_SLOT1N_BYTE/**/] >> CAN_MSG_CFG2949_SLOT1N_BIT) & 0xF;
	byte  slot2P = (canMsg.data[CAN_MSG_CFG2949_SLOT2P_BYTE/**/] >> CAN_MSG_CFG2949_SLOT2P_BIT) & 0xF;
	byte  slot2N = (canMsg.data[CAN_MSG_CFG2949_SLOT2N_BYTE/**/] >> CAN_MSG_CFG2949_SLOT2N_BIT) & 0xF;
	byte  slotfP = (canMsg.data[CAN_MSG_CFG2949_SLOTFP_BYTE/**/] >> CAN_MSG_CFG2949_SLOTFP_BIT) & 0xF;
	byte  slotfN = (canMsg.data[CAN_MSG_CFG2949_SLOTFN_BYTE/**/] >> CAN_MSG_CFG2949_SLOTFN_BIT) & 0xF;

	measACC /*  */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_ACC_BYTE/*       */], 1 << CAN_MSG_CFG2949_ACC_BIT/*       */);
	measI1 /*   */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_I1_BYTE/*   */], 1 << CAN_MSG_CFG2949_MEAS_I1_BIT/*   */);
	measI2 /*   */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_I2_BYTE/*   */], 1 << CAN_MSG_CFG2949_MEAS_I2_BIT/*   */);
	measP1 /*   */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_P1_BYTE/*   */], 1 << CAN_MSG_CFG2949_MEAS_P1_BIT/*   */);
	measP2 /*   */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_P2_BYTE/*   */], 1 << CAN_MSG_CFG2949_MEAS_P2_BIT/*   */);
	measSLOT1 /**/ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_SLOT1_BYTE/**/], 1 << CAN_MSG_CFG2949_MEAS_SLOT1_BIT/**/);
	measSLOT2 /**/ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_SLOT2_BYTE/**/], 1 << CAN_MSG_CFG2949_MEAS_SLOT2_BIT/**/);
	measVCC /*  */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_VCC_BYTE/*  */], 1 << CAN_MSG_CFG2949_MEAS_VCC_BIT/*  */);
	measVREF /* */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_VREF_BYTE/* */], 1 << CAN_MSG_CFG2949_MEAS_VREF_BIT/* */);
	measBAT /*  */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_BAT_BYTE/*  */], 1 << CAN_MSG_CFG2949_MEAS_BAT_BIT/*  */);
	measTEMP /* */ = bitMaskSetChk(canMsg.data[CAN_MSG_CFG2949_MEAS_TEMP_BYTE/* */], 1 << CAN_MSG_CFG2949_MEAS_TEMP_BIT/* */);

	canSendMsgSlowCycleTime100ms = canMsg.can_dlc > CAN_MSG_CFG2949_MEAS_PERIOD_BYTE ? canMsg.data[CAN_MSG_CFG2949_MEAS_PERIOD_BYTE] : 0xFFU; // default is disabled
	sendCANSlowTimeout = millis() + 100UL * canSendMsgSlowCycleTime100ms;
	if (cfgFast)
	{
		canSendMsgFastCycleTime1ms = canMsg.can_dlc > CAN_MSG_CFG2949_FAST_MEAS_PERIOD_BYTE ? canMsg.data[CAN_MSG_CFG2949_FAST_MEAS_PERIOD_BYTE] : 0; // default is disable 
		sendCANFastTimeoutMicros = micros() + canSendMsgFastCycleTime1ms * 1000;
	}
	else
	{
		canSendMsgFastCycleTime1ms = 0;
		sendCANFastTimeoutMicros = 0;
	}

	byte  error = ChkDeviceReadyAndWakeUp();
	if (PrintErrCan(error, false))
		return;

	for (byte i = 0; i < 2; i++)
	{
		if ((i == 0 && bitMaskClrChk(adcCfg, LTC2949_BM_ADCCONF_NTC1)) || (i == 1 && bitMaskClrChk(adcCfg, LTC2949_BM_ADCCONF_NTC2)))
			continue;
		switch (ntcType[i])
		{
		case 0:
			NtcCfgWrite(i + 1, NTC_STH_RREF, NTC_STH_A, NTC_STH_B, NTC_STH_C, true);
			break;
		case 1:
			NtcCfgWrite(i + 1, NTC_STHW_RREF, NTC_STHW_A, NTC_STHW_B, NTC_STHW_C, true);
			break;
		case 2:
			NtcCfgWrite(i + 1, NTC_STH1_RREF, NTC_STH1_A, NTC_STH1_B, NTC_STH1_C, true);
			break;
		case 3:
		default:
			// nothing will be written. This allows the user to provide other values via RAWRW CAN message
			break;
		}
		Serial.println();
	}

	LTC2949_SlotsCfg(
		slot1P,
		slot1N,
		slot2P,
		slot2N
	);
	LTC2949_SlotFastCfg(
		slotfP,
		slotfN
	);

	Serial.print(slot1P);
	PrintComma();
	Serial.print(slot1N);
	PrintComma();
	Serial.print(slot2P);
	PrintComma();
	Serial.print(slot2N);
	PrintComma();
	Serial.print(slotfP);
	PrintComma();
	Serial.print(slotfN);
	PrintComma();
	SerialPrintByteArrayHex(&cfgFast, 1, true);
	PrintComma();
	SerialPrintByteArrayHex(&adcCfg, 1, true);
	PrintComma();

	if (measEnable)
	{
		byte data;
		error |= LTC2949_ADCConfigRead(&data); // read again to update internal state of ADCConfig
		// read, report & clear status
		error |= LTC2949_ReadChkStatusFaults(true, true);
		error |= LTC2949_GoCont(cfgFast, adcCfg);
	}
	else
	{
		LTC2949_WriteFastCfg(false, false, false, false);
		LTC2949_OpctlIdle();
	}
	Serial.println();

	PrintErrCan(error, false);
}

boolean PrintErrCan(byte  error, boolean printOK)
{
	if (error)
	{
		measEnable = false;// stop measurement

		PrintOkErr(error);

		canMsg.can_dlc = 1;
		canMsg.data[0] = error;
		canMsg.can_id = CAN_ID_ERR;

		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);

		return true; // true means error
	}

	if (printOK)
		PrintOkErr(0);

	return false; // false means no error
}

void DoCan()
{
	if (!mcp2515->checkReceive())
		return;
	if (mcp2515->readMessage(&canMsg) != MCP2515::ERROR_OK)
	{
		Serial.println(F("CAN: readMessage failed"));
		return;
	}

	PrintCanMsg();
	if (canMsg.can_id == CAN_ID_CFG2949)
		CanMsg_CFG2949();
	else if (canMsg.can_id == CAN_ID_FASTRQ)
		CanMsg_FastRequest();
	else if (canMsg.can_id == CAN_ID_RAWRW)
		CanMsg_RawRW();
}

void DoSlowMeas()
{
	if (canSendMsgSlowCycleTime100ms == 0xFFU)
		return;

	if (!LTC_TIMEOUT_CHECK(millis(), sendCANSlowTimeout))
		return;

	sendCANSlowTimeout = millis() + 100UL * canSendMsgSlowCycleTime100ms;

	byte  error;
	if (!LTC2949_ChkUpdate(&error))
	{
		PrintErrCan(error, false);
		return;
	}

	canMsg.can_dlc = 0;
	// Current Power Voltage
	memset(canMsg.data, 0, 8 * sizeof(byte));
	if (measI1)
	{
		error |= LTC2949_READ(LTC2949_VAL_I1, 3, &(canMsg.data[0]));
		canMsg.can_dlc = 3;
	}
	else
		canMsg.data[0] = 0x80; // all other bytes are zero, see memset. Setting the MSBit makes it the most negative value, which indicates this channel was not measured
	if (measP1 && !LTC2949_P1IsVolt())
	{
		error |= LTC2949_READ(LTC2949_VAL_P1, 3, &(canMsg.data[3]));
		canMsg.can_dlc = 6;
	}
	else
		canMsg.data[3] = 0x80;
	if (measBAT)
	{
		error |= LTC2949_READ(LTC2949_VAL_BAT, 2, &(canMsg.data[6]));
		canMsg.can_dlc = 8;
	}
	else
		canMsg.data[6] = 0x80;
	if (canMsg.can_dlc)
	{
		canMsg.can_id = CAN_ID_IP1V;

		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		if (PrintErrCan(error, false))
			return;
	}

	canMsg.can_dlc = 0;
	// Current Power Temp
	memset(canMsg.data, 0, 8 * sizeof(byte));
	if (measI2)
	{
		error |= LTC2949_READ(LTC2949_VAL_I2, 3, &(canMsg.data[0]));
		canMsg.can_dlc = 3;
	}
	else
		canMsg.data[0] = 0x80;
	if (measP2 && !LTC2949_P2IsVolt())
	{
		error |= LTC2949_READ(LTC2949_VAL_P2, 3, &(canMsg.data[3]));
		canMsg.can_dlc = 6;
	}
	else
		canMsg.data[3] = 0x80;
	if (measTEMP)
	{
		error |= LTC2949_READ(LTC2949_VAL_TEMP, 2, &(canMsg.data[6]));
		canMsg.can_dlc = 8;
	}
	else
		canMsg.data[6] = 0x80;
	if (canMsg.can_dlc)
	{
		canMsg.can_id = CAN_ID_IP2T;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		if (PrintErrCan(error, false))
			return;
	}

	canMsg.can_dlc = 0;
	// Power as Voltage
	memset(canMsg.data, 0, 6 * sizeof(byte));
	if (measP1 && LTC2949_P1IsVolt())
	{
		error |= LTC2949_READ(LTC2949_VAL_P1, 3, &(canMsg.data[0]));
		canMsg.can_dlc = 3;
	}
	else
		canMsg.data[0] = 0x80;
	if (measP2 && LTC2949_P2IsVolt())
	{
		error |= LTC2949_READ(LTC2949_VAL_P2, 3, &(canMsg.data[3]));
		canMsg.can_dlc = 6;
	}
	else
		canMsg.data[3] = 0x80;
	if (canMsg.can_dlc)
	{
		canMsg.can_id = CAN_ID_PASV;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		if (PrintErrCan(error, false))
			return;
	}

	canMsg.can_dlc = 0;
	// SLOTS + VCC/VREF
	memset(canMsg.data, 0, 8 * sizeof(byte));
	if (measSLOT1 && !LTC2949_Slot1IsTemp())
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, &(canMsg.data[0]));
		canMsg.can_dlc = 2;
	}
	else
		canMsg.data[0] = 0x80;
	if (measSLOT2 && !LTC2949_Slot2IsTemp())
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT2, 2, &(canMsg.data[2]));
		canMsg.can_dlc = 4;
	}
	else
		canMsg.data[2] = 0x80;
	if (measVREF)
	{
		error |= LTC2949_READ(LTC2949_VAL_VREF, 2, &(canMsg.data[4]));
		canMsg.can_dlc = 6;
	}
	else
		canMsg.data[4] = 0x80;
	if (measVCC)
	{
		error |= LTC2949_READ(LTC2949_VAL_VCC, 2, &(canMsg.data[6]));
		canMsg.can_dlc = 8;
	}
	else
		canMsg.data[6] = 0x80;
	if (canMsg.can_dlc)
	{
		canMsg.can_id = CAN_ID_SVR;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		if (PrintErrCan(error, false))
			return;
	}

	canMsg.can_dlc = 0;
	// SLOTS as temp via NTC
	memset(canMsg.data, 0, 4 * sizeof(byte));
	if (measSLOT1 && LTC2949_Slot1IsTemp())
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT1, 2, &(canMsg.data[0]));
		canMsg.can_dlc = 2;
	}
	else
		canMsg.data[0] = 0x80;
	if (measSLOT2 && LTC2949_Slot2IsTemp())
	{
		error |= LTC2949_READ(LTC2949_VAL_SLOT2, 2, &(canMsg.data[2]));
		canMsg.can_dlc = 4;
	}
	else
		canMsg.data[2] = 0x80;
	if (canMsg.can_dlc)
	{
		canMsg.can_id = CAN_ID_SLOTSNTC;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		if (PrintErrCan(error, false))
			return;
	}

	if (!measACC)
		return;

	// Accumulators C1, E1, ...
	canMsg.can_dlc = 6;
	canMsg.can_id = CAN_ID_ACC_C1;
	error |= LTC2949_READ(LTC2949_VAL_C1, 6, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 6;
	canMsg.can_id = CAN_ID_ACC_E1;
	error |= LTC2949_READ(LTC2949_VAL_E1, 6, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 4;
	canMsg.can_id = CAN_ID_ACC_TB1;
	error |= LTC2949_READ(LTC2949_VAL_TB1, 4, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

#ifndef ENABLE_CAN_ID_ACC_C2_TO_TB4
	return;
#endif

	canMsg.can_dlc = 6;
	canMsg.can_id = CAN_ID_ACC_C2;
	error |= LTC2949_READ(LTC2949_VAL_C2, 6, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 6;
	canMsg.can_id = CAN_ID_ACC_E2;
	error |= LTC2949_READ(LTC2949_VAL_E2, 6, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 4;
	canMsg.can_id = CAN_ID_ACC_TB2;
	error |= LTC2949_READ(LTC2949_VAL_TB2, 4, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 8;
	canMsg.can_id = CAN_ID_ACC_C3;
	error |= LTC2949_READ(LTC2949_VAL_C3, 8, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 8;
	canMsg.can_id = CAN_ID_ACC_E4;
	error |= LTC2949_READ(LTC2949_VAL_E4, 8, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 4;
	canMsg.can_id = CAN_ID_ACC_TB3;
	error |= LTC2949_READ(LTC2949_VAL_TB3, 4, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

	canMsg.can_dlc = 4;
	canMsg.can_id = CAN_ID_ACC_TB4;
	error |= LTC2949_READ(LTC2949_VAL_TB4, 4, &(canMsg.data[0]));
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	if (PrintErrCan(error, false))
		return;

}

void DoFastMeas()
{
	if (canSendMsgFastCycleTime1ms == 0)
		return; // no cyclic fast measurement

	if (!LTC_TIMEOUT_CHECK(micros(), sendCANFastTimeoutMicros))
		return; // cycle time not yet elapsed
	sendCANFastTimeoutMicros = micros();

	int16_t fastData2949[LTC2949_RDFASTDATA_LENGTH];
	byte  error = 0;

	if (bitMaskClrChk(cfgFast, LTC2949_BM_FACTRL_FACONV)) // fast single shot
	{
		// check if ADCV was already send
		if (bitMaskSetChk(cfgFast, CFGFAST_ADCVSEND_FLAG))
		{
			// store time for next ADCV to be send
			sendCANFastTimeoutMicros += canSendMsgFastCycleTime1ms * 1000 - 1200 - LTC2949_TimeLastPEC;
			bitMaskClr(cfgFast, CFGFAST_ADCVSEND_FLAG); // clear flag
			//error = LTC2949_PollFastData(fastData2949);
			error = LTC2949_RdFastData(fastData2949);
			if (!LTC2949_FASTSSHT_HS_OK(fastData2949))
				error |= LTC2949_ERRCODE_FIFOTAGERR;
		}
		else
		{
			// make new fast single shot
			LTC2949_ADxx();
			// calculate the time it took from time out to ADCV send
			unsigned long adcvTime = LTC2949_TimeLastPEC - sendCANFastTimeoutMicros;
			sendCANFastTimeoutMicros = LTC2949_TimeLastPEC + 1200;// at this time results are ready to be read
			LTC2949_TimeLastPEC = adcvTime; // store this delta for subtract it later.
			bitMaskSet(cfgFast, CFGFAST_ADCVSEND_FLAG); // set flag
			return;
		}
	}
	else // fast continuous
	{
		// store time for next fast event
		sendCANFastTimeoutMicros += canSendMsgFastCycleTime1ms * 1000;
		if (fastSamplesCnt <= 1)
		{
			// read latest result only
			error = LTC2949_PollFastData(fastData2949);
		}
		else
		{
			error = ReportFifos(false);
		}
	}

	if (PrintErrCan(error, false))
		return;

	// send single shot result
	SendFastData(fastData2949);
}

byte  ChkDeviceReadyAndWakeUp()
{
	byte data;
	byte error = LTC2949_READ(LTC2949_REG_OPCTRL, 1, &data);
	// CRC error or device not yet ready?
	boolean wakeupRequired = error & LTC2949_ERRCODE_PECERR_MASK || data == LTC2949_BM_OPCTRL_SLEEP;

	if (wakeupRequired)
	{
		Serial.print(F("WK:"));
		delay(100);
		PrintOkErr(LTC2949_WakeupAndAck());
	}
	error = LTC2949_ReadChkStatusFaults(true, true);
	PrintComma();
	PrintOkErr(error);
	error = LTC2949_ADCConfigRead(&data);
	PrintOkErr(error);
	if (error)
		return error;
	return 0;
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

void SendFastData(int16_t * fastData2949)
{
	if (movingAverage)
	{
		/* option to send also BAT and AUX here
			canMsg.can_dlc = 8;
			memcpy(canMsg.data + 4, fastData2949 + 2, 4); // copy BAT and AUX
		*/
		canMsg.can_id = CAN_ID_FSSHTMA;
		//canMsg.can_dlc = 6;
		canMsg.can_dlc = 4;

		// calc moving average
		movingAverage->sum -= movingAverage->samples[movingAverage->i];
		movingAverage->sum += fastData2949[LTC2949_RDFASTDATA_I2];
		movingAverage->samples[movingAverage->i] = fastData2949[LTC2949_RDFASTDATA_I2];
		movingAverage->i = (movingAverage->i + 1) % movingAverage->len;
		// copy results
		((int32_t*)(canMsg.data))[0] = movingAverage->sum / (float)movingAverage->len * 1024.0;
		//((int16_t*)(canMsg.data))[2] = fastData2949[LTC2949_RDFASTDATA_I2];
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
		//return;
	}

	canMsg.can_id = CAN_ID_FSSHT;
	canMsg.can_dlc = 8;
	memcpy(canMsg.data, fastData2949, 8); // I1, I2 BAT, AUX
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
}

////// void AdjustFastData(int16_t * fastData2949)
////// {
////// 	boolean ch1OrCh2 = false;
////// 
////// 	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH1))
////// 		ch1OrCh2 = true;
////// 	else
////// 		fastData2949[LTC2949_RDFASTDATA_I1] = 0x8000; // channel not enabled, report most negative RAW value
////// 
////// 	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH2))
////// 		ch1OrCh2 = true;
////// 	else
////// 		fastData2949[LTC2949_RDFASTDATA_I2] = 0x8000; // channel not enabled, report most negative RAW value
////// 
////// 	if (ch1OrCh2 && LTC2949_AnyPasV())
////// 		;
////// 	else
////// 		fastData2949[LTC2949_RDFASTDATA_BAT] = 0x8000; // channel not enabled, report most negative RAW value
////// 
////// 	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACHA))
////// 		;
////// 	else
////// 		fastData2949[LTC2949_RDFASTDATA_AUX] = 0x8000; // channel not enabled, report most negative RAW value
////// }

byte  ReportFifos(boolean reportAllSamples)
{
	boolean ch1OrCh2 = false;
	byte  error = 0;

	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH1))
	{
		ch1OrCh2 = true;
		error |= RdFifoSendCAN(0, reportAllSamples);
	}
	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACH2))
	{
		ch1OrCh2 = true;
		error |= RdFifoSendCAN(1, reportAllSamples);
	}
	if (ch1OrCh2 && LTC2949_AnyPasV())
	{
		error |= RdFifoSendCAN(2, reportAllSamples);
	}
	if (bitMaskSetChk(cfgFast, LTC2949_BM_FACTRL_FACHA))
	{
		error |= RdFifoSendCAN(3, reportAllSamples);
	}
	return error;
}

#ifdef FIFO_NORMAL_READ
byte  RdFifoSendCAN(byte  ch, boolean reportAllSamples)
{
	uint16_t len = fastSamplesCnt;
	for (uint16_t samplesAvailable;;)
	{
		// get number of samples available in FIFO
		byte  error = LTC2949_RdFifoSampleCount(ch + LTC2949_REG_FIFOI1, &samplesAvailable);
		if (error)
			return error;
		len = min(len, samplesAvailable);
		break;
	}
	if (len == 0)
		return 0;

	uint16_t totalSamplesRead = 0;
	byte  k = 0, rdBurstsCnt = 0;
	uint32_t average = 0;

	// ID of the FIFO channel
	canMsg.can_id = CAN_ID_FIFO + ch;
	canMsg.can_dlc = 8;

	while (len > 0)
	{
		// we read requested samples in bursts of maximum 100
		int16_t buffer[100];
		uint16_t samplesRead = min(100, len);

		// read FIFO samples
		byte  error = LTC2949_ReadFifo(
			/*byte  addr:      */ ch + LTC2949_REG_FIFOI1,
			/*uint16_t *len:     */ &samplesRead,
			/*int16_t * samples: */ buffer);
		if (error)
			return error;
		rdBurstsCnt++;

		if (rdBurstsCnt > 10)
			// we can't read more than 10x100 samples!
			return LTC2949_ERRCODE_TIMEOUT;

		totalSamplesRead += samplesRead;
		len -= samplesRead;
		for (byte j = 0; j < samplesRead; j++)
		{
			// sum up all samples
			average += buffer[j];

			if (!reportAllSamples)
				continue;

			// buffer is int16_t which is stored little-endian on arduino 
			// and most controllers, so canMsg.data is also little-endian
			memcpy(&(canMsg.data[k]), &(buffer[j]), 2);
			k += 2;
			if (k == 8)
			{
				k = 0;
				if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
					PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
			}
		}
	}
	if (reportAllSamples && k != 0)
	{
		// send remaining samples
		canMsg.can_dlc = k;
		if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
			PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	}
	// always send the average
	canMsg.can_id = ch + CAN_ID_FIFOAVG;
	// 32bit fixed point integer (10 fractional bits), average of all samples
	// 16bit unsigned int, number of samples used for average
	canMsg.can_dlc = 6;
	((int32_t*)(canMsg.data))[0] = average / (float)totalSamplesRead * 1024.0;
	((uint16_t*)(canMsg.data))[2] = totalSamplesRead;
	if (mcp2515->sendMessage(&canMsg) != MCP2515::ERROR_OK)
		PrintOkErr(LTC2949_ERRCODE_TIMEOUT);
	return 0;
}

#endif
