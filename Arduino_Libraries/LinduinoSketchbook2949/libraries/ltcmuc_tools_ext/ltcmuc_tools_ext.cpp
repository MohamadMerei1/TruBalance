/************************************
  
Copyright 2018(c) Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 - Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 - Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in
   the documentation and/or other materials provided with the
   distribution.
 - Neither the name of Analog Devices, Inc. nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.
 - The use of this software may or may not infringe the patent rights
   of one or more patent holders.  This license does not release you
   from the requirement that you obtain separate licenses from these
   patent holders to use this software.
 - Use of the software either in source or binary form, must be run
   on or directly connected to an Analog Devices Inc. component.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************/

#include "ltcmuc_tools_ext.h"

template<typename TIN>
void ToBytes(TIN value, byte *bytes, int len)
{
	for (int i = len - 1; i >= 0; i--)
	{
		bytes[i] = value;
		value = value >> 8;
	}
}
template void ToBytes<>(unsigned long, byte *, int);

// Read 0x000000FFFF0 or 1234
// Example:
// Hex:     0x11 (0x prefix)
// Decimal: 17
uint16_t SerialReadByteArray(byte * bytes, uint16_t maxLength)
{
	// read string
	String serialInputString = "";
	while (true)
	{
		if (!Serial.available())
			continue;
		char character = Serial.read(); // Receive a single character from the software serial port
		if (IsTermChar(character))
			break;
		serialInputString.concat(character);
	}
	return StringToBytes(serialInputString, maxLength, bytes);
}

template<typename T>
unsigned int toInts(String str, unsigned int iStart, T ints[], unsigned int len)
{
	unsigned int i = 0;
	for (; i < len;)
	{
		int val;
		boolean lastElement = !stringSplitter(str, &iStart, &val);
		ints[i++] = val;
		if (lastElement)
			break;
	}
	return i;
}
template unsigned int toInts<>(String, unsigned int, uint8_t[], unsigned int);

template<typename TA, typename TB>
bool CompareType(TA a, TB b)
{
	return false /* the following anding is just dummy code to prevent compiler warning!  && (a || b)*/;
}

template<typename T>
bool CompareType(T a, T b)
{
	return true /* the following oring is just dummy code to prevent compiler warning!  || (a && b)*/;
}


template<typename T, typename TL> void PrintArraysAsCsv(T data[], TL len)
{
	if (len > 0)
	{
		for (TL i = 0;;)
		{
			Serial.print(data[i++]);
			if (i < len)
				PrintComma();
			else
				break;
		}
	}
}
template void PrintArraysAsCsv<>(double[], uint8_t);
template void PrintArraysAsCsv<>(int16_t[], uint8_t);
template void PrintArraysAsCsv<>(uint16_t[], uint8_t);
template void PrintArraysAsCsv<>(uint8_t[], uint8_t);
template void PrintArraysAsCsv<>(int32_t[], uint8_t);

template<typename T, typename TL> void PrintArraysAsCsv(T data[], TL len, int f)
{
	if (len > 0)
	{
		for (TL i = 0;;)
		{
			Serial.print(data[i++], f);
			if (i < len)
				PrintComma();
			else
				break;
		}
	}
}
template void PrintArraysAsCsv<>(double[], uint8_t, int);


void PrintBytes2Int16sAsCsv(byte * data, uint8_t len)
{
	if (len > 0)
	{
		for (uint8_t i = 0;;)
		{
			int16_t v0 = LTC_2BytesToInt16(data + i);
			Serial.print(v0);
			i += 2;
			if (i < len)
				PrintComma();
			else
				break;
		}
	}
}

void PrintBytes2Int24sAsCsv(byte * data, uint8_t len)
{
	if (len > 0)
	{
		for (uint8_t i = 0;;)
		{
			int32_t v0 = LTC_3BytesToInt32(data + i);
			Serial.print(v0);
			i += 3;
			if (i < len)
				PrintComma();
			else
				break;
		}
	}
}


#ifdef FLOAT2SCIENTIFICTEST
boolean float2ScientificTest(int digits, int runs, int maxExp, boolean verbose)
{
	boolean ok = true;
	for (int i = 0; i < runs; i++)
	{
		float e = random() / (float)RANDOM_MAX;
		e = e * 2 - 1;
		e = e * maxExp;
		float val = random() / (float)RANDOM_MAX;
		val = val * 2 - 1;
		val = val * powf(10, e);
		String fstr = float2Scientific(val, digits);
		float valr = fstr.toFloat();
		float valrErr = (valr / val - 1)*1e6; // in ppm!
		Serial.print(i);
		boolean notOK = (abs(valrErr) > 100);
		ok &= !notOK;
		if (verbose || notOK)
		{
			PrintComma();
			Serial.print(val, digits);
			PrintComma();
			Serial.print(fstr);
			PrintComma();
			Serial.print(valr, digits);
			PrintComma();
			Serial.print(valrErr);
		}
		Serial.println();
	}
	return ok;
}
#endif


