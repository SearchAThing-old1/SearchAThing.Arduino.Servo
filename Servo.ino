/*
* The MIT License(MIT)
* Copyright(c) 2016 Lorenzo Delana, https://searchathing.com
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.
*/

//===========================================================================
// PROJECT	: Servo.ino
//===========================================================================
// Setup()
// Loop()
//---------------------------------------------------------------------------
// Suggested defines
//---------------------------------------------------------------------------
// SEARCHATHING_DISABLE;DPRINT_SERIAL
//

// SearchAThing.Arduino debug macro definitions
#include <SearchAThing.Arduino.Utils\DebugMacros.h>

//---------------------------------------------------------------------------
// Libraries
//---------------------------------------------------------------------------
#include <MemoryFree.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <SearchAThing.Arduino.Utils\Util.h>
#include <SearchAThing.Arduino.Utils\SList.h>
using namespace SearchAThing::Arduino;

#include <SearchAThing.Arduino.Net\EthNet.h>
using namespace SearchAThing::Arduino::Net;

#include <SearchAThing.Arduino.Net\SRUDP_Client.h>
using namespace SearchAThing::Arduino::Net::SRUDP;

#include <SearchAThing.Arduino.Enc28j60\Driver.h>
using namespace SearchAThing::Arduino::Enc28j60;

#include "LoopProc.h"
#include "AnalogPortMap.h"
using namespace SearchAThing::Arduino::Servo;

//---------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------

// network card driver
EthDriver *drv;

// network manager
EthNet *net;

OneWire *ds18b20OneWire;
DallasTemperature *ds18b20;

LoopProc loopProc;

IPEndPoint remoteEndPoint;

// Flash strings - declaration
const __FlashStringHelper *F_free_SP;
const __FlashStringHelper *F_ds18b20_setup_SP;
const __FlashStringHelper *F_ds18b20_get_SP;
const __FlashStringHelper *F_digital_setup_SP;
const __FlashStringHelper *F_digital_write_SP;
const __FlashStringHelper *F_digital_read_SP;
const __FlashStringHelper *F_analog_read_SP;
const __FlashStringHelper *F_analog_read_pack_SP;
const __FlashStringHelper *F_ERR_space;
const __FlashStringHelper *F_MissingSetup;
const __FlashStringHelper *F_OutOfMemory;
const __FlashStringHelper *F_SamplesOutOfPkt;
const __FlashStringHelper *F_InvalidSyntax;
const __FlashStringHelper *F_OK;
const __FlashStringHelper *F_OKnl;
const __FlashStringHelper *F_nl;
const __FlashStringHelper *F_IN;
const __FlashStringHelper *F_OUT;
const __FlashStringHelper *F_ON;

//---------------------------------------------------------------------------
// Setup
//---------------------------------------------------------------------------
void setup()
{
	// Flash strings - initialization
	F_free_SP = F("free ");
	F_ds18b20_setup_SP = F("ds18b20_setup ");
	F_ds18b20_get_SP = F("ds18b20_get ");
	F_digital_setup_SP = F("digital_setup ");
	F_ERR_space = F("ERR ");
	F_MissingSetup = F("MissingSetup");
	F_InvalidSyntax = F("InvalidSyntax");
	F_OutOfMemory = F("OutOfMemory");
	F_SamplesOutOfPkt = F("SamplesOutOfPkt");
	F_OK = F("OK");
	F_OKnl = F("OK\r\n");
	F_nl = F("\r\n");
	F_IN = F("IN");
	F_OUT = F("OUT");
	F_digital_write_SP = F("digital_write ");
	F_digital_read_SP = F("digital_read ");
	F_analog_read_SP = F("analog_read ");
	F_analog_read_pack_SP = F("analog_read_pack ");

	// init
	{
		{
			// network card init ( mac = 00:00:6c:00:00:[01] )
			drv = new Driver(PrivateMACAddress(1));

			// network manager init [dynamic-mode]
			net = new EthNet(drv);
		}

		DPrint(F("MAC\t")); DPrintHexBytesln(net->MacAddress());
		net->PrintSettings(); // print network settings

		net->AddProcess(loopProc);

		DPrintln(F("setup done"));
		DNewline();
	}

	remoteEndPoint = IPEndPoint(net->IpAddress(), 50000);
}

void SyntaxError(SearchAThing::Arduino::Net::SRUDP::Client& client)
{
	client.Write(RamData(F_ERR_space)
		.Append(F_InvalidSyntax)
		.Append(F_nl));
}

void OutOfMemoryError(SearchAThing::Arduino::Net::SRUDP::Client& client)
{
	client.Write(RamData(F_ERR_space)
		.Append(F_OutOfMemory)
		.Append(F_nl));
}

void OKNL(SearchAThing::Arduino::Net::SRUDP::Client& client)
{
	client.Write(RamData(F_OKnl));
}

//---------------------------------------------------------------------------
// Loop
//---------------------------------------------------------------------------
void loop()
{
	DPrintln(F("Waiting client..."));
	auto client = Client::Listen(net, remoteEndPoint);

	DPrint(F("client connected: ")); client.RemoteEndPoint().ToString().PrintAsChars(); DNewline();

	while (client.State() == ClientState::Connected)
	{

		RamData data;

		if (client.Read(data) == TransactionResult::Successful)
		{
			//-----------------------------------------------------------
			// free
			// https://searchathing.com/?page_id=886#free
			//-----------------------------------------------------------
			if (data.StartsWith(F_free_SP))
			{
				auto type = data.StripBegin(F_free_SP).ToUInt16_t();

				int free = (type == 0) ? FreeMemorySum() : FreeMemoryMaxBlock();

				// <free-ram-bytes>
				byte buf[UINT_CHARS + 1];
				utoa(free, (char *)buf, DEC);

				// OK<nl><free-ram-bytes><nl>
				client.Write(RamData(F_OKnl)
					.Append(RamData(buf, strlen((const char *)buf)))
					.Append(F_nl));
			}

			//-----------------------------------------------------------
			// uptime
			// https://searchathing.com/?page_id=886#uptime
			//-----------------------------------------------------------
			else if (data.Equals(F("uptime")))
			{
				// <OK><nl><days>.milliseconds><nl>
				client.Write(RamData(F_OKnl)
					.Append(loopProc.UpTimeStr())
					.Append(F_nl));
			}

			//-----------------------------------------------------------
			// ds18b20 port setup
			// https://searchathing.com/?page_id=886#ds18b20_setup
			//-----------------------------------------------------------
			else if (data.StartsWith(F_ds18b20_setup_SP))
			{
				// ds18b20_setup <port><nl>
				auto port = data.StripBegin(F_ds18b20_setup_SP).ToUInt16_t();

				if (ds18b20OneWire != NULL)
				{
					delete ds18b20;
					delete ds18b20OneWire;
				}

				ds18b20OneWire = new OneWire(port);
				ds18b20 = new DallasTemperature(ds18b20OneWire);
				ds18b20->begin();

				// OK<nl>
				OKNL(client);
			}

			//-----------------------------------------------------------
			// ds18b20 get device count
			// https://searchathing.com/?page_id=886#ds18b20_count
			//-----------------------------------------------------------
			else if (data.Equals(F("ds18b20_count")))
			{
				// ds18b20_count<nl>
				if (ds18b20 == NULL)
				{
					client.Write(RamData(F_ERR_space).Append(F_MissingSetup).Append(F_nl));
				}
				else
				{
					// OK<nl><count><nl>
					client.Write(
						RamData(F_OKnl)
						.Append(RamData::FromUInt16(ds18b20->getDeviceCount()))
						.Append(F_nl));
				}
			}

			//-----------------------------------------------------------
			// ds18b20 get temperature C
			// https://searchathing.com/?page_id=886#ds18b20_get
			//-----------------------------------------------------------
			else if (data.StartsWith(F_ds18b20_get_SP))
			{
				// ds18b20_get <idx><nl>
				if (ds18b20 == NULL)
				{
					client.Write(RamData(F_ERR_space).Append(F_MissingSetup).Append(F_nl));
				}
				else
				{
					RamData res;
					{
						auto idx = data.StripBegin(F_ds18b20_get_SP).ToUInt16_t();
						ds18b20->requestTemperaturesByIndex(idx);
						auto tempC = ds18b20->getTempCByIndex(idx);
						char tmp[20];
						FloatToString(tmp, tempC, 2);
						// OK<nl><temperature_C><nl>
						res = RamData(F_OKnl).Append(tmp).Append(F_nl);
					}

					client.Write(res);
				}
			}

			//-----------------------------------------------------------
			// set digital port mode
			// https://searchathing.com/?page_id=886#digital_setup
			//-----------------------------------------------------------
			else if (data.StartsWith(F_digital_setup_SP))
			{
				// digital_setup <port> <node><nl>
				auto words = data.StripBegin(F_digital_setup_SP).Split(' ');
				
				if (words.Size() != 2)
				{
					// ERR InvalidSyntax<nl>
					SyntaxError(client);
				}
				else
				{					
					auto port = words.Get(0).ToUInt16_t();
					auto mode = words.Get(1);
					
					// use of StartsWith instead of Equals cause the token
					// contains leading newline characters

					if (mode.StartsWith(F_IN))
					{
						// OK<nl>
						pinMode(port, INPUT);
						OKNL(client);
					}
					else if (mode.StartsWith(F_OUT))
					{
						// OK<nl>
						pinMode(port, OUTPUT);
						OKNL(client);
					}
					else
					{
						// ERR InvalidSyntax<nl>
						SyntaxError(client);
					}
				}
			}

			//-----------------------------------------------------------
			// digital port write
			// https://searchathing.com/?page_id=886#digital_write
			//-----------------------------------------------------------
			else if (data.StartsWith(F_digital_write_SP))
			{
				// digital_write <port> <value><nl>
				auto words = data.StripBegin(F_digital_write_SP).Split(' ');
				if (words.Size() != 2)
				{
					// ERR InvalidSyntax<nl>
					SyntaxError(client);
				}
				else
				{
					auto port = words.Get(0).ToUInt16_t();
					auto value = words.Get(1).ToUInt16_t();
					digitalWrite(port, value);
					OKNL(client);
				}
			}

			//-----------------------------------------------------------
			// digital port read
			// https://searchathing.com/?page_id=886#digital_read
			//-----------------------------------------------------------
			else if (data.StartsWith(F_digital_read_SP))
			{
				// digital_read <port><nl>
				auto port = data.StripBegin(F_digital_read_SP).ToUInt16_t();
				auto value = digitalRead(port);

				// OK<nl><value><nl>
				auto res = RamData(F_OKnl);
				{
					char tmp[7];
					utoa(value, tmp, DEC);
					res = res
						.Append(RamData((const byte *)tmp, strlen(tmp)))
						.Append(F_nl);
				}
				client.Write(res);
			}

			//-----------------------------------------------------------
			// analog port read
			// https://searchathing.com/?page_id=886#analog_read
			//-----------------------------------------------------------
			else if (data.StartsWith(F_analog_read_SP))
			{
				// analog_read <port><nl>
				auto port_n = data.StripBegin(F_analog_read_SP).ToUInt16_t();

				auto value = analogRead(analog_port_map[port_n]);

				// OK<nl><value><nl>
				auto res = RamData(F_OKnl);
				{
					char tmp[7];
					utoa(value, tmp, DEC);
					res = res
						.Append(RamData((const byte *)tmp, strlen(tmp)))
						.Append(F_nl);
				}
				client.Write(res);
			}

			//-----------------------------------------------------------
			// analog port read pack
			// https://searchathing.com/?page_id=886#analog_read_pack
			//-----------------------------------------------------------
			else if (data.StartsWith(F_analog_read_pack_SP))
			{
				// analog_read <port> <saples-count><nl>
				auto port_n = 0;
				auto samples_count = 0;
				{
					auto words = data.StripBegin(F_analog_read_pack_SP).Split(' ');

					port_n = words.Get(0).ToUInt16_t();
					samples_count = words.Get(1).ToUInt16_t();
				}

				auto aport = analog_port_map[port_n];

				// OK<nl><data-pack>				
				auto sampleCountMax = (client.MaxDataBytes() - 4) / 2;

				if (samples_count > sampleCountMax)
				{
					client.Write(RamData(F_ERR_space).Append(F_SamplesOutOfPkt).Append(F_nl));
				}
				else
				{
					const int bufSize = 2 * samples_count;
					auto buf = (byte *)malloc(bufSize);
					if (buf == NULL)
					{
						OutOfMemoryError(client);
					}
					else
					{

						auto begin = millis();
						auto i = 0;
						while (i < bufSize)
						{
							auto value = analogRead(aport);
							buf[i++] = (value >> 8) & 0xff;
							buf[i++] = value & 0xff;
						}

						{
							char tmp[10];

							ultoa(TimeDiff(begin, millis()), tmp, DEC); // interval ms
							client.Write(RamData(F_OKnl).Append(tmp).Append(F_nl));
						}

						client.Write(buf, bufSize); // data

						free(buf);
					}
				}
			}
		}
	}

	DPrintln(F("client received a disconnect"));

	PrintFreeMemory();
}
