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

#ifndef _SEARCHATHING_ARDUINO_SERVO_LOOP_PROC_H
#define _SEARCHATHING_ARDUINO_SERVO_LOOP_PROC_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <SearchAThing.Arduino.Utils\DebugMacros.h>

#include <SearchAThing.Arduino.Utils\RamData.h>
using namespace SearchAThing::Arduino;

#include <SearchAThing.Arduino.Net\EthProcess.h>
using namespace SearchAThing::Arduino::Net;

namespace SearchAThing
{

	namespace Arduino
	{

		namespace Servo
		{

			// nr. of characters required to store an unsigned int
			// 0-65536
			const int UINT_CHARS = 6;

			// nr. of characters required to store an unsigned long
			// 0-4 xxx xxx xxx
			const int ULONG_CHARS = 10;

			class LoopProc : public EthProcess
			{

				uint16_t uptimeDays = 0;
				unsigned long lastMilllis;

			public:
				LoopProc();

				void LoopProcessImpl(EthNet *net);

				RamData UpTimeStr() const;

			};

		}

	}

}

#endif
