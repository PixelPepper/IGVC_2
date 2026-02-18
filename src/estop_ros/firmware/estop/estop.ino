/**
 * Copyright (C) 2023  kbkn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * status = status | (estop << 2) | (reset << 3) | (remote << 4);
 *
 * status: normal(00),stop(10),reset(01)
 *        0x00     0x01     0x02     0x03     0x04     0x05     0x06     0x07
 * remote |       0|       0|       0|       0|       0|       0|       0|       0|
 * reset  |     on0|     on0|     on0|     on0|     on0|     on0|     on0|     on0|
 * estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
 * status |      00|      01|      10|      11|      00|      01|      10|      11|
 * --------------------------------------------------------------------------------
 *        |  stop10|  stop10|  stop10|  stop10| reset01|normal00| reset01|normal00|
 * 0x02,0x02,0x02,0x02,0x01,0x00,0x01,0x00,
 *        0x08     0x09     0x0a     0x0b     0x0c     0x0d     0x0e     0x0f
 * remote |       0|       0|       0|       0|       0|       0|       0|       0|
 * reset  |    off1|    off1|    off1|    off1|    off1|    off1|    off1|    off1|
 * estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
 * status |      00|      01|      10|      11|      00|      01|      10|      11|
 * --------------------------------------------------------------------------------
 *        |  stop10|  stop10|  stop10|  stop10|normal00|normal00|  stop10|  stop10|
 * 0x02,0x02,0x02,0x02,0x00,0x00,0x02,0x02,
 *        0x10     0x11     0x12     0x13     0x14     0x15     0x16     0x17
 * remote |       1|       1|       1|       1|       1|       1|       1|       1|
 * reset  |     on0|     on0|     on0|     on0|     on0|     on0|     on0|     on0|
 * estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
 * status |      00|      01|      10|      11|      00|      01|      10|      11|
 * --------------------------------------------------------------------------------
 *        |  stop10|  stop10|  stop10|  stop10| reset01| reset01| reset01| reset01|
 * 0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,
 *        0x18     0x19     0x1a     0x1b     0x1c     0x1d     0x1e     0x1f
 * remote |       1|       1|       1|       1|       1|       1|       1|       1|
 * reset  |    off1|    off1|    off1|    off1|    off1|    off1|    off1|    off1|
 * estop  |     on0|     on0|     on0|     on0|    off1|    off1|    off1|    off1|
 * status |      00|      01|      10|      11|      00|      01|      10|      11|
 * --------------------------------------------------------------------------------
 *        |  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|  stop10|
 * 0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02
 */

#include <TimerTC3.h>

#define DEBUG
#define SERIAL_PORT Serial // to USBC
#define SERIAL_PORT1 Serial1 // to txDenPin
#define USE_USBCON  // seeeduino xiao model needs this flag to be defined

#define txDenPin 2
#define estopPin 3
#define resetPin 4
#define remotePin 5

const char mode[] = { 
  0x02,0x02,0x02,0x02,0x01,0x00,0x01,0x00,
  0x02,0x02,0x02,0x02,0x00,0x00,0x02,0x02,
  0x02,0x02,0x02,0x02,0x01,0x01,0x01,0x01,
  0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02
};

const char estop_cmd[] = { 0x01, 0x06, 0x20, 0x0E, 0x00, 0x05, 0x23, 0xCA };
const char reset_cmd[] = { 0x01, 0x06, 0x20, 0x0E, 0x00, 0x06, 0x63, 0xCB };

volatile int timercount = 0;
volatile int count = 0;
volatile int status = 1;
volatile int estop = 0;
volatile int remote = 0;
volatile int reset = 0;

void TimerCnt()
{
  timercount++;
  count++;
}

void setup()
{
  SERIAL_PORT.begin(115200);
  SERIAL_PORT1.begin(115200);

  pinMode(txDenPin, OUTPUT);
  pinMode(estopPin, INPUT_PULLUP);
  pinMode(resetPin, INPUT_PULLUP);
  pinMode(remotePin, INPUT_PULLUP);

  status = 0x0c;
  digitalWrite(txDenPin, LOW);

  TimerTc3.initialize(1000);
  TimerTc3.attachInterrupt(TimerCnt);
  TimerTc3.start();

  pinMode(LED_BUILTIN, OUTPUT);
  while (SERIAL_PORT.available())
    SERIAL_PORT.read();

  delay(1000);
}

void loop()
{
  if (count % 500 == 0)
  {
    remote = digitalRead(remotePin);
    estop = digitalRead(estopPin);
    reset = digitalRead(resetPin);
    status &= 0x03;
    status = mode[status | (estop << 2) | (reset << 3) | (remote << 4)];

    switch (status)
    {
      case 0:  // normal
        break;

      case 1:  // reset
        sendSignal(0x00, reset_cmd, sizeof(reset_cmd));
        break;

      case 2:  // stop
        sendSignal(0x01, estop_cmd, sizeof(estop_cmd));
        break;

      default:
        break;
    }
  }
}

void sendSignal(uint8_t value, const char *cmd, size_t cmd_size)
{
  digitalWrite(txDenPin, HIGH);
  timercount = 0;

  SERIAL_PORT.write(value);
  SERIAL_PORT1.write(cmd, cmd_size);
  while (timercount < 1)
    SERIAL_PORT.flush();
    SERIAL_PORT1.flush();

  digitalWrite(txDenPin, LOW);

  while (timercount < 2)
    while (SERIAL_PORT.available())
      SERIAL_PORT.read();

  digitalWrite(LED_BUILTIN, value == 0x00 ? HIGH : LOW);
}
