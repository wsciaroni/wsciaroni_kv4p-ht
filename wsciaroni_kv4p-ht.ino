/*
KV4P-HT (see http://kv4p.com)
Copyright (C) 2024 Vance Vagell

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <DRA818.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>

// Headers
#include "CommandValueEnum.hpp"
#include "ModeEnum.hpp"
#include "MsgTypeEnum.hpp"
#include "Constants.hpp"

// https://github.com/pschatzmann/arduino-audio-tools/
#include "AudioTools.h"

////////////////////////////////////////////////////////////////////////////////
/// AudioTools Globals
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// Application Globals
////////////////////////////////////////////////////////////////////////////////

const byte FIRMWARE_VER[8] = {'0', '0', '0', '0', '0', '0', '0', '1'}; // Should be 8 characters representing a zero-padded version, like 00000001.
const byte VERSION_PREFIX[7] = {'V', 'E', 'R', 'S', 'I', 'O', 'N'};    // Must match RadioAudioService.VERSION_PREFIX in Android app.

// Mode of the app, which is essentially a state machine
Mode mode = Mode::MODE_STOPPED;

// Object used for radio module serial comms
DRA818 *dra = new DRA818(&Serial2, DRA818_VHF);

////////////////////////////////////////////////////////////////////////////////
/// Forward Declarations
////////////////////////////////////////////////////////////////////////////////

/// Setup Functions
void setInitialState();

void setupSerial();
void setupWDT();
void setupLED();
void setupDRA818();
void setupAudioTools();

/// State Transition Functions
void setMode(Mode newMode);
void handleCMD(CommandValue command);
void handleDATA();
int16_t getLengthOfDataToRead();
void tuneTo(float freqTx, float freqRx, int tone, int squelch);
void stopTx();
void startTx();
void stopRx();
void startRx();

void setup()
{
  setupSerial();
  setupWDT();
  setupLED();
  setupDRA818();
  setInitialState();
  setupAudioTools();
}

MsgType msgType;

void handleIncomingSerial()
{
  msgType = static_cast<MsgType>(Serial.read());

  switch (msgType)
  {
  case MsgType::CMD:
    handleCMD(static_cast<CommandValue>(Serial.read()));
    break;
  case MsgType::DATA:
    handleDATA();
    break;
  default:
    break;
  }
}

void loop()
{
  if (Serial.available())
  {
    handleIncomingSerial();
  }

  esp_task_wdt_reset();
}

void setMode(Mode newMode)
{
  if (mode == newMode)
  {
    return;
  }
  if (Mode::MODE_TX == mode)
  {
    stopTx();
  }
  if (Mode::MODE_TX == newMode)
  {
    startTx();
  }
  if (Mode::MODE_RX == mode)
  {
    stopRx();
  }
  if (Mode::MODE_RX == newMode)
  {
    startRx();
  }
  mode = newMode;
  switch (mode)
  {
  case Mode::MODE_STOPPED:
    digitalWrite(LED_PIN, LOW);
    digitalWrite(PTT_PIN, HIGH);
    break;
  case Mode::MODE_RX:
    digitalWrite(LED_PIN, LOW);
    digitalWrite(PTT_PIN, HIGH);
    // initI2SRx();
    break;
  case Mode::MODE_TX:
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(PTT_PIN, LOW);
    break;
  }
}

void handleCMD(CommandValue command)
{
  // TODO: Convert this to be a Metadata Callback from Binary
  switch (command)
  {
  case CommandValue::COMMAND_PTT_DOWN:
  {
    // output->start(I2S_NUM_0, i2sPins, sampleSource);
    setMode(Mode::MODE_TX);
  }
  break;
  case CommandValue::COMMAND_PTT_UP:
  {
    // output->stop(I2S_NUM_0);
    setMode(Mode::MODE_RX);
  }
  break;
  case CommandValue::COMMAND_TUNE_TO:
  {
    // Example:
    // 145.450144.850061
    // 7 chars for tx, 7 chars for rx, 2 chars for tone, 1 char for squelch (17 bytes total for params)
    setMode(Mode::MODE_RX);

    // If we haven't received all the parameters needed for COMMAND_TUNE_TO, wait for them before continuing.
    // This can happen if ESP32 has pulled part of the command+params from the buffer before Android has completed
    // putting them in there. If so, we take byte-by-byte until we get the full params.
    int paramBytesMissing = 17;
    String paramsStr = "";
    if (paramBytesMissing > 0)
    {
      uint8_t paramPartsBuffer[paramBytesMissing];
      for (int j = 0; j < paramBytesMissing; j++)
      {
        unsigned long waitStart = micros();
        while (!Serial.available())
        {
          // Wait for a byte.
          if ((micros() - waitStart) > 500000)
          { // Give the Android app 0.5 second max before giving up on the command
            esp_task_wdt_reset();
            return;
          }
        }
        paramPartsBuffer[j] = Serial.read();
      }
      paramsStr += String((char *)paramPartsBuffer);
      paramBytesMissing--;
    }
    float freqTxFloat = paramsStr.substring(0, 7).toFloat();
    float freqRxFloat = paramsStr.substring(7, 14).toFloat();
    int toneInt = paramsStr.substring(14, 16).toInt();
    int squelchInt = paramsStr.substring(16, 17).toInt();

    // Serial.println("PARAMS: " + paramsStr.substring(0, 16) + " freqTxFloat: " + String(freqTxFloat) + " freqRxFloat: " + String(freqRxFloat) + " toneInt: " + String(toneInt));

    tuneTo(freqTxFloat, freqRxFloat, toneInt, squelchInt);
  }
  break;
  case CommandValue::COMMAND_FILTERS:
  {
    int paramBytesMissing = 3; // e.g. 000, in order of emphasis, highpass, lowpass
    String paramsStr = "";
    if (paramBytesMissing > 0)
    {
      uint8_t paramPartsBuffer[paramBytesMissing];
      for (int j = 0; j < paramBytesMissing; j++)
      {
        unsigned long waitStart = micros();
        while (!Serial.available())
        {
          // Wait for a byte.
          if ((micros() - waitStart) > 500000)
          { // Give the Android app 0.5 second max before giving up on the command
            esp_task_wdt_reset();
            return;
          }
        }
        paramPartsBuffer[j] = Serial.read();
      }
      paramsStr += String((char *)paramPartsBuffer);
      paramBytesMissing--;
    }
    bool emphasis = (paramsStr.charAt(0) == '1');
    bool highpass = (paramsStr.charAt(1) == '1');
    bool lowpass = (paramsStr.charAt(2) == '1');

    dra->filters(emphasis, highpass, lowpass);
  }
  break;
  case CommandValue::COMMAND_STOP:
  {
    // Serial.flush();
  }
  break;
  case CommandValue::COMMAND_GET_FIRMWARE_VER:
  {
    Serial.write(VERSION_PREFIX, sizeof(VERSION_PREFIX));
    Serial.write(FIRMWARE_VER, sizeof(FIRMWARE_VER));
  }
  break;
  default:
    break;
  }
}

void handleDATA()
{
  int16_t numberOfBytesToRead = getLengthOfDataToRead();
  while (numberOfBytesToRead > 0)
  {
    int16_t bytesAvailableForRead = std::min(TX_AUDIO_CHUNK_SIZE, Serial.available());
    int16_t bytesToReadThisTime = std::min(numberOfBytesToRead, bytesAvailableForRead);
    uint8_t *bytes = new uint8_t[bytesToReadThisTime];
    Serial.readBytes(bytes, bytesToReadThisTime);
    // TODO: write these bytes to the stream
    delete[] bytes;
    numberOfBytesToRead -= bytesToReadThisTime;
    // TODO: Determine if we need to reset the WDT here
  }
}

int16_t getLengthOfDataToRead()
{
  uint8_t bytes[2];
  while (Serial.available() < 2)
  {
    // Wait for two bytes
  }
  Serial.readBytes(bytes, 2);
  return ((bytes[0] << 8) | bytes[1]);
}

void tuneTo(float freqTx, float freqRx, int tone, int squelch)
{
  int result = dra->group(DRA818_25K, freqTx, freqRx, tone, squelch, 0);
}

void stopTx()
{
  // TODO: Stop the Tx audio streams
}

void startTx()
{
  // TODO: Start the Tx audio streams
}

void stopRx()
{
  // TODO: Stop the Rx audio streams
}

void startRx()
{
  // TODO: Start the Rx audio streams
}

////////////////////////////////////////////////////////////////////////////////
// Setup Functions
////////////////////////////////////////////////////////////////////////////////

void setupSerial()
{
  // Communication with Android via USB cable
  Serial.begin(921600);
  Serial.setRxBufferSize(USB_BUFFER_SIZE);
  Serial.setTxBufferSize(USB_BUFFER_SIZE);
}

void setupWDT()
{
  // Configure watch dog timer (WDT), which will reset the system if it gets stuck somehow.
  esp_task_wdt_init(10, true); // Reboot if locked up for a bit
  esp_task_wdt_add(NULL);      // Add the current task to WDT watch
}

void setupLED()
{
  // Debug LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void setupDRA818()
{
  /////////////////////////////// Setup Radio Module
  // Set up radio module defaults
  pinMode(PD_PIN, OUTPUT);
  digitalWrite(PD_PIN, HIGH); // Power on
  pinMode(SQ_PIN, INPUT);
  pinMode(PTT_PIN, OUTPUT);
  digitalWrite(PTT_PIN, HIGH); // Rx

  // Communication with DRA818V radio module via GPIO pins
  Serial2.begin(9600, SERIAL_8N1, RXD2_PIN, TXD2_PIN);

  int result = -1;
  while (result != 1)
  {
    result = dra->handshake(); // Wait for module to start up
  }

  // Serial.println("handshake: " + String(result));
  // tuneTo(146.700, 146.700, 0, 0);
  result = dra->volume(8);
  // Serial.println("volume: " + String(result));
  result = dra->filters(false, false, false);
  // Serial.println("filters: " + String(result));
}

void setInitialState()
{
  // Begin in STOPPED mode
  setMode(Mode::MODE_STOPPED);
}

void setupAudioTools()
{
  
}
