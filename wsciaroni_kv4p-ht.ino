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
#include "TuneToMetaData.hpp"
#include "FiltersMetaData.hpp"

// https://github.com/pschatzmann/arduino-audio-tools/
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/ContainerBinary.h"

////////////////////////////////////////////////////////////////////////////////
/// AudioTools Globals
////////////////////////////////////////////////////////////////////////////////
#define AUDIO_USE_SIN_FOR_TESTING

AudioInfo info(44100, 1, 16);

#ifndef AUDIO_USE_SIN_FOR_TESTING
// AnalogAudioStream in;
#else
SineWaveGenerator<int16_t> sineWave(32000);
GeneratedSoundStream<int16_t> in(sineWave);
#endif

auto &serial = Serial2;
EncoderL8 enc;
EncodedAudioStream enc_stream(&serial, &enc);
Throttle throttle(enc_stream);
StreamCopy copierOut(throttle, in, 256); // copies sound into Serial

AnalogAudioStream out;
BinaryContainerDecoder cont_dec(new DecoderL8());
EncodedAudioStream decoder(&serial, &cont_dec);
StreamCopy copier(out, decoder);

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
void metadataCallback(uint8_t *data, int len, void *ref);
void handleSimpleCmd(CommandValue command);
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

void loop()
{
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

void metadataCallback(uint8_t *data, int len, void *ref)
{
  assert(0 != len);
  CommandValue cmd = static_cast<CommandValue>(data[0]);
  if (len == 1)
  {
    handleSimpleCmd(cmd);
  }
  else if (CommandValue::COMMAND_TUNE_TO == cmd)
  {
    assert((len - 1) == sizeof(TuneToMetaData));
    // This is a TuneTo command
    TuneToMetaData tuneToCommand((TuneToMetaData *)(&data[1]));
    tuneTo(tuneToCommand.getTxFreq(), tuneToCommand.getRxFreq(), tuneToCommand.getToneInt(), tuneToCommand.getSquelchInt());
  }
  else if (CommandValue::COMMAND_FILTERS == cmd)
  {
    assert((len - 1) == sizeof(FiltersMetaData));
    FiltersMetaData filtersData((FiltersMetaData*)*(&data[1]));
    dra->filters(filtersData.getEmphasis(), filtersData.getHighpass(), filtersData.getHighpass());
  }
}

void handleSimpleCmd(CommandValue command)
{
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
#ifndef AUDIO_USE_SIN_FOR_TESTING
  auto config = in.defaultConfig(RX_MODE);
  config.copyFrom(info);
  in.begin(config);
#else
  sineWave.begin(info, N_B4);
  in.begin(info);
#endif

  throttle.begin(info);
  enc_stream.begin(info);

  // TX
  auto config2 = out.defaultConfig(TX_MODE);
  config2.copyFrom(info);
  out.begin(config2);

  decoder.begin(info);
  cont_dec.setMetaCallback(metadataCallback);
}
