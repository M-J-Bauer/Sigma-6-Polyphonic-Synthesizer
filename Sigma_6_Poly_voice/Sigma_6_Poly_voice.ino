/*
 * File:       Sigma_6_Poly_voice (.ino)
 *
 * Project:    Sigma-6 Voice Module for Polyphonic Instrument (no display)
 *
 * Platform:   RobotDyn SAMD21 M0-MINI dev board (MCU: ATSAMD21G18)
 *
 * Author:     M.J.Bauer, 2025 -- www.mjbauer.biz
 *
 * Licence:    Open Source (Unlicensed) -- free to copy, distribute, modify
 *
 * Version:    1.2  01-SEP-2025  (See Revision History file)
 */
#include <fast_samd21_tc3.h>
#include <Wire.h>
#include "m0_synth_def.h"

#define TESTPOINT3    7        // pin D7 is N/C on Poly-voice circuit

void  TC3_Handler(void);       // Audio ISR - defined in "m0_synth_engine"

ConfigParams_t  g_Config;      // structure holding config param's
uint8_t  g_MidiChannel;        // 1..16  (16 = broadcast, omni)
uint8_t  g_MidiMode;           // OMNI_ON_MONO or OMNI_OFF_MONO
uint8_t  g_MidiRegisParam;     // Registered Param # (0: PB range, 1: Fine Tuning)

//---------------------------------------------------------------------------------------
//
void  setup()
{
  uint8_t  channelSwitches = 0;

  pinMode(CHAN_SWITCH_S1, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S2, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S3, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S4, INPUT_PULLUP);

  pinMode(TESTPOINT1, OUTPUT);  // scope test-point TP1 (ISR)
  pinMode(TESTPOINT2, OUTPUT);  // scope test-point TP2 (GATE)
  pinMode(TESTPOINT3, OUTPUT);  // scope test-point TP3 (DEBUG)
  pinMode(SPI_DAC_CS, OUTPUT);
  if (!USE_SPI_DAC_FOR_AUDIO) pinMode(A0, OUTPUT);  // Use MCU on-chip DAC for audio
  digitalWrite(SPI_DAC_CS, HIGH);  // Set DAC CS High (idle)

  if (digitalRead(CHAN_SWITCH_S1) == HIGH)  channelSwitches += 1;
  if (digitalRead(CHAN_SWITCH_S2) == HIGH)  channelSwitches += 2;
  if (digitalRead(CHAN_SWITCH_S3) == HIGH)  channelSwitches += 4;
  if (digitalRead(CHAN_SWITCH_S4) == HIGH)  channelSwitches += 8;
  
  g_MidiChannel = channelSwitches;
  if (channelSwitches == 0)  g_MidiMode = OMNI_ON_MONO;
  else  g_MidiMode = OMNI_OFF_MONO;

  Serial1.begin(31250);        // initialize UART for MIDI IN
  Wire.begin();                // initialize IIC as master
  Wire.setClock(400*1000);     // set IIC clock to 400kHz
  analogReadResolution(10);    // set ADC resolution to 10 bits
  DefaultConfigData();         // Sigma-6 Poly Voice has NO EEPROM!
  PresetSelect(13);            // initialize synth engine!

  // Set wave-table sampling interval for audio ISR - Timer/Counter #3
  fast_samd21_tc3_configure((float) 1000000 / SAMPLE_RATE_HZ);  // period = 31.25us
  fast_samd21_tc3_start();
}

// Main background process loop...
//
void  loop()
{
  static uint32_t last_millis;

  MidiInputService();

  if (millis() != last_millis)  // once every millisecond...
  {
    last_millis = millis();
    SynthProcess();
  }
}


/*`````````````````````````````````````````````````````````````````````````````````````````````````
 * Function:     Copy patch parameters from a specified preset patch in flash
 *               program memory to the "active" patch parameter array in data memory.
 *
 * Entry args:   preset = index into preset-patch definitions array g_PresetPatch[]
 *
 */
void  PresetSelect(uint8_t preset)
{
  if (preset < GetNumberOfPresets())
  {
    memcpy(&g_Patch, &g_PresetPatch[preset], sizeof(PatchParamTable_t));
    SynthPrepare();
    g_Config.PresetLastSelected = preset;
//  StoreConfigData();  // Sigma-6 Poly Voice has NO EEPROM!
  }
}


/*`````````````````````````````````````````````````````````````````````````````````````````````````
 * Function:  MidiInputService()
 *
 * MIDI IN service routine, executed frequently from within main loop.
 * This routine monitors the serial MIDI INPUT stream and whenever a complete message is
 * received, it is processed.
 *
 * The synth module responds to valid messages addressed to the configured MIDI IN channel and,
 * if MIDI mode is set to 'Omni On' (channel-select switches set to 0), it will respond to all
 * messages received, regardless of which channel(s) the messages are addressed to.
 * The module also responds to valid messages addressed to channel 16, regardless of the channel
 * switch setting, so that the host controller can transmit a "broadcast" message to all modules
 * on the MIDI network simultaneously.
 */
void  MidiInputService()
{
  static  uint8_t  midiMessage[MIDI_MSG_MAX_LENGTH];
  static  short  msgBytesExpected;
  static  short  msgByteCount;
  static  short  msgIndex;
  static  uint8_t  msgStatus;     // last command/status byte rx'd
  static  bool   msgComplete;   // flag: got msg status & data set

  uint8_t  msgByte;
  uint8_t  msgChannel;  // 1..16 !
  BOOL     gotSysExMsg = FALSE;

  if (Serial1.available() > 0)  // unread byte(s) available in Rx buffer
  {
    msgByte = Serial1.read();

    if (msgByte & 0x80)  // command/status byte received (bit7 High)
    {
      if (msgByte == SYSTEM_MSG_EOX)
      {
        msgComplete = TRUE;
        gotSysExMsg = TRUE;
        midiMessage[msgIndex++] = SYSTEM_MSG_EOX;
        msgByteCount++;
      }
      else if (msgByte <= SYS_EXCLUSIVE_MSG)  // Ignore Real-Time messages
      {
        msgStatus = msgByte;
        msgComplete = FALSE;  // expecting data byte(s))
        midiMessage[0] = msgStatus;
        msgIndex = 1;
        msgByteCount = 1;  // have cmd already
        msgBytesExpected = MIDI_GetMessageLength(msgStatus);
      }
    }
    else  // data byte received (bit7 LOW)
    {
      if (msgComplete && msgStatus != SYS_EXCLUSIVE_MSG)
      {
        if (msgByteCount == 0)  // start of new data set -- running status
        {
          msgIndex = 1;
          msgByteCount = 1;
          msgBytesExpected = MIDI_GetMessageLength(msgStatus);
        }
      }
      if (msgIndex < MIDI_MSG_MAX_LENGTH)
      {
        midiMessage[msgIndex++] = msgByte;
        msgByteCount++;
      }
    }

    if ((msgByteCount != 0 && msgByteCount == msgBytesExpected) || gotSysExMsg)
    {
      msgComplete = TRUE;
      msgChannel = (midiMessage[0] & 0x0F) + 1;  // 1..16

      if (msgChannel == g_MidiChannel || msgChannel == 16
      ||  g_MidiMode == OMNI_ON_MONO  || msgStatus == SYS_EXCLUSIVE_MSG)
      {
        ProcessMidiMessage(midiMessage, msgByteCount);
//      g_MidiRxSignal = TRUE;  // signal to UI (not used in Poly voiuce)
        msgBytesExpected = 0;
        msgByteCount = 0;
        msgIndex = 0;
      }
    }
  }
}


void  ProcessMidiMessage(uint8_t *midiMessage, short msgLength)
{
  static uint8_t  noteKeyedFirst;
  uint8_t  statusByte = midiMessage[0] & 0xF0;
  uint8_t  noteNumber = midiMessage[1];  // New note keyed
  uint8_t  velocity = midiMessage[2];
  uint8_t  program = midiMessage[1];
  uint8_t  leverPosn_Lo = midiMessage[1];  // modulation
  uint8_t  leverPosn_Hi = midiMessage[2];
  short  bipolarPosn;
  bool   executeNoteOff = FALSE;
  bool   executeNoteOn = FALSE;

  switch (statusByte)
  {
    case NOTE_OFF_CMD:
    {
      SynthNoteOff(noteNumber);
      digitalWrite(TESTPOINT2, LOW);  // "Gate" LED off
      break;
    }
    case NOTE_ON_CMD:
    {
      if (velocity == 0) 
      {
        SynthNoteOff(noteNumber);
        digitalWrite(TESTPOINT2, LOW);  // "Gate" LED off
      }
      else  
      {
        SynthNoteOn(noteNumber, velocity);
        digitalWrite(TESTPOINT2, HIGH);  // "Gate" LED on
      }
      break;
    }
    case CONTROL_CHANGE_CMD:
    {
      ProcessControlChange(midiMessage);
      break;
    }
    case PROGRAM_CHANGE_CMD:
    {
      PresetSelect(program);
      break;
    }
    case PITCH_BEND_CMD:
    {
      bipolarPosn = ((short)(leverPosn_Hi << 7) | leverPosn_Lo) - 0x2000;
      SynthPitchBend(bipolarPosn);
      break;
    }
    case SYS_EXCLUSIVE_MSG:
    {
      ProcessMidiSystemExclusive(midiMessage, msgLength);
      break;
    }
    default:  break;
  }  // end switch
}


void  ProcessControlChange(uint8_t *midiMessage)
{
  static uint8_t  modulationHi = 0;    // High byte of CC data (7 bits)
  static uint8_t  expressionHi = 0;    // High byte of CC data (7 bits)
  uint8_t  CCnumber = midiMessage[1];  // Control Change 'register' number
  uint8_t  dataByte = midiMessage[2];  // Control Change data value
  uint8_t  oscnum;
  int    data14;

  if (CCnumber == 2 || CCnumber == 7 || CCnumber == 11)  // High byte
  {
    expressionHi = dataByte;
    data14 = (int) expressionHi << 7;
    SynthExpression(data14);
  }
  else if (CCnumber == 34 || CCnumber == 39 || CCnumber == 43)  // Low byte
  {
    data14 = (((int) expressionHi) << 7) + dataByte;
    SynthExpression(data14);
  }
  else if (CCnumber == 1)  // Modulation High Byte (01)
  {
    modulationHi = dataByte;
    data14 = ((int) modulationHi) << 7;
    SynthModulation(data14);
  }
  else if (CCnumber == 33)  // Modulation Low Byte
  {
    data14 = (((int) modulationHi) << 7) + dataByte;
    SynthModulation(data14);
  }
  // The following CC numbers are to set synth Configuration parameters:
  // ```````````````````````````````````````````````````````````````````
  else if (CCnumber == 100)  // MIDI "Registered Parameter" ID
  {
    g_MidiRegisParam = dataByte; 
  }
  else if (CCnumber == 38)  // Parameter "Data Entry" (LSB) message
  {
    if (g_MidiRegisParam == 0x00 && dataByte <= 12) g_Config.PitchBendRange = dataByte;
    if (g_MidiRegisParam == 0x01) g_Config.FineTuning_cents = (short)dataByte - 64;
  }
  else if (CCnumber == 86)  // Set audio ampld control mode
  {
    if (dataByte < 4)  g_Config.AudioAmpldCtrlMode = dataByte;
  }
  else if (CCnumber == 87)  // Set vibrato control mode
  {
    if (dataByte < 4)  g_Config.VibratoCtrlMode = dataByte;
  }
  else if (CCnumber == 88)  // Set pitch-bend control mode
  {
    if (dataByte < 4)  g_Config.PitchBendMode = dataByte;
  }
  else if (CCnumber == 89)  // Set reverb mix level
  {
    if (dataByte <= 100)  g_Config.ReverbMix_pc = dataByte;
  }
  // The following CC numbers are to set synth Patch parameters:
  // ```````````````````````````````````````````````````````````
  else if (CCnumber == 70)  // Set osc. mixer output gain (unit = 0.1)
  {
    if (dataByte != 0)  g_Patch.MixerOutGain_x10 = dataByte;
  }
  else if (CCnumber == 71)  // Set ampld limiter level (%), 0 => OFF
  {
    if (dataByte <= 95)  g_Patch.LimiterLevelPc = dataByte;
  }
  else if (CCnumber == 72)  // Set Ampld ENV Release Time (unit = 100ms)
  {
    if (dataByte != 0 && dataByte <= 100)  
      g_Patch.EnvReleaseTime = (uint16_t) dataByte * 100;
  }
  else if (CCnumber == 73)  // Set Ampld ENV Attack Time (unit = 10ms)
  {
    if (dataByte != 0 && dataByte <= 100)  
      g_Patch.EnvAttackTime = (uint16_t) dataByte * 10;
  }
  else if (CCnumber == 74)  // Set Ampld ENV Peak Hold Time (unit = 10ms)
  {
    if (dataByte <= 100)  g_Patch.EnvHoldTime = (uint16_t) dataByte * 10;
  }
  else if (CCnumber == 75)  // Set Ampld ENV Decay Time (unit = 100ms)
  {
    if (dataByte != 0 && dataByte <= 100)  
      g_Patch.EnvDecayTime = (uint16_t) dataByte * 100;
  }
  else if (CCnumber == 76)  // Set Ampld ENV Sustain Level (unit = 1%)
  {
    if (dataByte <= 100)  g_Patch.EnvSustainLevel = (uint16_t) dataByte;
  }
  else if (CCnumber == 77)  // Set LFO frequency (data = Hz, max 50)
  {
    if (dataByte != 0 && dataByte <= 50)  
      g_Patch.LFO_Freq_x10 = (uint16_t) dataByte * 10;
  }
  else if (CCnumber == 78)  // Set LFO ramp time (unit = 100ms)
  {
    if (dataByte <= 100)  g_Patch.LFO_RampTime = (uint16_t) dataByte * 100;
  }
  else if (CCnumber == 79)  // Set LFO FM (vibrato) depth (unit = 5 cents)
  {
    if (dataByte <= 120)  g_Patch.LFO_FM_Depth = (uint16_t) dataByte * 5;
  }
  else if (CCnumber == 80)  // Set Osc. Mixer Input Level
  {
    oscnum = (dataByte >> 4) % 6;  // MS digit (0..5)
    g_Patch.MixerInputStep[oscnum] = dataByte & 0x0F;  // LS digit (0..15)
  }
  else if (CCnumber == 112)  // LFO phase sync
  {
    SynthLFO_PhaseSync();  // Data byte ignored.
  }
  // Mode Change messages
  // ````````````````````````````````````````````````````````````````````````
  else if (CCnumber == 120 || CCnumber == 123)
  {
    SynthPrepare();  // All Sound Off & Kill note playing
  }
}


/*
 * The "manufacturer ID" (2nd byte of msg) is first validated to ensure the message
 * can be correctly interpreted, i.e. it's a Bauer exclusive message which contains
 * information about a Bauer MIDI controller (e.g. REMI) connected to the MIDI IN port.
 * Byte 3 of the message is a code to identify the type of message content.
 */
void  ProcessMidiSystemExclusive(uint8_t *midiMessage, short msgLength)
{
  if (midiMessage[1] == SYS_EXCL_REMI_ID)  // "Manufacturer ID" match
  {
      // Nothing to be done in this version !
  }
}


int  MIDI_GetMessageLength(uint8_t statusByte)
{
  uint8_t  command = statusByte & 0xF0;
  uint8_t  length = 0;  // assume unsupported or unknown msg type

  if (command == PROGRAM_CHANGE_CMD || command == CHAN_PRESSURE_CMD)  length = 2;
  if (command == NOTE_ON_CMD || command == NOTE_OFF_CMD
  ||  command == CONTROL_CHANGE_CMD || command == PITCH_BEND_CMD)
  {
      length = 3;
  }
  if (statusByte == SYS_EXCLUSIVE_MSG)  length = MIDI_MSG_MAX_LENGTH;

  return  length;
}


/*`````````````````````````````````````````````````````````````````````````````````````````````````
 *   Set default values for configuration param's, except those which are assigned values
 *   by reading config switches at start-up, e.g. MIDI channel and mode.
 *   Config param's may be changed subsequently by MIDI CC messages from the Master board.
 *
 *   Options for AudioAmpldCtrlMode, VibratoCtrlMode, PitchBendMode and MasterTuneOffset
 *   are defined in the header file: "m0_synth_def.h".
 */
void  DefaultConfigData(void)
{
  g_Config.AudioAmpldCtrlMode = AUDIO_CTRL_ENV1_VELO;
  g_Config.VibratoCtrlMode = VIBRATO_DISABLED;
  g_Config.PitchBendMode = PITCH_BEND_BY_MIDI_MSG;
  g_Config.PitchBendRange = 2;         // semitones (max. 12)
  g_Config.ReverbMix_pc = 15;          // 0..100 % (typ. 15)
  g_Config.PresetLastSelected = 1;     // user preference
  g_Config.FineTuning_cents = 0;
}


// ================================================================================================
// ==========  Instrument Presets -- Array of patch parameter tables in flash memory  =============
//
// ... Values defined for g_Patch.OscFreqMult[] ............................
// |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  | 10  | 11  | <- index
// | 0.5 |  1  | 4/3 | 1.5 |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |
// `````````````````````````````````````````````````````````````````````````
//
// ... Values defined for g_Patch.MixerInputStep[] ...........................................
// | 0 | 1 | 2 | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10  | 11  | 12  | 13  | 14  |  15 |  16  |
// | 0 | 5 | 8 | 11 | 16 | 22 | 31 | 44 | 63 | 88 | 125 | 177 | 250 | 353 | 500 | 707 | 1000 |
// ```````````````````````````````````````````````````````````````````````````````````````````
//
// ... Values defined for g_Patch.OscAmpldModSource[] .........................
// |  0   |   1   |   2   |  3   |  4   |    5   |    6   |  7  |  8   |  9   | <- index
// | None | CONT+ | CONT- | ENV2 | MODN | EXPRN+ | EXPRN- | LFO | VEL+ | VEL- |
// ````````````````````````````````````````````````````````````````````````````
//
// For EWI controllers, Presets 24 thru 31 have 'Ampld Control Mode' set to 'Expression' (3).
// ````````````````````````````````````````````````````````````````````````````````````````````````
//
const  PatchParamTable_t  g_PresetPatch[] =
{
  {
    "Sound Test",                   // 00
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 15, 13, 11, 0, 0, 0 },        // Mixer Input levels (0..16)
    5, 0, 200, 80, 200, 2,          // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay, Sus %
    50, 500, 20, 20,                // LFO: Hz x10, Ramp, FM%, AM%
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  //  Presets with percussive ampld envelope profile, some with piano semblance
  {
    "Electric Piano #1",            // 01
    { 1, 3, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..7)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 14, 12, 8, 8, 5, 0 },         // Osc Mixer level/step (0..16)
    10, 70, 1500, 0, 500, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 1000, 95,                // Contour Env (S-D-R-H)
    200, 16,                        // ENV2: Dec, Sus %
    30, 500, 0, 20,                 // LFO: Hz x10, Ramp, FM %, AM %
    33, 60,                         // Mixer Gain x10, Limit %FS
  },
  {
    "Electric Piano #2",            // 02  
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 3, 3, 3, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 12, 13, 9, 9, 6 },        // Osc Mixer level/step (0..16)
    10, 50, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 1000, 95,                // Contour Env (S-D-R-H)
    700, 50,                        // ENV2: Decay/Rel, Sus %
    30, 500, 0, 20,                 // LFO: Hz x10, Ramp, FM %, AM %
    20, 50,                         // Mixer Gain x10, Limit %FS
  },
  {
    "Trashy Toy Piano",             // 03
    { 1, 1, 1, 4, 6, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { -18, 0, 19, -14, 0, 16 },     // Osc Detune, cents (+/-600)
    { 13, 13, 13, 11, 9, 7 },       // Osc Mixer level/step (0..16)
    5, 50, 500, 0, 300, 2,          // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 1000, 95,                // Contour Env (S-D-R-H)
    200, 50,                        // ENV2: Decay/Rel, Sus %
    30, 500, 30, 20,                // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Steel-tine Clavier",           // 04
    { 1, 4, 5, 8, 9, 10 },          // Osc Freq Mult index (0..11)
    { 0, 2, 1, 2, 7, 1 },           // Osc Ampld Modn src (0..9)
    { 0, -21, 19, -27, -31, 0 },    // Osc Detune cents (+/-600)
    { 12, 12, 12, 10, 12, 12 },     // Osc Mixer levels (0..16)
    5, 20, 700, 0, 700, 2,          // Amp Env (A-H-D-S-R), Amp Mode
    5, 50, 800, 80,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    100, 500, 0, 55,                // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Tubular Bells",                // 05
    { 1, 6, 9, 8, 0, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 7, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 33, -35, 0, 0, 0 },        // Osc Detune, cents (-600..+600)
    { 9, 13, 13, 0, 0, 0 },         // Mixer Input levels (0..16)
    5, 100, 2000, 0, 2000, 2,       // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay, Sus %
    30, 500, 10, 20,                // LFO: Hz x10, Ramp, FM%, AM%
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Smart Vibraphone",             // 06
    { 0, 1, 4, 6, 7, 11 },          // Osc Freq Mult index (0..11)
    { 7, 7, 0, 7, 0, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 0, 13, 0, 9, 0, 13 },         // Osc Mixer level/step (0..16)
    5, 50, 2000, 0, 2000, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    0, 0, 200, 100,                 // Contour Env (S-D-R-H)
    500, 35,                        // ENV2: Decay/Rel, Sus %
    80, 5, 0, 40,                   // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Guitar Synthetique",           // 07
    { 1, 5, 6, 8, 9, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 1, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 13, 7, 11, 8, 9, 8 },         // Osc Mixer level/step (0..16)
    5, 200, 2000, 4, 700, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    25, 0, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 20,                // LFO: Hz x10, Ramp, FM %, AM %
    20, 60,                         // Mixer Gain x10, Limit %FS
  },
  // Presets with organ-like sounds; some with transient envelope(s)...
  {
    "Jazz Organ #1",                // 08
    { 0, 1, 5, 8, 0, 0 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 3, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, 0, -3, 4, 0 },          // Osc Detune cents (+/-600)
    { 10, 13, 15, 12, 0, 0 },       // Osc Mixer levels (0..16)
    10, 0, 400, 100, 300, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    100, 50,                        // ENV2: Dec, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Jazz Organ #2",                // 09
    { 0, 1, 4, 5, 8, 0 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, -8, 4, -10, 0 },        // Osc Detune cents (+/-600)
    { 11, 14, 7, 14, 11, 0 },       // Osc Mixer levels (0..16)
    10, 0, 400, 100, 300, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    200, 50,                        // ENV2: Dec, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Rock Organ #3",                // 10  (aka 'Rock Organ #3')
    { 0, 3, 1, 4, 6, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, -6, -7, 0, 0 },         // Osc Detune cents (+/-600)
    { 13, 13, 13, 13, 0, 0 },       // Osc Mixer levels (0..16)
    10, 0, 400, 100, 300, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Floyd Organ",             // 11
    { 0, 3, 6, 0, 3, 6 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 6, 5, 4, -6, -5, -4 },        // Osc Detune cents (+/-600)
    { 13, 10, 10, 13, 10, 10 },     // Osc Mixer levels (0..16)
    30, 0, 200, 100, 200, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 15, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Hammondish Organ",             // 12
    { 1, 3, 4, 5, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 3 },           // Osc Ampld Modn source (0..7)
    { 0, -7, 12, 4, 0, 3 },         // Osc Detune cents (+/-600)
    { 13, 3, 0, 9, 0, 15 },         // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Dec, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Bauer Organ #1",               // 13
    { 1, 4, 6, 8, 10, 0 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 4, -4, 3, -2, 3 },         // Osc Detune, cents (-600..+600)
    { 13, 13, 0, 9, 13, 11 },       // Mixer Input levels (0..16)
    20, 20, 400, 70, 300, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM%, AM%
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Meditation Pipe",              // 14  (* todo:  Add AM using Contour *)
    { 1, 4, 6, 7, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 6 },           // Osc Ampld Modn src (0..9) <== todo
    { 0, -5, 0, 4, 0, 0 },          // Osc Detune cents (+/-600)
    { 13, 14, 0, 9, 10, 9 },        // Osc Mixer levels (0..16)
    50, 0, 200, 80, 200, 2,         // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    100, 50,                        // ENV2: Dec, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Full Swell Organ",             // 15  (Good for bass!)
    { 0, 1, 4, 5, 6, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, 4, -3, 3, -3 },         // Osc Detune cents (+/-600)
    { 8, 14, 13, 11, 10, 7 },       // Osc Mixer levels (0..16)
    5, 0, 5, 100, 300, 2,           // Amp Env (A-H-D-S-R), Amp Mode
    0, 50, 300, 100,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  // Miscellaneous experimental "instruments"  --------------------------------
  {
    "Morph Harmonium",              // 16
    { 0, 3, 1, 7, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 1, 2, 1 },           // Osc Ampld Modn src (0..9)
    { 3, -3, 0, -3, 3, -3 },        // Osc Detune cents (+/-600)
    { 13, 12, 13, 8, 9, 11 },       // Osc Mixer levels (0..16)
    30, 0, 10, 80, 500, 2,          // Amp Env (A-H-D-S-R), Amp Mode
    10, 50, 300, 90,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    70, 500, 10, 55,                // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Ring Modulator",               // 17
    { 1, 3, 4, 5, 8, 0 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 0, 11, 14, 0, 0, 0 },         // Osc Mixer levels (0..16)
    10, 0, 400, 100, 300, 2,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Bass Overdrive",               // 18
    { 0, 1, 4, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 3, 0, 3 },           // Osc Ampld Modn source (0..7)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 14, 12, 8, 10, 0, 12 },       // Osc Mixer level/step (0..16)
    5, 0, 200, 80, 200, 2,          // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 20, 20,                // LFO: Hz x10, Ramp, FM %, AM %
    33, 50,                         // Mixer Gain x10, Limit %FS
  },
  {
    "Bellbird  (JPM)",              // 19  (created by JPM)
    { 9, 5, 8, 1, 8, 5 },           // Osc Freq Mult index (0..11)
    { 7, 3, 3, 3, 7, 7 },           // Osc Ampld Modn source (0..7)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 4, 8, 2, 10, 14, 15 },        // Osc Mixer level/step (0..16)
    70, 50, 100, 50, 700, 2,        // Ampld Env (A-H-D-S-R), Amp Mode
    0, 200, 500, 100,               // Contour Env (S-D-R-H)
    3000, 50,                       // ENV2: Dec, Sus %
    30, 70, 40, 35,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Dull Steel Drum",              // 20  (old name: Dull Tone)
    { 1, 4, 5, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 1, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, -14, 0, 23, 0, 0 },        // Osc Detune, cents (+/-600)
    { 15, 10, 4, 11, 8, 6 },        // Osc Mixer level/step (0..16)
    10, 50, 500, 0, 500, 2,         // Ampld Env (A-H-D-S-R), Amp Mode
    20, 0, 50, 80,                  // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    100, 0, 0, 0,                   // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Wobulator",                    // 21
    { 0, 1, 2, 3, 4, 5 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 7, 7, 7 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 0, 13, 14, 0, 14 },       // Osc Mixer level/step (0..16)
    30, 300, 1000, 25, 1000, 2,     // Ampld Env (A-H-D-S-R), Amp Mode
    20, 0, 200, 80,                 // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    80, 200, 30, 25,                // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Hollow Wood Drum",             // 22  (created by JPM)
    { 0, 1, 2, 3, 4, 5 },           // Osc Freq Mult index (0..11)
    { 6, 7, 6, 2, 0,  2 },          // Osc Ampld Modn source (0..7)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 5, 5, 4, 12, 8, 14 },         // Osc Mixer level/step (0..16)
    5, 20, 100, 0, 300, 2,          // Ampld Env (A-H-D-S-R), Amp Mode
    20, 0, 200, 80,                 // Contour Env (S-D-R-H)
    1500, 25,                       // ENV2: Dec, Sus %
    40, 5, 20, 20,                  // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Soft-attack Accordian",        // 23
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 1, 1, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 12, 11, 10, 9, 8 },       // Osc Mixer level/step (0..16)
    100, 200, 2000, 10, 70, 2,      // Ampld Env (A-H-D-S-R), Amp Mode
    100, 0, 300, 30,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 20,                // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  // Presets with Amp Control by Expression (for EWI controllers)...
  {
    "Terrible Recorder",            // 24  (aka 'Treble Recorder')
    { 1, 5, 7, 9, 11, 0 },          // Osc Freq Mult index (0..11)
    { 0, 0, 5, 0, 5, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 11, 13, 9, 13, 0 },       // Osc Mixer level/step (0..16)
    50, 0, 200, 80, 200, 3,         // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Psychedelic Oboe",             // 25  (* Add AM using exprn &/or mod'n *)
    { 1, 3, 4, 5, 6, 9 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)  <== todo
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 11, 0, 11, 12, 14, 0 },       // Osc Mixer levels (0..16)
    30, 0, 200, 80, 200, 3,         // Amp Env (A-H-D-S-R), Amp Mode
    100, 10, 1000, 25,              // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Stopped Flute",                // 26  (* Add AM using exprn &/or mod'n *)
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9) <== todo
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 15, 9, 6, 0, 0, 5 },          // Osc Mixer levels (0..16)
    50, 0, 200, 80, 200, 3,         // Amp Env (A-H-D-S-R), Amp Mode
    0, 50, 300, 100,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 15, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Spaced Out Pipe",              // 27  (aka 'Pink Floyd Organ')
    { 0, 3, 6, 0, 3, 6 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9)
    { 6, 5, 4, -6, -5, -4 },        // Osc Detune cents (+/-600)
    { 13, 10, 10, 13, 10, 10 },     // Osc Mixer levels (0..16)
    30, 0, 200, 80, 200, 3,         // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 15, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Mellow Reed",                  // 28  (* Add AM using exprn &/or mod'n *)
    { 1, 5, 6, 7, 8, 0 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn src (0..9) <== todo
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 14, 9, 6, 12, 12, 0 },        // Osc Mixer levels (0..16)
    30, 0, 200, 80, 200, 3,         // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Dec, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Melody Organ #2",              // 29  (aka 'Bauer Organ #2')
    { 1, 3, 4, 5, 8, 0 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 0 },           // Osc Ampld Modn src (0..9)
    { 0, 4, -4, 3, -2, 3 },         // Osc Detune, cents (-600..+600)
    { 13, 13, 10, 12, 14, 12 },     // Mixer Input levels (0..16)
    20, 20, 400, 70, 300, 3,        // Amp Env (A-H-D-S-R), Amp Mode
    5, 20, 600, 40,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay, Sus %
    70, 500, 30, 0,                 // LFO: Hz x10, Ramp, FM%, AM%
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Reed Overdrive",               // 30
    { 1, 4, 5, 7, 8, 9 },           // Osc Freq Mult index (0..11)
    { 0, 5, 5, 5, 5, 5 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 12, 10, 10, 13, 13 },     // Osc Mixer level/step (0..16)
    70, 0, 200, 80, 200, 3,         // Ampld Env (A-H-D-S-R), Amp Mode
    5, 20, 500, 95,                 // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    25, 50,                         // Mixer Gain x10, Limit %FS

  },
  {
    "Deep Saxophoney",              // 31
    { 0, 1, 4, 5, 6, 7 },           // Osc Freq Mult index (0..11)
    { 5, 0, 5, 0, 5, 5 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 9, 10, 13, 0, 13, 12 },       // Osc Mixer level/step (0..16)
    70, 0, 200, 80, 200, 3,         // Ampld Env (A-H-D-S-R), Amp Mode
    0, 50, 300, 100,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    20, 50,                         // Mixer Gain x10, Limit %FS
  },
};


// Function returns the number of Predefined Patch definitions...
//
int  GetNumberOfPresets(void)
{
  return  (int) sizeof(g_PresetPatch) / sizeof(PatchParamTable_t);
}

