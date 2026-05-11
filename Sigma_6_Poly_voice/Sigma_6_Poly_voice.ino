/*
 * File:       Sigma_6_Poly_voice (.ino)
 *
 * Project:    Sigma-6 Voice Module for Polyphonic synth. (no display, no EEPROM)
 *
 * Platform:   RobotDyn SAMD21 M0-MINI dev board (MCU: ATSAMD21G18)
 *             <!> In Arduino IDE, select board type: "Arduino Zero (Native USB)"
 *
 * Author:     M.J.Bauer, 2025 -- www.mjbauer.biz
 *
 * Licence:    Open Source (Unlicensed) -- free to copy, distribute, modify
 *
 * Version:    1.8  11-05-2026   (See Revision History file)
 *             Now supports "Mono-voice" mode for Multi-timbral operation.
 */
#include <fast_samd21_tc3.h>
#include <Wire.h>
#include "m0_synth_def.h"

#define GPIOA_PIN_MODE_OUT(bitnum)  (PORT_IOBUS->Group[0].DIRSET.reg = (1 << bitnum))
#define GPIOA_PIN_SET_HIGH(bitnum)  (PORT_IOBUS->Group[0].OUTSET.reg = (1 << bitnum))
#define GPIOA_PIN_SET_LOW(bitnum)   (PORT_IOBUS->Group[0].OUTCLR.reg = (1 << bitnum))

#define TX_LED  27  // PORT_A bit 27

void  TC3_Handler(void);       // Audio ISR - defined in "m0_synth_engine"

ConfigParams_t  g_Config;      // structure holding config param's

uint8_t  voiceChannel;         // MIDI channel DIP-switch setting: 1..15
uint8_t  midiRegstParam;       // Registered Param # (0: PB range, 1: Fine Tuning)
uint8_t  midiMessageFlag;      // Signal MIDI RX activity (for debug purposes)
bool     isMonoVoice;          // TRUE if voice channel is independent of Poly Master
bool     isBroadcast;          // TRUE if last received MIDI message is broadcast

//---------------------------------------------------------------------------------------
//
void  setup()
{
  uint8_t  channelSwitches = 0;

  pinMode(CHAN_SWITCH_S1, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S2, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S3, INPUT_PULLUP);
  pinMode(CHAN_SWITCH_S4, INPUT_PULLUP);
  GPIOA_PIN_SET_HIGH(TX_LED);  // TX LED off
  GPIOA_PIN_MODE_OUT(TX_LED);
  pinMode(TESTPOINT1, OUTPUT);  // scope test-point TP1 (ISR)
  pinMode(TESTPOINT2, OUTPUT);  // scope test-point TP2 (GATE)
  pinMode(SPI_DAC_CS, OUTPUT);
  if (!USE_SPI_DAC_FOR_AUDIO) pinMode(A0, OUTPUT);  // Use MCU on-chip DAC for audio
  digitalWrite(SPI_DAC_CS, HIGH);  // Set DAC CS High (idle)

  if (digitalRead(CHAN_SWITCH_S1) == HIGH)  channelSwitches += 1;
  if (digitalRead(CHAN_SWITCH_S2) == HIGH)  channelSwitches += 2;
  if (digitalRead(CHAN_SWITCH_S3) == HIGH)  channelSwitches += 4;
  if (digitalRead(CHAN_SWITCH_S4) == HIGH)  channelSwitches += 8;
  voiceChannel = channelSwitches;

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
  static uint32_t TXLED_ONtime_begin;

  MidiInputService();

  if (millis() != last_millis)  // once every millisecond...
  {
    last_millis = millis();
    SynthProcess();
  }
  
/***  Debug usage only ...

  if (midiMessageFlag)
  {
    TXLED_ONtime_begin = millis();
    GPIOA_PIN_SET_LOW(TX_LED);  // TX LED On
    midiMessageFlag = 0;
  }

  if ((millis() - TXLED_ONtime_begin) >= 100)  // ON-time expired
    GPIOA_PIN_SET_HIGH(TX_LED);  // TX LED Off
***/	
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
  }
}


/*```````````````````````````````````````````````````````````````````````````````````````
 * Function:  MidiInputService()
 *
 * MIDI IN service routine, executed frequently from within main loop.
 * This routine monitors the serial MIDI INPUT stream and whenever a complete message is
 * received, it is processed.
 *
 * The module responds to valid messages addressed to the configured MIDI IN channel.
 * The module also responds to valid messages addressed to channel 16, regardless of the 
 * channel switch setting, so that the host controller can transmit a "broadcast" message
 * to all modules on the "MIDI bus" simultaneously.
 */
void  MidiInputService()
{
  static  uint8_t  midiMessage[MIDI_MSG_MAX_LENGTH];
  static  short  msgBytesExpected;
  static  short  msgByteCount;
  static  short  msgIndex;
  static  uint8_t  msgStatus;  // last command/status byte rx'd
  static  uint8_t  msgChannel;  // 1..16 ! (0 = invalid, 16 = broadcast)
  static  bool   runningStatus;  // flag: got msg status & data set

  uint8_t  msgByte;
  
  BOOL     gotSysExMsg = FALSE;

  if (Serial1.available() > 0)  // unread byte(s) available in Rx buffer
  {
    msgByte = Serial1.read();

    if ((msgByte & 0x80) && msgByte < 0xF0)  // command/status byte
    {
        msgStatus = msgByte;
        msgChannel = (msgStatus & 0x0F) + 1;  // 1..16
        if (msgChannel == 16) isBroadcast = TRUE;
        else  isBroadcast = FALSE;
        runningStatus = FALSE;  // expecting data byte(s)
    }

    if (msgChannel == voiceChannel || isBroadcast)
    {
      if ((msgByte & 0x80) && msgByte < 0xF0)  // command/status byte
      {
        midiMessage[0] = msgStatus;
        msgIndex = 1;
        msgByteCount = 1;  // have cmd already
        msgBytesExpected = MIDI_GetMessageLength(msgStatus);
      }
      else if ((msgByte & 0x80) == 0)  // data byte (bit7 = 0)
      {
        if (runningStatus && msgByteCount == 0)  // start new data set
        {
          msgIndex = 1;
          msgByteCount = 1;  // have status byte already
          msgBytesExpected = MIDI_GetMessageLength(msgStatus);
        }
        if (msgIndex < MIDI_MSG_MAX_LENGTH)
        {
          midiMessage[msgIndex++] = msgByte;
          msgByteCount++;
        }
      }
      if (msgByteCount != 0 && msgByteCount == msgBytesExpected)
      {
        runningStatus = TRUE;  // have complete message
        ProcessMidiMessage(midiMessage, msgByteCount);
        midiMessageFlag = 1;
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
      digitalWrite(TESTPOINT2, LOW);
      GPIOA_PIN_SET_HIGH(TX_LED);  // "Gate" LED off
      break;
    }
    case NOTE_ON_CMD:
    {
      if (velocity == 0) 
      {
        SynthNoteOff(noteNumber);
        digitalWrite(TESTPOINT2, LOW);
        GPIOA_PIN_SET_HIGH(TX_LED);  // "Gate" LED off
      }
      else  
      {
        SynthNoteOn(noteNumber, velocity);
        digitalWrite(TESTPOINT2, HIGH);
        GPIOA_PIN_SET_LOW(TX_LED);  // "Gate" LED on
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
      if (!isMonoVoice) PresetSelect(program);  // case: Poly-voice channel
      else if (!isBroadcast) PresetSelect(program);  // case: Mono-voice
      break;
    }
    case PITCH_BEND_CMD:
    {
      bipolarPosn = ((short)(leverPosn_Hi << 7) | leverPosn_Lo) - 0x2000;
      SynthPitchBend(bipolarPosn);
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
  // The following CC numbers are to set synth configuration parameters:
  // ```````````````````````````````````````````````````````````````````
  else if (CCnumber == 100)  // MIDI "Registered Parameter" ID
  {
    midiRegstParam = dataByte; 
  }
  else if (CCnumber == 38)  // Parameter "Data Entry" (LSB) message
  {
    if (midiRegstParam == 0x00 && dataByte <= 12) g_Config.PitchBendRange = dataByte;
    if (midiRegstParam == 0x01) g_Config.FineTuning_cents = (short)dataByte - 64;
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
    if (dataByte <= 100)
    {
      g_Config.ReverbMix_pc = dataByte;
      SynthSetReverbMix(dataByte);  // effective immediately
    }
  }
  // Sigma-6 Poly/Mono voice specific messages
  // `````````````````````````````````````````
  if (CCnumber == 64)  // Set ENV1 Hold/Sustain state (on or off)
  {
    // Broadcast Hold/Sustain messages are ignored by Mono-voice modules
    if (!isMonoVoice || !isBroadcast) SynthSetHoldSustain((dataByte >= 64) ? 1 : 0);
  }
  else if (CCnumber == 112)  // LFO phase sync
  {
    SynthLFO_PhaseSync();  // Data byte ignored.
  }
  else if (CCnumber == 113)  // Set 'Mono' (independent) or 'Poly' voice mode
  {
    if (dataByte >= 64) isMonoVoice = TRUE;  
    else  isMonoVoice = FALSE;
  }
  else if (CCnumber == 120 || CCnumber == 123)
  {
    SynthPrepare();  // All Sound Off & Kill note playing
  }

  // Mono-voice must reject broadcasts which alter patch parameters
  if (isMonoVoice && isBroadcast)  return;  

  // The following CC numbers are to set synth Patch parameters:
  // ```````````````````````````````````````````````````````````
  if (CCnumber == 70)  // Set osc. mixer output gain (unit = 0.1)
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
  else if (CCnumber == 77)  // Set LFO frequency (data unit = 1Hz, max 50)
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
 *   by reading config switches at start-up, e.g. MIDI channel.
 *   Config param's may be changed subsequently by MIDI CC messages from the Master board.
 *
 *   Options for AudioAmpldCtrlMode, VibratoCtrlMode, PitchBendMode and MasterTuneOffset
 *   are defined in the header file: "m0_synth_def.h".
 */
void  DefaultConfigData(void)
{
  g_Config.AudioAmpldCtrlMode = AUDIO_CTRL_ENV1_VELO;
  g_Config.VibratoCtrlMode = VIBRATO_AUTOMATIC;
  g_Config.PitchBendMode = PITCH_BEND_DISABLED;
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
    "Keith Emerson Organ",          // 10 (Alt.)
    { 0, 3, 1, 4, 5, 6 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { -3, 3, 0, -3, 3, 0 },         // Osc Detune, cents (+/-600)
    { 12, 12, 12, 12, 12, 10 },     // Osc Mixer level/step (0..16)
    20, 0, 5, 100, 300, 2,          // Ampld Env (A-H-D-S-R), Amp Mode
    0, 50, 300, 100,                // Contour Env (S-D-R-H)
    700, 35,                        // ENV2: Decay/Rel, Sus %
    70, 300, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
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
    "Bellbird Recall",              // 19  (alt. revised)
    { 4, 6, 7, 8, 9, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 3, 3, 3 },           // Osc Ampld Modn source (0..7)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune cents (+/-600)
    { 0, 8, 12, 12, 12, 8 },        // Osc Mixer level/step (0..16)
    50, 20, 3000, 0, 2000, 2,       // Ampld Env (A-H-D-S-R) - Bell
    0, 200, 500, 100,               // Contour Env (S-D-R-H)
    2000, 50,                       // ENV2: Dec, Sus %
    200, 50, 60, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
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
    { 5, 0, 5, 4, 5, 4 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 13, 10, 10, 10, 11, 9 },      // Osc Mixer level/step (0..16)
    70, 0, 200, 80, 200, 3,         // Ampld Env (A-H-D-S-R), Amp Mode
    0, 50, 300, 100,                // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    25, 40,                         // Mixer Gain x10, Limit %FS
  },
  // ============  Banks 3 & 4 (32 Presets) added in version 3.0  =============
  //         ( See project web page for explanation of nomenclature. )
  {
    "Pink No Osc AM",               // 32  Fender Rhodes ripoff
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 12, 10, 0, 0, 6  },       // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Modn Osc 3 & 4",          // 33
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 4, 4, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 11, 11, 12, 8 },      // Osc Mixer level/step (0..16)
    10, 100, 2000, 0, 2000, 2,      // Ampld Env (A-H-D-S-R) - Bell
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Modn Osc 3 & 5",          // 34
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 4, 0, 4, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 12, 11, 11, 11, 9 },      // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R) - Organ
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Contour 3 & 5",           // 35
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 2, 0, 1, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 6, 13, 8, 13, 0 },        // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R)
    20, 50, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Contour 3,4,5,6",         // 36
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 2, 2, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 14, 12, 12, 12, 12, 12 },     // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Transient 4,5,6",         // 37
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 3, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 8, 12, 10, 12, 10 },      // Osc Mixer level/step (0..16)
    10, 200, 2000, 4, 700, 2,       // Ampld Env (A-H-D-S-R) - Guitar
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 0,                         // ENV2: Decay/Rel, Sus %
    60, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Pink Transient 5 & 6",         // 38
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 8, 9, 8, 12, 12, 12 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    7, 0,                           // Mixer Gain x10, Limit %FS
  },
  {
    "Pink No AM detune 2-6",        // 39
    { 1, 4, 5, 6, 7, 8 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 6, -6, 6, -6, 6, -6 },        // Osc Detune, cents (+/-600)
    { 12, 7, 8, 10, 6, 10 },        // Osc Mixer level/step (0..16)
    10, 200, 2000, 4, 700, 2,       // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Grey No Osc AM",               // 40
    { 1, 3, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 13, 12, 11, 10, 9, 8 },       // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Grey Modn Osc 3 & 4",          // 41
    { 1, 3, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 4, 4, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 12, 12, 12, 0, 0 },       // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Grey Contour 3 & 6",           // 42
    { 1, 3, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 1, 0, 0, 2 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 0, 12, 9, 10, 10 },       // Osc Mixer level/step (0..16)
    100, 0, 200, 80, 200, 2,        // Ampld Env (A-H-D-S-R)
    0, 50, 1000, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Grey Transient 5 & 6",         // 43
    { 1, 3, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 11, 8, 8, 0, 10, 10 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    20, 200, 1000, 80,              // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Cyan No Osc AM",               // 44
    { 1, 4, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 6, 0, 6, 5 },         // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Cyan Modn Osc 4,5,6",          // 45
    { 1, 4, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 4, 4, 4 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 0, 10, 0, 8 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Cyan Contour Osc 3,6",         // 46
    { 1, 4, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 1, 0, 0, 2 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 0, 12, 10, 0, 12 },       // Osc Mixer level/step (0..16)
    100, 0, 200, 80, 200, 2,        // Ampld Env (A-H-D-S-R) - Flute
    25, 100, 500, 100,              // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    70, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Cyan EG2 5,6  detune",         // 47
    { 1, 4, 5, 7, 9, 11 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, -12, 12, -12, 12, 0 },     // Osc Detune, cents (+/-600)
    { 6, 10, 12, 10, 8, 0 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    20, 200, 1000, 80,              // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow No Osc AM",             // 48
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 11, 10, 9, 8, 7 },        // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Modn Osc 3,4",          // 49
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 4, 4, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 10, 13, 13, 10, 6 },      // Osc Mixer level/step (0..16)
    10, 100, 2000, 0, 2000, 2,      // Ampld Env (A-H-D-S-R) - Bell
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Modn Osc 3,5",          // 50
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 4, 0, 4, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 12, 12, 12, 12, 7 },      // Osc Mixer level/step (0..16)
    100, 0, 200, 80, 200, 2,        // Ampld Env (A-H-D-S-R) - Flute
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Contour 4 & 5",         // 51
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 1, 1, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 10, 12, 12, 0 },      // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R)
    20, 0, 500, 100,                // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Contour 3..6",          // 52
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 2, 2, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 12, 10, 12, 12 },     // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Transient 4-6",         // 53
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 3, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 10, 10, 10, 12, 10 },     // Osc Mixer level/step (0..16)
    10, 200, 2000, 4, 700, 2,       // Ampld Env (A-H-D-S-R) - Guitar
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow Transient 5,6",         // 54
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 12, 12, 11, 12, 10 },     // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R)
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    500, 50,                        // ENV2: Decay/Rel, Sus %
    60, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Yellow No AM, detune",         // 55
    { 0, 1, 4, 6, 8, 10 },          // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, -6, 6, -6, 6, -6 },        // Osc Detune, cents (+/-600)
    { 12, 11, 10, 9, 8, 7 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R) - Organ
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Orange No Osc AM",             // 56
    { 0, 3, 5, 6, 8, 9 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 13, 12, 11, 10, 9 },      // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) 
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Orange Modn Osc 5 & 6",        // 57
    { 0, 3, 5, 6, 8, 9 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 4, 4 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 10, 10, 10, 10, 13, 13 },     // Osc Mixer level/step (0..16)
    100, 0, 200, 80, 200, 2,        // Ampld Env (A-H-D-S-R) - Flute
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Orange Contour+ 3, 4",         // 58
    { 0, 3, 5, 6, 8, 9 },           // Osc Freq Mult index (0..11)
    { 0, 0, 1, 1, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 10, 0, 12, 0, 6 },        // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    10, 50, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Orange Contour 3 & 6",         // 59
    { 0, 3, 5, 6, 8, 9 },           // Osc Freq Mult index (0..11)
    { 0, 0, 1, 0, 0, 2 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 0, 12, 10, 0, 12 },       // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R) - Organ
    10, 50, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Green No Osc AM",              // 60
    { 1, 2, 3, 4, 5, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 0, 0 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 12, 0, 10, 10, 6, 5 },        // Osc Mixer level/step (0..16)
    20, 70, 1500, 0, 300, 2,        // Ampld Env (A-H-D-S-R) - Piano
    0, 100, 500, 100,               // Contour Env (S-D-R-H) - Med 100
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 500, 0, 0,                  // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Green Modn Osc 4 & 6",         // 61
    { 1, 2, 3, 4, 5, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 4, 0, 4 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 13, 0, 11, 13, 0, 0 },        // Osc Mixer level/step (0..16)
    100, 0, 200, 80, 200, 2,        // Ampld Env (A-H-D-S-R) - Flute
    0, 100, 500, 100,               // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    50, 200, 10, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Green Contur Osc 3-6",         // 62
    { 1, 2, 3, 4, 5, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 2, 2, 1, 1 },           // Osc Ampld Modn source (0..9)
    { 0, 0, 0, 0, 0, 0 },           // Osc Detune, cents (+/-600)
    { 6, 0, 10, 10, 8, 8 },         // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R) - Organ
    10, 200, 1000, 100,             // Contour Env (S-D-R-H)
    200, 25,                        // ENV2: Decay/Rel, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  },
  {
    "Green EG2 5,6 detune",         // 63
    { 1, 2, 3, 4, 5, 7 },           // Osc Freq Mult index (0..11)
    { 0, 0, 0, 0, 3, 3 },           // Osc Ampld Modn source (0..9)
    { 0, -12, 12, -12, 12, 0 },     // Osc Detune, cents (+/-600)
    { 13, 0, 0, 10, 10, 9 },        // Osc Mixer level/step (0..16)
    10, 0, 400, 100, 300, 2,        // Ampld Env (A-H-D-S-R) - Organ
    20, 200, 1000, 80,              // Contour Env (S-D-R-H)
    1000, 5,                        // ENV2: Decay/Rel, Sus %
    70, 500, 20, 0,                 // LFO: Hz x10, Ramp, FM %, AM %
    10, 0,                          // Mixer Gain x10, Limit %FS
  }
};


// Function returns the number of Predefined Patch definitions...
//
int  GetNumberOfPresets(void)
{
  return  (int) sizeof(g_PresetPatch) / sizeof(PatchParamTable_t);
}

