/*
 * File:       Sigma_6_Poly_master (.ino)
 *
 * Project:    Sigma-6 Poly-synth Master Controller application
 *
 * Platform:   RobotDyn SAMD21 M0-MINI dev board (MCU: ATSAMD21G18)
 *
 * Author:     M.J.Bauer, 2025 -- www.mjbauer.biz
 *
 * Reference:  https://www.mjbauer.biz/Sigma6_Poly_synth_weblog.htm
 *
 * Licence:    Open Source (Unlicensed) -- free to copy, distribute, modify
 *
 * Notes:   1. The value defined by the symbol NUMBER_OF_VOICES (see #define on line 21)
 *             should match the synth hardware configuration, typically 6, 8, 10 or 12 voices.
 *             (The symbol value may be lower than the actual number of voices.)
 *             
 *          2. The table of Preset patch definitions - g_PresetPatch[] - must be an exact
 *             copy of the respective table defined in the Poly-voice firmware code.
 */
#include <Wire.h>

#define FIRMWARE_VERSION  "0.88"

#define NUMBER_OF_VOICES   6  // Set according to hardware configuration

#define MIDI_MSG_MAX_LENGTH  32
#define SYS_EXCLUSIVE_MSG  0xF0
#define SYSTEM_MSG_EOX  0xF7
#define OMNI_ON     1       // MIDI IN mode: Omni-On Poly
#define OMNI_OFF    3       // MIDI IN mode: Omni-Off Poly
#define BROADCAST   16      // MIDI OUT channel for broadcast
#define GATE_OFF    0xFF    // Voice/Channel status
#define INVALID     0xFFFF  // Patch param value not assigned
#define HOME_SCREEN_ID  2   // Defined in "poly_synth_panel.ino"

#define NOTE_OFF_CMD         0x80    // 3-byte message
#define NOTE_ON_CMD          0x90    // 3-byte message
#define KEY_PRESSURE_CMD     0xA0    // 2-byte message
#define CONTROL_CHANGE_CMD   0xB0    // 3-byte message
#define PROGRAM_CHANGE_CMD   0xC0    // 2-byte message
#define CHAN_PRESSURE_CMD    0xD0    // 2-byte message
#define PITCH_BEND_CMD       0xE0    // 3-byte message

// MCU I/O pin assignments.................
#define KEYBOARD_SS     8    // SPI slave-select: Keyboard interface
#define DEBUG_LED       8    // Temporary
#define HEARTBEAT_LED   2    // Temporary

#define QUOTE  (char)34  // ASCII double-quote char
#define OFF    0
#define ON     1
#ifndef FALSE
#define FALSE  0
#define TRUE  (!FALSE)
#endif

typedef struct table_of_configuration_params
{
  uint8_t  MidiChannel;             // MIDI IN channel (1..16, 0: Omni-On)
  uint8_t  PitchBendEnable;         // Pitch Bend TX messages (0:Off, 1:On)
  uint8_t  PitchBendRange;          // Pitch Bend range, semitones (1..12)
  uint8_t  ReverbMix_pc;            // Reverb. wet/dry mix (0..100 %)
  uint8_t  PresetLastSelected;      // Preset last selected (0..127)
  uint8_t  MasterTuneOffset;        // Master tuning (cents + 64)
  uint8_t  DisplayBrightness;       // OLED contrast setting (5..100 %)
  uint8_t  VoiceTuning[16];         // Voice fine tuning (cents + 64)
  uint8_t  UserPresetBase[8];       // Base Presets of the 8 Favorites
  //
  uint32_t EEpromCheckWord;         // Data integrity check...
                                    // EEPROM data defaulted if this changes!
} ConfigParams_t;

ConfigParams_t  g_Config;  // structure holding configuration param's

// Data structure for active patch (g_Patch); also 'User Presets' in EEPROM:
typedef  struct  synth_patch_param_table
{
  char     PresetName[24];        // Preset (patch) name, up to 22 chars
  uint16_t OscFreqMult[6];        // One of 12 options (encoded 0..11)
  uint16_t OscAmpldModSource[6];  // One of 10 options (encoded 0..9)
  short    OscDetune[6];          // Unit = cents (range 0..+/-600)
  uint16_t MixerInputStep[6];     // Mixer Input Levels (encoded 0..15)
  ////
  uint16_t EnvAttackTime;         // 5..5000+ ms
  uint16_t EnvHoldTime;           // 0..5000+ ms (if zero, skip Decay)
  uint16_t EnvDecayTime;          // 5..5000+ ms
  uint16_t EnvSustainLevel;       // Unit = 1/100 (range 0..100 %)
  uint16_t EnvReleaseTime;        // 5..5000+ ms
  uint16_t AmpControlMode;        // One of 4 options (encoded 0..3)
  ////
  uint16_t ContourStartLevel;     // Unit = 1/100 (range 0..100 %)
  uint16_t ContourDelayTime;      // 0..5000+ ms
  uint16_t ContourRampTime;       // 5..5000+ ms
  uint16_t ContourHoldLevel;      // Unit = 1/100 (range 0..100 %)
  uint16_t Env2DecayTime;         // 5..5000+ ms
  uint16_t Env2SustainLevel;      // Unit = 1/100 (range 0..100 %)
  ////
  uint16_t LFO_Freq_x10;          // LFO frequency x10 (range 5..250)
  uint16_t LFO_RampTime;          // 5..5000+ ms
  uint16_t LFO_FM_Depth;          // Unit = 1/100 semitone (cents, max. 600)
  uint16_t LFO_AM_Depth;          // Unit = 1/100 (0..100 %FS)
  uint16_t MixerOutGain_x10;      // Unit = 1/10  (value = gain x10, 0..100)
  uint16_t LimiterLevelPc;        // Audio limiter level (%), 0: Disabled

} PatchParamTable_t;

PatchParamTable_t  g_Patch;   // Active patch data
extern  const   PatchParamTable_t  g_PresetPatch[];

uint8_t  g_channelStatus[16];   // Note number (12..120) if GATE ON; 0xFF if GATE_OFF
uint8_t  g_FavoriteSelected;    // Favorite Preset selected (1..8, 0 => None)
uint8_t  g_NumberOfPresets;     // Number of Presets defined in Poly-voice firmware
uint8_t  g_MidiMode;            // OMNI_ON_POLY (1) or OMNI_OFF_POLY (3)
uint8_t  g_MidiRegisParam;      // Registered Param # (0: PB range, 1: Master Tune)
bool     g_PatchModified;       // True if any active patch param is modified
bool     g_DisplayEnabled;      // True if OLED display initialized OK
bool     g_MidiRxSignal;        // Signal MIDI message received (for GUI icon)
bool     g_EEpromFaulty;        // True if EEPROM error or not fitted
bool     g_MonophonicTestMode;  // True in monophonic test mode (e.g. voice tuning)
uint8_t  g_VoiceUnderTest;      // Voice/channel # in monophonic test mode

//---------------------------------------------------------------------------------------

void  setup()
{
  uint8_t  n;

  Serial.begin(57600);         // initialize USB port for serial CLI
  Serial1.begin(31250);        // initialize UART for MIDI IN/OUT
  Wire.begin();                // initialize IIC as master
  Wire.setClock(400*1000);     // set IIC clock to 400kHz
  analogReadResolution(10);    // set ADC resolution to 10 bits
  pinMode(HEARTBEAT_LED, OUTPUT);  // Temp.  todo: remove
  pinMode(DEBUG_LED, OUTPUT);      // Temp.  todo: remove
  
  if (EEpromACKresponse() == FALSE)
    { g_EEpromFaulty = TRUE; }  // IIC bus error or EEprom not fitted

  if (FetchConfigData() == 0 || g_Config.EEpromCheckWord != 0xABCDE085)  // Data corrupted
  {
    DefaultConfigData();  
    for (n = 0;  n < 8;  n++)  // Default the 8 Fav. Presets...
    {
      g_Config.UserPresetBase[n] = n + 1;  // 1..8
      memcpy(&g_Patch, &g_PresetPatch[n+1], sizeof(PatchParamTable_t));
      strcpy(g_Patch.PresetName, "Unnamed_Fav_Preset");
      StoreUserPreset(n);
    }
    StoreConfigData();
  }

  if (SSD1309_Init())  // If OLED controller responding on IIC bus...
  {
    g_DisplayEnabled = TRUE;
    while (millis() < 200) ;   // delay for OLED init
    SSD1309_Test_Pattern();    // test OLED display
    while (millis() < 800) ;   // delay to view test pattern
    Disp_ClearScreen();
    SSD1309_SetContrast(g_Config.DisplayBrightness);  // Saved setting
    GoToNextScreen(0);         // 0 => STARTUP SCREEN
  }

  if (g_Config.MidiChannel == 0)  g_MidiMode = OMNI_ON;
  else  g_MidiMode = OMNI_OFF;

  // Initialize the voice channels............
  MIDI_SendProgramChange(BROADCAST, g_Config.PresetLastSelected);  // Recall last preset
  MIDI_SendControlChange(BROADCAST, 68, 0);   // Legato Mode: always Disabled
  MIDI_SendControlChange(BROADCAST, 86, 2);   // Ampld Control: always ENV1*VELO
  MIDI_SendControlChange(BROADCAST, 89, g_Config.ReverbMix_pc);
  MIDI_SendControlChange(BROADCAST, 88, g_Config.PitchBendEnable);
  // If pitch bend enabled, send MIDI msg to disable vibrato, and vice-versa...
  if (g_Config.PitchBendEnable) MIDI_SendControlChange(BROADCAST, 87, 0);   // Vibrato disabled
  else  MIDI_SendControlChange(BROADCAST, 87, 3);   // Vibrato auto-ramp
  // The following 2 messages must be sent in sequence...
  MIDI_SendControlChange(BROADCAST, 100, 0);  // Reg. Param 0 = Pitch-Bend range
  MIDI_SendControlChange(BROADCAST, 38, g_Config.PitchBendRange);  // Data Entry

  for (n = 0; n < NUMBER_OF_VOICES; n++)
  {
    g_channelStatus[n] = GATE_OFF;
    MIDI_SendControlChange(n+1, 100, 1);  // Reg. Param 1 = Fine Tuning
    MIDI_SendControlChange(n+1, 38, g_Config.VoiceTuning[n]);  // Data Entry
  }

  g_NumberOfPresets = GetNumberOfPresets();
  PresetSelect(g_Config.PresetLastSelected);

  // todo : turn off FAV Preset LED indicators
}


// Main background process loop...
//
void  loop()
{
  static uint32_t startPeriod_5ms;
  static uint32_t startPeriod_50ms;
  static uint32_t startPeriod_500ms;

  MidiInputService();
  ServicePortRoutine();

  if ((millis() - startPeriod_5ms) >= 5)  // 5ms period ended
  {
    startPeriod_5ms = millis();
    PotService();
  }

  if ((millis() - startPeriod_50ms) >= 50)  // 50ms period ended
  {
    startPeriod_50ms = millis();
    ButtonScan();
    //digitalWrite(HEARTBEAT_LED, OFF);  // Temp. - todo: remove
    if (g_DisplayEnabled) UserInterfaceTask();
  }

  if ((millis() - startPeriod_500ms) >= 500)  // 500ms period ended
  {
    startPeriod_500ms = millis();
    //digitalWrite(HEARTBEAT_LED, ON);  // Temp. - todo: remove
  }
}


/**
 * Function:   Send MIDI Program Change message to all voice-channels;
 *             Save preset/program number as PresetLastSelected in EEPROM.
 *             Copy preset patch parameters from flash PM to active patch in data RAM.
 *
 * Entry arg:  preset/program number (max. NumberOfPresets)
 */
void  PresetSelect(uint8_t preset)
{
  if (preset < g_NumberOfPresets)
  {
    g_Config.PresetLastSelected = preset;
    StoreConfigData();
    MIDI_SendProgramChange(BROADCAST, preset);
    memcpy(&g_Patch, &g_PresetPatch[preset], sizeof(PatchParamTable_t));
    g_FavoriteSelected = 0;  // None
    g_PatchModified = FALSE;  // pending changes
  }
}


/**
 * This function configures the voice modules with patch parameters associated with a
 * particular User Preset, aka 'Favorite' (= arg. favNum).  
 *
 * Each of the 8 Favorites is based on the Preset selected at the time the active patch
 * was saved. This may be a Factory Preset or a User Preset. In the case of a Factory
 * Preset, the "base preset" for the Favorite is therefore g_Config.PresetLastSelected.
 *
 * The configuration parameter g_Config.PresetLastSelected is *never* replaced by a User
 * Preset base, so that at power-on/restart, the last selected Voice Preset is reloaded
 * regardless of whether a Favorite may have been selected at the time of power-off.
 */
void  RecallUserPreset(uint8_t favNum)  // favNum = 0..7
{
  uint8_t  basePreset = g_Config.UserPresetBase[favNum];
  uint8_t  oscNum, dataValue;

  MIDI_SendProgramChange(BROADCAST, basePreset);  // Voice patch := User Preset base
  FetchUserPreset(favNum);  // Load active patch from EEPROM
  g_FavoriteSelected = favNum + 1;  // 1..8
  g_PatchModified = FALSE;  // pending changes

  // Send all potentially modified patch param's to voice modules...
  MIDI_SendControlChange(BROADCAST, 79, g_Patch.LFO_FM_Depth / 5);
  MIDI_SendControlChange(BROADCAST, 77, g_Patch.LFO_Freq_x10 / 10);
  MIDI_SendControlChange(BROADCAST, 78, g_Patch.LFO_RampTime / 100);
  MIDI_SendControlChange(BROADCAST, 73, g_Patch.EnvAttackTime / 10);
  MIDI_SendControlChange(BROADCAST, 74, g_Patch.EnvHoldTime / 10);
  MIDI_SendControlChange(BROADCAST, 75, g_Patch.EnvDecayTime / 100);
  MIDI_SendControlChange(BROADCAST, 76, g_Patch.EnvSustainLevel);
  MIDI_SendControlChange(BROADCAST, 72, g_Patch.EnvReleaseTime / 100);
  MIDI_SendControlChange(BROADCAST, 70, g_Patch.MixerOutGain_x10);
  MIDI_SendControlChange(BROADCAST, 71, g_Patch.LimiterLevelPc);
  
  for (oscNum = 0;  oscNum < 6;  oscNum++)
  {
    dataValue = (oscNum << 4) + ((uint8_t)g_Patch.MixerInputStep[oscNum] & 0x0F);
    MIDI_SendControlChange(BROADCAST, 80, dataValue);
  }
}


/*````````````````````````````````````````````````````````````````````````````````````````
 * Function:  MidiInputService()
 *
 * MIDI IN service routine, executed frequently from within main loop.
 * This routine monitors the serial MIDI INPUT stream and whenever a complete message
 * is received, it is processed.
 *
 * The Master responds to valid messages addressed to the configured MIDI IN channel
 * and, if MIDI mode is set to 'Omni On' (MIDI IN channel = 0), it will respond to any
 * message received, regardless of which channel the message is addressed to.
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
  uint8_t  setChannel = g_Config.MidiChannel;
  bool     gotSysExMsg = FALSE;

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

    if (msgByteCount != 0 && msgByteCount == msgBytesExpected)
    {
      msgComplete = TRUE;
      msgChannel = (midiMessage[0] & 0x0F) + 1;  // 1..16

      if (msgChannel == setChannel || msgChannel == 16 || g_MidiMode == OMNI_ON)
      {
        ProcessMidiMessage(midiMessage, msgByteCount);
        g_MidiRxSignal = TRUE;  // signal to GUI to flash MIDI Rx icon
        msgBytesExpected = 0;
        msgByteCount = 0;
        msgIndex = 0;
      }
    }
  }
}


void  ProcessMidiMessage(uint8_t *midiMessage, short msgLength)
{
  static uint8_t  lastChannel;  // last channel/voice allocated a note
  uint8_t  statusByte = midiMessage[0] & 0xF0;
  uint8_t  noteNumber = midiMessage[1];
  uint8_t  velocity = midiMessage[2];
  uint8_t  program = midiMessage[1];
  uint16_t data14bits = ((uint16_t)midiMessage[1] << 8) + midiMessage[2];
  bool     executeNoteOff = FALSE;
  bool     executeNoteOn = FALSE;
  uint8_t  count = 0;
  uint8_t  voice = lastChannel;

  switch (statusByte)
  {
    case NOTE_OFF_CMD:
    {
      executeNoteOff = TRUE;
      break;
    }
    case NOTE_ON_CMD:
    {
      if (velocity == 0) executeNoteOff = TRUE;
      else  executeNoteOn = TRUE;
      break;
    }
    case CONTROL_CHANGE_CMD:
    {
      ProcessControlChange(midiMessage);
      break;
    }
    case PROGRAM_CHANGE_CMD:
    {
      PresetSelect(program);  // ignored if program # undefined
      break;
    }
    case PITCH_BEND_CMD:
    {
      if (g_Config.PitchBendEnable) MIDI_SendPitchBend(BROADCAST, data14bits);
      break;
    }
    default:  break;
  }  // end switch

  if (executeNoteOff)  // Terminate note on key release
  {
    if (g_MonophonicTestMode)
    {
      voice = g_VoiceUnderTest;  // 0..N-1
      MIDI_SendNoteOff(voice+1, noteNumber);
      return;
    }
    // Normal polyphonic mode...
    for (voice = 0;  voice < NUMBER_OF_VOICES;  voice++)
    {
      if (g_channelStatus[voice] == noteNumber)
      {
        MIDI_SendNoteOff(voice+1, noteNumber);
        g_channelStatus[voice] = GATE_OFF;
      }
    }
    return;
  }

  if (executeNoteOn)  // Initiate new note keyed
  {
    if (g_MonophonicTestMode)
    {
      voice = g_VoiceUnderTest;  // 0..N-1
      MIDI_SendNoteOn(voice+1, noteNumber, velocity);
      return;
    }
    // Normal polyphonic mode...
    voice = lastChannel + 1;  // oldest channel last active
    for (count = 0; count < NUMBER_OF_VOICES; count++, voice++)
    {
      if (voice >= NUMBER_OF_VOICES) voice = 0;  // wrap
      if (g_channelStatus[voice] == GATE_OFF)
      {
        MIDI_SendNoteOn(voice+1, noteNumber, velocity);
        g_channelStatus[voice] = noteNumber;
        lastChannel = voice;
        break;  // activate one voice only
      }
    }
    // todo: If all channels are busy (gated), terminate the oldest note, 
    // i.e. the note playing in (lastChannel + 1) % g_Config.NumberOfVoices;
    // then initiate the new note in that channel, deferred by 5ms.
  }
}


// A few received CC messages are intended for the Master Controller only;
// some others are filtered out, i.e. not passed through;
// the rest are passed through to the voice channels in a broadcast msg.
//
void  ProcessControlChange(uint8_t *midiMessage)
{
  uint8_t CCnumber = midiMessage[1];
  uint8_t dataByte = midiMessage[2];  // CC data value

  if (CCnumber == 100)  g_MidiRegisParam = dataByte; // "Registered Param" ID
  else if (CCnumber == 38)  // Parameter "Data Entry" message
  {
    if (g_MidiRegisParam == 0x00 && dataByte <= 12) 
    {
      g_Config.PitchBendRange = dataByte;
      StoreConfigData();
      MIDI_SendControlChange(BROADCAST, 100, 0);  // Reg. Param 0 = Pitch-Bend range
      MIDI_SendControlChange(BROADCAST, 38, g_Config.PitchBendRange);  // Data Entry
    }
    if (g_MidiRegisParam == 0x01)  // Master Tune param
    {
      g_Config.MasterTuneOffset = dataByte;
      // todo: Adjust each voice fine-tuning param according to g_Config.MasterTuneOffset
      StoreConfigData();
      // todo: Send individual fine-tuning param's to voice modules.
    }
  }
  else if (CCnumber == 80) { ; }  // Set Osc. mixer level - blocked (TX only)
  else if (CCnumber == 112) { ; }  // LFO phase sync. - blocked (TX only)
  // else...
  // Pass message through to all voice-channels...
  else  MIDI_SendControlChange(BROADCAST, CCnumber, dataByte);
}


uint8_t  MIDI_GetMessageLength(uint8_t statusByte)
{
  uint8_t  command = statusByte & 0xF0;
  uint8_t  length = 0;  // assume unsupported or unknown msg type

  if (command == PROGRAM_CHANGE_CMD || command == CHAN_PRESSURE_CMD)  length = 2;
  if (command == NOTE_ON_CMD || command == NOTE_OFF_CMD
  ||  command == CONTROL_CHANGE_CMD || command == PITCH_BEND_CMD)
  {
    length = 3;
  }
  return  length;
}

/*
 * Function:     Transmit MIDI Note-On/Velocity message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               noteNum = MIDI standard note number. (Note #60 = C4 = middle-C.)
 *               velocity = output level of ampld envelope shaper (DCA)
 */
void  MIDI_SendNoteOn(uint8_t chan, uint8_t noteNum, uint8_t velocity)
{
  uint8_t statusByte = 0x90 | ((chan - 1) & 0xF);

  Serial1.write(statusByte);
  Serial1.write(noteNum & 0x7F);
  Serial1.write(velocity & 0x7F);
}

/*
 * Function:     Transmit MIDI Note-Off message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               noteNum = MIDI note number.
 */
void  MIDI_SendNoteOff(uint8_t chan, uint8_t noteNum)
{
  uint8_t statusByte = 0x80 | ((chan - 1) & 0xF);

  Serial1.write(statusByte);
  Serial1.write(noteNum & 0x7F);
  Serial1.write((const uint8_t) 0);
}

/*
 * Function:     Transmit MIDI Pitch Bend message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               value = Pitch deviation value (14 bits).
 */
void  MIDI_SendPitchBend(uint8_t chan, uint16_t value)
{
  uint8_t statusByte = 0xE0 | ((chan - 1) & 0xF);

  Serial1.write(statusByte);
  Serial1.write(value & 0x7F);           // 7 LS bits
  Serial1.write((value >> 7) & 0x7F);    // 7 MS bits
}

/*
 * Function:     Transmit MIDI Control Change message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               ctrlNum = Control Number (0..119) -- not range checked.
 *               value = Controller data value (MSB or LSB).
 */
void  MIDI_SendControlChange(uint8_t chan, uint8_t ctrlNum, uint8_t value)
{
  uint8_t statusByte = 0xB0 | ((chan - 1) & 0xF);

  Serial1.write(statusByte);
  Serial1.write(ctrlNum & 0x7F);
  Serial1.write(value & 0x7F);
}

/*
 * Function:     Transmit MIDI Program Change message.
 *
 * Entry args:   chan = MIDI channel number (1..16)
 *               progNum = Program (instrument/voice) number. Depends on MIDI device.
 */
void  MIDI_SendProgramChange(uint8_t chan, uint8_t progNum)
{
  uint8_t statusByte = 0xC0 | ((chan - 1) & 0xF);

  Serial1.write(statusByte);
  Serial1.write(progNum & 0x7F);
}


/*`````````````````````````````````````````````````````````````````````````````````````````````````
 *   Set "factory default" values for configuration param's.
 *   Some param's may be changed later by MIDI CC messages or the control panel.
 */
void  DefaultConfigData(void)
{
  uint8_t voice, favLocn;

  g_Config.MidiChannel = 0;          // 0: Omni-ON-Poly mode
  g_Config.PitchBendEnable = 0;      // 0: Disabled
  g_Config.PitchBendRange = 2;       // semitones (max. 12)
  g_Config.ReverbMix_pc = 15;        // 0..100 % (typ. 15)
  g_Config.PresetLastSelected = 1;
  g_Config.MasterTuneOffset = 0;
  g_Config.DisplayBrightness = 30;   // %
  g_Config.EEpromCheckWord = 0xABCDE085;

  for (voice = 0; voice < 12; voice++)  // max. 12 voices
    { g_Config.VoiceTuning[voice] = 64; }  // Reset (64 = zero offset)
}

void  StoreConfigData()
{
  uint16_t  promAddr = 0;
  uint8_t  *pData = (uint8_t *) &g_Config;
  short  bytesToCopy = (short) sizeof(g_Config);
  int    errorCode;

  while (bytesToCopy > 0)
  {
    errorCode = EEpromWriteData(pData, promAddr, (bytesToCopy >= 32) ? 32 : bytesToCopy);
    if (errorCode != 0)  break;
    promAddr += 32;  pData += 32;  bytesToCopy -= 32;
  }
}

uint8_t  FetchConfigData()
{
  uint16_t  promAddr = 0;
  short  bytesToCopy = (short) sizeof(g_Config);
  uint8_t  *pData = (uint8_t *) &g_Config;
  uint8_t  count = 0;

  while (bytesToCopy > 0)
  {
    count += EEpromReadData(pData, promAddr, (bytesToCopy >= 32) ? 32 : bytesToCopy);
    if (count == 0)  break;
    promAddr += 32;  pData += 32;  bytesToCopy -= 32;
  }
  return  count;  // number of bytes read;  0 if an error occurred
}


void  StoreUserPreset(uint8_t favNum)  // Favorite number, favNum = 0..7
{
  uint16_t promAddr = 0x100 + favNum * 128;  // assume sizeof(g_Patch) <= 128
  uint8_t  *pData = (uint8_t *) &g_Patch;  // active patch addr
  short  bytesToCopy = (short) sizeof(g_Patch);

  if (g_EEpromFaulty || favNum > 7)  return;

  while (bytesToCopy > 0)
  {
    EEpromWriteData(pData, promAddr, (bytesToCopy >= 32) ? 32 : bytesToCopy);
    promAddr += 32;  pData += 32;  bytesToCopy -= 32;
  }
}

void  FetchUserPreset(uint8_t favNum)  // Favorite number, favNum = 0..7
{
  uint16_t promAddr = 0x100 + favNum * 128;  // assume sizeof(g_Patch) <= 128
  uint8_t  *pData = (uint8_t *) &g_Patch;  // active patch addr
  short  bytesToCopy = (short) sizeof(g_Patch);
  uint8_t  count = 0;

  if (g_EEpromFaulty || favNum > 7)  return;

  while (bytesToCopy > 0)
  {
    count += EEpromReadData(pData, promAddr, (bytesToCopy >= 32) ? 32 : bytesToCopy);
    if (count == 0)  break;  // error
    promAddr += 32;  pData += 32;  bytesToCopy -= 32;
  }
}


//=================  24LC64 IIC EEPROM Low-level driver functions  ======================
//                   ````````````````````````````````````````````
#define EEPROM_WRITE_INHIBIT()   {}    // Not used... WP tied to GND
#define EEPROM_WRITE_ENABLE()    {}

/*
 * Function:    EEpromACKresponse() -- Checks if EEPROM responds on the IIC bus
 *
 * Returns:     TRUE if the device responds with ACK to a control byte
 */
bool  EEpromACKresponse(void)
{
  Wire.beginTransmission(0x50);  // Send control byte
  return  (Wire.endTransmission() == 0);  // ACK rec'd
}

/**
 * Function:    EEpromWriteData() -- Writes up to 32 bytes on a 32-byte boundary
 *
 * Entry arg's: pData = pointer to source data (byte array)
 *              begAddr = EEPROM beginning address (0..8190)
 *              nbytes = number of bytes to write (max. 32 - see note)
 *
 * <!> Note:    24LC64 page buffer is 32 bytes.
 *
 * Returns:     Error code, 0xBE if I2C bus error detected;  0 if write OK
 */
int  EEpromWriteData(uint8_t *pData, uint16_t begAddr, uint8_t nbytes)
{
  short  npolls = 1000;  // time-out = 25ms @ 400kHz SCK
  int    errcode = 0;

  EEPROM_WRITE_ENABLE();   // Set WP Low

  if (EEpromACKresponse())
  {
    Wire.beginTransmission(0x50);  // Control byte
    Wire.write(begAddr >> 8);  // Addr Hi byte
    Wire.write(begAddr & 0xFF);  // Addr Lo byte
    Wire.write(pData, nbytes);
    errcode = Wire.endTransmission();  // Stop
    while (npolls--)  // ACK polling -- exit when ACK rec'd
      { if (EEpromACKresponse()) break; }
    if (npolls == 0)  errcode = 0xBE;
    else  EEpromACKresponse();  // (redundant ???)
  }

  EEPROM_WRITE_INHIBIT();  // Set WP High (or float)
  return errcode;
}

/**
 * Function:    EEpromReadData() -- Reads up to 32 bytes from the EEPROM.
 *
 * Entry arg's: pData = pointer to destination (byte array)
 *              begAddr = EEPROM beginning address (0..8190)
 *              nbytes = number of bytes to read (max. 32 - see note)
 *
 * <!> Note:    Arduino IIC 'Wire' library uses a 32-byte read/write buffer.
 *
 * Returns:     Number of bytes received from EEPROM;  0 if I2C bus error
 */
int  EEpromReadData(uint8_t *pData, uint16_t begAddr, uint8_t nbytes)
{
  int  bcount = 0;

  if (EEpromACKresponse())
  {
    Wire.beginTransmission(0x50);  // Control byte
    Wire.write(begAddr >> 8);  // Addr Hi byte
    Wire.write(begAddr & 0xFF);  // Addr Lo byte
    if (Wire.endTransmission() != 0)  return 0;  // an error occurred
    Wire.requestFrom(0x50, nbytes);
    while (bcount < nbytes)  { *pData++ = Wire.read();  bcount++; }
  }

  return  bcount;
}


//=================================================================================================
//===========   Command-Line User Inerface (CLI) -- USB-serial functions  =========================
//
#define CMD_LINE_MAX_LEN  80  // Max length of command line (chars)
#define CLI_ARG_MAX_LEN   24  // Max length of cmd arg string (chars)
#define SPACE      32
#define ASCII_CR   13
#define ASCII_BS    8
#define ASCII_CAN  24
#define ASCII_ESC  27

char  cmdLine[CMD_LINE_MAX_LEN + 2];   // Command Line buffer
char  cmdName[CLI_ARG_MAX_LEN + 1];    // Command name
char  argStr1[CLI_ARG_MAX_LEN + 1];    // Command argument #1 (if any)
char  argStr2[CLI_ARG_MAX_LEN + 1];    // Command argument #2 (if any)


bool  GetCommandLine(char *buffer, uint8_t maxlen)
{
	static uint8_t  index;  // index into buffer[] - saved across calls
	static uint8_t  count;  // number of chars buffered
	char  rxb;              // received char
	bool  status = FALSE;   // return value

	if (Serial.available()) 
	{
		rxb = Serial.read();

		if (rxb == ASCII_CR)   // CR code -- got complete command line
		{
			Serial.write("\r\n");  // Echo CR + LF
			buffer[index] = 0;  // add NUL terminator
			index = 0; 
			count = 0;
			status = TRUE;
		}
		else if (rxb >= SPACE && count < maxlen) // printable char
		{
			Serial.write(rxb);  // echo rxb back to user
			buffer[index] = rxb;  // append to buffer
			index++;
			count++;
		}
		else if (rxb == ASCII_BS && count != 0)  // Backspace
		{
			Serial.write(ASCII_BS);  // erase offending char
			Serial.write(SPACE);
			Serial.write(ASCII_BS);  // re-position cursor
			index--;  // remove last char in buffer
			count--;
		}
		else if (rxb == ASCII_CAN || rxb == ASCII_ESC)  // Cancel line
		{
			Serial.print(" ^X^ \n");
			Serial.print("> ");        // prompt
			index = 0;
			count = 0;
		}
	}
	return  status;
}


void  ServicePortRoutine()
{
  uint8_t  offset = 0;   // marker of next argument in cmdLine[]
	uint8_t  argCount;     // Number of cmd "arguments" incl. cmdName
	uint8_t  cmdLineLength = strlen(cmdLine);

  if (GetCommandLine(cmdLine, CMD_LINE_MAX_LEN))  // TRUE => have complete command 
  {
    cmdName[0] = 0;  // clear cmd string
    argStr1[0] = argStr2[0] = 0;  // clear arg's
    if (cmdLineLength != 0)
    {
      offset = ExtractArg(cmdLine, 0, cmdName);
      argCount = 1;  // assume we have arg[0] = cmd
      if (offset < cmdLineLength)  // get 1st arg, if any
      {
        offset = ExtractArg(cmdLine, offset, argStr1);  
        argCount++;
      }
      if (offset < cmdLineLength)  // get 2nd arg, if any
      {
        offset = ExtractArg(cmdLine, offset, argStr2);
        argCount++;
      }
      if (strMatch(cmdName, "help"))  HelpCommand();
      else if (strMatch(cmdName, "patch"))  PatchCommand();
      else if (strMatch(cmdName, "save"))  SaveCommand();
      else  Serial.println("! Undefined command !");
    }
    Serial.print("\n> ");  // prompt
  }
}


void  HelpCommand()
{
  Serial.println("Command usage:");
  Serial.println("``````````````");
  Serial.println("help     | Show this info. ");
  Serial.println("patch    | List active patch param's ");
  Serial.println("save  <fav#>  [name]   | Save active patch as Fav. Preset");
  Serial.println("... where <fav#> = Fav. Preset number (1..8) ");
  Serial.println("    and name (optional) = 20 chars max. (no spaces) ");
}


void  PatchCommand()
{
  ListActivePatch();
}


void  SaveCommand()
{
  uint8_t favID = (uint8_t) atoi((const char *)argStr1);

  if (favID == 0 || favID > 8)
  {
    Serial.println("! Command error:  Fav ID range is 1..8");
    return;
  }

  argStr2[20] = 0;  // max. 20 chars
  if (strlen(argStr2) != 0) strcpy(g_Patch.PresetName, (const char *)argStr2);
  StoreUserPreset(favID - 1);  // 0..7
  g_FavoriteSelected = favID;  // 1..8
  g_PatchModified = FALSE;  // redundant -- for clarity
  Disp_ClearScreen();
  GoToNextScreen(HOME_SCREEN_ID);  // Refresh Home screen
}


uint8_t  ExtractArg(char *source, uint8_t offset, char *dest)
{
	uint8_t  index = offset;  // index into input array, source[]
	uint8_t  outdex = 0;  // index into output array, dest[]
	uint8_t  count = 0;

	if (source[index] < SPACE)  return index;  // end-of message
	while (source[index] == SPACE)  { index++; }  // skip space(s)

	while (count < CLI_ARG_MAX_LEN)   // copy chars to dest[]
	{
		if (source[index] <= SPACE) break;  // control code or space
		dest[outdex++] = source[index++];   // copy 1 char
		dest[outdex] = 0;  // terminate string
		count++;
	}
	if (source[index] < SPACE)  return index;  // end-of message
	while (source[index] == SPACE)  { index++; }  // skip space(s)
	return  index;
}


bool  strMatch(char *str1, const char *str2)
{
  uint8_t  k;
  char   c1, c2;
  bool   result = TRUE;

  for (k = 0;  k < 255;  k++)
  {
    c1 = tolower( str1[k] );
    c2 = tolower( str2[k] );
    if (c1 != c2)  result = FALSE;
    if (c1 == 0 || c2 == 0)  break;  // found NUL -- exit
  }
  return result;
}


/**
 *  Utility to list the active patch parameter values via the 'console' serial port.
 *  Output text is in C source code format, suitable for importing into the array of
 *  preset patch definitions - g_PresetPatch[] - in the Poly-voice firmware, e.g:
 * 
 *       "Preset_patch_name",
 *       { 1, 2, 5, 7, 9, 11 },             // Osc Freq. Mult index (0..11)
 *       { 0, 0, 0, 0, 0, 0 },              // Osc Modulation source (0..9)
 *       { 0, 0, 0, 0, 0, 0 },              // Osc Detune, cents (+/-600)
 *       { 13, 0, 11, 10, 9, 8 },           // Osc Mixer level/step (0..15)
 *       5, 0, 200, 80, 200, 3,             // Ampld Env (A-H-D-S-R), Amp Mode
 *       5, 20, 500, 95,                    // Contour Env (S-D-R-H)
 *       500, 50,                           // ENV2: Decay/Rel, Sus %
 *       50, 500, 20, 20,                   // LFO Freq, Ramp, FM %st, AM %
 *       10, 0                              // Mixer Gain x10, Limit %FS
 * 
 *  The console port uses baud rate = 57600.  Set PC terminal app accordingly.
 */
void  ListActivePatch(void)
{
    char   numBuf[20];
    
    Serial.print("\n    ");
    Serial.print(QUOTE);
    Serial.print((char *) g_Patch.PresetName);
    Serial.print(QUOTE);
    Serial.print("\n");

    ListParamsFromArray((short *) &g_Patch.OscFreqMult[0], 6, 1);
    Serial.print("Osc Freq. Mult index (0..11)\n");
    
    ListParamsFromArray((short *) &g_Patch.OscAmpldModSource[0], 6, 1);
    Serial.print("Osc Modulation source (0..9)\n");
    
    ListParamsFromArray((short *) &g_Patch.OscDetune[0], 6, 1);
    Serial.print("Osc Detune, cents (+/-600)\n");
    
    ListParamsFromArray((short *) &g_Patch.MixerInputStep[0], 6, 1);
    Serial.print("Osc Mixer level/step (0..15)\n");
    
    ListParamsFromArray((short *) &g_Patch.EnvAttackTime, 6, 0);
    Serial.print("Ampld Env (A-H-D-S-R), Amp Mode \n");
    
    ListParamsFromArray((short *) &g_Patch.ContourStartLevel, 4, 0);
    Serial.print("Contour Env (S-D-R-H) \n");
    
    ListParamsFromArray((short *) &g_Patch.Env2DecayTime, 2, 0);
    Serial.print("ENV2: Decay/Rel, Sus % \n");
    
    ListParamsFromArray((short *) &g_Patch.LFO_Freq_x10, 4, 0);
    Serial.print("LFO: Hz x10, Ramp, FM %, AM %\n");
    
    ListParamsFromArray((short *) &g_Patch.MixerOutGain_x10, 2, 0);
    Serial.print("Mixer Gain x10, Limit %FS\n");
    Serial.print("\n");
}


void  ListParamsFromArray(short *sourceData, short paramCount, bool putBraces)
{
  char  numBuf[20], outBuff[100];
  uint8_t  padSize, n, b;
  
  if (putBraces) strcpy(outBuff, "    { ");  // indent 4 places, put brace
  else  strcpy(outBuff, "    ");  // indent 4 places, no brace
  
  for (n = 0;  n < paramCount;  n++)
  {
    sprintf(numBuf, "%d", (int) sourceData[n]);
    strcat(outBuff, numBuf);
    if (!putBraces || (n < 5)) strcat(outBuff, ", ");
  }
  if (putBraces) strcat(outBuff, " }, ");  

  padSize = 40 - strlen(outBuff);  // pad to column 41
  for (b = 0;  b < padSize;  b++)  { strcat(outBuff, " "); }
  strcat(outBuff, "// ");
  Serial.print((const char *)outBuff);
}


// ==========  'Factory Presets' -- Array of patch parameter tables in flash memory  =============
// <!> This table must be an exact copy of the presets defined in the Poly-voice MCU firmware.
//
// ... Values defined for g_Patch.OscFreqMult[] ............................
// |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  | 10  | 11  | <- index
// | 0.5 |  1  | 4/3 | 1.5 |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  | <- Freq.Mult
// `````````````````````````````````````````````````````````````````````````
//
// ... Values defined for g_Patch.OscAmpldModSource[] .........................
// |  0   |   1   |   2   |  3   |  4   |    5   |    6   |  7  |  8   |  9   | <- index
// | None | CONT+ | CONT- | ENV2 | MODN | EXPRN+ | EXPRN- | LFO | VEL+ | VEL- | <- AM source
// ````````````````````````````````````````````````````````````````````````````
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
