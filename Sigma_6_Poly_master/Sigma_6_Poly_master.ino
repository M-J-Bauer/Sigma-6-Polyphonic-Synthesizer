/*
 * File:       Sigma_6_Poly_master (.ino)
 *
 * Project:    Sigma-6 Poly-synth Master Controller application
 *
 * Platform:   RobotDyn SAMD21 M0-MINI dev board (MCU: ATSAMD21G18)
 *             <!> In Arduino IDE, select board type: "Adafruit ItsyBitsy M0 Express (SAMD21)"
 *
 * Author:     M.J.Bauer, 2025 -- www.mjbauer.biz
 *
 * Reference:  https://www.mjbauer.biz/Sigma6_Poly_synth_weblog.htm
 *
 * Licence:    Open Source (Unlicensed) -- free to copy, distribute, modify
 *
 * Notes:   1. The value defined by the symbol TOTAL_NUMBER_OF_VOICES (see #define on line 28)
 *             should match the synth hardware configuration, typically 6, 8, 10 or 12 voices.
 *             To exclude an unwanted/unavailable voice module, set its MIDI channel (DIP-switch
 *             setting) higher than than the TOTAL_NUMBER_OF_VOICES defined here.
 *
 *          2. The table of Preset patch definitions - g_PresetPatch[] - must be an exact
 *             copy of the respective table defined in the Poly-voice firmware code.
 */
#include <Wire.h>
#include <SPI.h>

#define FIRMWARE_VERSION  "1.20"

#define TOTAL_NUMBER_OF_VOICES   6  // Set according to hardware configuration

#define DEFAULT_MONO_VOICE_PRESETS  { 2, 18, 11, 13 }  // List up to 8 presets

#define MIDI_MSG_MAX_LENGTH  32
#define SYS_EXCLUSIVE_MSG  0xF0
#define SYSTEM_MSG_EOX     0xF7
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
#define EXTDIO_SS       8    // SPI slave-select: Ext. I/O port
#define TX_LED          27   // PA27 = on-board TX_LED (for 'heartbeat')

#define GPIOA_PIN_MODE_OUT(bit)  (PORT_IOBUS->Group[0].DIRSET.reg = (1 << bit))
#define GPIOA_PIN_SET_HIGH(bit)  (PORT_IOBUS->Group[0].OUTSET.reg = (1 << bit))
#define GPIOA_PIN_SET_LOW(bit)   (PORT_IOBUS->Group[0].OUTCLR.reg = (1 << bit))

#define QUOTE  (char)34  // ASCII double-quote char
#define OFF    0
#define ON     1
#ifndef FALSE
#define FALSE  0
#define TRUE  (!FALSE)
#endif

#define DO_NOTHING()   {;}


typedef struct table_of_configuration_params
{
  uint8_t  MidiChannel;             // MIDI IN channel (1..16, 0: Omni-On)
  uint8_t  NumberOfPolyVoices;      // Number of voices in Polyphonic group
  uint8_t  PitchBendEnable;         // Pitch Bend TX messages (0:Off, 1:On)
  uint8_t  PitchBendRange;          // Pitch Bend range, semitones (1..12)
  uint8_t  ReverbMix_pc;            // Reverb. wet/dry mix (0..100 %)
  uint8_t  PresetLastSelected;      // Preset last selected (0..127)
  uint8_t  MasterTuneOffset;        // Master tuning (cents + 64)
  uint8_t  DisplayBrightness;       // OLED contrast setting (5..100 %)
  uint8_t  VoiceTuning[16];         // Voice fine tuning (cents + 64)
  uint8_t  MonoVoicePreset[8];      // Preset numbers for Mono voices (8 max)
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

uint8_t  g_DefaultMonoPreset[] = DEFAULT_MONO_VOICE_PRESETS;
uint8_t  g_channelStatus[16];   // Note number (12..120) if GATE ON; 0xFF if GATE_OFF
uint8_t  g_FavoriteSelected;    // Favorite Preset selected (1..8, 0 => None)
uint8_t  g_NumberOfPresets;     // Number of Presets defined in Poly-voice firmware
uint8_t  g_MidiRegisParam;      // Registered Param # (0: PB range, 1: Master Tune)
bool     g_PatchModified;       // True if any active patch param is modified
bool     g_DisplayEnabled;      // True if OLED display initialized OK
bool     g_MidiRxSignal;        // Signal MIDI message received (for GUI icon)
bool     g_EEpromFaulty;        // True if EEPROM IIC bus error (hard fault)
bool     g_EEpromDefault;       // True if EEPROM data defaulted on startup
bool     g_MonophonicTestMode;  // True in monophonic test mode (voice tuning)
uint8_t  g_LastChannel;         // Last channel/voice allocated a note
uint8_t  g_VoiceUnderTest;      // Voice-channel # in monophonic test mode
uint8_t  g_NotePending;         // Note number of deferred Note-On (0 = none)
uint8_t  g_ChannelPending;      // Voice-channel to use for Note Pending
uint8_t  g_VelocityPending;     // Key velocity of deferred Note-On
uint32_t g_NoteOnDelayBegin;    // Captured time (ms)

//---------------------------------------------------------------------------------------

void  setup()
{
  short  n;

  Serial.begin(57600);         // initialize USB port for serial CLI
  Serial1.begin(31250);        // initialize UART for MIDI IN/OUT
  Wire.begin();                // initialize IIC as master
  Wire.setClock(400*1000);     // set IIC clock to 400kHz
  SPI.begin();                 // initialize SPI port
  analogReadResolution(10);    // set ADC resolution to 10 bits
  GPIOA_PIN_MODE_OUT(TX_LED);  // Heartbeat LED (PA27)
  ButtonLEDstate(88, ON);      // Turn ON all FAV Preset LED indicators

  if (EEpromACKresponse() == FALSE)
    { g_EEpromFaulty = TRUE; }  // IIC bus error or EEprom not fitted

  if (FetchConfigData() == 0 || g_Config.EEpromCheckWord != 0xABCDE120) 
  {
    DefaultConfigData();
    for (n = 0;  n < 8;  n++)  // Default the 8 Fav. Presets...
    {
      g_Config.UserPresetBase[n] = n + 1;  // 1..8
      memcpy(&g_Patch, &g_PresetPatch[n+1], sizeof(PatchParamTable_t));
      strcpy(g_Patch.PresetName, "Favorite_Preset");
      StoreUserPreset(n);
    }
    StoreConfigData();
    g_EEpromDefault = TRUE;
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

  for (n = 0; n < TOTAL_NUMBER_OF_VOICES; n++)  g_channelStatus[n] = GATE_OFF;

  g_NumberOfPresets = GetNumberOfPresets();
  PresetSelect(g_Config.PresetLastSelected);
  ButtonLEDstate(88, OFF);  // Turn off all FAV Preset LED indicators

  // NB: Function "InitializeVoiceModules()" is called from "UserState_StartupScreen()"
  // defined in source file: "poly_synth_panel.ino", about 5 seconds after MCU reset.
}


// Main background process loop...
//
void  loop()
{
  static uint32_t startPeriod_5ms, startPeriod_50ms, count_to_10;
  static uint32_t lastMillisec;

  MidiInputService();
  ServicePortRoutine();

  if (millis() != lastMillisec)  // 1ms interval ended
  {
    lastMillisec = millis();
    PotService();
  }

  if ((millis() - startPeriod_5ms) >= 5)  // 5ms period ended
  {
    startPeriod_5ms = millis();
    // No task to execute here yet, but keep this place-holder
  }

  if ((millis() - startPeriod_50ms) >= 50)  // 50ms period ended
  {
    startPeriod_50ms = millis();
    ButtonScan();
    if (g_DisplayEnabled) UserInterfaceTask();
    ////
    if (count_to_10 == 0)  GPIOA_PIN_SET_LOW(TX_LED);   // Heartbeat LED on
    if (count_to_10 == 2)  GPIOA_PIN_SET_HIGH(TX_LED);  // Heartbeat LED off
    if (++count_to_10 == 10) count_to_10 = 0;  // Reset heartbeat LED period
  }
}


// Function called from UI screen 'Startup' in file "poly_synth_panel.ino",
// which waits 5 seconds (min.) after power-on/reset before calling this function
// to allow voice modules time to start up.  
//
void  InitializeVoiceModules()
{
  uint8_t  voice;  // index value: 0..n (MIDI channel = voice + 1)
  uint8_t  monoVoice1 = g_Config.NumberOfPolyVoices;
  uint8_t  numberOfMonoVoices = TOTAL_NUMBER_OF_VOICES - monoVoice1;

  for (voice = 0;  voice < TOTAL_NUMBER_OF_VOICES;  voice++)
  {
    if (voice < monoVoice1)  // voice is in Poly group
    {
      MIDI_SendControlChange(voice+1, 113, 0);  // Set 'Poly Voice' mode
    }
    else  // voice is Mono
    {
      MIDI_SendControlChange(voice+1, 113, 100);  // Set 'Mono Voice' mode
      MIDI_SendProgramChange(voice+1, g_Config.MonoVoicePreset[voice]);
    } 
  }
  
  MIDI_SendControlChange(BROADCAST, 86, 2);   // Ampld Control: always ENV1*VELO
  MIDI_SendControlChange(BROADCAST, 89, g_Config.ReverbMix_pc);
  MIDI_SendControlChange(BROADCAST, 88, g_Config.PitchBendEnable);
  // If pitch bend enabled, send MIDI msg to disable vibrato, and vice-versa...
  if (g_Config.PitchBendEnable) MIDI_SendControlChange(BROADCAST, 87, 0);
  else  MIDI_SendControlChange(BROADCAST, 87, 3);   // Vibrato auto-ramp
  // The following 2 messages must be sent in sequence...
  MIDI_SendControlChange(BROADCAST, 100, 0);  // Reg. Param 0 = Pitch-Bend range
  MIDI_SendControlChange(BROADCAST, 38, g_Config.PitchBendRange);  // Data Entry
  // Send fine tuning param's to voice modules
  ExecuteVoiceTuning();
  // Program Change *BROADCAST* message is ignored by Mono-voice modules
  MIDI_SendProgramChange(BROADCAST, g_Config.PresetLastSelected);
}


void  ExecuteVoiceTuning()
{
  short  voice, tuningValue;

  for (voice = 0;  voice < TOTAL_NUMBER_OF_VOICES;  voice++)
  {
    tuningValue = (short) g_Config.VoiceTuning[voice] - 64;  // signed (0 +/- 60 cents)
    tuningValue += (short) g_Config.MasterTuneOffset - 64;
    if (tuningValue < -60)  tuningValue = 0 - 60;  // min.
    if (tuningValue > 60)  tuningValue = 60;  // max.
    MIDI_SendControlChange(voice+1, 100, 1);  // Reg. Param 1 = Fine Tuning
    MIDI_SendControlChange(voice+1, 38, (uint8_t)(tuningValue + 64));
  }
}


/**
 * Function:   Send MIDI Program Change message to all Poly voices via broadcast.
 *             (Mono voices reject broadcast messages.)
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

  // Send all potentially modified patch param's to Poly-voices
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
 * addressed to the Poly/master is received, it is processed.
 *
 * Messages NOT addressed to the Poly/master are passed through to the MIDI OUT port
 * (VM BUS) for any independent (mono) voice modules operating in 'Multi-timbral' mode.
 */
void  MidiInputService()
{
  static  uint8_t  midiMessage[MIDI_MSG_MAX_LENGTH];
  static  short  msgBytesExpected;
  static  short  msgByteCount;
  static  short  msgIndex;
  static  uint8_t  msgStatus;  // last command/status byte rx'd
  static  uint8_t  msgChannel;  // 1..16 ! (0 = invalid)
  static  bool   runningStatus;  
  uint8_t  monoVoice1 = g_Config.NumberOfPolyVoices;
  uint8_t  msgByte;

  if (Serial1.available() > 0)  // unread byte(s) available in Rx buffer
  {
    msgByte = Serial1.read();

    if ((msgByte & 0x80) && msgByte < 0xF0)  // command/status byte
    {
      msgStatus = msgByte;
      msgChannel = (msgStatus & 0x0F) + 1;  // 1..16
      runningStatus = FALSE;  // expecting data byte(s)
    }
    
    if (msgChannel == g_Config.MidiChannel)  // Message addressed to Poly Master
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
        g_MidiRxSignal = TRUE;  // signal to GUI to flash MIDI Rx icon
        msgByteCount = 0;
        msgIndex = 0;
      }
    }
    else if (msgChannel > g_Config.NumberOfPolyVoices)  // Message addressed to Mono Voice
    {
      Serial1.write(msgByte);  // Pass thru to MIDI OUT port (VM Bus)
    }
  }

  // Check for deferred Note-On pending and delay-time expired:
  if (g_NotePending && (millis() - g_NoteOnDelayBegin) >= 5)  // 5ms up
  {
    MIDI_SendNoteOn(g_ChannelPending+1, g_NotePending, g_VelocityPending);
    g_channelStatus[g_ChannelPending] = g_NotePending;
    g_LastChannel = g_ChannelPending;
    g_NotePending = 0;  // signal done
  }
}


void  ProcessMidiMessage(uint8_t *midiMessage, short msgLength)
{
  uint8_t  statusByte = midiMessage[0] & 0xF0;
  uint8_t  noteNumber = midiMessage[1];
  uint8_t  velocity = midiMessage[2];
  uint8_t  program = midiMessage[1];
  uint16_t data14bits = ((uint16_t)midiMessage[1] << 8) + midiMessage[2];
  bool     executeNoteOff = FALSE;
  bool     executeNoteOn = FALSE;
  uint8_t  count = 0;
  uint8_t  voice = g_LastChannel;
  uint8_t  oldestNote;

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
      PresetSelect(program);  // Function affects Poly-voices only
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
    for (voice = 0;  voice < g_Config.NumberOfPolyVoices;  voice++)
    {
      if (g_channelStatus[voice] == noteNumber)
      {
        MIDI_SendNoteOff(voice+1, noteNumber);
        g_channelStatus[voice] = GATE_OFF;
      }
    }
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
    voice = g_LastChannel + 1;  // oldest channel last active
    for (count = 0; count < g_Config.NumberOfPolyVoices; count++, voice++)
    {
      if (voice >= g_Config.NumberOfPolyVoices) voice = 0;  // wrap
      if (g_channelStatus[voice] == GATE_OFF)
      {
        MIDI_SendNoteOn(voice+1, noteNumber, velocity);
        g_channelStatus[voice] = noteNumber;
        g_LastChannel = voice;
        break;  // activate one voice only
      }
    }
    // Following code implements "N-key rollover" algorithm:
    if (count == g_Config.NumberOfPolyVoices)  // all channels are busy (gated)...
    {
      voice = (g_LastChannel + 1) % g_Config.NumberOfPolyVoices;  // oldest used
      oldestNote = g_channelStatus[voice];
      MIDI_SendNoteOff(voice+1, oldestNote);  // Terminate the oldest note
      g_channelStatus[voice] = GATE_OFF;  // voice is now free to use
      g_ChannelPending = voice;
      g_NotePending = noteNumber;  // Initiate deferred Note-On
      g_VelocityPending = velocity;
      g_NoteOnDelayBegin = millis();
    }
  }
}


/*
 * A few CC message types intended for the Master Controller's exclusive use are 
 * intercepted and actioned. Other specific CC messages addressed to the basic channel
 * are forwarded to all voice modules in a broadcast; e.g. Modulation control (CC01).
 * All other CC messages including patch parameter changes are ignored because these
 * settings are not meant to be made by the host MIDI controller.
 */
void  ProcessControlChange(uint8_t *midiMessage)
{
  uint8_t CCnumber = midiMessage[1];
  uint8_t dataByte = midiMessage[2];  // CC data value

  if (CCnumber == 100)  // Data byte is a "Registered Parameter" ID number
  {
	  g_MidiRegisParam = dataByte; 
  }
  else if (CCnumber == 38)  // Parameter "Data Entry" message
  {
    if (g_MidiRegisParam == 0x00 && dataByte <= 12)  // Reg. Param 0 = Pitch-Bend range
    {
      g_Config.PitchBendRange = dataByte;
      StoreConfigData();
      MIDI_SendControlChange(BROADCAST, 100, 0);  
      MIDI_SendControlChange(BROADCAST, 38, g_Config.PitchBendRange);  // Data Entry
    }
    if (g_MidiRegisParam == 0x01)  // Reg. Param 1 = Master Tune
    {
      g_Config.MasterTuneOffset = dataByte;
      StoreConfigData();
      ExecuteVoiceTuning();
    }
  }
  else if (CCnumber == 64)  // Hold/Sustain switch
  {
    if (dataByte >= 64) MIDI_SendControlChange(BROADCAST, 64, 100);  // Hold ON
    else  MIDI_SendControlChange(BROADCAST, 64, 0);  // Hold Off
  }
  else if (CCnumber == 1 || CCnumber == 33 || CCnumber == 120 || CCnumber == 123)
  {
	// Pass Modulation and 'All Notes Off' messages thru to voice modules
	MIDI_SendControlChange(BROADCAST, CCnumber, dataByte);  
  }
}


uint8_t  MIDI_GetMessageLength(uint8_t statusByte)
{
  uint8_t  command = statusByte & 0xF0;
  uint8_t  length = 0;  // assume unsupported or unknown msg type

  if (command == PROGRAM_CHANGE_CMD || command == CHAN_PRESSURE_CMD)  
    length = 2;

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
 *
 *   To restore the User Presets to "factory" defaults, change the EEpromCheckWord value
 *   here and in the EEPROM check code in the setup function, then do a firmware update.
 */
void  DefaultConfigData(void)
{
  uint8_t  voice, favLocn;
  uint8_t  maxMonoPresets = sizeof(g_DefaultMonoPreset);

  g_Config.MidiChannel = 1;          // 0: Omni-ON-Poly mode
  g_Config.NumberOfPolyVoices = TOTAL_NUMBER_OF_VOICES;
  g_Config.PitchBendEnable = 0;      // 0: Disabled
  g_Config.PitchBendRange = 2;       // semitones (max. 12)
  g_Config.ReverbMix_pc = 15;        // 0..100 % (typ. 15)
  g_Config.PresetLastSelected = 8;
  g_Config.MasterTuneOffset = 64;    // 64 represents zero
  g_Config.DisplayBrightness = 30;   // %
  g_Config.EEpromCheckWord = 0xABCDE120;

  for (voice = 0; voice < 12; voice++)  // max. 12 voices
    g_Config.VoiceTuning[voice] = 64;   // 64 = zero offset
	
  for (voice = 0; voice < 8; voice++)
  {
	if (voice < maxMonoPresets)
	  g_Config.MonoVoicePreset[voice] = g_DefaultMonoPreset[voice];
    else  g_Config.MonoVoicePreset[voice] = 1;  // pad (unused)
  }
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
            Serial.print(" ^X^ \r\n");
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
      else if (strMatch(cmdName, "sysinfo"))  SysInfoCommand();  // Hidden cmd!
      else  Serial.println("! Undefined command !");
    }
    Serial.print("\r\n> ");  // prompt
  }
}


void  HelpCommand()
{
  Serial.println("Command usage:");
  Serial.println("``````````````");
  Serial.println("help     | Show available commands ");
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


void  SysInfoCommand()  // Hidden
{
  static uint16_t *SYSCTRL_XOSC32K = (uint16_t *)0x40000814;  // register address
  static uint32_t *GCLK_GENCTRL = (uint32_t *)0x40000C04;  // register address
  uint32_t  clockSource = (*GCLK_GENCTRL >> 8) & 0x1F;  // bits 12:8

/***  Deprecated (retired) code ... no longer useful !
  Serial.print("SYSCTRL->XOSC32K reg: ");
  Serial.print((uint32_t)*SYSCTRL_XOSC32K, HEX);
  if (*SYSCTRL_XOSC32K & 0x02)  Serial.print("  [bit2 = 1 => XOSC32K Enabled]");
  else  Serial.print("  bit2 = 0 -> XOSC32K Disabled");
  Serial.print("\r\n");
  Serial.print("GCLK->GENCTRL reg: ");
  Serial.print((uint32_t)*GCLK_GENCTRL, HEX);
  if (clockSource == 5)  Serial.print("  [Clock source = 5 => XOSC32K]");
  if (clockSource == 6)  Serial.print("  [Clock source = 6 => OSC8M]");
  Serial.print("\r\n");
***/
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

    Serial.print("\r\n    ");
    Serial.print(QUOTE);
    Serial.print((char *) g_Patch.PresetName);
    Serial.print(QUOTE);
    Serial.print("\r\n");

    ListParamsFromArray((short *) &g_Patch.OscFreqMult[0], 6, 1);
    Serial.print("Osc Freq. Mult index (0..11)\r\n");

    ListParamsFromArray((short *) &g_Patch.OscAmpldModSource[0], 6, 1);
    Serial.print("Osc Modulation source (0..9)\r\n");

    ListParamsFromArray((short *) &g_Patch.OscDetune[0], 6, 1);
    Serial.print("Osc Detune, cents (+/-600)\r\n");

    ListParamsFromArray((short *) &g_Patch.MixerInputStep[0], 6, 1);
    Serial.print("Osc Mixer level/step (0..15)\r\n");

    ListParamsFromArray((short *) &g_Patch.EnvAttackTime, 6, 0);
    Serial.print("Ampld Env (A-H-D-S-R), Amp Mode \r\n");

    ListParamsFromArray((short *) &g_Patch.ContourStartLevel, 4, 0);
    Serial.print("Contour Env (S-D-R-H) \r\n");

    ListParamsFromArray((short *) &g_Patch.Env2DecayTime, 2, 0);
    Serial.print("ENV2: Decay/Rel, Sus % \r\n");

    ListParamsFromArray((short *) &g_Patch.LFO_Freq_x10, 4, 0);
    Serial.print("LFO: Hz x10, Ramp, FM %, AM %\r\n");

    ListParamsFromArray((short *) &g_Patch.MixerOutGain_x10, 2, 0);
    Serial.print("Mixer Gain x10, Limit %FS\r\n");
    Serial.print("\r\n");
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
    "Deep Saxophoney",              // 31  (revised)
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
