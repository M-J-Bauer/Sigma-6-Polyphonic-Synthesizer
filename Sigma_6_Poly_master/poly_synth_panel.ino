/**
 * File:       poly_synth_panel (.ino)
 *
 * Module:     Front-panel user-interface for Sigma-6 Poly-synth Master Controller
 *             comprising IIC OLED display, 3 push-buttons and Data Entry Pot.
 *
 * Author:     M.J.Bauer, 2025 -- www.mjbauer.biz
 */
#include "oled_display_lib.h"

#define BUTT_POS_A      0    // X-pos of button A legend
#define BUTT_POS_B      44   // X-pos of button B legend
#define BUTT_POS_C      88   // X-pos of button C legend

// MCU I/O pin assignments.................
#define SW_DRIVE_COL0   10   // Button/switch matrix drive line 0
#define SW_DRIVE_COL1   11   // Button/switch matrix drive line 1
#define SW_DRIVE_COL2   12   // Button/switch matrix drive line 2
#define SW_DRIVE_COL3   13   // Button/switch matrix drive line 3
#define SW_SENSE_ROW0   5    // Button/switch matrix sense line 0
#define SW_SENSE_ROW1   6    // Button/switch matrix sense line 1
#define SW_SENSE_ROW2   7    // Button/switch matrix sense line 2
#define LFO_FREQ_POT    A8   // LFO (vibrato) frequency pot
#define LFO_DEPTH_POT   A9   // LFO (vibrato) depth pot
#define LED_REG_LE      2    // SPI slave-select: LED register 'LE'

extern ConfigParams_t  g_Config;  // structure holding config param's
extern PatchParamTable_t  g_Patch;   // Active patch param's

enum User_Interface_States  // aka 'Screen identifiers'
{
  STARTUP = 0,
  CONFIRM_DEFAULT = 1,
  HOME_SCREEN = 2,
  PRESET_SELECT,
  SETUP_MENU,
  PATCH_MENU,
  // setup ...
  SET_USER_PRESET,
  SET_PITCH_BEND,
  SET_REVERB_LEVEL,
  SET_MIDI_CHAN,
  SET_DISP_BRIGHT,
  SET_VOICE_TUNING,
  // patch ...
  SET_MIXER_LEVELS,
  SET_LFO_DEPTH,
  SET_LFO_FREQ,
  SET_LFO_RAMP,
  SET_ENV_ATTACK,
  SET_ENV_HOLD,
  SET_ENV_DECAY,
  SET_ENV_SUSTAIN,
  SET_ENV_RELEASE,
  SET_MIXER_GAIN,
  SET_LIMITER_LVL
};

/*
 * Bitmap image definition
 * Image name: sigma_6_icon_24x21, width: 24, height: 21 pixels
 */
bitmap_t sigma_6_icon_24x21[] = {
  0x00, 0x00, 0x7C, 0x00, 0x01, 0xFC, 0x00, 0x03, 0xFC, 0x00, 0x03, 0xC0, 0x00, 0x07, 0x80, 0x00,
  0x07, 0x00, 0x00, 0x07, 0x00, 0x07, 0xF7, 0xF0, 0x1F, 0xF7, 0xFC, 0x3F, 0xF7, 0xFE, 0x71, 0x87,
  0x8E, 0x71, 0xC7, 0x0F, 0xE0, 0xE7, 0x07, 0xE0, 0xE7, 0x07, 0xE0, 0xE7, 0x07, 0xE0, 0xE7, 0x07,
  0xF1, 0xE7, 0x8F, 0x71, 0xC3, 0x8E, 0x7F, 0xC3, 0xFE, 0x3F, 0x81, 0xFC, 0x0E, 0x00, 0x70
};

/*
 * Bitmap image definition
 * Image name: config_icon_7x7,  width: 7, height: 7 pixels
 */
bitmap_t config_icon_9x9[] = {
  0x14, 0x00, 0x5D, 0x00, 0x22, 0x00, 0xC1, 0x80, 0x41, 0x00, 0xC1, 0x80, 0x22, 0x00,
  0x5D, 0x00, 0x14, 0x00
};

/*
 * Bitmap image definition
 * Image name: patch_icon_7x7,  width: 7, height: 7 pixels
 */
bitmap_t patch_icon_9x9[] = {
  0x49, 0x00, 0xFF, 0x80, 0x49, 0x00, 0x49, 0x00, 0xFF, 0x80, 0x49, 0x00, 0x49, 0x00,
  0xFF, 0x80, 0x49, 0x00
};

/*
 * Bitmap image definition
 * Image name: midi_conn_icon_9x9,  width: 9, height: 9 pixels
 */
bitmap_t midi_conn_icon_9x9[] = {
  0x3E, 0x00, 0x77, 0x00, 0xDD, 0x80, 0xFF, 0x80, 0xBE, 0x80,
  0xFF, 0x80, 0xFF, 0x80, 0x7F, 0x00, 0x36, 0x00
};

/*
 * Bitmap image definition
 * Image name: gate_signal_icon_8x7,  width: 8, height: 7 pixels
 */
bitmap_t gate_signal_icon_8x7[] = {
  0xC0, 0xF0, 0xFC, 0xFF, 0xFC, 0xF0, 0xC0
};

/*
 * Bitmap image definition
 * Image name: Adafruit_logo_11x12,  width: 11, height: 12 pixels
 */
bitmap_t Adafruit_logo_11x12[] = {
  0x02, 0x00, 0x06, 0x00, 0xEE, 0x00, 0x7E, 0x00, 0x73, 0x00, 0x21, 0xC0,
  0x21, 0xE0, 0x73, 0x80, 0x7E, 0x00, 0xEE, 0x00, 0x06, 0x00, 0x02, 0x00
};

/*
 * Bitmap image definition
 * Image name: RobotDyn_logo_11x11,  width: 11, height: 11 pixels
 */
bitmap_t RobotDyn_logo_11x11[] = {
  0x7F, 0xC0, 0xCE, 0x60, 0xCE, 0x60, 0xFF, 0xE0, 0x7F, 0xC0, 0x04, 0x00,
  0x04, 0x40, 0x3F, 0x80, 0x4E, 0x00, 0x11, 0x00, 0x19, 0x80
};

const uint8_t  percentLogScale[] =       // 16 values, 3dB log scale (approx.)
        { 0, 1, 2, 3, 4, 5, 8, 10, 12, 16, 25, 35, 50, 70, 100, 100  };

const uint16_t  timeValueQuantized[] =     // 16 values, logarithmic scale
        { 0, 10, 20, 30, 50, 70, 100, 200, 300, 500, 700, 1000, 1500, 2000, 3000, 5000 };
		
const uint8_t  oscFreqMultConst[] =     // Dummy values at idx = 0, 2, 3 (non-integer)
        { 0, 1, 0, 0, 2, 3, 4, 5, 6, 7, 8, 9, 0, 0, 0, 0 };
		

//===================   P U S H - B U T T O N   F U N C T I O N S  ======================
//
const char  ButtonCodeLUT[] =
    { '1','2','3','4', '5','6','7','8', 'A','B','C','D' };

bool  ButtonHitDetected;
char  ButtonCodeLastHit;

// This function is called at periodic intervals of 30 ~ 50 milliseconds.
//
void  ButtonScan()
{
  static bool  PrepDone;
  static bool  AnyButtonPressedLastScan;
  uint8_t  row, col;
  uint8_t  buttonIdx;
  bool  button_pressed = FALSE;

  if (!PrepDone)  // One-time initialization at start-up
  {
    pinMode(SW_SENSE_ROW0, INPUT_PULLUP);  // D5 = buttons 1,2,3,4
    pinMode(SW_SENSE_ROW1, INPUT_PULLUP);  // D6 = buttons 5,6,7,8
    pinMode(SW_SENSE_ROW2, INPUT_PULLUP);  // D7 = buttons A,B,C,D
    PrepDone = TRUE;
  }

  for (col = 0; col < 4; col++)  // Activate (i.e. set LOW) one column at a time
  {
    if (col == 0) { pinMode(SW_DRIVE_COL0, OUTPUT); digitalWrite(SW_DRIVE_COL0, LOW); }
    if (col == 1) { pinMode(SW_DRIVE_COL1, OUTPUT); digitalWrite(SW_DRIVE_COL1, LOW); }
    if (col == 2) { pinMode(SW_DRIVE_COL2, OUTPUT); digitalWrite(SW_DRIVE_COL2, LOW); }
    if (col == 3) { pinMode(SW_DRIVE_COL3, OUTPUT); digitalWrite(SW_DRIVE_COL3, LOW); }

    delayMicroseconds(2);   // delay for output to settle

    for (row = 0; row < 3; row++)  // Test each row for button press
    {
      if (row == 0 && digitalRead(SW_SENSE_ROW0) == 0) break;
      if (row == 1 && digitalRead(SW_SENSE_ROW1) == 0) break;
      if (row == 2 && digitalRead(SW_SENSE_ROW2) == 0) break;
    }
    if (row < 3)  // a button is pressed...
    {
      button_pressed = TRUE;
      buttonIdx = (row << 2) + col;  // ASCII table index
    }

    if (col == 0) pinMode(SW_DRIVE_COL0, INPUT_PULLUP);  // Restore col Hi-Z with pullup
    if (col == 1) pinMode(SW_DRIVE_COL1, INPUT_PULLUP);
    if (col == 2) pinMode(SW_DRIVE_COL2, INPUT_PULLUP);
    if (col == 3) pinMode(SW_DRIVE_COL3, INPUT_PULLUP);
  }

  if (button_pressed && !AnyButtonPressedLastScan)  // button hit detected
  {
    ButtonHitDetected = TRUE;
    ButtonCodeLastHit = ButtonCodeLUT[buttonIdx];
  }
  AnyButtonPressedLastScan = button_pressed;
}

/*
 * Function:   Test for a button hit, i.e. transition from not pressed to pressed.
 * Entry arg:  buttonCode = '1','2','3','4', '5','6','7','8', 'A','B','C','D'
 * Return val: TRUE if the given button was hit since the last call, else FALSE.
 */
bool  ButtonHit(char buttonCode)
{
  bool result = FALSE;

  if (ButtonHitDetected && ButtonCodeLastHit == buttonCode)
  {
    result = TRUE;
    ButtonHitDetected = FALSE;
  }
  return result;
}

/*
 * Function:   Set favorite Button LED state (arg2 = ON or OFF)
 * Arg1:  buttonID = 0..7  (0 => button #1 (LHS); 7 => button #8)
 * Note:  All other button LEDs will be turned off.
 *        Call with buttonID > 7 to set all LEDs to same state.
 */
void  ButtonLEDstate(char buttonID, bool state)
{
  static bool  PrepDone;
  uint8_t  LED_states;  // 1 bit per LED, 1 = ON, 0 = OFF
  uint8_t  bitMask = 1 << (7 - buttonID);

  if (!PrepDone)  // One-time initialization at start-up
  {
    pinMode(LED_REG_LE, OUTPUT);
    digitalWrite(LED_REG_LE, HIGH);
    PrepDone = TRUE;
  }

  if (state == OFF)  LED_states = 0x00;  // Clear all bits
  else  LED_states = bitMask;  // Set 1 bit
  if (buttonID > 7)  LED_states = (state == OFF) ? 0x00 : 0xFF;  // All bits = state

  SPI.beginTransaction(SPISettings(500000, LSBFIRST, SPI_MODE2));
  digitalWrite(LED_REG_LE, LOW);
  SPI.transfer(LED_states ^ 0xFF);  // Active LOW
  digitalWrite(LED_REG_LE, HIGH);
  SPI.endTransaction();
}


//=================  D A T A   E N T R Y   P O T   F U N C T I O N S  ===================
//
static long   potReadingAve;  //  Data Entry pot reading smoothed [24:8 fixed-pt]
static short  sliderPotReading[6];  // Raw ADC readings (10 bits)
static short  LFO_FreqPotReading, LFO_DepthPotReading;
//
/*
 * Overview:  Service Routine for front-panel data-entry pot and other control pots.
 *            Non-blocking "task" called at 5ms intervals from main loop.
 *
 * Detail:    The routine reads the Data Entry pot input and keeps a rolling average of 
 *            the ADC readings in fixed-point format (24:8 bits);  range 0.0 ~ 1023.99.
 *            The current pot position can be read by a call to function DataPotPosition().
 *            Slider pots and LFO control pot readings are not smoothed.
 */
void  PotService()
{
  static const uint32_t  pinid[] = { A0, A1, A2, A3, A4, A5 };
  static uint8_t  call;  // call sequence = 0, 1, 2 ... 7, 0, 1, ...
  short  dummyRead;
  long   potReading;

  dummyRead = analogRead(A11);  // 1st reading invalid
  potReading = analogRead(A11);  // valid reading (10 bits)
  // Apply 1st-order IIR filter (K = 0.25)
  potReading = potReading << 8;  // convert to fixed-point (24:8 bits)
  potReadingAve -= (potReadingAve >> 2);
  potReadingAve += (potReading >> 2);

  if (call < 6)  // on calls 0..5
  {
    dummyRead = analogRead(pinid[call]);  // 1st reading invalid
    sliderPotReading[call] = analogRead(pinid[call]);
  }
  else if (call == 6)
  {
    dummyRead = analogRead(A8);  // 1st reading invalid
    LFO_FreqPotReading = analogRead(A8);
  }
  else if (call == 7)
  {
    dummyRead = analogRead(A9);  // 1st reading invalid
    LFO_DepthPotReading = analogRead(A9);
  }
  
  if (++call >= 8)  call = 0;  // repeat ADC read sequence
}

/*
 * Overview:  Returns TRUE if the pot position has changed by more than 2% since a
 *            previous call to the function which also returned TRUE. (20/1024 -> 2%)
 *
 * Returns:   (bool) status flag, TRUE or FALSE
 */
bool  DataPotMoved()
{
  static long lastReading;
  bool result = FALSE;

  if (labs(potReadingAve - lastReading) > (20 << 8))
  {
    result = TRUE;
    lastReading = potReadingAve;
  }
  return result;
}

/*
 * Function:     DataPotPosition()
 *
 * Overview:     Returns the current position of the data-entry pot, averaged over
 *               several ADC readings, as an 8-bit unsigned integer.
 *               Full-scale ADC reading is 1023.  Divide by 4 to get 255.
 *
 * Return val:   (uint8_t) Pot position, range 0..255.
 */
uint8_t  DataPotPosition()
{
  return (uint8_t)(potReadingAve >> 10);    // (Integer part) / 4
}


/*
 * Overview:  Returns TRUE if the pot position has changed by more than 2% since a
 *            previous call to the function which also returned TRUE. (20/1024 := 2%)
 *
 * Entry arg: potid = pot ID number, one of 6, range 0..5
 * Returns:   (bool) status flag, TRUE or FALSE
 */
bool  SliderPotMoved(uint8_t potid)
{
  static short  lastReading[6];
  bool result = FALSE;

  if (abs(sliderPotReading[potid] - lastReading[potid]) > 20)  // approx. 2% change
  {
    result = TRUE;
    lastReading[potid] = sliderPotReading[potid];
  }
  return result;
}

/*
 * Overview:  Returns current position of a specified slider pot.
 *
 * Entry arg: potid = pot ID number, one of 6, range 0..5
 * Returns:   (uint8_t) Slider pot position, range 0..255
 */
uint8_t  SliderPosition(uint8_t potid)
{
  return  (uint8_t)(sliderPotReading[potid] >> 2);  // 0..255
}

// ***** todo:  Add functions to support LFO control pots  *****


//=============  S C R E E N   N A V I G A T I O N   F U N C T I O N S  =================
//
static bool screenSwitchReq;      // flag: Request to switch to next screen
static bool isNewScreen;          // flag: Screen switch has occurred
static uint8_t currentScreen;     // ID number of current screen displayed
static uint8_t previousScreen;    // ID number of previous screen displayed
static uint8_t nextScreen;        // ID number of next screen to be displayed
//
/*
 * Display title bar text left-justified top of screen in 12pt font with underline.
 * The title bar area is assumed to be already erased.
 * Maximum length of titleString is 16 ~ 17 chars.
 */
void  DisplayTitleBar(const char *titleString)
{
  Disp_Mode(SET_PIXELS);
  Disp_PosXY(4, 0);
  Disp_SetFont(PROP_12_NORM);
  Disp_PutText(titleString);
  Disp_PosXY(0, 12);
  Disp_DrawLineHoriz(128);
}

/*
 * Display a text string (6 chars max) centred in a field of fixed width (41px)
 * using 8pt mono-spaced font, at the specified horizontal position (arg = x-coord).
 * This version assumes 3 UI buttons (A, B, C) located under the screen.
 * Vertical position (y-coord) is fixed at 53 (bottom of screen).
 */
void  DisplayButtonLegend(uint8_t x, const char *str)
{
  int len = strlen(str);
  int i, wpix;

  if (len > 6) len = 6;
  wpix = len * 6;  // text width, pixels (max. 36)

  Disp_Mode(SET_PIXELS);  // Draw the button (bar)
  Disp_PosXY(x, 53);  // Y-coord = 53 (fixed)
  Disp_DrawBar(41, 9);
  Disp_Mode(CLEAR_PIXELS);  // Write the string
  Disp_SetFont(MONO_8_NORM);
  x = x + (42 - wpix) / 2;
  Disp_PosXY(x, 54);
  Disp_PutText(str);
  Disp_Mode(SET_PIXELS);
}

/*
 * Function returns the screen ID number of the currently displayed screen.
 */
uint8_t GetCurrentScreenID()
{
  return currentScreen;
}

/*
 * Function GoToNextScreen() triggers a screen switch to a specified screen ID.
 * The real screen switch business is done by the UserInterfaceTask() function
 * when next executed following the call to GoToNextScreen().
 *
 * Entry arg:  screenID = ID number of next screen required.
 */
void  GoToNextScreen(uint8_t screenID)
{
  nextScreen = screenID;
  screenSwitchReq = TRUE;
}

/*
 * Called at periodic intervals of 50 milliseconds from the main loop, the
 * User Interface task allows the user to view and adjust various operational
 * parameters using the Data Entry Pot, push-buttons and OLED display.
 */
void  UserInterfaceTask(void)
{
  if (screenSwitchReq)  // Screen switch requested
  {
    screenSwitchReq = FALSE;
    isNewScreen = TRUE;   // Signal to render a new screen
    if (nextScreen != currentScreen)  Disp_ClearScreen();
    previousScreen = currentScreen;  // make the switch
    currentScreen = nextScreen;      // next screen becomes current
    DataPotMoved();  // clear 'pot moved' flag...
  }

  switch (nextScreen)
  {
    case STARTUP:            UserState_StartupScreen();      break;
    case CONFIRM_DEFAULT:    UserState_ConfirmDefault();     break;
    case HOME_SCREEN:        UserState_HomeScreen();         break;
    case PRESET_SELECT:      UserState_PresetSelect();       break;
    case SETUP_MENU:         UserState_SetupMenu();          break;
    case PATCH_MENU:         UserState_PatchMenu();          break;
    // setup
    case SET_USER_PRESET:    UserState_SetUserPreset();      break;
    case SET_PITCH_BEND:     UserState_SetPitchBend();       break;
    case SET_REVERB_LEVEL:   UserState_SetReverbLevel();     break;
    case SET_MIDI_CHAN:      UserState_SetMidiChannel();     break;
    case SET_DISP_BRIGHT:    UserState_SetDisplayBright();   break;
    case SET_VOICE_TUNING:   UserState_SetVoiceTuning();     break;
    // patch
    case SET_MIXER_LEVELS:   UserState_SetMixerLevels();     break;
    case SET_LFO_DEPTH:      UserState_Set_LFO_Depth();      break;
    case SET_LFO_FREQ:       UserState_Set_LFO_Freq();       break;
    case SET_LFO_RAMP:       UserState_Set_LFO_RampTime();   break;
    case SET_ENV_ATTACK:     UserState_Set_ENV_Attack();     break;
    case SET_ENV_HOLD:       UserState_Set_ENV_Hold();       break;
    case SET_ENV_DECAY:      UserState_Set_ENV_Decay();      break;
    case SET_ENV_SUSTAIN:    UserState_Set_ENV_Sustain();    break;
    case SET_ENV_RELEASE:    UserState_Set_ENV_Release();    break;
    case SET_MIXER_GAIN:     UserState_Set_MixerGain();      break;
    case SET_LIMITER_LVL:    UserState_Set_LimiterLevel();   break;
  }
  
  isNewScreen = FALSE;  // New screen prep done
}


//===========  A P P L I C A T I O N - S P E C I F I C   F U N C T I O N S  =============

void  UserState_StartupScreen()
{
  static uint32_t  stateTimer;  // unit = 50ms
  static bool  doneEEpromCheck;
  static uint8_t  led;
  bool  exit = FALSE;

  if (isNewScreen)
  {
    led = stateTimer = 0;
    if (g_EEpromFaulty && !doneEEpromCheck)  // EEPROM check... One time only
    {
      DisplayTitleBar("    EEPROM Fault");
      Disp_SetFont(PROP_12_BOLD);
      Disp_PosXY(4, 0);
      Disp_PutText("<!>");
      Disp_SetFont(MONO_8_NORM);
      Disp_PosXY(8, 28);
      Disp_PutText("* Using default");
      Disp_PosXY(8, 40);
      Disp_PutText("  configuration.");
      return;
    }

    DisplayTitleBar("Start-up");
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 16);  // info line 1
    Disp_PutText("Firmware version: ");
    Disp_SetFont(MONO_8_NORM);
    Disp_PutText(FIRMWARE_VERSION);
    Disp_SetFont(PROP_8_NORM);

    Disp_PosXY(4, 28);  // info line 2
    Disp_PutText("MIDI IN chan: ");
    Disp_SetFont(MONO_8_NORM);
    if (g_MidiMode == OMNI_ON) Disp_PutText("Omni On");
    else  Disp_PutDecimal(g_Config.MidiChannel, 1);
    Disp_SetFont(PROP_8_NORM);

    Disp_PosXY(4, 40);  // info line 3
    Disp_PutText("Voices: ");
    Disp_SetFont(MONO_8_NORM);
    Disp_PutDecimal(NUMBER_OF_VOICES, 1);
    Disp_SetFont(PROP_8_NORM);
    Disp_PutText(",  Presets: ");
    Disp_SetFont(MONO_8_NORM);
    Disp_PutDecimal(g_NumberOfPresets, 1);

    DisplayButtonLegend(BUTT_POS_A, "Home");
    DisplayButtonLegend(BUTT_POS_B, "-");
    if (!g_EEpromFaulty) DisplayButtonLegend(BUTT_POS_C, "Config");
    else  DisplayButtonLegend(BUTT_POS_C, "-");
  } // end if (isNewScreen)

  if (ButtonHit('A'))  exit = TRUE;  // Home
  if (ButtonHit('B'))  exit = TRUE;  // Home
  if (ButtonHit('C') && !g_EEpromFaulty)  // goto 'Default Config' prompt screen...
    { ButtonLEDstate(88, OFF);  GoToNextScreen(CONFIRM_DEFAULT); }

  if (led >= 8)  led = 0;  // LED chaser...
  if (stateTimer >= (3 * led))  ButtonLEDstate(led++, ON);
 
  if (g_EEpromFaulty && !doneEEpromCheck)
  {
    if (++stateTimer > 40)   // 2 sec time-out
    {
      doneEEpromCheck = TRUE;
      Disp_ClearScreen();
      GoToNextScreen(STARTUP);  // Repeat startup without EEprom check
    }
  }
  else if (++stateTimer > 100) exit = TRUE;  // 5 sec time-out

  if (exit) { ButtonLEDstate(88, OFF);  GoToNextScreen(HOME_SCREEN); }
}


void  UserState_ConfirmDefault()
{
  static bool  affirmed;
  static uint32_t timeSinceAffirm_ms;

  if (isNewScreen)
  {
    DisplayTitleBar("Default Config");
    Disp_Mode(SET_PIXELS);
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(8, 16);
    Disp_PutText("Restore configuration");
    Disp_PosXY(8, 26);
    Disp_PutText("parameters to default");
    Disp_PosXY(8, 36);
    Disp_PutText("settings ?");
    DisplayButtonLegend(BUTT_POS_A, "Cancel");
    DisplayButtonLegend(BUTT_POS_B, "-");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    timeSinceAffirm_ms = 0;
    affirmed = FALSE;
  }

  if (ButtonHit('A')) GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B')) GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('C') && !affirmed)
  {
    DefaultConfigData();
    StoreConfigData();
    Disp_PosXY(0, 16);
    Disp_BlockClear(128, 30);  // Erase messages
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(48, 26);  // mid-screen
    Disp_PutText("Done!");
    DisplayButtonLegend(BUTT_POS_A, "");
    DisplayButtonLegend(BUTT_POS_B, "");
    DisplayButtonLegend(BUTT_POS_C, "");
    affirmed = TRUE;
    timeSinceAffirm_ms = 0;
  }

  if (affirmed)  // waiting 1.5 sec to view message
  {
    if (timeSinceAffirm_ms >= 1500)  GoToNextScreen(HOME_SCREEN);
    timeSinceAffirm_ms += 50;
  }
}


void  UserState_HomeScreen()
{
  static uint8_t  lastPresetShown, lastFavShown;
  static short  midiNoActivityTimer;  // unit = 50ms
  static bool  midiIconVisible;
  static bool  patchIconVisible;
  uint8_t  osc, fav;

  if (isNewScreen)
  {
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(0, 0);
    Disp_BlockFill(28, 25);
    Disp_Mode(CLEAR_PIXELS);
    Disp_PosXY(2, 2);
    Disp_PutImage(sigma_6_icon_24x21, 24, 21);

    Disp_Mode(SET_PIXELS);
    Disp_SetFont(PROP_12_BOLD);
    Disp_PosXY(32, 2);
    Disp_PutText("Sigma");
    Disp_PosXY(Disp_GetX() + 2, 2);
    Disp_PutChar('6');

    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(32, 17);
    Disp_PutText("Polyphonic");
    Disp_PosXY(0, 28);
    Disp_DrawLineHoriz(128);
    Disp_PosXY(116, 1);  // Display Robodyn logo at upper RHS
    Disp_PutImage(RobotDyn_logo_11x11, 11, 11);

    DisplayButtonLegend(BUTT_POS_A, "PRESET");
    DisplayButtonLegend(BUTT_POS_B, "SETUP");
    DisplayButtonLegend(BUTT_POS_C, "PATCH");

    for (osc = 0;  osc < 6;  osc++)  
      SliderPotMoved(osc);  // clear 'pot moved' flags
    
    lastPresetShown = 255;  // force refresh
    lastFavShown = 255;
    midiIconVisible = FALSE;
    patchIconVisible = FALSE;
    g_MonophonicTestMode = FALSE;
  }

  if (ButtonHit('A')) GoToNextScreen(PRESET_SELECT);
  if (ButtonHit('B')) GoToNextScreen(SETUP_MENU);
  if (ButtonHit('C')) GoToNextScreen(PATCH_MENU);

  for (fav = 0;  fav < 8;  fav++)  // Monitor Favorite buttons
  {
    if (ButtonHit('1' + fav))
    {
      RecallUserPreset(fav);  // 0..7
      ButtonLEDstate(fav, ON);  // Turn on LED for fav preset
      lastFavShown = 255;  // force screen refresh
      break;
    }
  }

  for (osc = 0;  osc < 6;  osc++)  // Monitor Slider Pots
    if (SliderPotMoved(osc))  GoToNextScreen(SET_MIXER_LEVELS);

  // todo:  Monitor LFO control pots 

  // Refresh Preset displayed if selection changed...
  if (lastPresetShown != g_Config.PresetLastSelected && !g_FavoriteSelected)
  {
    Disp_PosXY(0, 36);
    Disp_BlockClear(128, 8);  // erase existing line
    Disp_SetFont(MONO_8_NORM);
    Disp_PutDecimal(g_Config.PresetLastSelected, 2);  // Preset #
    Disp_SetFont(PROP_8_NORM);
    Disp_PutChar(' ');
    Disp_PutText((char *)g_Patch.PresetName);
    lastPresetShown = g_Config.PresetLastSelected;
  }

  // Refresh Favorite displayed if selection changed...
  if (g_FavoriteSelected && lastFavShown != g_FavoriteSelected)
  {
    Disp_PosXY(0, 36);
    Disp_BlockClear(128, 8);  // erase existing line
    Disp_SetFont(MONO_8_NORM);
    Disp_PutChar('F');  // Prefix for Fav.
    Disp_PutDecimal(g_FavoriteSelected, 1);  // Fav. # (1..8)
    Disp_SetFont(PROP_8_NORM);
    Disp_PutChar(' ');
    Disp_PutText((char *)g_Patch.PresetName);
    lastFavShown = g_FavoriteSelected;
  }

  if (g_MidiRxSignal && !midiIconVisible)  // MIDI message incoming
  {
    g_MidiRxSignal = FALSE;  // prevent repeats
    midiNoActivityTimer = 0;
    Disp_PosXY(102, 1);
    Disp_PutImage(midi_conn_icon_9x9, 9, 9);
    midiIconVisible = TRUE;
  }
  if (midiIconVisible && ++midiNoActivityTimer >= 20)  // 1 sec time-out...
  {
    Disp_PosXY(102, 1);
    Disp_BlockClear(10, 9);  // erase MIDI icon
    midiIconVisible = FALSE;
  }
  if (g_PatchModified && !patchIconVisible)  // Active patch modified
  {
    Disp_PosXY(117, 16);
    Disp_PutImage(patch_icon_9x9, 9, 9);
    patchIconVisible = TRUE;
  }
  if (!g_PatchModified && patchIconVisible)  // Active patch not modified
  {
    Disp_PosXY(117, 16);
    Disp_BlockClear(10, 9);
    patchIconVisible = FALSE;
  }
}


void  UserState_PresetSelect()
{
  static uint32_t timeSinceLastRefresh_ms;  // unit = ms
  static uint8_t  bank, setting;
  static bool settingChanged;
  uint8_t numBanks = (g_NumberOfPresets + 15) / 16;  // 16 presets per bank
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("     PRESET");
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Bank>");
    DisplayButtonLegend(BUTT_POS_C, "Select");
    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(100, 16);
    Disp_PutText("Bank");
    setting = g_Config.PresetLastSelected;
    bank = setting / 16;  // 0, 1, 2...
    settingChanged = TRUE;  // signal to refresh preset shown
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16 + (bank * 16);
    if (setting >= g_NumberOfPresets) setting = g_NumberOfPresets - 1;
    settingChanged = TRUE;
    doRefresh = TRUE;
  }

  if (ButtonHit('A')) GoToNextScreen(HOME_SCREEN);  // no change
  if (ButtonHit('B'))  // Next bank
  {
    if (++bank >= numBanks) bank = 0;
    setting = DataPotPosition() / 16 + (bank * 16);
    if (setting >= g_NumberOfPresets) setting = g_NumberOfPresets - 1;
    settingChanged = TRUE;
    doRefresh = TRUE;
  }
  if (ButtonHit('C'))  // Affirm new setting
  {
    PresetSelect(setting);
    g_FavoriteSelected = 0;  // redundant, for clarity only
    g_PatchModified = FALSE;
    ButtonLEDstate(88, OFF);  // Turn off FAV Preset LED(s)
    GoToNextScreen(HOME_SCREEN);
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 20);  // Display preset #
    Disp_BlockClear(40, 16);
    Disp_PutDecimal(setting, 2);
    Disp_SetFont(PROP_12_BOLD);
    Disp_PosXY(112, 26);  // Show bank #
    Disp_BlockClear(8, 10);
    Disp_PutDecimal(bank+1, 1);
    timeSinceLastRefresh_ms = 0;  // reset timer
  }

  if (settingChanged && timeSinceLastRefresh_ms >= 200)
  {
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(8, 38);
    Disp_BlockClear(120, 8);  // erase existing line
    Disp_PutText((char *)g_PresetPatch[setting].PresetName);  
    settingChanged = FALSE;  // prevent repeat refresh
    timeSinceLastRefresh_ms = 0;
  }

  timeSinceLastRefresh_ms += 50;  // Call period is 50ms
}


void  UserState_SetupMenu()
{
  static const char *selectionName[] =
    { "User Preset", "Pitch Bend", "Reverb", "MIDI channel", "Brightness", "Voice Tuning" };
  static uint8_t  item;
  bool  doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("   SETUP MENU");
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 18);
    Disp_PutText("Selected item:");
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Item>");
    DisplayButtonLegend(BUTT_POS_C, "Enter");
    item = 0;  // always start with User Patch screen
    doRefresh = TRUE;
  }

  if (ButtonHit('A')) GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))
  {
    item = (item + 1) % 6;  // 0..5
    doRefresh = TRUE;
  }
  if (ButtonHit('C'))  // Enter selected screen...
  {
    switch (item)
    {
      case 0:  GoToNextScreen(SET_USER_PRESET);   break;
      case 1:  GoToNextScreen(SET_PITCH_BEND);    break;
      case 2:  GoToNextScreen(SET_REVERB_LEVEL);  break;
      case 3:  GoToNextScreen(SET_MIDI_CHAN);     break;
      case 4:  GoToNextScreen(SET_DISP_BRIGHT);   break;
      case 5:  GoToNextScreen(SET_VOICE_TUNING);  break; 
    }
  }

  if (doRefresh)
  {
    Disp_SetFont(PROP_12_NORM);  // Display parameter name
    Disp_PosXY(16, 32);
    Disp_BlockClear(96, 12);  // clear existing data
    Disp_PutText(selectionName[item]);
  }
}


void  UserState_PatchMenu()
{
  static const char *parameterName[] =
    { "Mixer Levels", "LFO Depth", "LFO Freq", "LFO Ramp", "ENV Attack", "ENV Hold",
      "ENV Decay", "ENV Sustain", "ENV Release", "Mixer Gain", "Limiter" };
  static uint8_t item;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("   PATCH MENU");
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 18);
    Disp_PutText("Parameter to adjust:");
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next>");
    DisplayButtonLegend(BUTT_POS_C, "Select");
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    item = DataPotPosition() / 24;  // 0..10
    doRefresh = TRUE;
  }

  if (ButtonHit('A')) GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))
  {
    item = (item + 1) % 11;  // 0..10
    doRefresh = TRUE;
  }
  if (ButtonHit('C'))  // Affirm selection...
  {
    switch (item)  // 0..10
    {
      case  0:  GoToNextScreen(SET_MIXER_LEVELS);  break;
      case  1:  GoToNextScreen(SET_LFO_DEPTH);     break;
      case  2:  GoToNextScreen(SET_LFO_FREQ);      break;
      case  3:  GoToNextScreen(SET_LFO_RAMP);      break;
      case  4:  GoToNextScreen(SET_ENV_ATTACK);    break;
      case  5:  GoToNextScreen(SET_ENV_HOLD);      break;
      case  6:  GoToNextScreen(SET_ENV_DECAY);     break;
      case  7:  GoToNextScreen(SET_ENV_SUSTAIN);   break;
      case  8:  GoToNextScreen(SET_ENV_RELEASE);   break;
      case  9:  GoToNextScreen(SET_MIXER_GAIN);    break;
      case 10:  GoToNextScreen(SET_LIMITER_LVL);   break;
    }
  }

  if (doRefresh)
  {
    Disp_SetFont(PROP_12_NORM);  // Display parameter name
    Disp_PosXY(16, 32);
    Disp_BlockClear(96, 12);  // clear existing data
    Disp_PutText(parameterName[item]);
  }
}


void  UserState_SetUserPreset()
{
  static bool  affirmed;
  static uint32_t timeSinceAffirm_ms;
  static uint8_t  setting;  // Fav. Preset (0..7)
  bool  doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("User Preset");
    Disp_Mode(SET_PIXELS);
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 20);
    Disp_PutText("Save Active Patch");
    Disp_PosXY(4, 30);
    Disp_PutText("as Favorite Preset #");
    DisplayButtonLegend(BUTT_POS_A, "Cancel");
    DisplayButtonLegend(BUTT_POS_B, "Fav >");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    timeSinceAffirm_ms = 0;
    affirmed = FALSE;
    doRefresh = TRUE;
  }

  if (ButtonHit('A')) GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B')) // Next Fav. posn
  {
    if (++setting >= 8) setting = 0;  // Next Fav. (0..7)
    doRefresh = TRUE;
  }
  if (ButtonHit('C') && !affirmed)
  {
    StoreUserPreset(setting);  // Patch now saved as Favorite!
    if (!g_FavoriteSelected)  // Factory Preset was selected last
      g_Config.UserPresetBase[setting] = g_Config.PresetLastSelected;
    StoreConfigData();
    g_FavoriteSelected = setting + 1;  // 1..8
    g_PatchModified = FALSE;
    ButtonLEDstate(setting, ON);  // Turn on LED for fav preset selected
    Disp_ClearScreen();
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(32, 26);  // mid-screen
    if (g_EEpromFaulty) Disp_PutText("EEPROM fault!");
    else  Disp_PutText("Saved OK!");
    affirmed = TRUE;
    timeSinceAffirm_ms = 0;
  }
  if (affirmed)  // waiting 1.5 sec to view message
  {
    timeSinceAffirm_ms += 50;
    if (timeSinceAffirm_ms >= 1500) GoToNextScreen(HOME_SCREEN);
  }

  if (doRefresh)
  {
    Disp_PosXY(104, 30);
    Disp_BlockClear(16, 16);
    Disp_SetFont(MONO_16_NORM);
    Disp_PutDecimal(setting + 1, 1);
  }
}


void  UserState_SetPitchBend()  // Set on/off or bend range
{
  static uint8_t setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("Pitch Bend");
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 16);
    Disp_PutText("Bend Enable:");
    Disp_PosXY(4, 30);
    Disp_PutText("Bend Range:");
    Disp_PosXY(72, 40);
    Disp_PutText("semitones");
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "On/Off");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Config.PitchBendRange;
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 20;  // 0..12
    if (setting == 0) setting = 1;  // min.
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(SETUP_MENU);  // Exit
  if (ButtonHit('B'))
  {
    g_Config.PitchBendEnable ^= 1;  // toggle
    g_Config.PitchBendEnable &= 1;
    StoreConfigData();
	  // If pitch bend enabled, send MIDI msg to disable vibrato, and vice-versa...
    if (g_Config.PitchBendEnable) MIDI_SendControlChange(BROADCAST, 87, 0);   // Vibrato disabled
    else  MIDI_SendControlChange(BROADCAST, 87, 3);   // Vibrato auto-ramp
    doRefresh = TRUE;
  }
  if (ButtonHit('C'))  // Affirm
  {
    g_Config.PitchBendRange = setting;
    StoreConfigData();
    MIDI_SendControlChange(BROADCAST, 100, 0);  // Reg. Param 0 = Pitch-Bend range
    MIDI_SendControlChange(BROADCAST, 38, setting);  // Data Entry value (LSB)
    GoToNextScreen(SETUP_MENU);
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(72, 16);
    Disp_BlockClear(24, 8);  // erase existing text
    if (g_Config.PitchBendEnable) Disp_PutText("ON");
    else  Disp_PutText("OFF");
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(72, 27);
    Disp_BlockClear(24, 10);  // erase existing text
    Disp_PutDecimal(setting, 1);
  }
}


void  UserState_SetReverbLevel()
{
  static uint8_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("Reverb Level");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Home");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Config.ReverbMix_pc;
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = ((int)DataPotPosition() * 100) / 255;  // 0..100
    setting = (setting / 5) * 5;  // quantize, step size = 5
    g_Config.ReverbMix_pc = setting;
    MIDI_SendControlChange(BROADCAST, 89, setting);
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(SETUP_MENU);  // exit
  if (ButtonHit('B'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('C'))  // Affirm -- save and exit
  {
    StoreConfigData();
    GoToNextScreen(SETUP_MENU);  // Skip CV options and Master Tune
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 24);
    Disp_BlockClear(64, 16);  // clear existing data
    Disp_PutDecimal(g_Config.ReverbMix_pc, 2);
    Disp_SetFont(PROP_12_NORM);
    Disp_PutText(" %");
  }
}


void  UserState_SetMidiChannel()
{
  static uint8_t  setting;
  bool  doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("MIDI channel");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Home");
    DisplayButtonLegend(BUTT_POS_C, "Set");
    setting = g_Config.MidiChannel;
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(SETUP_MENU);  // exit
  if (ButtonHit('B'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('C'))
  {
    g_Config.MidiChannel = setting;
    StoreConfigData();
    GoToNextScreen(SETUP_MENU);
  }

  if (doRefresh)
  {
    Disp_SetFont(PROP_12_BOLD);
    Disp_PosXY(32, 24);
    Disp_BlockClear(64, 16);
    if (setting == 0) Disp_PutText("Omni On");
    else
    {
      Disp_SetFont(MONO_16_NORM);
      Disp_PosXY(48, 24);
      Disp_PutDecimal(setting, 1);
    }
  }
}


void  UserState_SetDisplayBright()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("     Display");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(32, 16);
    Disp_PutText("Brightness");
    DisplayButtonLegend(BUTT_POS_A, "Home");
    DisplayButtonLegend(BUTT_POS_B, "Save");
    DisplayButtonLegend(BUTT_POS_C, "Exit");
    setting = g_Config.DisplayBrightness;  // Last saved
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = ((int)DataPotPosition() * 100) / 255;  // 0..100
    setting = (setting / 5) * 5;  // quantize, step size = 5
    if (setting < 5) setting = 5;  // min. 5%
    if (setting > 95) setting = 95;  // max. 95%
    SSD1309_SetContrast(setting);
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))  // Save setting in EEPROM
  {
    g_Config.DisplayBrightness = setting;  // range 0..95
    StoreConfigData();  // commit
	GoToNextScreen(HOME_SCREEN);
  }
  if (ButtonHit('C'))  GoToNextScreen(SETUP_MENU);

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 28);
    Disp_BlockClear(64, 16);  // clear existing data
    Disp_PutDecimal(setting, 1);
    Disp_SetFont(PROP_12_NORM);
    Disp_PutText(" %");
  }
}


void  UserState_SetVoiceTuning()
{
  static short setting;  // signed (+/-64)
  static uint8_t voice;  // Voice-channel selected
  short  absValue;
  uint8_t offsetVal;  // setting offset by 64 (zero +64)
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("Voice Tuning");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(config_icon_9x9, 9, 9);  // Config icon
    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(16, 16);
    Disp_PutText("Voice");
    Disp_PosXY(72, 16);
    Disp_PutText("cents");
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(16, 42);
    Disp_PutText("Monophonic test mode");
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Save");
    DisplayButtonLegend(BUTT_POS_C, "Voice");
    setting = (short)g_Config.VoiceTuning[voice] - 64;  // +/-64
    g_VoiceUnderTest = voice;
    g_MonophonicTestMode = TRUE;
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = (short)DataPotPosition() - 128;  // -128 ~ +127
    if (setting >= 0)  // pot posn is right of centre
    {
      if (setting < 6) setting = 0;  // dead-band
      else  setting = ((setting - 6) * 64) / 120;  // +6 ~ +64
      if (setting > 60) setting = 60;
    }
    else  // pot posn is left of centre (setting < 0)
    {
      if (setting > -6) setting = 0;  // dead-band
      else  setting = ((setting + 6) * 64) / 120;  // -6 ~ -64
      if (setting < -60) setting = -60;
    }
    setting = (setting / 3) * 3;  // make multiple of 3
    offsetVal = (uint8_t) (setting + 64);
    MIDI_SendControlChange(voice+1, 100, 1);  // Reg. Param 1 = Fine Tuning
    MIDI_SendControlChange(voice+1, 38, offsetVal);  // Data Entry value (LSB)
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  // Exit -- last setting not saved
  {
    g_MonophonicTestMode = FALSE;
    GoToNextScreen(SETUP_MENU);
  }
  if (ButtonHit('B'))  // Save last setting in EEPROM
  {
    offsetVal = (uint8_t) (setting + 64);
    g_Config.VoiceTuning[voice] = offsetVal;
    StoreConfigData();
  }
  if (ButtonHit('C'))  // Next voice -- last setting not saved
  {
    if (++voice >= NUMBER_OF_VOICES) voice = 0;
    setting = g_Config.VoiceTuning[voice] - 64;  // +/-64
    g_VoiceUnderTest = voice;
    doRefresh = TRUE;
  }

  if (doRefresh)
  {
    Disp_PosXY(0, 26);
    Disp_BlockClear(120, 14);  // clear data area
    Disp_SetFont(MONO_16_NORM);  // show voice #
    Disp_PosXY(24, 26);
    Disp_PutDecimal(voice+1, 1);
    //
    Disp_PosXY(64, 26);  // show setting (signed)
    absValue = (setting >= 0) ? setting : (0 - setting);
    if (setting < 0)  Disp_PutChar('-');
    else if (setting > 0)  Disp_PutChar('+');
    else  Disp_PutChar(' ');
    Disp_PutDecimal(absValue, 2);
  }
}

//=======================================================================================

void  UserState_Set_LFO_Depth()
{
  static uint32_t timeSinceLastChange_ms;  // unit = ms
  static bool settingChanged;
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("LFO FM Depth");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Home");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Done");
    setting = g_Patch.LFO_FM_Depth;
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = ((int)DataPotPosition() * 205) / 255;  // 0..200
    setting = (setting / 5) * 5;  // cents quantized, step size = 5
    if (setting > 200) setting = 200;  // max. 200 cents
    settingChanged = TRUE;
    g_PatchModified = TRUE;
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))  GoToNextScreen(SET_LFO_FREQ);
  if (ButtonHit('C'))  GoToNextScreen(PATCH_MENU);
  
  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 24);
    Disp_BlockClear(80, 16);  // clear existing data
    Disp_PutDecimal(setting, 3);  // unit = cent
    Disp_SetFont(MONO_8_NORM);
    Disp_PosXY(Disp_GetX(), 30);
    Disp_PutText(" cents");
  }

  if (settingChanged && timeSinceLastChange_ms >= 200)
  {
    g_Patch.LFO_FM_Depth = setting;  // unit = cents (1/100 semitone)
    MIDI_SendControlChange(BROADCAST, 79, (setting / 5));  // unit = 5 cents
    settingChanged = FALSE;  // prevent repeat update
    timeSinceLastChange_ms = 0;
  }
  timeSinceLastChange_ms += 50;  // Call period is 50ms
}


void  UserState_Set_LFO_Freq()
{
  static uint32_t timeSinceLastChange_ms;  // unit = ms
  static bool settingChanged;
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("LFO Frequency");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Home");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Done");
    setting = g_Patch.LFO_Freq_x10 / 10;
    if (setting == 0) setting = 1;  // min. 1Hz
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    if (setting == 0) setting = 1;  // min. 1Hz
    settingChanged = TRUE;
    g_PatchModified = TRUE;
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))  GoToNextScreen(SET_LFO_RAMP);
  if (ButtonHit('C'))  GoToNextScreen(PATCH_MENU);

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 24);
    Disp_BlockClear(64, 16);  // clear existing data
    Disp_PutDecimal(setting, 1);
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(Disp_GetX(), 28);
    Disp_PutText(" Hz");
  }

  if (settingChanged && timeSinceLastChange_ms >= 200)
  {
    g_Patch.LFO_Freq_x10 = setting * 10;
    MIDI_SendControlChange(BROADCAST, 77, setting);  // 1..15 Hz
    settingChanged = FALSE;  // prevent repeat update
    timeSinceLastChange_ms = 0;
  }
  timeSinceLastChange_ms += 50;  // Call period is 50ms
}


void  UserState_Set_LFO_RampTime()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("LFO Ramp Time");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.LFO_RampTime;  // unit = ms
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    setting = timeValueQuantized[setting];
    if (setting < 100) setting = 0;  // make multiple of 100
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_ENV_ATTACK);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.LFO_RampTime = (uint16_t) setting;  // range 0..5000 ms
    MIDI_SendControlChange(BROADCAST, 78, setting / 100);  // unit = 100 ms
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 24);
    Disp_BlockClear(80, 16);  // clear existing data
    Disp_PutDecimal(setting, 1);  // up to 4 digits
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(Disp_GetX(), 28);
    Disp_PutText(" ms");
  }
}


void  UserState_Set_ENV_Attack()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("ENV Attack");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.EnvAttackTime;  // unit = ms
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 20;  // 0..11
    setting = timeValueQuantized[setting];  // one of 12 values
    if (setting < 10) setting = 10;  // minimum
    if (setting > 1000) setting = 1000;  // maximum
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_ENV_HOLD);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.EnvAttackTime = (uint16_t) setting;  // range 10..1000 ms
    MIDI_SendControlChange(BROADCAST, 73, setting / 10);  // unit = 10 ms
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 24);
    Disp_BlockClear(80, 16);  // clear existing data
    Disp_PutDecimal(setting, 1);  // up to 4 digits
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(Disp_GetX(), 28);
    Disp_PutText(" ms");
  }
}


void  UserState_Set_ENV_Hold()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("ENV Peak-Hold");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    Disp_SetFont(PROP_8_NORM);
    Disp_PosXY(4, 38);
    Disp_PutText("0 = bypass Peak & Decay");
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.EnvHoldTime;  // unit = ms
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 20;  // 0..11
    setting = timeValueQuantized[setting];  // one of 12 values
    if (setting > 1000) setting = 1000;  // maximum
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_ENV_DECAY);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.EnvHoldTime = (uint16_t) setting;  // range 0..1000 ms
    MIDI_SendControlChange(BROADCAST, 74, setting / 10);  // unit = 10 ms
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 20);
    Disp_BlockClear(80, 16);  // clear existing data
    if (setting == 0) Disp_PosXY(56, 20);
    Disp_PutDecimal(setting, 1);  // up to 4 digits
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(Disp_GetX(), 24);
    if (setting != 0) Disp_PutText(" ms");
  }
}


void  UserState_Set_ENV_Decay()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("ENV Decay");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.EnvDecayTime;  // unit = ms
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    setting = timeValueQuantized[setting];  // one of 16 values
    if (setting < 100) setting = 100;  // minimum
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_ENV_SUSTAIN);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.EnvDecayTime = (uint16_t) setting;  // range 100..5000 ms
    MIDI_SendControlChange(BROADCAST, 75, setting / 100);  // unit = 100 ms
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(8, 38);
    Disp_BlockClear(120, 8);  // clear existing message
    Disp_PosXY(40, 20);
    Disp_BlockClear(80, 16);  // clear existing data
    if (g_Patch.EnvHoldTime != 0)
    {
      Disp_PutDecimal(setting, 3);
      Disp_SetFont(PROP_12_NORM);
      Disp_PosXY(Disp_GetX(), 24);
      Disp_PutText(" ms");
    }
    else  // EnvHoldTime == 0 : Decay segment bypassed
    {
      Disp_PutText(" X ");
      Disp_SetFont(MONO_8_NORM);
      Disp_PosXY(32, 38);
      Disp_PutText("* BYPASSED *");
    }
  }
}


void  UserState_Set_ENV_Sustain()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("ENV Sustain");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.EnvSustainLevel;  // unit = %FS
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    setting = percentLogScale[setting];  // 0..100 %
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_ENV_RELEASE);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.EnvSustainLevel = (uint16_t) setting;  // range 0..100
    MIDI_SendControlChange(BROADCAST, 76, setting);  // unit = percent
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 20);
    Disp_BlockClear(64, 16);  // clear existing data
    Disp_PutDecimal(setting, 2);
    Disp_SetFont(PROP_12_NORM);
    Disp_PutText(" %");
  }
}


void  UserState_Set_ENV_Release()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("ENV Release");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.EnvReleaseTime;  // unit = ms
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 16;  // 0..15
    setting = timeValueQuantized[setting];  // one of 16 values
    if (setting < 100) setting = 100;  // minimum
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_MIXER_GAIN);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.EnvReleaseTime = (uint16_t) setting;  // range 100..5000 ms
    MIDI_SendControlChange(BROADCAST, 72, setting / 100);  // unit = 100 ms
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(40, 24);
    Disp_BlockClear(80, 16);  // clear existing data
    Disp_PutDecimal(setting, 1);  // up to 4 digits
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(Disp_GetX(), 28);
    Disp_PutText(" ms");
  }
}


void  UserState_Set_MixerGain()
{
  static uint8_t optMixerGain_x10[] = { 2, 3, 5, 7, 10, 15, 20, 25, 33, 50 };
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("Mixer Gain");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    Disp_SetFont(PROP_12_NORM);
    Disp_PosXY(36, 28);
    Disp_PutChar('x');
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Next");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.MixerOutGain_x10;  // unit = 0.1
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = DataPotPosition() / 25;  // 0..10
    if (setting > 9)  setting = 9;  // 0..9 (index)
    setting = (uint16_t) optMixerGain_x10[setting];
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_LIMITER_LVL);
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.MixerOutGain_x10 = (uint16_t) setting;  // range 2..50
    MIDI_SendControlChange(BROADCAST, 70, setting);  // unit = 0.1
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 24);
    Disp_BlockClear(64, 16);  // clear existing data
    Disp_PutDecimal(setting / 10, 1);  // integer part
    Disp_PutChar('.');
    Disp_PutDecimal(setting % 10, 1);  // fractional part
  }
}


void  UserState_Set_LimiterLevel()
{
  static uint16_t  setting;
  bool doRefresh = FALSE;

  if (isNewScreen)
  {
    DisplayTitleBar("Limiter Level");
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(116, 1);
    Disp_PutImage(patch_icon_9x9, 9, 9);  // Patch icon
    DisplayButtonLegend(BUTT_POS_A, "Exit");
    DisplayButtonLegend(BUTT_POS_B, "Back");
    DisplayButtonLegend(BUTT_POS_C, "Affirm");
    setting = g_Patch.LimiterLevelPc;  // unit = %
    doRefresh = TRUE;
  }

  if (DataPotMoved())
  {
    setting = ((int)DataPotPosition() * 100) / 255;  // 0..100
    setting = (setting / 5) * 5;  // quantize, step size = 5
    if (setting < 20) setting = 0;  // floor = 20% (0:Off)
    if (setting > 95) setting = 95;  // ceiling = 95%
    doRefresh = TRUE;
  }

  if (ButtonHit('A'))  GoToNextScreen(PATCH_MENU);
  if (ButtonHit('B'))  GoToNextScreen(SET_MIXER_GAIN);  // back
  if (ButtonHit('C'))  // Affirm:  Send to voice modules
  {
    g_Patch.LimiterLevelPc = (uint16_t) setting;  // range 0..95
    MIDI_SendControlChange(BROADCAST, 71, setting);  // unit = percent
    g_PatchModified = TRUE;
  }

  if (doRefresh)
  {
    Disp_SetFont(MONO_16_NORM);
    Disp_PosXY(48, 24);
    Disp_BlockClear(64, 16);  // clear existing data
    if (setting == 0) Disp_PutText("OFF");
    else
    {
      Disp_PutDecimal(setting, 2);
      Disp_SetFont(PROP_12_NORM);
      Disp_PutText(" %");
    }
  }
}

//=======================================================================================
//
// Function renders a dotted line horizontally at the current cursor position (x, y).
// If length is not a multiple of 8 pix, the length will be truncated to nearest 8 pix.
//
void  DrawDottedLineHoriz(uint8_t length)  
{
  static uint8_t  bar8pix = 0xAA;  // dotted bar, w=8, h=1 px
  uint8_t  xpos = Disp_GetX();
  uint8_t  ypos = Disp_GetY();
  uint8_t  savePosX = xpos;

  while (length >= 8)  
  { 
    Disp_PosXY(xpos, ypos);
    Disp_PutImage((uint8_t *)&bar8pix, 8, 1);
    xpos += 8;
    length -= 8; 
  }
  Disp_PosXY(savePosX, ypos);  // restore caller's cursor position
}


void  UpdateBarGraphLevel(uint8_t osc, uint8_t level)
{
  static uint8_t  bar8pix = 0xFF;  // solid bar, w=8, h=1 px
  uint8_t  xpos = 28 + osc * 16;
  uint8_t  ypos = 22;  // height at level 15 (max.)
  uint8_t  steps = 15;

  while (steps--)  // erase existing bar (15 steps)
  {
    Disp_PosXY(xpos, ypos);
    Disp_BlockClear(8, 1);  // erase 1 step
    ypos += 2;  // go down 2 pixels
  }
  ypos = 52;  // height at level 0
  while (level--)  // skip if level == 0
  {
    ypos -= 2;  // go up 2 pixels
    Disp_PosXY(xpos, ypos);
    Disp_PutImage((uint8_t *)&bar8pix, 8, 1);
  }
}


void  UserState_SetMixerLevels()  // Oscillator Mixer Input Levels
{
  static uint32_t  timeSinceLastChange_ms;  // unit = ms
  static uint8_t  setting[6];
  static bool  settingChanged[6];
  static bool  doRefresh[6];
  uint8_t  idx, osc, level, xpos, ypos, dataValue;

  if (isNewScreen)  // Render constant screen info
  {
    Disp_SetFont(PROP_8_NORM);
    Disp_Mode(SET_PIXELS);
    Disp_PosXY(3, 1);
    Disp_PutText("OSC");
    Disp_PosXY(0, 10);
    Disp_PutText("Freq");
    for (osc = 0;  osc < 6;  osc++)  
    {
      SliderPotMoved(osc);  // clear 'pot moved' flags
      settingChanged[osc] = FALSE;
      doRefresh[osc] = TRUE;
      // Show Osc. numbers 1..6, hi-lighted, top of screen
      xpos = 26 + osc * 16;
      Disp_Mode(SET_PIXELS);
      Disp_PosXY(xpos, 0);
      Disp_DrawBar(12, 9);
      Disp_Mode(CLEAR_PIXELS);  // invert pixels
      Disp_SetFont(MONO_8_NORM);
      Disp_PosXY(xpos + 4, 1);
      Disp_PutDecimal(osc + 1, 1);  // 1..6
      // Show Osc. Freq. Multiplier settings
      xpos = 26 + osc * 16;
      idx = g_Patch.OscFreqMult[osc];  // range 0..11 (1 of 12 options)
      Disp_Mode(SET_PIXELS);
      Disp_SetFont(PROP_8_NORM);
      Disp_PosXY(xpos, 11);
      if (idx == 0) Disp_PutText("1'2");  // 0.5 x fo
      else if (idx == 2) Disp_PutText("4'3");  // 1.333 x fo
      else if (idx == 3) Disp_PutText("3'2");  // 1.5 x fo
      else  
      {
        Disp_SetFont(MONO_8_NORM);
        Disp_PosXY(xpos + 4, 11);
        Disp_PutDecimal(oscFreqMultConst[idx], 1);  // single digit (1..9)
      }
      UpdateBarGraphLevel(osc, g_Patch.MixerInputStep[osc]);  // Initialize bar graph
    }
    // Draw division lines and labels at levels: 0, 4, 8, 12 & 16
    Disp_SetFont(PROP_8_NORM);
    Disp_Mode(SET_PIXELS);
    for (idx = 0;  idx < 5;  idx++)  
    {
      level = idx * 4;
      ypos = 52 - idx * 8;  // 8 pix per division
      Disp_PosXY(12, ypos - 1);
      if (level < 10)  Disp_PosXY(16, ypos - 1);  // single digit
      Disp_PutDecimal(level, 1);
      Disp_PosXY(24, ypos);
      if (idx == 0 || idx == 4) Disp_DrawBar(96, 1);  // Draw solid line
      else  DrawDottedLineHoriz(96);  // Draw dotted line
    }
    timeSinceLastChange_ms = 0;
  }

  if (ButtonHit('A'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('B'))  GoToNextScreen(HOME_SCREEN);
  if (ButtonHit('C'))  GoToNextScreen(HOME_SCREEN);

  for (osc = 0;  osc < 6;  osc++)
  {
    if (SliderPotMoved(osc))
    {
      setting[osc] = SliderPosition(osc) / 16;  // 0..15
      settingChanged[osc] = TRUE;
      doRefresh[osc] = TRUE;
    }
    if (settingChanged[osc] && timeSinceLastChange_ms >= 200)
    {
      // Update voices and patch parameter...
      g_Patch.MixerInputStep[osc] = setting[osc];
      g_PatchModified = TRUE;
      dataValue = (osc << 4) + (setting[osc] & 0x0F);
      MIDI_SendControlChange(BROADCAST, 80, dataValue);
      settingChanged[osc] = FALSE;  // prevent repeat update
      timeSinceLastChange_ms = 0;
    }
    if (doRefresh[osc])  // Show new Osc. Mixer Input Level
    {
      Disp_Mode(SET_PIXELS);
      Disp_SetFont(MONO_8_NORM);
      level = g_Patch.MixerInputStep[osc];
      xpos = 26 + osc * 16;
      Disp_PosXY(xpos, 56);  // erase existing value
      Disp_BlockClear(16, 8);
      if (level < 10)  xpos += 4;  // single digit
      Disp_PosXY(xpos, 56);
      Disp_PutDecimal(level, 1);  // 0..16
      UpdateBarGraphLevel(osc, level);
      doRefresh[osc] = FALSE;  // done
    }
  }

  // If no change in last 10 seconds, return home...
  if (timeSinceLastChange_ms >= 10000)  GoToNextScreen(HOME_SCREEN);
  timeSinceLastChange_ms += 50;  // Call interval is 50ms
}
