# 'Sigma-6' Polyphonic Synthesizer
__DIY Digital Polyphonic Synthesizer using unique Additive Synthesis technique__

![Sigma-6-Poly-HOME-screen-web](https://github.com/user-attachments/assets/5409557d-349e-40b0-8359-043d28ec8b8d)

For details of concept, design, construction and operation, please refer to the project web page, here...  
https://www.mjbauer.biz/Sigma6_Poly_synth_weblog.htm

__Firmware Installation__

Download and install the latest version of Arduino IDE on your PC and follow the instructions provided by Adafruit, here:

[Arduino IDE Setup](https://learn.adafruit.com/introducing-itsy-bitsy-m0/setup)

Install the Adafruit SAMDxx Boards Manager. When you connect
the board to your computer, Arduino usually determines the board type automatically, but in any case be sure to select board
type 'Adafruit ItsyBitsy M0 Express (SAMD21)' for the Sigma-6 Poly __MASTER__ controller firmware build, because the
Arduino Zero board package does not support analog inputs A8, A9 and A11.

Select board type 'Arduino Zero (Native USB)' for the Sigma-6 Poly __VOICE__ module firmware build. This is preferable
to the Adafruit M0 board package because the Arduino Zero startup code enables the 32.768kHz crystal oscillator for
the MCU system clock. (The Adafruit ItsyBitsy M0 board uses the MCU internal 8MHz RC oscillator which is not as precise or stable.)

The Robotdyn SAMD21 M0-Mini board is mostly compatible with the "retired" (obsolete) Arduino M0 board, but the Robotdyn MCU bootloader
is a copy of the Arduino Zero bootloader which is incompatible! Hence it is impossible to upload the firmware code with the 
Arduino M0 board package selected in the IDE... (Ugh! What were the board developers thinking?)

It may be necessary to reset the MCU and/or unplug and reconnect the USB cable to get the bootloader to start, after
which it may also be necessary to re-select the board type and/or USB-serial port in the drop-down box in Arduino IDE.

To build the __Sigma-6 Poly VOICE module__ firmware, you also need to install a "fast timer" library in the Arduino IDE.
Open the Arduino Library Manager. From the 'Type' drop-down list, choose 'All'. In the 'Filter' box, write "fast_samd21_tc"
and click 'INSTALL'. Then choose Type = "Installed". The library should then appear in your Arduino IDE as in this screen-shot:

![Screenshot_SAMD21_fast_timer_library](https://github.com/user-attachments/assets/398ecf9a-11e7-4b22-b53f-e896f9cf998e)

Steps to compile and "upload" the __Sigma-6 Poly VOICE module__ firmware:

    > Download the Sigma-6 Poly-voice source files from the repository here.
    > Create a project folder in your computer local drive named "Sigma_6_Poly_voice".
    > Copy the downloaded source files into the project folder.
    > Double-click on the file "Sigma_6_Poly_voice.ino" -- this should open Arduino IDE and load
      all source files into the editor window. (Alternatively, open Arduino IDE first, then open the
      source file "Sigma_6_Poly_voice.ino".)
    > Connect the first Voice MCU to your computer USB port and select board type 'Arduino Zero (Native USB)'.
    > Compile the code and upload the firmware to the Sigma-6 Voice MCU.^
    > Repeat the upload procedure for all remaining voice MCU's.

Steps to compile and "upload" the __Sigma-6 Poly MASTER controller__ firmware:

    > Download the Sigma_6_Poly_master source files from the repository here.
    > Create a project folder in your computer local drive named "Sigma_6_Poly_master".
    > Copy the downloaded source files into the project folder.
    > Double-click on the file "Sigma_6_Poly_master.ino" -- this should open Arduino IDE and load
      all source files into the editor window. (Alternatively, open Arduino IDE first, then open the
      source file "Sigma_6_Poly_master.ino".)
    > Connect the Master MCU to your computer USB port and select board type 'Adafruit ItsyBitsy M0 Express'.
    > Make any required changes to the source code to suit your synth configuration.*
    > Compile the code and upload the firmware to your Sigma-6 Master Controller MCU).^

* Edit any applicable #define lines in the main file "Sigma_6_Poly_master.ino" (see comments therein);
  in particular to specify the number of voices implemented in your build.

^ Sometimes it is necessary to retry an upload more than once, or to select (again) the board COM port, 
  and/or to press the MCU reset button to enter the bootloader. This is "normal" for Arduino!

![Coded-by-a-Human-no-AI](https://github.com/user-attachments/assets/e2479440-66a7-4c50-b6dd-358b75b23dfc)
