# 'Sigma-6' Polyphonic Synthesizer
__DIY Digital Polyphonic Synthesizer using unique Additive Synthesis technique__

![Sigma-6-Poly-synth-profile](https://github.com/user-attachments/assets/de2b180a-744f-4f6d-af62-0a37b76fa0b2)

For details of concept, design, construction and operation, please refer to the project web page, here...  
https://www.mjbauer.biz/Sigma6_Poly_synth_weblog.htm

__Firmware Installation__

Download and install the latest version of Arduino IDE on your PC and follow the instructions provided by Adafruit, here:

[Arduino IDE Setup](https://learn.adafruit.com/introducing-itsy-bitsy-m0/setup)

Select board type __Arduino Zero (Native USB)__ if it is not already auto-detected by Arduino IDE. This is preferable
to the Adafruit SAMD21 board package, especially for the Poly-voice firmware, because the Arduino Zero startup code enables the 32.768kHz crystal oscillator for
the MCU system clock. The Adafruit ItsyBitsy M0 board uses the MCU internal 8MHz RC oscillator which is not as precise or stable.

To build the Sigma-6 Poly-voice firmware, you also need to install a "fast timer" library in the Arduino IDE.
Open the Arduino Library Manager. From the 'Type' drop-down list, choose 'All'. In the 'Filter' box, write "fast_samd21_tc"
and click 'INSTALL'. Then choose Type = "Installed". The library should then appear in your Arduino IDE as in this screen-shot:

![Screenshot_SAMD21_fast_timer_library](https://github.com/user-attachments/assets/398ecf9a-11e7-4b22-b53f-e896f9cf998e)

Steps to compile and "upload" the __Sigma-6 Poly-voice__ firmware:

    > Download the Sigma-6 Poly-voice source files from the repository here.
    > Create a project folder in your computer local drive named "Sigma_6_Poly_voice".
    > Copy the downloaded source files into the project folder.
    > Double-click on the file "Sigma_6_Poly_voice.ino" -- this should open Arduino IDE and load
      all source files into the editor window. (Alternatively, open Arduino IDE first, then open the
      source file "Sigma_6_Poly_voice.ino".)
    > Connect the first Voice MCU to your computer USB port.
    > Select board type 'Arduino Zero (Native USB)' as noted above.
    > Compile the code and upload the firmware to the Sigma-6 Voice MCU.
    > Repeat the upload procedure for all remaining voice MCU's.

Steps to compile and "upload" the __Sigma-6 Poly Master__ firmware:

    > Download the Sigma_6_Poly_master source files from the repository here.
    > Create a project folder in your computer local drive named "Sigma_6_Poly_master".
    > Copy the downloaded source files into the project folder.
    > Double-click on the file "Sigma_6_Poly_master.ino" -- this should open Arduino IDE and load
      all source files into the editor window. (Alternatively, open Arduino IDE first, then open the
      source file "Sigma_6_Poly_master.ino".)
    > Connect the Master MCU to your computer USB port.
    > Select board type 'Arduino Zero (Native USB)' as noted above.
    > Make any required changes to the source code to suit your synth configuration.*
    > Compile the code and upload the firmware to your Sigma-6 Master Controller MCU.

* Edit any applicable #define lines in the main file "Sigma_6_Poly_master.ino" (see comments therein);
  in particular to specify the total number of voices implemented in your build.

NB: It may be necessary to reset the MCU and/or unplug and reconnect the USB cable to get the bootloader to start, after
which it may also be necessary to re-select the board type and/or USB-serial port in the drop-down box in Arduino IDE.

![Coded-by-a-Human-no-AI](https://github.com/user-attachments/assets/e2479440-66a7-4c50-b6dd-358b75b23dfc)
