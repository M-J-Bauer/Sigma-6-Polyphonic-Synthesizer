# 'Sigma-6' Polyphonic Synthesizer
__DIY Digital Polyphonic Synthesizer using unique Additive Synthesis technique__

![Sigma-6-Poly-HOME-screen-web](https://github.com/user-attachments/assets/5409557d-349e-40b0-8359-043d28ec8b8d)

For details of concept, design, construction and operation, please refer to the project web page, here...  
https://www.mjbauer.biz/Sigma6_Poly_synth_weblog.htm

__Firmware Installation__

Download and install the latest version of Arduino IDE on your PC and follow the instructions provided by Adafruit, here:

[Arduino IDE Setup](https://learn.adafruit.com/introducing-itsy-bitsy-m0/setup)

Install the Adafruit SAMDxx Boards Manager. When you connect
the board to your computer, Arduino should determine the board type automatically, but in any case be sure to select board
type 'Adafruit ItsyBitsy M0 Express (SAMD21)' to ensure that the firmware will compile with compatible library functions.

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
    > Compile the code and upload the firmware to each of the Sigma-6 Voice MCU's.^

Steps to compile and "upload" the __Sigma-6 Poly MASTER controller__ firmware:

    > Download the Sigma_6_Poly_master source files from the repository here.
    > Create a project folder in your computer local drive named "Sigma_6_Poly_master".
    > Copy the downloaded source files into the project folder.
    > Double-click on the file "Sigma_6_Poly_master.ino" -- this should open Arduino IDE and load
      all source files into the editor window. (Alternatively, open Arduino IDE first, then open the
      source file "Sigma_6_Poly_master.ino".)
    > Make any required changes to the source code to suit your synth configuration.*
    > Compile the code and upload the firmware to your Sigma-6 Master Controller MCU).^

* Edit any applicable #define lines in the main file "Sigma_6_Poly_master.ino" (see comments therein),
  for example to specify the number of voices implemented in your build.

^ Sometimes it is necessary to retry an upload more than once, or to select (again) the board COM port, 
  and/or to press the MCU reset button to enter the bootloader. This is "normal" for Arduino!

![Coded-by-a-Human-no-AI](https://github.com/user-attachments/assets/e2479440-66a7-4c50-b6dd-358b75b23dfc)
