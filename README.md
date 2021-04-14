# Helping Hand ESP32
Code for the the ESP32 in the helping hand project. Meant to be run as an Arduino project. This code is taken from another (private) repo but with the directory structure reworked and unused files removed.

By Hayden Liao and Ryan Regier.

## Installing

After cloning the repository, you need to setup the Arduino IDE to work with an ESP32
1) Go to File -> Preferences and put this link in "additional board manager URLs": https://dl.espressif.com/dl/package_esp32_index.json
2) Go to Tools -> Board -> Board Manager. Search for "esp32" and install the most recent version. This may take a few minutes.
3) Then in Tools -> Board -> ESP32 Arduino select either NodeMCU-32S or ESP32 Dev Board (both appear to work).
4) Special drivers are needed to upload to the ESP32 over USB. Go [here](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers) to install them. If you are on Linux you may already have them installed.
5) (On Windows) Run Device Manager and go to Ports (COM and LPT). Check which COM port is connected to the Silicon Labs USB to UART Bridge. If Ports (COM and LPT) does not show up on Device Manager, you will need to do the following:
    1) Go to Action -> Add legacy hardware. Click "Next"
    2) Select "Install the hardware that I manually select from a list (Advanced)" and click "Next"
    3) Scroll down to find and select "Ports (COM & LPT)," then click "Next"
    4) Select the manufacturer you need. The default should be fine. Click "Next"
6) Return to the Arduino IDE. Plug in the ESP32 to the correct USB port. In the menu, nagivate to Tools -> Port and select the COM port attached to the ESP.
7) In the Arduino IDE, go to Sketch -> Include Library -> Manage Libraries. Search for and install ESP32Encoder and ESP32Servo.

## Usage
#### Serial Commands
For both files in this repo, the same serial command format is used to communicate with the ESP32. The format is `<x,x,x,x,x>\n`, where the xs are replaced with the numbers to send to the ESP. The number of values required is equal to the NUM_JOINTS macro set in both files. 

The source of the serial commands depends on the value of the SigSerial macro. Setting this to "Serial" will allow communication over the serial monitor in the Arduino IDE. Setting it to a different Serial object (we reccomend Serial2) will allow communication over GPIO pins. When writing this format with the serial monitor the newline will be automatically added, but it must be manually written if you are sending serial commands through other means (such as the Raspberry Pi). The baud rate defaults to 115200 but can be changed through BAUD_RATE macro.

#### Serial PID Find Zero
The [serial_pid_find_zero](serial_pid_find_zero/serial_pid_find_zero.ino) code is meant to be running during normal operations of the prosthetic hand. The find zero code currently in the file does not work consistently, so instead this code assumes at all encoders are at the zero position (fully open) when starting up. Serial commands sent to the ESP are interpreted as target angles of the joints, in the units of degrees. These are converted to the units of encoder ticks then run through a PID loop to determine what PWM signal to send to the motors. Minimal bounds and obstruction checking is also present to prevent damage to the device, but in practice this will not prevent the line from snapping if you are not careful.

#### Serial Joystick
The [serial_joystick](serial_joystick/serial_joystick.ino) code is used to manually control PWM signals sent to the motors, which is useful for testing. In addition, it will print out encoder values for all motors. This is useful for checking that your encoders are wired correctly and manually driving the motors (such as for the initial windup). The serial command sent will be the PWMs sent to each motor (so the first number to the first motor, the second to the second, and so forth). You must have a PWM value for every motor even if you intend to keep the same value. If the serial command cannot be parsed, a neutral PWM is written to every motor. This means that an emergency stop command can be sent to the ESP simply by sending any invalid command like `q`.

Note: as of now not all macros defined in the Serial Commands section are followed in the joystick code.
