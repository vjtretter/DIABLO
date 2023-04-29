Github repo for Montana State Capstone II DIABLO project

Credit to Kai Morich for foundation of bluetooth app
https://github.com/kai-morich

Considering the difficulty of using the DJI drone for this use case, the app is not functional within the project as of the end of this capstone course.  However, in order to get the bluetooth app to work without any DJI functionality, copy the SerialSocket.java file into the corresponding location in Kai Morich's SimpleBluetoothTerminal (https://github.com/kai-morich/SimpleBluetoothTerminal).  This is located at:\
SimpleBluetoothTerminal/app/src/main/java/de/kai_morich/simple_bluetooth_terminal/SerialSocket.java

The app in this state is intended solely to interperet simple commands sent to the device the app is hosted on. It is formatted as an 8 byte array, with the first byte being reserved for custom commands, and the following 7 bytes being reserved for data needed for the execution of that command.


The file SkyboxESP32/SkyboxESP32.ino is the heart of this project.  This code allows some stimuli (in this case a button connected to GPIO14) to begin a launch procedure.  This involves an attempted extension of the platform, followed by a series of flight checks, including wind speed, temperature, and limit switches.
There is a pin reserved for battery checks, but as of now, it is not implemented.

The pinout of the ESP32 is:\
MotorDriver PWM		==> 4\
MotorDriverDirection1	==> 27\
MotorDriverDirection2	==> 15\
ExtendLimitSwitch	==> 26\
RetractLimitSwitch	==> 25\
StartButton		==> 14\
BatteryLogic		==> 5

Anenometer		==> 36 (A4)
