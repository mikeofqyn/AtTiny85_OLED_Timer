# AtTiny85_OLED_Timer

Use an Attiny82 microcontroller to manage a timer switch. 

An exercise on managing hardware with the minimal resources provided by a AtTiny85 microcontroller: 6K program memory, 512 bytes dynamic memory, 6 I/O pins. 

The duration is set trough a 5-pin encoder, and the switch is started with the encoder pushbutton. Time set and remaining is displayend on a 1603 (128x32) OLED display with I2C interface. 

The output signal is used to drive a MOSFET transistor. The same pin is used to read the encoder's pushbutton, so once turned on the switch has to be turned off by rotating the encoder's knob. 

The example circuit provided can be powered with 6 to 12 volts. To use 5 volts, remove C2 and the 7805 regulator and connect the power input directly to  the ground and output pins of the regulator (2 and 3).

