# Setting up

C2 is a 2-pin protocol for Arduino Uno. Just need to make sure that the correct pins are mapped for your Arduino Uno.  

# Setup Arduino
- Connect the clocks of all ESCs to Arduino Uno pin 6
- C2D pins to Arduino pin 2,5,11 and 12. 
- Connect GND to Arduino GND.
- Program the Arduino Uno with [arduino_uno.ino](prog/arduino_uno/arduino_uno.ino)

# Software

You need to have Python installed.  Then, install some required python modules.

Use Python 3.x and Pyserial for [flash36.py](https://github.com/christophe94700/efm8-arduino-programmer/blob/master/flash36.py)

```
pip install -r requirements.txt
```

# Running

Programming one target.

```
python flash36.py <serial-port> <firmware.hex>
```

Example for Linux: 
```flash35.py /dev/ttyACM0 RF_Brige.hex```

Example for Windows: 
```python flash36.py COM8 RF_Bridge.hex```


# Troubleshooting

- Some modules need sudo on some systems

# Changing the communication speed


Edit the following lines:
In the Python program modify the following line to switch to a speed of 115200baud / sec:

    self.ser = serial.Serial (com, 115200, timeout = 1)
In the program of your Arduino:

    Serial.begin (115200);
