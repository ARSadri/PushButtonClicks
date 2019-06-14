# A Push Button Event Modelling Library
This library is written for simple modelling of events made by a switch by monitoring it periodically. This repo is different from [famous PushButton Library](https://github.com/pololu/pushbutton-arduino), in that it is easier to use and supports modelling long press/release besides a click, double click, and any other number of clicks. This makes it suitable for touchpads or counting laser pulses as well. 

## Summary
This is a repo for a C++ library to support detection of events coming from a PushButton, a switch, a touchpad or laser pulses.This library was first designed for modeling any kind of 1-bit digital event. It is simply advertised for Arduino and PushButtons but the library can be used in any other platform.

## Supported platforms
This library is written in simple C++ with no dependencies. However, it is tested for Arduino - STM32duino Maple mini and ESP32.
Look at the example file written for Arduino.

## Getting started

### Hardware
Just connect a pin to a switch whose other side is connected to the ground and activate the pull-up resistor inside the microcontroller.

### software
An object PushButton must be constructed globally and its check routine must be called periodically. The output of the routine shows the events. Please check the example code and consider the following.

```
#include "Arduino.h"
#include "PushButtonClicks.h"
PushButton myButton;	//default settings are recommended!
#define myButtonPin PC15
void setup()
{
	Serial.begin(115200);
	pinMode(myButtonPin, INPUT_PULLUP);
}
void loop()
{
	switch(myButton.buttonCheck(millis(), digitalRead(myButtonPin))) {
		case 0 : Serial.println("Nothing or Bounce"); break;
		case 1 : Serial.println("Pressed and not released for a long time"); break;
		case 2 : Serial.println("Pressed and released after a long time"); break;
		case 3 : Serial.println("A click"); break;
		case 4 : Serial.println("Double click"); break;
		case 5 : Serial.println("Triple click"); break;
	}
	delay(35);
}
```

the construction of the object accepts inputs:

* DeadDuration: That is the duration that push-button is dead between events; DEFAULT: 150
* longDurationThreshold: That is the duration that a key must be pressed in to be considered as long time; DEFAULT: 500
* debounceThreshold: That is the minimum duration that the button must be pressed to be considered as an event, i.e. debouncing; DEFAULT: 75    
* pressed: the voltage state when pressed; DEFAULT: 0   (This is because usually, microcontrollers have an internal pull-up and button is active LOW)

The check routine must be called together with two inputs:

* Current time: for example in Arduino millis() is perfect which gives the time of running the system in milliseconds.
* current state of the PushButton voltage: 0 or 1. e.g. in Arduino digitalRead(PushButtonPin). 

The output of the Check routine can be numbers 0,1,2,3,...:

0. nothing or bounce
1. long press with no release
2. long press and release
3. A click
4. Double-click
5. Triple-click
6. four clicks
7. and up to n=255: n-2 clicks
