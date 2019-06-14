# PushButton
This is a repo for a C++ library to support detection of events comming from a PushButton.

This library is written in simple C++ with no dependency. It is tested for Arduino - STM32duino Maple mini and ESP32.

An object PushButton must be constructred globally and its check rutine must be called periodically. The output of the routin shows the events. 

the construction accepts inputs:
1- DeadDuration: That is the duration that push button is dead between events
    DEFAULT: 150
2- longDurationThreshold: That is the duration that a key must be pressed to be considered as long time
    DEFAULT: 500
3- debounceThreshold: That is the minimum duration that the button must be pressed to be considered as an event, i.e. debouncing
    DEFAULT: 75
4- pressed: the voltage state when pressed
    DEFAULT: 0   (This is because usually microcontrollers have an internal pullup and button is active LOW)

The check routine must be called together with two inputs:
1- Current time : for example in Arduino millis() is perfect which gives the time of running the system in milliseconds.
2- current state of the PushButton voltage: 0 or 1. e.g. in Arduino digitalRead(PushButtonPin)

The output of the Check routine can be:
0: nothing or bounce
1: long press with no release
2: long press and release
3: A click
4: Double-click
5: Triple-click
6: four clicks
n: n-2 clicks up to n = 255

Good Luck!
