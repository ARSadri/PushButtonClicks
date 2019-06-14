/* Example of initilization:
PushButton myButton(75, 500, 150, 1);	//example of how to set the button
DeadDuration is set to 150ms
longDurationThreshold is set to 500ms
debounceThreshold is set to 75ms
pressed voltage state is HIGH 1
    
output: 0 nothing, noise or bounce    
    ____                            
____|  |____________________________
    50ms < DeadDuration : output 0           
    
    
output: 1 pressed and held for a long time and not released yet
output: 2 released aftera a long time
     ____________________________________________________
  __|                                                    |________
  700ms > longDurationThreshold : output 1    when released output : 2
  notiec that when output 1 is produced, output 2 will be produced later necessarily.
output: 3, 4 and more respectively: A click, double click, three clicks, ...
     _______ 
_____|     |___________________
    100ms :  A clicl because it is low after the click for DeadDuration
    
     _______            _______ 
_____|     |____________|     |______
             The gap is less than DeadDuration and it is a double click
and so on
*/

// This code is written for STM32duino
// for Arduino Uno make sure that pinMode is INPUT and 
// pullups are activated by digitalWrite(PushButtonPin, HIGH)

#include "Arduino.h"
#include "PushButtonClicks.h"

PushButton myButton;	//default settings are recommended!
#define myButtonPin PC15
void setup()
{
	Serial.begin(115200);
	pinMode(myButtonPin, INPUT_PULLUP);
}

// The loop function is called in an endless loop
void loop()
{
	switch(myButton.buttonCheck(millis(), digitalRead(myButtonPin))) {
		case 0 : Serial.println("Nothing or Bounce"); break;
		case 1 : Serial.println("Pressed and not released for a long time"); break;
		case 2 : Serial.println("Pressed and released after a long time"); break;
		case 3 : Serial.println("A click"); break;
		case 4 : Serial.println("Double click"); break;
		case 5 : Serial.println("Triple click"); break;
		case 6 : Serial.println("Four clicks"); break;
		case 7 : Serial.println("Five clicks"); break;
	}
	delay(35);	//its best if the delay between checkings is less than half of the debouncing duration
}
