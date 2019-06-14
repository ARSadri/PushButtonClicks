#include "Arduino.h"
#include "PushButton.h"

PushButton myButton;	//default settings are recommended!
//PushButton myButton(200, 1000, 100, 1);	//example of how to set the button
#define myButtonPin PC15

void setup()
{
	Serial.begin(115200);
	pinMode(myButtonPin, INPUT_PULLUP);
}

// The loop function is called in an endless loop
void loop()
{
	unsigned long current_millis = millis();
	bool current_pin_state = digitalRead(myButtonPin);
	unsigned char myButton_clicked_mode = myButton.buttonCheck(current_millis, current_pin_state);
	switch(myButton_clicked_mode) {
		//case 0 : Serial.println("Nothing or Bounce"); break;
		case 1 : Serial.println("Pressed and not released for a long time"); break;
		case 2 : Serial.println("Pressed and released after a long time"); break;
		case 3 : Serial.println("A click"); break;
		case 4 : Serial.println("Double click"); break;
		case 5 : Serial.println("Triple click"); break;
		case 6 : Serial.println("Four clicks"); break;
		case 7 : Serial.println("Five clicks"); break;
	}
	delay(10);
}
