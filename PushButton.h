/*
 * PushButton.h
 *
 *  Created on: 13 Jun. 2019
 *      Author: Alireza Sadri
 */

// define a button as global and keep calling the buttonCheck function with current millis()
// and the voltage on the pin. The default is that the button is active low.
// This library's output is the output of buttonCheck() and models clicks as follows:
// 0: debounce
// 1: Long press without release
// 2: Long press and release
// 3: A click
// 4: Double-click
// 5: Three clicks
// 6: Four clicks
// n: n-2 clicks up to 255

// Any event n>0 is followed by n=0. For example if n=1 is the output
// if button is pressed and not released, when it is released the output
// n will be 3 and then it will be zero.

#pragma once

#ifndef PUSHBUTTON_H_
#define PUSHBUTTON_H_

class PushButton{
	unsigned long DeadDuration;				//be dead for a while after the last event
											//may init,  default = 100 (debounce x 2)
	unsigned long longDurationThreshold;	//Threshold at which its been a long press
											//may init,  default = 2000
	unsigned long debounceThreshold;		//Threshold below which its a bounce
											//may init,  default = 50
	bool pressed;							//Pin voltage level when pressed
											//may init,  default = LOW
	unsigned long lastCheckedTime;
	unsigned long pressedDuratoin;
	unsigned long lastEventTime;
	bool longPressflag;

public:
	PushButton(unsigned long inDeadDuration = 100,
			   unsigned long inlongDurationThreshold = 1000,
			   unsigned long indebounceThreshold = 50,
			   bool inpressed = 0);

	unsigned char buttonCheck(const unsigned int current_millies, bool currentPinStatus);
};

#endif /* PUSHBUTTON_H_ */
