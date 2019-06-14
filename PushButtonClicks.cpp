/*
 * PushButtonClicks.cpp
 *
 *  Created on: 13 Jun. 2019
 *      Author: Alireza Sadri
 */

#include "PushButtonClicks.h"
//The constructor
PushButton::PushButton(unsigned long inDeadDuration, unsigned long inlongDurationThreshold, unsigned long indebounceThreshold, bool inpressed) {
	DeadDuration = inDeadDuration;
	longDurationThreshold = inlongDurationThreshold;
	debounceThreshold = indebounceThreshold;
	pressed = inpressed;
	lastCheckedTime = 0;
	pressedDuratoin = 0;
	lastEventTime = 0;
	longPressflag = 0;
	shortPressedCnt = 2;
}

/* Long Press/hold/release is supported.
If the button is pressed a counter counts how much time it has been in pressed state.
if while it is held active, the duration passes a threshold, it is a long_press
if released and the duration counter is higher than that threshold, its a long_release
If it is not a long release and still the duration was higher than debouncing threshold, its a click.
if another click does not arrive within the duration of bein dead, it was a click, 
if it came and it was a short one its a double click and so on
*/
unsigned char PushButton::buttonCheck(const unsigned int current_millies, bool currentPinStatus) {
	unsigned char output = 0;
	if (currentPinStatus==pressed) {
		pressedDuratoin += current_millies - lastCheckedTime;
		if (pressedDuratoin > longDurationThreshold && longPressflag == 0) {
			output = 1;	//long Event without release
			longPressflag = 1;
		}
	}
	else {
		if (pressedDuratoin > longDurationThreshold) {
			output = 2;	//long Event with release
			lastEventTime = current_millies;
			shortPressedCnt = 2;
		}

		if(pressedDuratoin > debounceThreshold && pressedDuratoin<=longDurationThreshold) {
			shortPressedCnt++;
			lastEventTime = current_millies;
		}
		if(current_millies > lastEventTime + DeadDuration && shortPressedCnt>2) {
			output = shortPressedCnt;
			shortPressedCnt = 2;
		}

		longPressflag = 0;
		pressedDuratoin = 0;
	}
	lastCheckedTime = current_millies;
	return(output);
}
