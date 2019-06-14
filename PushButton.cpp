/*
 * PushButton.cpp
 *
 *  Created on: 13 Jun. 2019
 *      Author: Alireza Sadri
 */

#include "PushButton.h"

PushButton::PushButton(unsigned long inDeadDuration, unsigned long inlongDurationThreshold, unsigned long indebounceThreshold, bool inpressed) {
	DeadDuration = inDeadDuration;
	longDurationThreshold = inlongDurationThreshold;
	debounceThreshold = indebounceThreshold;
	pressed = inpressed;
	lastCheckedTime = 0;
	pressedDuratoin = 0;
	lastEventTime = 0;
	longPressflag = 0;
}

unsigned char shortPressedCnt=2;

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
