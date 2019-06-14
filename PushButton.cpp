
#include "PushButton.h"
#include "Arduino.h"

#include "src-gen/CNCSimCtrl.h"
#include "src-gen/CNCSimCtrlRequired.h"
#include "scutil/sc_timer_service.h"

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Fonts/FreeSans7pt7b.h>
#include <SPI.h>
#define CS     PB12
#define RST    PB13  // you can also connect this to the Arduino reset in which case, set this #define pin to -1!
#define DC     PB14
#define LCD_microSD_CS	PB2
#define BLACK	0x0000
#define WHITE	0xFFFF
#define BLUE	0x001F
#define RED		0xF800
#define GREEN	0x07E0
#define CYAN	0x07FF
#define MAGENTA	0xF81F
#define YELLOW	0xFFE0
#define AMBER	0xFE00
// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
Adafruit_ST7735 lcd = Adafruit_ST7735(CS,  DC, RST);


#define modeB_pin 		PA0
#define setB_pin 		PC15
#define Encpin1			PB6
#define Encpin2			PB7
#define led_white_pin 	PA15
#define led_green_pin	PB3
#define led_yellow_pin 	PB4
#define led_red_pin 	PB5
#define rly_run_pin 	PA8
#define rly_cycle_pin 	PB15

#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS                        2.5f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25                                760.0f
#define ADC_REFERENCE_VOLTAGE_MV                                    3300.0f
#define ADC_MAX_OUTPUT_VALUE                                        4095.0f
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define TEMP110                                                     110.0f
#define TEMP30                                                      30.0f

//#define inBufIntPin 	18
//#define outBufIntPin 	19
#define Serial2_TX_Select_Pin PA1
#define Serial3_TX_Select_Pin PB0
#define Machine_Select_1	PC14
#define Machine_Select_2	PC13

// We need to declare the data exchange
// variable to be volatile - the value is
// read from memory.
#define MAX_TIMERS 20 //number of timers our timer service can use
#define CYCLE_PERIOD 0 //number of milliseconds that pass between each statechart cycle
#define REPLENISH_PERIOD 	0
#define LCDUpdate_PERIOD	200

static unsigned long cycle_count = 0L; //number of passed cycles
static unsigned long last_cycle_time = 0L; //timestamp of last cycle
static unsigned long last_LCDUpdate_time = 0L;
static unsigned long last_REPLENISH_time = 0L;

#define DEBOUNCE_THRSH		70	//milliseconds
#define LONG_PRESS_THRSH	500	//milliseconds

//pushbutton_t pushbutton;		//pushbutton
static CNCSimCtrl cncsimctrl;

static sc_timer_service_t timer_service;

static sc_timer_t timers[MAX_TIMERS];

int setMenuUpdated=1;
int setMenurefreshed = 0;
int oldPosition=0;
int setFlag=0;
int State_0_counter=0;
int State_0_Flag=1;
int State_1_Flag=1;
int State_2_Flag=1;
int State_21_flag=1;
int State_3_counter=0;
int State_3_Flag=1;
int State_4_counter=0;
int State_4_Flag=1;
int State_41_flag=1;
int State_42_Flag=1;
int State_42_1_flag=1;
int State_5_Flag=1;
int State_5_counter=0;
int State_52_Flag=1;
int inBufferval;
int outBufferVal;
int aProdInProcess=0;
int State_32_counter=0;
int materialAvailableFlag = 0;
int productRequestedFlag = 0;
int waitingForMaterialFlag = 1;
int connection_flag = 0;
int setMenuSel;
int setMenuSel_perv;
int State_101_Flag=1;
int presetMenuSel;
int presetMenuSel_perv;
int set_PB_released=1;
int mode_PB_released=1;
unsigned long modePushButtonRaiseTime=0;
unsigned long  setPushButtonRaiseTime=0;
unsigned char myRand;

double numStepsCircleTurning=48.0;

int myEnctmp=0;
int myEnc=0;
int EncState=0;
bool pin1_old;
bool pin2_old;
void EncInts() {
	bool pin1val = digitalRead(Encpin1);
	bool pin2val = digitalRead(Encpin2);
	EncState = 8*pin1_old + 4*pin2_old + 2*pin1val + pin2val;
	pin1_old = pin1val;
	pin2_old = pin2val;
	if ((EncState==0b0001) || (EncState==0b0111) || (EncState==0b1110) || (EncState==0b1000))
		myEnctmp--;
	else if ((EncState==0b0010) || (EncState==0b1011) || (EncState==0b1101) || (EncState==0b0100))
		myEnctmp++;
	myEnc = myEnctmp/4;
}

//! callback implementation for the setting up time events
void cNCSimCtrl_setTimer(CNCSimCtrl* handle, const sc_eventid evid, const sc_integer time_ms, const sc_boolean periodic){
	sc_timer_start(&timer_service, (void*) handle, evid, time_ms, periodic);
}

//! callback implementation for canceling time events.
void cNCSimCtrl_unsetTimer(CNCSimCtrl* handle, const sc_eventid evid) {
	sc_timer_cancel(&timer_service, evid);
}

//static void button_changed(pushbutton_t *button) {
//	if (!pushbutton.state) cNCSimCtrlIface_raise_mode(&cncsimctrl);
//}




PushButton modeButton;
PushButton setButton;


//The setup function is called once at startup of the sketch
void setup()

{
	pinMode(Encpin1,INPUT_PULLUP);
	pinMode(Encpin2,INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(Encpin1), EncInts, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Encpin2), EncInts, CHANGE);

	pinMode(Serial3_TX_Select_Pin, OUTPUT);
	digitalWrite(Serial3_TX_Select_Pin, LOW);
	pinMode(Serial2_TX_Select_Pin, OUTPUT);
	digitalWrite(Serial2_TX_Select_Pin, LOW);
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial2.begin(115200);
	Serial3.begin(115200);
	pinMode(Machine_Select_1, OUTPUT);
	digitalWrite(Machine_Select_1, HIGH);
	pinMode(Machine_Select_2, OUTPUT);
	digitalWrite(Machine_Select_2, HIGH);

	pinMode(modeB_pin, INPUT_PULLUP);
	pinMode(setB_pin, INPUT_PULLUP);

	pinMode(PA3, INPUT_PULLUP);
	pinMode(PB11, INPUT_PULLUP);
	pinMode(PA10, INPUT_PULLUP);

	//setup_pushbutton(&pushbutton, modeB_pin, button_changed);

	pinMode(led_red_pin, OUTPUT);
	pinMode(led_green_pin, OUTPUT);
	pinMode(led_yellow_pin, OUTPUT);
	pinMode(led_white_pin, OUTPUT);

    pinMode(rly_run_pin, OUTPUT);
	pinMode(rly_cycle_pin, OUTPUT);

	lcd.initR(INITR_GREENTAB);
	lcd.setFont(&FreeSans7pt7b);
	lcd.setTextSize(1);
	lcd.setRotation(3);
	lcd.setTextWrap(false);


	sc_timer_service_init(
			&timer_service,
			timers,
			MAX_TIMERS,
			(sc_raise_time_event_fp) &cNCSimCtrl_raiseTimeEvent
			);

	cNCSimCtrl_init(&cncsimctrl); //initialize statechart

	cNCSimCtrl_enter(&cncsimctrl); //enter the statechart

	adc_reg_map *regs = ADC1->regs;
	regs->CR2 |= ADC_CR2_TSVREFE;
	regs->SMPR1 = (0b111 << (3*6));

	Serial.println("Machine Simulator initialized");
	set_PB_released = 0;
}

unsigned char currentmode_but_mode_perv=0;
unsigned char currentmode_but_mode=0;

// The loop function is called in an endless loop
void loop()
{
	long newPosition;
	unsigned long current_millies;

	current_millies =  millis();
	currentmode_but_mode_perv = currentmode_but_mode;
	while(1) {

		switch(modeButton.buttonCheck(millis(), digitalRead(modeB_pin))) {
			case 3 : cNCSimCtrlIface_raise_mode(&cncsimctrl); Serial.println("mode_short"); break;
			case 1 : cNCSimCtrlIface_raise_modeLongPress(&cncsimctrl); Serial.println("mode_long"); break;
		}

		switch(setButton.buttonCheck(millis(), digitalRead(setB_pin))) {
			case 3 : cNCSimCtrlIface_raise_set(&cncsimctrl); Serial.println("set_short"); break;
		    case 1 : cNCSimCtrlIface_raise_setLongPress(&cncsimctrl);; Serial.println("set_long"); break;
		}


		if (Serial1.available()) {
			char c = Serial1.read();
			Serial.print("Got this:");
			Serial.println(c);
			if (c == 0xCC)
					cNCSimCtrlIface_raise_mode(&cncsimctrl);

		}

		newPosition = myEnc;
		if (newPosition > oldPosition ) {
			oldPosition = newPosition;
			cNCSimCtrlIface_raise_encoderUp(&cncsimctrl);
			setMenuUpdated = 1;
		}
		else if (newPosition < oldPosition ) {
			oldPosition = newPosition;
			cNCSimCtrlIface_raise_encoderDown(&cncsimctrl);
			setMenuUpdated = 1;
		}

		current_millies = millis();
		if ( cycle_count == 0L || (current_millies >= last_cycle_time + CYCLE_PERIOD) ) {
			sc_timer_service_proceed(&timer_service, current_millies - last_cycle_time);
			cNCSimCtrl_runCycle(&cncsimctrl);
			last_cycle_time = current_millies;
			cycle_count++;

		}
		if ( current_millies >= last_REPLENISH_time + REPLENISH_PERIOD ) {
			my_synchronize(&cncsimctrl);
			last_REPLENISH_time = current_millies;
			myRand = random(255);
			while (myRand == 0xAA || myRand == 0xCC || myRand == 0)
				myRand = random(255);
			cNCSimCtrlIface_set_myRandomNum(&cncsimctrl, myRand);
		}
		if ( current_millies >= last_LCDUpdate_time + LCDUpdate_PERIOD ) {
			my_synchronize_LCD(&cncsimctrl);
			last_LCDUpdate_time = current_millies;
			randomSeed(myRand + adc_read(ADC1, 16));
		}

	}
}

void UpdateBuffersOnLCD() {
	int curX;
	int curY;
	String inString;
	String outString;
	int inBufferval= cNCSimCtrlIfaceMachine_get_inBuffer(&cncsimctrl);
	int outBufferVal= cNCSimCtrlIfaceMachine_get_outBuffer(&cncsimctrl);
	inString = String(inBufferval)+ "/" + String(cNCSimCtrlIfaceMachine_get_inBufferLimit(&cncsimctrl));
	lcd.setTextColor(AMBER, BLACK);
	lcd.setCursor(5, 127-(60+40)+10);
	curX = lcd.getCursorX();
	curY = lcd.getCursorY();
	lcd.fillRect(0, curY-9, 127, 11, BLACK);
	lcd.print(inString+"  ");
	outString = String(outBufferVal)+ "/" + String(cNCSimCtrlIfaceMachine_get_outBufferLimit(&cncsimctrl));
	lcd.setTextColor(AMBER, BLACK);
	lcd.setCursor(85, 127-(60+40)+10);
	lcd.print(outString+"  ");

	lcd.setCursor(0, 0);
	lcd.fillRect(5, 127-80,   20,    40, AMBER);
	lcd.fillRect(105, 127-80,   20,    40, AMBER);
	lcd.fillRect(105+2, 127-(80-2),   16,    36-ceil(3.6*outBufferVal), BLACK);
	lcd.fillRect(105+2, 127-(42+ceil(3.6*outBufferVal)),   16,    ceil(3.6*outBufferVal), RED);
	lcd.fillRect(5+2, 127-(80-2),   16,    36-ceil(3.6*inBufferval), BLACK);
	lcd.fillRect(5+2, 127-(42+ceil(3.6*inBufferval)),   16,    ceil(3.6*inBufferval), RED);
}
void cNCSimCtrlIface_synchronize(const CNCSimCtrl *handle)
{
	//Serial.println("Synch");
}

void my_synchronize_LCD(const CNCSimCtrl *handle) {
	String inString;
	String outString;

	int currentState = cNCSimCtrlIface_get_myState(handle);
	int STEPNUM;
	float STEPSZ;
	int boxHeight;
	int curX;
	int curY;

	if(currentState==0)
	{
		if (State_0_Flag) {
			lcd.fillScreen(BLACK);
			lcd.setTextColor(WHITE, BLACK);
			lcd.setCursor(0, 15);
			lcd.setTextColor(YELLOW, BLACK);
			lcd.println("The Factory In A Box");
			lcd.setTextColor(BLUE, BLACK);
			lcd.println("       A project of:   ");
			lcd.setTextColor(CYAN, BLACK);
			lcd.println("      Sutton Tools,   ");
			lcd.println("     DMTC, AMGC,     ");
			lcd.println("     RMIT and UQ     ");
			lcd.setTextColor(GREEN, BLACK);
			lcd.println("      By:S.Dowey");
			lcd.println("      & A.R..Sadri");
		}
		State_0_Flag=0;

		State_0_counter++;
		if (State_0_counter==numStepsCircleTurning) {
			State_0_counter = 0;
		}
		//lcd.fillCircle(64+50*sin((State_0_counter-3)*2*3.14159/numStepsCircleTurning), 64+50*cos((State_0_counter-3)*2*3.14159/numStepsCircleTurning), 5, BLACK);
		//lcd.fillCircle(64+50*sin(State_0_counter*2*3.14159/numStepsCircleTurning), 64+50*cos(State_0_counter*2*3.14159/numStepsCircleTurning), 5, WHITE);
	}
	else
		State_0_Flag=1;

	if(currentState==2)
	{
		if (State_2_Flag) {
			lcd.fillScreen(BLACK);
			lcd.setTextColor(BLUE, BLACK);
			lcd.setCursor(0, 10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, BLACK);
			lcd.print("            STOP      ");
			lcd.drawLine(10+20, 127-34, 92, 127-34, WHITE);
			lcd.drawLine(10+20, 127-14, 92, 127-14, WHITE);
			lcd.drawCircle(10+20, 127-24, 10, YELLOW);
			lcd.drawCircle(92, 127-24, 10, YELLOW);
			lcd.drawLine(10+20, 127-14, 10+20, 127-34, YELLOW);
			lcd.drawLine(92, 127-14, 92, 127-34, YELLOW);
		}
		State_2_Flag=0;

		if(cNCSimCtrlIfaceMachine_get_bufferUpdate(handle)==true) {
			cNCSimCtrlIfaceMachine_set_bufferUpdate(&cncsimctrl, false);
			UpdateBuffersOnLCD();
		}
	}
	else
		State_2_Flag=1;

	if(currentState==3)
	{
		if (State_3_Flag) {
			//lcd.fillScreen(BLACK);
			lcd.setTextColor(GREEN, BLACK);
			lcd.setCursor(5, 10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, BLACK);
			lcd.print("           RUN       ");
			lcd.drawLine(10+20, 127-34, 92, 127-34, WHITE);
			lcd.drawLine(10+20, 127-14, 92, 127-14, WHITE);
			lcd.drawCircle(10+20, 127-24, 10, YELLOW);
			lcd.drawCircle(92, 127-24, 10, YELLOW);
			lcd.drawLine(10+20, 127-14, 10+20, 127-34, YELLOW);
			lcd.drawLine(92, 127-14, 92, 127-34, YELLOW);
			State_32_counter=0;
		}
		State_3_Flag=0;

		if(cNCSimCtrlIfaceMachine_get_bufferUpdate(handle)==true) {
			cNCSimCtrlIfaceMachine_set_bufferUpdate(&cncsimctrl, false);
			UpdateBuffersOnLCD();
		}

		STEPNUM = floor(cNCSimCtrlIface_get_busyTime(handle)/LCDUpdate_PERIOD)-1;
		if (STEPNUM<1)
			STEPNUM = 1;
		STEPSZ = 40.0/STEPNUM;
		boxHeight = 20-floor(State_32_counter*STEPSZ/3);
		if (cNCSimCtrlIfaceMachine_get_cycle(handle)) {
			if (State_32_counter<=STEPNUM) {
				if (State_32_counter>0)
					lcd.fillRect(5+25+floor((State_32_counter-1)*STEPSZ), 127-40-20,   20,    20, BLACK);
				lcd.fillRect(5+25+floor((State_32_counter)*STEPSZ), 127-(40+boxHeight),   20,    boxHeight, RED);
				State_32_counter++;
			}
		}
		else if (State_32_counter>0){
			lcd.fillRect(5+25+floor((State_32_counter-1)*STEPSZ), 127-40-20,   20,    20, BLACK);
			State_32_counter=0;
		}

		State_3_counter++;
		if (State_3_counter==24)
			State_3_counter = 0;
		lcd.fillCircle(10+20, 127-24, 9, BLACK);
		lcd.fillCircle(92, 127-24, 9, BLACK);
		lcd.drawLine(10+20 + 10*sin(State_3_counter*0.2617), 127-(24+ 10*cos(State_3_counter*0.2617)), 10+20+ 10*sin(3.1415+State_3_counter*0.2617), 127-( 24+ 10*cos(3.1415+State_3_counter*0.2617)), YELLOW);
		lcd.drawLine(92 + 10*sin(State_3_counter*0.2617), 127-(24+ 10*cos(State_3_counter*0.2617)), 92+ 10*sin(3.1415+State_3_counter*0.2617), 127-(24+ 10*cos(3.1415+State_3_counter*0.2617)), YELLOW);
	}
	else
		State_3_Flag=1;

	if(currentState==4)
	{
		if(cNCSimCtrlIfaceMachine_get_bufferUpdate(handle)==true) {
			cNCSimCtrlIfaceMachine_set_bufferUpdate(&cncsimctrl, false);
			UpdateBuffersOnLCD();
		}

		State_4_counter++;
		if (State_4_counter==4) {
			lcd.setCursor(0, 10);
			lcd.setTextColor(YELLOW, BLACK);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, BLACK);
			lcd.print("         Warning     ");
		}
		if (State_4_counter==8) {
			State_4_counter=0;
			lcd.setTextColor(BLACK, YELLOW);
			lcd.setCursor(0, 10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, YELLOW);
			lcd.print("         Warning     ");
		}
		if (State_4_counter>8)
			State_4_counter=0;
	}

	if(currentState==42)
	{
		if (State_42_Flag) {
			lcd.setTextColor(YELLOW, BLACK);
			lcd.setCursor(0, 10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, BLACK);
			lcd.print("         Warning     ");
		}
		State_42_Flag=0;

		if(cNCSimCtrlIfaceMachine_get_bufferUpdate(handle)==true) {
			cNCSimCtrlIfaceMachine_set_bufferUpdate(&cncsimctrl, false);
			UpdateBuffersOnLCD();
		}
	}
	else
		State_42_Flag=1;


	if(currentState==5)
	{
		if (State_5_Flag) {
			lcd.fillScreen(BLACK);
		}
		State_5_Flag=0;
		State_5_counter++;
		if (State_5_counter==4) {
			lcd.setTextColor(RED, YELLOW);
			lcd.setCursor(0, 55+10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, YELLOW);
			lcd.print("          ALARM      ");
		}
		if (State_5_counter==8) {
			State_5_counter=0;
			lcd.setTextColor(YELLOW, RED);
			lcd.setCursor(0, 55+10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, RED);
			lcd.print("          ALARM      ");
		}
		if (State_5_counter>8)
			State_5_counter=0;
	}
	else
		State_5_Flag=1;

	if(currentState==52) {
		if (State_52_Flag==1) {
			State_52_Flag = 0;
			lcd.setTextColor(RED, BLACK);
			lcd.setCursor(0, 55+10);
			curX = lcd.getCursorX();
			curY = lcd.getCursorY();
			lcd.fillRect(0, curY-9, 127, 11, BLACK);
			lcd.print("          ALARM      ");
		}
	}
	else
		State_52_Flag=1;
}

void my_synchronize(const CNCSimCtrl *handle) {
	String tmp;
	int curX;
	int curY;
	int currentState = cNCSimCtrlIface_get_myState(handle);

	if (currentState==1) {
		if(cNCSimCtrlIface_get_clearPage(handle)) {
			lcd.fillScreen(BLACK);
			cNCSimCtrlIface_set_clearPage(&cncsimctrl, false);
		}
		if(cNCSimCtrlIface_get_clearPage2(handle)) {
			lcd.fillScreen(BLACK);
			cNCSimCtrlIface_set_clearPage2(&cncsimctrl, false);
		}
		lcd.setCursor(0, 10);

		setMenuSel = cNCSimCtrlIface_get_menuSel(handle);
		if(setMenuSel != 0) {
			cNCSimCtrlIface_set_menuSel(&cncsimctrl, 0);
			if (setMenuSel<=4) {
				lcd.setTextColor(GREEN, BLACK);
				lcd.println("         Timings");
				lcd.drawLine(0, 14, 127, 14, GREEN);

				if (setMenuSel==1) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Idle: ");
				tmp  = String((handle->iface.idleTime)/1000.0);
				tmp += "s    ";
				lcd.println(tmp);

				if (setMenuSel==2) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Busy: ");
				tmp  = String((handle->iface.busyTime)/1000.0);
				tmp += "s     ";
				lcd.println(tmp);

				if (setMenuSel==3) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Repair: ");
				tmp  = String((handle->iface.repairTime)/1000.0);
				tmp += "s     ";
				lcd.println(tmp);

				if (setMenuSel==4) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Response:");
				tmp  = String((handle->iface.responseTime)/1000.0);
				tmp += "s     ";
				lcd.println(tmp);
			}
			else if(setMenuSel>4 && setMenuSel<9) {
				lcd.setTextColor(GREEN, BLACK);
				lcd.println("     Other Settings");
				lcd.drawLine(0, 14, 127, 14, GREEN);

				if (setMenuSel==5) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Reliability:");
				tmp  = String((handle->iface.reliability)/10);
				tmp += "%      ";
				lcd.println(tmp);

				if (setMenuSel==6) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Simulation: ");
				if(handle->iface.sim==true)
					tmp  = "Yes";
				else
					tmp = "No";
				tmp += "       ";
				lcd.println(tmp);

				if (setMenuSel==7) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Inf Input: ");
				if(handle->iface.infiniteInput==true)
					tmp  = "Yes";
				else
					tmp = "No";
				tmp += "       ";
				lcd.println(tmp);

				if (setMenuSel==8) {
					curX = lcd.getCursorX();
					curY = lcd.getCursorY();
					lcd.fillRect(0, curY-9, 127, 11, BLACK);
					lcd.setTextColor( YELLOW,  BLACK);
				}
				else
					lcd.setTextColor(WHITE, BLACK);
				lcd.print(" Inf Output: ");
				if(handle->iface.infiniteOutput==true)
					tmp  = "Yes";
				else
					tmp = "No";
				tmp += "       ";
				lcd.println(tmp);
			}
			else {
				lcd.setTextColor(GREEN, BLACK);
				lcd.println("           INFO       ");
				lcd.drawLine(0, 14, 127, 14, GREEN);
				lcd.setTextColor(WHITE, BLACK);
				lcd.println("                  ");
				lcd.println("  CNC Machine Sim");
				lcd.println("      Software v2.1");
				lcd.setTextColor(YELLOW, BLACK);
				lcd.println("  CPU: STM32F103");
				lcd.println("    ARM Cortex M3");
			}

		}
	}


	if (currentState==101) {
		if(State_101_Flag) {
			lcd.fillScreen(BLACK);
			presetMenuSel_perv = 0;
		}
		State_101_Flag=0;
		String tmp;
		presetMenuSel = cNCSimCtrlIface_get_presetMenuSel(handle);

		if(presetMenuSel != presetMenuSel_perv) {
			presetMenuSel_perv = presetMenuSel;
			lcd.setCursor(0, 10);

			lcd.setTextColor(GREEN, BLACK);
			lcd.println("          Presets");
			lcd.drawLine(0, 14, 127, 14, GREEN);


			if (presetMenuSel==1)
				lcd.setTextColor( YELLOW,  RED);
			else
				lcd.setTextColor(WHITE, BLACK);
			lcd.println("Preset: Slow");
			if (presetMenuSel==2)
				lcd.setTextColor( YELLOW,  RED);
			else
				lcd.setTextColor(WHITE, BLACK);
			lcd.println("Preset: Medium");
			if (presetMenuSel==3)
				lcd.setTextColor( YELLOW,  RED);
			else
				lcd.setTextColor(WHITE, BLACK);
			lcd.println("Preset: Fast");
			if (presetMenuSel==4)
				lcd.setTextColor( YELLOW,  RED);
			else
				lcd.setTextColor(WHITE, BLACK);
			lcd.println("Preset: Chaotic");
		}
	}
	else
		State_101_Flag=1;


	digitalWrite(led_white_pin, cNCSimCtrlIfaceStackLight_get_white(handle));
	digitalWrite(led_green_pin, cNCSimCtrlIfaceStackLight_get_green(handle));
	digitalWrite(led_yellow_pin, cNCSimCtrlIfaceStackLight_get_amber(handle));
	digitalWrite(led_red_pin, cNCSimCtrlIfaceStackLight_get_red(handle));
	digitalWrite(rly_cycle_pin, !cNCSimCtrlIfaceMachine_get_cycle(handle));
	delay(1);
	digitalWrite(rly_run_pin, !cNCSimCtrlIfaceMachine_get_run(handle));

	///////////////////////////////////////////////////////////////////////
	// For input buffer, we need to toggle the channel
	digitalWrite(Machine_Select_1, cNCSimCtrlIface_get_inputMachineNumber(handle));
	digitalWrite(Machine_Select_2, cNCSimCtrlIface_get_outputMachineNumber(handle));
	delay(1);
 	////////////////////////////////////////////////////////////////////
	int c;
	if(cNCSimCtrlIface_get_materialRequired_byC(handle)==true) {
		digitalWrite(Serial2_TX_Select_Pin,HIGH);
		delay(1);
		c = cNCSimCtrlIface_get_requestID(handle);
		Serial2.write(c);
		Serial2.flush();
		digitalWrite(Serial2_TX_Select_Pin,LOW);
		cNCSimCtrlIface_set_materialRequired_byC(&cncsimctrl, false);
	}
	if(cNCSimCtrlIface_get_waitingForHandShake(handle)==true) {
		if (Serial2.available()) {
			c = Serial2.read();
			if (c>0) {
				cNCSimCtrlIface_set_materialID(&cncsimctrl,c);
				cNCSimCtrlIface_set_waitingForHandShake(&cncsimctrl,false);
			}
		}
	}

	if(cNCSimCtrlIface_get_materialObtained_byC(handle)==true) {
		cNCSimCtrlIface_set_materialObtained_byC(&cncsimctrl,false);
	}

	//////////////////////////////////////////////////////////////////////
	if(cNCSimCtrlIface_get_waitingForRequest(handle)==true) {
		if (Serial3.available()) {
			c = Serial3.read();
			if(c>0) {
				cNCSimCtrlIface_set_productReqID(&cncsimctrl,c);
				cNCSimCtrlIface_set_waitingForRequest(&cncsimctrl,false);
			}
		}
	}
	if(cNCSimCtrlIface_get_productDelivered_byC(handle)==false) {
		digitalWrite(Serial3_TX_Select_Pin,HIGH);
		delay(1);
		c = cNCSimCtrlIface_get_productReqID(handle);
		Serial3.write(c);
		Serial3.flush();
		digitalWrite(Serial3_TX_Select_Pin,LOW);
		cNCSimCtrlIface_set_productDelivered_byC(&cncsimctrl,true);

	}
	if(cNCSimCtrlIface_get_productSent_byC(handle)==true) {
		cNCSimCtrlIface_set_productSent_byC(&cncsimctrl,false);
	}

}
