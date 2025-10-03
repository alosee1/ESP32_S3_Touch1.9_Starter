#include "lcd_bsp.h"
#include "lcd_bl_pwm_bsp.h"
#include "ui.h"
#include "Display_ST7789.h"
#include "LVGL_Driver.h"
#include "ui.h"
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <EEPROM.h>  //Needed to record user settings
#include <ModbusIP_ESP8266.h>
#include <TimeOut.h>
#include <FastLED_NeoPixel.h>


 // Which pin on the Arduino is connected to the LEDs?
#define DATA_PIN 42

// How many LEDs are attached to the Arduino?
#define NUM_LEDS 10

// How many NeoPixels are attached to the Arduino?
//#define LED_COUNT 10

// LED brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50

Adafruit_NeoPixel strip(NUM_LEDS, DATA_PIN, NEO_GRB + NEO_KHZ800);

TimeOut timeout0;

//EEPROM locations to store 4-byte variables
#define EEPROM_SIZE 100      //Allocate 100 bytes of EEPROM
#define LOCATION_ADDRESS 0     //Float, requires 4 bytes of EEPROM


#define BAR_RETRACT 40640.0 * .4  //0.6 Inches
#define STEP_TOTAL 5
char buffer[80];



int option;
int stallThreshold = 20;
int stallThreshold2 = 20;
int interruptCount = 0;
bool diagFlagX = 0;
bool diagFlagZ = 0;

bool flagStartMeasureBeam = 0;
bool flaggotoBeamHome = 0;

//Beam Positions
	int topBarPosition = 0;

	int topBeamPosition = 8128.0 * -0.15;
	int topBeamPosition2 = 8128.0 * -0.2;

	int bottomBeamPosition = 8128.0 *(-.60);
	int bottomBeamPosition2 = 8128.0 *(-.65);

	int bottomBarPosition = 8128.0 * (-.8);

enum motion { INVALID,
              HOMING,
              STANDBY,
              SCANNING,
              ERROR
            };


typedef enum { STATE_STANDBY,
               STATE_HOMEING,   //X and Z stepper are in the process of homing
               STATE_HOME,     //X and Z steppers are homed
               STATE_SEEKING,   //Seeking the location to start measuring the Beam
               STATE_READY,     //AT Local Beam Measuting position
               STATE_MEASURE,
               STATE_HOMING_ERROR,
               STATE_CHECK_INDICATOR,
               STATE_SOMTHING
             } state;


typedef enum { BEAMFAMILY_NONE,
              BEAMFAMILY_C01,
              BEAMFAMILY_C2,
              BEAMFAMILY_TR1,
              BEAMFAMILY_TR2
            } beamFamily;

//Modbus Registers Offsets
const int LED_COIL = 100;
const int HREG_STATUS = 101;
const int HREG_SPEC = 102;
const int HREG_PASS_FAIL = 103;
const int HREG_FAMILY = 104;
const int HREG_BOT = 105;




int sysState = STATE_STANDBY;

// current state-machine state
state Xstate = STATE_STANDBY;
state Zstate = STATE_STANDBY;



// Mitutoyo SPC
#define HUMAN_READABLE_OUTPUT

//#define HUMAN_READABLE_DEBUG

//#define BEAM_SIZE_DEBUG
int req = 41;  //Arduino pin for REQ line, drives transistor to ground SPC port pin 5 (~REQ)
int dat = 39;  //Arduino pin for Data line, connects directly to SPC port pin 2 (DATA)
int clk = 40;  //Arduino pin for Clock line, connects directly to SPC port pin 3 (~CLK)
float indicatorValue = 0;
float homeOffset = 0;



#define STALL_VALUE 50  // [0..255]
#define EN_PIN 16       // Enable
#define EN_PIN2 7       // Enable
#define DIAG_PIN2 3     // DIAG PIN FOR HOMEING

#define DIR_PIN 18   // Direction
#define STEP_PIN 17  // Step
#define DIAG_PIN 15  // DIAG PIN FOR HOMEING

#define DIR_PIN2 6   // Direction
#define STEP_PIN2 5  // Step

//#define SW_RX            0 // TMC2208/TMC2224 SoftwareSerial receive pin
//#define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial0   // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00   // TMC2209 Driver address according to MS1 and MS2
#define DRIVER2_ADDRESS 0b01  // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f  // Match to your driver


AccelStepper stepperX(AccelStepper::FULL2WIRE, STEP_PIN, DIR_PIN);
AccelStepper stepperZ(AccelStepper::FULL2WIRE, STEP_PIN2, DIR_PIN2);
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&SERIAL_PORT, R_SENSE, DRIVER2_ADDRESS);

//ModbusIP object
ModbusIP mb;

struct MySettings {
	int xHomeLocation_C0;
	int zHomeLocation_C0;
	int xHomeLocation_C2;
	int zHomeLocation_C2;
	int xHomeLocation_TR1;
	int zHomeLocation_TR1;
	int xHomeLocation_TR2;
	int zHomeLocation_TR2;
	int homeAtStartup;
};

MySettings settings;


//Fucntion Prototypes
void task(void *);
void processState();
void handleRisingEdge();
void handleRisingEdge2();
void moveDown();
void moveLeft();
void moveRight();
void handleRisingEdge();
void stopAllMotors(lv_event_t *e);
void moveXnbLoop(int pos);
void moveXnb(int pos);
void moveZnb(int pos);
void homeXaxisAT();
void homeYaxisAT();
void homeSteppers();
void homeBeamFamily();
float measure();
void meaureBeam();
void homeServo();
void runMotor();
void beamSelectChangeModbus();
void moveUp();
float readIndicator();
void handleScreenUpdate();
void measureBeam();
void colorWipe(uint32_t color, unsigned long wait);

	void setup() {

  strip.begin();  // initialize strip (required!)
	strip.setBrightness(BRIGHTNESS);


	EEPROM.begin(EEPROM_SIZE);  //Some platforms need this. Comment this line if needed

	//Look up the Home Offsets
	EEPROM.get(LOCATION_ADDRESS, settings);
	if (settings.xHomeLocation_C0 == 0xFFFFFFFF) {
		settings.xHomeLocation_C0 = 0;
		settings.zHomeLocation_C0 = 0;
		settings.xHomeLocation_C2 = 0;
		settings.zHomeLocation_C2 = 0;
		settings.xHomeLocation_TR1 = 0;
		settings.zHomeLocation_TR1 = 0;
		settings.xHomeLocation_TR2 = 0;
		settings.zHomeLocation_TR2 = 0;
		settings.homeAtStartup = 0;
		EEPROM.put(LOCATION_ADDRESS, settings);
		EEPROM.commit();
	}


	// Mitutoyo SPC
	pinMode(req, OUTPUT);
	pinMode(clk, INPUT_PULLUP);
	pinMode(dat, INPUT_PULLUP);
	digitalWrite(req, LOW);  // set request at high


	Serial.begin(115200);


	lcd_lvgl_Init();
	setUpdutySubdivide(LCD_GPIO_MODE, LCD_GPIO_MODE_ON);  //LCD Backlight related
	ui_init();


	WiFi.begin("Corp", "absoluteFumble");


	Serial.print("My IP address: ");
	Serial.println(WiFi.localIP()); // Print the assigned IP address

	//Setup Modbus Server
	mb.server();
	mb.addHreg(HREG_STATUS, 0);
	mb.addHreg(HREG_PASS_FAIL, 2);
	mb.addHreg(HREG_SPEC, 0x0000);
	mb.addHreg(HREG_FAMILY,0);
	mb.addCoil(LED_COIL);


	//Stepper startup
	SERIAL_PORT.begin(115200);  // HW UART drivers
	driver.begin();
	driver.toff(4);
	driver.blank_time(24);
	driver.rms_current(800);  // mA
	driver.microsteps(2);
	driver.TCOOLTHRS(0xFFFFF);  // 20bit max
	driver.semin(5);
	driver.semax(2);
	driver.sedn(0b01);
	driver.SGTHRS(stallThreshold);


	//Stepper startup Driver 2
	driver2.begin();
	driver2.toff(4);
	driver2.blank_time(24);
	driver2.rms_current(800);  // mA
	driver2.microsteps(2);
	driver2.TCOOLTHRS(0xFFFFF);  // 20bit max
	driver2.semin(5);
	driver2.semax(2);
	driver2.sedn(0b01);
	driver2.SGTHRS(stallThreshold2);

	//Home the stepper and set the zero
	stepperX.setCurrentPosition(0);
	stepperX.setPinsInverted(false, false, true);
	stepperX.setMaxSpeed(5000.0);
	stepperX.setAcceleration(10000.0);

	//Home the stepper and set the zero
	stepperZ.setCurrentPosition(0);
	stepperZ.setPinsInverted(true, false, true);
	stepperZ.setMaxSpeed(5000.0);
	stepperZ.setAcceleration(10000.0);

//Incase the steppers are moving home - kill on reset. 
driver.VACTUAL(0);
driver2.VACTUAL(0);

	pinMode(EN_PIN, OUTPUT);
	digitalWrite(EN_PIN, 0);

	pinMode(EN_PIN2, OUTPUT);
	digitalWrite(EN_PIN2, 0);


	pinMode(DIAG_PIN, INPUT);  // Use INPUT_PULLUP for a stable signal
	attachInterrupt(digitalPinToInterrupt(DIAG_PIN), handleRisingEdge, RISING);
	attachInterrupt(digitalPinToInterrupt(DIAG_PIN2), handleRisingEdge2, RISING);


	lv_label_set_text_fmt(ui_Label2, "STANDBY");
	lv_label_set_text_fmt(ui_LabelResult, "----");

	Serial.println("Setup Done...");




	//Attempting to pin the modbus and screen update task to core zero
	xTaskCreatePinnedToCore(
	    task,        //Function to implement the task
	    "task",  //Name of the task
	    6000,        //Stack size in words
	    NULL,        //Task input parameter
	    0,           //Priority of the task
	    NULL,        //Task handle.
	    0);          //Core where the task should run





	if(settings.homeAtStartup == 1)
		lv_obj_add_state(ui_Switch2,LV_STATE_CHECKED);


	if(settings.homeAtStartup == true)
	{
		homeSteppers();
		lv_label_set_text_fmt(ui_Label2, "HOMEING");
		sysState = STATE_HOMEING;
	}


}




void loop() {

	//theaterChase(strip.Color(255, 255, 255), 250, 3, 5);  // 


	//colorWipe(strip.Color(255, 255, 255), 1);  // white

  static int family = 0;
	processState();
  TimeOut::handler();


	//Check WiFi
	if (WiFi.status() != WL_CONNECTED)
	{
		lv_obj_add_flag(ui_Image6, LV_OBJ_FLAG_HIDDEN);
	}
	else
  {
		lv_obj_clear_flag(ui_Image6, LV_OBJ_FLAG_HIDDEN);
    //Serial.print("My IP address: ");
		//Serial.println(WiFi.localIP()); // Print the assigned IP address
  }


  if(family != mb.Hreg(HREG_FAMILY))   //Beam family change by modbus
{
    family =  mb.Hreg(HREG_FAMILY);
    option =  mb.Hreg(HREG_FAMILY);
    Serial.println(option);
    lv_dropdown_set_selected(ui_Dropdown2,option);
    beamSelectChangeModbus();
}



	if (mb.Coil(LED_COIL) || flagStartMeasureBeam)
	{
		flagStartMeasureBeam = 0;
		mb.Coil(LED_COIL, 0);
		mb.Hreg(HREG_PASS_FAIL, 2);
	  colorWipe(strip.Color(0, 0, 255), 1);  // GREEN

    if(mb.Hreg(HREG_FAMILY) == BEAMFAMILY_C01 && sysState == STATE_READY)
		{
      topBeamPosition = 8128.0 * -0.15;
      topBeamPosition2 = 8128.0 * -0.2;
      bottomBeamPosition = 8128.0 *(-.60);
      bottomBeamPosition2 = 8128.0 *(-.65);
      bottomBarPosition = 8128.0 * (-.8);
			measureBeam();
			sysState == STATE_READY;
		}
    if(mb.Hreg(HREG_FAMILY) == BEAMFAMILY_C2 && sysState == STATE_READY)
		{
      topBeamPosition = 8128.0 * -0.15;
      topBeamPosition2 = 8128.0 * -0.2;
      bottomBeamPosition = 8128.0 *(-.85);
      bottomBeamPosition2 = 8128.0 *(-.9);
      bottomBarPosition = 8128.0 * (-1.05);
			measureBeam();
			sysState == STATE_READY;
		}
    if(mb.Hreg(HREG_FAMILY) == BEAMFAMILY_TR1 && sysState == STATE_READY)
		{
      topBeamPosition = 8128.0 * -0.15;
      topBeamPosition2 = 8128.0 * -0.2;
      bottomBeamPosition = 8128.0 *(-.577);
      bottomBeamPosition2 = 8128.0 *(-.627);
      bottomBarPosition = 8128.0 * (-.777);
			measureBeam();
			sysState == STATE_READY;
		}
    if(mb.Hreg(HREG_FAMILY) == BEAMFAMILY_TR2 && sysState == STATE_READY)
		{
      topBeamPosition = 8128.0 * -0.15;
      topBeamPosition2 = 8128.0 * -0.2;
      bottomBeamPosition = 8128.0 *(-.85);
      bottomBeamPosition2 = 8128.0 *(-.9);
      bottomBarPosition = 8128.0 * (-1.05);
			measureBeam();
			sysState == STATE_READY;
		}
	}


	//Flag set in one thread and then executed in another
	if(flaggotoBeamHome == 1)
	{
		if (sysState == STATE_HOME )
		{
      flaggotoBeamHome = 0;
      Serial.println("Home Beam Family");
      //delay(1000);

      //Incase the steppers are moving home - kill on reset. 
driver.VACTUAL(0);
driver2.VACTUAL(0);

			homeBeamFamily();
		}
	}


	//lv_timer_handler_run_in_period(100);

	if (Serial.available()) {
		byte incoming = Serial.read();

		if (incoming == 'w')
			moveUp();

		if (incoming == 's')
			moveDown();

		if (incoming == 'd')
			moveRight();

		if (incoming == 'a')
			moveLeft();

		if (incoming == 'm')
			measure();

		if (incoming == 'h')
			homeServo();

    if (incoming == 't')
          sysState = STATE_CHECK_INDICATOR;

		if (incoming == 'r')
    {
      	Serial.print("Distance: ");
	      float distInInches = (readIndicator() - homeOffset);
	      Serial.println(distInInches, 4);
    }
		

    if (incoming == 'i')
      Serial.println(WiFi.localIP());




		//if (incoming == 'b')
		//meaureBeam();

	}
	runMotor();
}

void blank(unsigned long wait) {
	strip.clear();
	strip.show();
	delay(wait);
}

void blinkError(uint32_t color, unsigned long wait) {

 static unsigned long lastupdate; 
 static bool toggle = 0;

 if(millis() - lastupdate > wait){
      toggle = !toggle;
      if (toggle) {
        for (unsigned int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, color);
          strip.show();
        }
      } else {
	strip.clear();
	strip.show();
      }

			lastupdate = millis();
 }
  
}


void colorWipe(uint32_t color, unsigned long wait) {
	for (unsigned int i = 0; i < strip.numPixels(); i++) {
		strip.setPixelColor(i, color);
		strip.show();
		delay(wait);
	}
}

void callback0()
{
  //Homeing has timmed out - flag error
  Serial.println("timeout callback");
  driver.VACTUAL(0);
  driver2.VACTUAL(0);
  sysState = STATE_HOMING_ERROR;

}


void processState() {
	switch (sysState) {

	case STATE_STANDBY:
      	mb.Hreg(HREG_STATUS, INVALID);
         
		break;

	case STATE_HOMEING:
    //theaterChase(strip.Color(0, 255, 255), 250, 3, 5);  // 
    //colorWipe(strip.Color(255, 255, 255), 1);  // BLUE
    //colorWipe(strip.Color(0, 255, 255), 1);  // ?
		if(diagFlagX == 1 && diagFlagZ == 1)
    {
			sysState = STATE_HOME;
      timeout0.cancel();
      delay(500);
    }
        mb.Hreg(HREG_STATUS, HOMING);
    break;


	case STATE_SEEKING:
		//
    //theaterChase(strip.Color(25, 0, 255), 250, 3, 5);  // 
    driver.VACTUAL(0);
    driver2.VACTUAL(0);

    //Wait for Both axis to get to locatoin of Beam Family
		if (Xstate == STATE_STANDBY &&  Zstate == STATE_STANDBY){
      //delay(500);
    	sysState = STATE_READY;
    }
    mb.Hreg(HREG_STATUS, INVALID);
		//Serial.println(Xstate);
		//Serial.print(Zstate);

		break;


	case STATE_MEASURE:
    //theaterChase(strip.Color(0, 0, 255), 250, 3, 5);  // 
    
    //colorWipe(strip.Color(0, 0, 255), 1);  // BLUE
		break;

	case STATE_HOME:
           timeout0.cancel();
         //theaterChase(strip.Color(255, 255, 255), 250, 3, 5);  // 
        	//mb.Hreg(HREG_STATUS, INVALID);
		break;

	case STATE_READY:
        	mb.Hreg(HREG_STATUS, STANDBY);
		break;



	default:
		break;

	}


}



void task(void*) {

	for( ;; )
	{
		mb.task();
		lv_timer_handler_run_in_period(20);
		handleScreenUpdate();
	}

}



void handleScreenUpdate()
{

	static unsigned long timer = 0;
	unsigned long interval = 50;
	if (millis() - timer >= interval) {
		timer = millis();
		//Update Diag screen info
		if (diagFlagX == true)
			lv_obj_clear_flag(ui_Label11, LV_OBJ_FLAG_HIDDEN);
		else
			lv_obj_add_flag(ui_Label11, LV_OBJ_FLAG_HIDDEN);

		if (diagFlagX == true)
			lv_obj_clear_flag(ui_LabelStall, LV_OBJ_FLAG_HIDDEN);
		else
			lv_obj_add_flag(ui_LabelStall, LV_OBJ_FLAG_HIDDEN);
		if (diagFlagZ == true)
			lv_obj_clear_flag(ui_LabelStall1, LV_OBJ_FLAG_HIDDEN);
		else
			lv_obj_add_flag(ui_LabelStall1, LV_OBJ_FLAG_HIDDEN);

		float spec = (int)mb.Hreg(HREG_SPEC);
		sprintf(buffer, "%.1f", spec);
		lv_label_set_text(ui_LabelSpec, buffer);

		//Print the current X,Z position for Servo Screen
		sprintf(buffer, "%d", stepperX.currentPosition());
		lv_label_set_text_fmt(ui_LabelXPos, buffer);

		sprintf(buffer, "%d", stepperZ.currentPosition());
		lv_label_set_text_fmt(ui_LabelZPos, buffer);
    

		//Display the position of the selected Beam
		switch (option) {
		case 0:
			sprintf(buffer, "%d", settings.xHomeLocation_C0);
			lv_label_set_text_fmt(ui_LabelXHome, buffer);

			sprintf(buffer, "%d", settings.zHomeLocation_C0);
			lv_label_set_text_fmt(ui_LabelZHome, buffer);
			break;

		case 1:
			sprintf(buffer, "%d", settings.xHomeLocation_C2);
			lv_label_set_text_fmt(ui_LabelXHome, buffer);

			sprintf(buffer, "%d", settings.zHomeLocation_C2);
			lv_label_set_text_fmt(ui_LabelZHome, buffer);

			break;

		default:
			break;

		}


		switch (sysState) {
		case STATE_STANDBY:
			lv_label_set_text_fmt(ui_Label2, "STAnDBY");
			break;

		case STATE_HOMEING:
			lv_label_set_text_fmt(ui_Label2, "HOMEING");
      blinkError(strip.Color(50, 255, 255), 100);  //
			break;

		case STATE_HOME:
			lv_label_set_text_fmt(ui_Label2, "HOME");
      colorWipe(strip.Color(50, 255, 255), 1);  // GREEN
			break;

		case STATE_SEEKING:
			lv_label_set_text_fmt(ui_Label2, "SEEKING");
      blinkError(strip.Color(0, 0, 250), 250);  // ?
      //colorWipe(strip.Color(0, 255, 0), 1);  // GREEN
			break;

		case STATE_READY:
			lv_label_set_text_fmt(ui_Label2, "READY");
			break;

		case STATE_MEASURE:
			lv_label_set_text_fmt(ui_Label2, "MEASURE");
			break;

		case STATE_HOMING_ERROR:
			lv_label_set_text_fmt(ui_Label2, "HM ERR");
      blinkError(strip.Color(255, 0, 0), 250);  //
			break;

    case STATE_CHECK_INDICATOR:
			lv_label_set_text_fmt(ui_Label2, "CHK IND");
      blinkError(strip.Color(255, 0, 0), 250);  //
			break;


		default:
    			lv_label_set_text_fmt(ui_Label2, "??????");

			break;

		}




	}

}



void measureBeam() {
  int error = 0;
	sysState = STATE_MEASURE;

	float topBar, botBar, beamTop, beamTop2, beamMiddle, beamBottom, beamBottom2;


	mb.Coil(LED_COIL, 0);
	mb.Hreg(HREG_PASS_FAIL, 2);
	mb.Hreg(HREG_STATUS, SCANNING);
	//mb.task();


	lv_bar_set_value(ui_Bar1, map(1, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);
	lv_label_set_text(ui_LabelPF, "----");
	lv_obj_set_style_text_color(ui_LabelPF, lv_color_hex(0x6C6C6C), LV_PART_MAIN | LV_STATE_DEFAULT);  //Gray


	moveXnb(0);

	stepperZ.move(-BAR_RETRACT);  //Move relative so we are Touchung bar
	stepperZ.runToPosition();
	homeServo();
  if (homeOffset == -1)
      error++;

	//stepperX.moveTo(topBarPosition);  //Move to start of Beam Scan
	topBar = 0;

	lv_bar_set_value(ui_Bar1, map(1, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);
	//Timer_Loop();
	stepperX.moveTo(topBeamPosition);  //Move to start of Beam Scan
	stepperX.runToPosition();
	
  beamTop = measure();
  if (beamTop == -1)
      error++;


	lv_bar_set_value(ui_Bar1, map(2, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);

	stepperX.moveTo(topBeamPosition2);  //Move to start of Beam Scan
	stepperX.runToPosition();
	beamTop2 = measure();
  if (beamTop2 == -1)
      error++;
	beamTop = ((beamTop + beamTop2)/2.0);


	lv_bar_set_value(ui_Bar1, map(3, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);

	stepperX.moveTo(bottomBeamPosition);
	stepperX.runToPosition();
	//moveXnb(bottomBeamPosition);

	beamBottom = measure();
    if (beamBottom == -1)
      error++;

	lv_bar_set_value(ui_Bar1, map(4, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);

	stepperX.moveTo(bottomBeamPosition2);
	stepperX.runToPosition();
	//moveXnb(bottomBeamPosition2);
	beamBottom2 = measure();
  if (beamBottom2 == -1)
    error++;

	beamBottom = ((beamBottom + beamBottom2)/2.0);


	lv_bar_set_value(ui_Bar1, map(5, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);

	stepperX.moveTo(bottomBarPosition);  //Move to start of Beam Scan
	stepperX.runToPosition();
	botBar = measure();
  if (botBar == -1)
    error++;


  //Validate Indicator - Make sure there are no connectivity issues or dead batteries, maybe the deice is off?
  if(error > 0)
  {
    	Serial.println("***--RESULTS INALID--***");
  }
  else
     	Serial.println("-----RESULTS------");

#ifdef HUMAN_READABLE_DEBUG

	Serial.print("beamLeft: ");
	Serial.print(beamBottom, 4);

	Serial.print(" beamRight: ");
	Serial.print(beamTop, 4);

	Serial.print(" LeftBar: ");
	Serial.print(botBar, 4);

	Serial.print(" rightBar: ");
	Serial.println(topBar, 4);

#endif  //HUMAN_READABLE_DEBUG

	float beamTopCorrected = beamTop - (((botBar / bottomBarPosition) * ((topBeamPosition + topBeamPosition2) /2.0)));
	float beamBotCorrected = beamBottom - ((botBar / bottomBarPosition) *  ((bottomBeamPosition + bottomBeamPosition2)/2.0));


	Serial.print("Left Corrected: ");
	Serial.print(beamBotCorrected, 4);

	Serial.print(" Right Corrected: ");
	Serial.println(beamTopCorrected, 4);


	lv_bar_set_value(ui_Bar1, map(5, 0, STEP_TOTAL, 0, 100), LV_ANIM_OFF);


	sprintf(buffer, "%.1f-%.1f", beamTopCorrected * 100.0, beamBotCorrected * 100);
	lv_label_set_text(ui_LabelResult, buffer);


	//Units until now was in inches but to convert to thouseands we need to multiply by 100
	float top = beamTopCorrected * 1000.0;
	float bot = beamBotCorrected * 1000.0;


	sprintf(buffer, "%.1f-%.1f", bot, top);
	lv_label_set_text(ui_LabelResult, buffer);
	//Timer_Loop();
	float spec = (int)mb.Hreg(HREG_SPEC);
	//Check the beamwidth to the specficiaiton using
	if ((top >= (spec - 2.0)) && (bot >= (spec - 2.0)) && (top <= (spec + 2.0)) && (bot <= (spec + 2.0))) {
		lv_label_set_text(ui_LabelPF, "PASS");
    colorWipe(strip.Color(0, 255, 0), 1);  // GREEN
    
    mb.Hreg(HREG_PASS_FAIL, 1);  //Modbus
		lv_obj_set_style_text_color(ui_LabelPF, lv_color_hex(0x0FC11F), LV_PART_MAIN | LV_STATE_DEFAULT);
		//Timer_Loop();  //Green
	} else {
		lv_label_set_text(ui_LabelPF, "FAIL");
		colorWipe(strip.Color(255, 0, 0), 1);  // RED
		mb.Hreg(HREG_PASS_FAIL, 0);
		lv_obj_set_style_text_color(ui_LabelPF, lv_color_hex(0xEE0909), LV_PART_MAIN | LV_STATE_DEFAULT);
		//Timer_Loop();  //Red
	}


	//stepperX.moveTo(0);  //Move to home
	stepperX.moveTo(0);
	//stepperX.moveTo(bottomBarPosition);  //Move to start of Beam Scan
	stepperX.runToPosition();

  if(error > 0)
    {
      sysState = STATE_CHECK_INDICATOR;
      mb.Hreg(HREG_STATUS, ERROR);
    }
    else
    {
      mb.Hreg(HREG_STATUS, STANDBY);
      sysState = STATE_READY;
    }
}


float readIndicator() {
	float rVal = 0;

	byte spcdata[13];  // The raw data sent by instrument
#ifdef HUMAN_READABLE_OUTPUT
	float value;  // The value calculated from raw data
	int decimal;  // Number of digits that are after decimal point
#endif
	int i = 0;
	int j = 0;
	int k = 0;
  int timestamp = millis();

  timestamp = millis();
	//delay(1);
	do {

		noInterrupts();
		digitalWrite(req, HIGH);  // generate set request
		for (i = 0; i < 13; i++) {
			k = 0;
			// Timing diagram indicates data bit has been valid for about 120
			// microseconds before the clock signal is raised, and remains
			// valid for about 120 microseconds afterwards. This code reads data
			// bit at the falling clock edge.

			for (j = 0; j < 4; j++) {
				while (digitalRead(clk) == LOW && ((millis() - timestamp) < 250)) {


				}  // hold until clock is high
				while (digitalRead(clk) == HIGH && ((millis() - timestamp) < 250)) {


				}  // hold until clock is low
				bitWrite(k, j, (digitalRead(dat) & 0x1));
			}

			// After reading the first 4-bit value, we can drop REQ output.
			if (i == 0) {
				digitalWrite(req, LOW);
			}
			spcdata[i] = k;
		}
		interrupts();
		delay(10);

	} while (spcdata[12] != 1 && ((millis() - timestamp) < 500));
 

#ifdef HEX_DATA_OUTPUT
	for (i = 0; i < 13; i++) {
		Serial.print(spcdata[i], HEX);
	}
#ifdef HUMAN_READABLE_OUTPUT
	// Need a separator if we're printing human readable as well
	Serial.print(" ");
#endif  // HUMAN_READABLE_OUTPUT
#endif  //HEX_DATA_OUTPUT

#ifdef HUMAN_READABLE_OUTPUT
	// Combine measurement digits into a number
	value = 0;
	for (i = 5; i <= 10; i++) {
		value *= 10;
		value += spcdata[i];
	}

	// Adjust number for decimal point position
	decimal = spcdata[11];
	value /= pow(10, decimal);

	// Adjust if number is negative
	if (spcdata[4] == 0x8) {
		value *= -1.0;
	}

	rVal = value;
#ifdef HUMAN_READABLE_DEBUG
	// Print resulting value to serial port, to specified
	// number of digits after decimal point.
	Serial.print(value, decimal);

	// Append units for value
	if (spcdata[12] == 0) {
		Serial.print(" mm");
	} else {
		Serial.print(" in");
	}
	Serial.println();
#endif  //HUMAN_READABLE_DEBUG
#endif  // HUMAN_READABLE_OUTPUT

	//Serial.print("T:");
	//Serial.println(rVal, 5);
  if(((millis() - timestamp) < 500))
	  return rVal;
  else
    return -1;  //TIMEOUT ERROR
}

void homeServo() {
	Serial.println("Home Offset Captured");
	indicatorValue = readIndicator();
	homeOffset = indicatorValue;
	stepperZ.setCurrentPosition(0);
	stepperZ.moveTo(BAR_RETRACT);
	stepperZ.runToPosition();
	stepperZ.setCurrentPosition(0);
	stepperX.setCurrentPosition(0);
}


float measure() {

	//Serial.println("Moving to touch");
	int homeFindPosition = 0;
	static unsigned long timer = 0;
	unsigned long interval = 500;
	stepperZ.moveTo(-BAR_RETRACT);
	stepperZ.runToPosition();


	Serial.print("Distance: ");

  float distInInches = -1;
  float indicatorRead = readIndicator();
  if(indicatorRead != -1)
	  distInInches = (indicatorRead - homeOffset);
    
	Serial.println(distInInches, 4);

	//Then move back
	stepperZ.stop();

	//Serial.println("Move Away");
	stepperZ.moveTo(0);
	stepperZ.runToPosition();

	//interrupts();
	return distInInches;
}

void runMotor() {

	// Stepper X run seed
	if (diagFlagX == true) {

		if (Xstate == STATE_HOMEING) {
			stepperX.setCurrentPosition(0);
			driver.VACTUAL(0);
			Xstate = STATE_STANDBY;
		}
	}

	// Stepper X run seed
	if (diagFlagZ == true) {

		if (Zstate == STATE_HOMEING) {
			stepperZ.setCurrentPosition(0);
			driver2.VACTUAL(0);
			Zstate = STATE_STANDBY;
		}
	}


	if (stepperX.distanceToGo() != 0)
		stepperX.run();

	if (stepperZ.distanceToGo() != 0)
		stepperZ.run();
}




//Move the stepper Position Up
void moveUp() {
	stepperZ.moveTo(stepperZ.currentPosition() - 5000);  //Move up
	stepperZ.runToPosition();
}

void moveDown() {
	stepperZ.moveTo(stepperZ.currentPosition() + 5000);  //Move Down
	stepperZ.runToPosition();
}


void moveLeft() {
	stepperX.moveTo(stepperX.currentPosition() - 5000);  //Move up
	stepperX.runToPosition();
}

void moveRight() {
	stepperX.moveTo(stepperX.currentPosition() + 5000);  //Move Down
	stepperX.runToPosition();
}


void handleRisingEdge() {
	// This function executes when a rising edge is detected on interruptPin
	interruptCount++;
	diagFlagX = true;
}


void handleRisingEdge2() {
	// This function executes when a rising edge is detected on interruptPin
	diagFlagZ = true;
}



// void product_modelChanged(lv_event_t* e) {

//   ui_Dropdown2 = lv_event_get_target(e);
//   option = (int)lv_dropdown_get_selected(ui_Dropdown2);
// }

void toggleChanged(lv_event_t* e) {

	if(lv_obj_has_state(ui_Switch2, LV_STATE_CHECKED))
		settings.homeAtStartup = 1;
	else
		settings.homeAtStartup = 0;

	EEPROM.put(LOCATION_ADDRESS, settings);
	EEPROM.commit();
}


void measureBeamCmd(lv_event_t* e) {

//meaureBeam();

  //For Test Force 
  sysState = STATE_READY;
	flagStartMeasureBeam = true;


}


void beamSelectChange(lv_event_t* e) {
	int lastOption = option;
	ui_Dropdown1 = lv_event_get_target(e);
	option = (int)lv_dropdown_get_selected(ui_Dropdown1);
	mb.Hreg(HREG_FAMILY, option);

  if(settings.homeAtStartup == 0)  //Bail if no home selected. 
  {
    sysState == STATE_READY;    //Force the ready state 
    return;
  }
  if (lastOption != option)
	{
		sysState = STATE_HOMEING;
		homeSteppers();
	}
	else if(sysState == STATE_STANDBY)
	{
		sysState = STATE_HOMEING;
		homeSteppers();

	}

  
	flaggotoBeamHome = 1;

}

void beamSelectChangeModbus() {

  if(settings.homeAtStartup == 0)  //Bail if no home selected. 
  {
    sysState == STATE_READY;    //Force the ready state 
    return;
  }
  if(sysState == STATE_READY)
	{
		sysState = STATE_HOMEING;
		homeSteppers();
  }
	else if(sysState == STATE_STANDBY)
	{
		sysState = STATE_HOMEING;
		homeSteppers();

	}
	flaggotoBeamHome = 1;

}





void stopAllMotors(lv_event_t* e) {
	driver.VACTUAL(0);
	driver2.VACTUAL(0);
	stepperX.moveTo(stepperX.currentPosition());
	stepperZ.moveTo(stepperZ.currentPosition());
}


void homeXaxis(lv_event_t* e) {

	homeXaxisAT();

}

void homeYaxis(lv_event_t* e) {

	homeYaxisAT();
}




void homeSteppers()
{
	sysState = STATE_HOMEING;
	diagFlagX = false;
	diagFlagZ = false;
	homeXaxisAT();
	homeYaxisAT(); 
  //delay(500);
  timeout0.timeOut(25000, callback0); //delay, callback function
  Serial.println("Homeing started");
}



void homeXaxisAT() {
	//diagFlagX = false;
	 stepperX.move(+ 200);
	 stepperX.runToPosition();
	//delay(250);
	for (int i = 0; i <= 700; i = i + 5) {
		driver.VACTUAL(i);
		Xstate = STATE_HOMEING;
		diagFlagX = false;
	}
}

void homeYaxisAT() {
	//diagFlagZ = false;
	 stepperZ.move(-200);
	 stepperZ.runToPosition();
   //delay(250);
	for (int i = 0; i <= 700; i = i + 5) {
		driver2.VACTUAL(i);
		Zstate = STATE_HOMEING;
		diagFlagZ = false;
	}
}






void btnPressedUp(lv_event_t* e) {

	stallThreshold++;
	//driver.SGTHRS(stallThreshold);
	diagFlagZ = false;
	//lv_obj_clear_flag(ui_Label11, LV_OBJ_FLAG_HIDDEN);


	stepperZ.moveTo(stepperZ.currentPosition() + 1000);
	//sprintf(buffer, "%d", stallThreshold);
	//lv_label_set_text_fmt(ui_Label5, buffer);
}

void btnPressedDown(lv_event_t* e) {
	diagFlagZ = false;
	stallThreshold--;
	//driver.SGTHRS(stallThreshold);

	stepperZ.moveTo(stepperZ.currentPosition() - 1000);

	//sprintf(buffer, "%d", stallThreshold);
	//lv_label_set_text_fmt(ui_Label5, buffer);
}

void btnPressedLeft(lv_event_t* e) {
	diagFlagX = false;
	//lv_label_set_text(ui_Label5, "Left");
	moveXnb(stepperX.currentPosition() - 250);
}

void btnPressedRight(lv_event_t* e) {
	diagFlagX = false;
	//lv_label_set_text(ui_Label5, "Right");
	moveXnb(stepperX.currentPosition() + 250);
}

void gotoBeamHome(lv_event_t* e) {
	flaggotoBeamHome = 1;
}

void homeBeamFamily() {



	switch (option) {
	
  case BEAMFAMILY_NONE:
  //No beam family selected
  break;
   
   
  case BEAMFAMILY_C01:
		Xstate = STATE_SEEKING;
		Zstate = STATE_SEEKING;
		sysState = STATE_SEEKING;
    delay(1000);
		stepperX.moveTo(settings.xHomeLocation_C0);
		stepperX.runToPosition();
		Xstate = STATE_STANDBY;
		stepperZ.moveTo(settings.zHomeLocation_C0);
		stepperZ.runToPosition();
		Zstate = STATE_STANDBY;
    colorWipe(strip.Color(0, 0, 255), 1);  // BLUE
		break;

	case BEAMFAMILY_C2:
		Xstate = STATE_SEEKING;
		Zstate = STATE_SEEKING;
		sysState = STATE_SEEKING;
    delay(1000);
		stepperX.moveTo(settings.xHomeLocation_C2);
		stepperX.runToPosition();
		Xstate = STATE_STANDBY;
		stepperZ.moveTo(settings.zHomeLocation_C2);
		stepperZ.runToPosition();
		Zstate = STATE_STANDBY;
        colorWipe(strip.Color(0, 0, 255), 1);  // BLUE
		break;


	case BEAMFAMILY_TR1:
		Xstate = STATE_SEEKING;
		Zstate = STATE_SEEKING;
		sysState = STATE_SEEKING;
        delay(1000);
		stepperX.moveTo(settings.xHomeLocation_TR1);
		stepperX.runToPosition();
		Xstate = STATE_STANDBY;
		stepperZ.moveTo(settings.zHomeLocation_TR1);
		stepperZ.runToPosition();
		Zstate = STATE_STANDBY;
        colorWipe(strip.Color(0, 0, 255), 1);  // BLUE
		break;


	case BEAMFAMILY_TR2:
		Xstate = STATE_SEEKING;
		Zstate = STATE_SEEKING;
		sysState = STATE_SEEKING;
        delay(1000);
		stepperX.moveTo(settings.xHomeLocation_TR2);
		stepperX.runToPosition();
		Xstate = STATE_STANDBY;
		stepperZ.moveTo(settings.zHomeLocation_TR2);
		stepperZ.runToPosition();
		Zstate = STATE_STANDBY;
        colorWipe(strip.Color(0, 0, 255), 1);  // BLUE
		break;


	default:
		break;
	}
}

void saveBeamHome(lv_event_t* e) {

	//Depends on option of Beam
	switch (option) {

  case 0: 
		break;
	case BEAMFAMILY_C01:
		stepperZ.move(BAR_RETRACT);
		stepperZ.runToPosition();
		settings.xHomeLocation_C0 = stepperX.currentPosition();
		settings.zHomeLocation_C0 = stepperZ.currentPosition();
		EEPROM.put(LOCATION_ADDRESS, settings);
		break;
	case BEAMFAMILY_C2:
		stepperZ.move(BAR_RETRACT);
		stepperZ.runToPosition();
		settings.xHomeLocation_C2 = stepperX.currentPosition();
		settings.zHomeLocation_C2 = stepperZ.currentPosition();
		EEPROM.put(LOCATION_ADDRESS, settings);
		break;
  case BEAMFAMILY_TR1:
		stepperZ.move(BAR_RETRACT);
		stepperZ.runToPosition();
		settings.xHomeLocation_TR1 = stepperX.currentPosition();
		settings.zHomeLocation_TR1 = stepperZ.currentPosition();
		EEPROM.put(LOCATION_ADDRESS, settings);
		break;
  case BEAMFAMILY_TR2:
		stepperZ.move(BAR_RETRACT);
		stepperZ.runToPosition();
		settings.xHomeLocation_TR2 = stepperX.currentPosition();
		settings.zHomeLocation_TR2 = stepperZ.currentPosition();
		EEPROM.put(LOCATION_ADDRESS, settings);
		break;
	default:
		break;
	}
	EEPROM.commit();  //Some platforms need this. Comment this line if needed
}

void moveXnbLoop(int pos) {

	static unsigned long timer = 0;
	unsigned long interval = 100;
	stepperX.moveTo(pos);
	while (stepperX.distanceToGo() != 0) {
		stepperX.run();
		if (millis() - timer >= interval) {
			timer = millis();
			//mb.task();
			Serial.print(".");
			//Timer_Loop();
			lv_timer_handler();
		}
	}
	Serial.println("Done move");
}


void moveXnb(int pos) {

	//static unsigned long timer = 0;
	//unsigned long interval = 100;
	stepperX.moveTo(pos);
}

void moveZnb(int pos) {

	static unsigned long timer = 0;
	unsigned long interval = 100;
	stepperZ.moveTo(pos);
	while (stepperZ.distanceToGo() != 0) {
		stepperZ.run();
		if (millis() - timer >= interval) {
			timer = millis();
			//mb.task();
			Timer_Loop();
		}
	}
}
