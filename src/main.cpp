#include "lcd_bsp.h"
#include "lcd_bl_pwm_bsp.h"
#include "ui.h"
#include "Display_ST7789.h"
#include "LVGL_Driver.h"
#include "ui.h"
//#include <TMCStepper.h>
//#include <AccelStepper.h>
#include <EEPROM.h> //Needed to record user settings
//#include <ModbusIP_ESP8266.h>
//#include <TimeOut.h>
//#include <FastLED_NeoPixel.h>

// Which pin on the Arduino is connected to the LEDs?
#define DATA_PIN 42

// How many LEDs are attached to the Arduino?
#define NUM_LEDS 10

// How many NeoPixels are attached to the Arduino?
// #define LED_COUNT 10

// LED brightness, 0 (min) to 255 (max)
#define BRIGHTNESS 50


// EEPROM locations to store 4-byte variables
#define EEPROM_SIZE 100	   // Allocate 100 bytes of EEPROM
#define LOCATION_ADDRESS 0 // Float, requires 4 bytes of EEPROM

char buffer[80];

// Mitutoyo SPC
#define HUMAN_READABLE_OUTPUT

// #define HUMAN_READABLE_DEBUG

// #define BEAM_SIZE_DEBUG
int req = 41; // Arduino pin for REQ line, drives transistor to ground SPC port pin 5 (~REQ)
int dat = 39; // Arduino pin for Data line, connects directly to SPC port pin 2 (DATA)
int clk = 40; // Arduino pin for Clock line, connects directly to SPC port pin 3 (~CLK)
float indicatorValue = 0;
float homeOffset = 0;

#define STALL_VALUE 50 // [0..255]
#define EN_PIN 16	   // Enable
#define EN_PIN2 7	   // Enable
#define DIAG_PIN2 3	   // DIAG PIN FOR HOMEING

#define DIR_PIN 18	// Direction
#define STEP_PIN 17 // Step
#define DIAG_PIN 15 // DIAG PIN FOR HOMEING

#define DIR_PIN2 6	// Direction
#define STEP_PIN2 5 // Step

// #define SW_RX            0 // TMC2208/TMC2224 SoftwareSerial receive pin
// #define SW_TX            5 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial0	 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00	 // TMC2209 Driver address according to MS1 and MS2
#define DRIVER2_ADDRESS 0b01 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver





// Fucntion Prototypes
void task(void *);
//void processState();
void handleScreenUpdate();

void setup()
{



	EEPROM.begin(EEPROM_SIZE); // Some platforms need this. Comment this line if needed


	Serial.begin(115200);

	lcd_lvgl_Init();
	setUpdutySubdivide(LCD_GPIO_MODE, LCD_GPIO_MODE_ON); // LCD Backlight related
	ui_init();


	lv_label_set_text_fmt(ui_Label2, "STARTER");
	lv_label_set_text_fmt(ui_LabelResult, "----");

	Serial.println("Setup Done...");

	// Attempting to pin the modbus and screen update task to core zero
	xTaskCreatePinnedToCore(
		task,	// Function to implement the task
		"task", // Name of the task
		6000,	// Stack size in words
		NULL,	// Task input parameter
		0,		// Priority of the task
		NULL,	// Task handle.
		0);		// Core where the task should run
}

void loop()
{

	if (Serial.available())
	{
		byte incoming = Serial.read();

		//if (incoming == 'i')
			//Serial.println(WiFi.localIP());
	}
}

void task(void *)
{

	for (;;)
	{
		//mb.task();
		lv_timer_handler_run_in_period(20);
		handleScreenUpdate();
	}
}

void handleScreenUpdate()
{
	static int i = 0;

	lv_bar_set_value(ui_Bar1, i, LV_ANIM_OFF);
	static unsigned long timer = 0;
	unsigned long interval = 50;
	if (millis() - timer >= interval)
	{
		timer = millis();
		// Update Diag screen info

		i++;
		if (i == 100)
			i = 0;
	}
}
