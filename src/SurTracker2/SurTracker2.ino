/*

SurTracker2

SPI1	: SD CARD
Serial2 : GSP MODULE

*/

#include <io.h>
#include <Keypad.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <NMEAGPS.h>
#include <GPSport.h>

#define LED_0	PC7
#define LED_1	PC9
#define LED_2	PC6
#define LED_3	PC8

#define LCD_CLK	PB13
#define LCD_DAT	PB15
#define LCD_CS	PB12
#define LCD_RST	PB2		

#define KEYPAD_ROW_CNT 	4
#define KEYPAD_ROW_0	PC4
#define KEYPAD_ROW_1	PB0
#define KEYPAD_ROW_2	PC5
#define KEYPAD_ROW_3	PB1

#define KEYPAD_COL_CNT	4
#define KEYPAD_COL_0	PC2
#define KEYPAD_COL_1	PC0
#define KEYPAD_COL_2	PC3
#define KEYPAD_COL_3	PC1

#define KEY_0	'0'
#define KEY_1	'1'
#define KEY_2	'2'
#define KEY_3	'3'
#define KEY_4	'4'
#define KEY_5	'5'
#define KEY_6	'6'
#define KEY_7	'7'
#define KEY_8	'8'
#define KEY_9	'9'
#define KEY_A	'A'
#define KEY_B	'B'
#define KEY_C	'C'
#define KEY_D	'D'
#define KEY_E	'E'
#define KEY_F	'F'

#define SW_0	PA8
#define SW_1	PA13

#define GPS_EN  PA1
#define GPS_TX	PA2
#define GPS_RX	PA3

#define SD_CS 	PA4
#define SD_SCK	PA5
#define SD_MISO	PA6
#define SD_MOSI	PA7

uint8 sw0, sw1;

uint8 KEYPAD_ROW_PINS[KEYPAD_ROW_CNT] = {KEYPAD_ROW_0, KEYPAD_ROW_1, KEYPAD_ROW_2, KEYPAD_ROW_3};
uint8 KEYPAD_COL_PINS[KEYPAD_COL_CNT] = {KEYPAD_COL_0, KEYPAD_COL_1, KEYPAD_COL_2, KEYPAD_COL_3};
char KEYPAD_KEYS[KEYPAD_ROW_CNT][KEYPAD_COL_CNT] = {
  {KEY_0, KEY_1, KEY_2, KEY_3},
  {KEY_4, KEY_5, KEY_6, KEY_7},
  {KEY_8, KEY_9, KEY_A, KEY_B},
  {KEY_C, KEY_D, KEY_E, KEY_F}
};
Keypad keypad = Keypad(makeKeymap(KEYPAD_KEYS), KEYPAD_ROW_PINS, KEYPAD_COL_PINS, KEYPAD_ROW_CNT, KEYPAD_COL_CNT);

U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, LCD_CLK, LCD_DAT, LCD_CS, LCD_RST);

NMEAGPS  gps;
gps_fix  fix;


//-----------------------------------------------------------------------------

void setup() {
	// init led pins
	pinMode(LED_0, OUTPUT);
	pinMode(LED_1, OUTPUT);
	pinMode(LED_2, OUTPUT);
	pinMode(LED_3, OUTPUT);

	// turn off all leds
	digitalWrite(LED_0, LOW); 
	digitalWrite(LED_1, LOW); 
	digitalWrite(LED_2, LOW); 
	digitalWrite(LED_3, LOW); 

	// --------------------------------

	digitalWrite(LED_0, HIGH); 

	// init sw pins
	pinMode(SW_0, INPUT);
	pinMode(SW_1, INPUT);

	// read sw states
	sw0 = digitalRead(SW_0);
	sw1 = digitalRead(SW_1);

	// init gps module pins
	pinMode(GPS_EN, OUTPUT);
	// enable gps module
	digitalWrite(GPS_EN, HIGH); 

	Serial.begin(9600);
	while (!Serial) {
		;
	}
	Serial.print( F("NMEAsimple.INO: started\n") );

	gpsPort.begin(9600);

	// Serial.print("Initializing SD card...");

	// // see if the card is present and can be initialized:
	// if (!SD.begin(SD_CS)) {
	// 	Serial.println("Card failed, or not present");
	// 	// don't do anything more:
	// 	while (1);
	// }
	// Serial.println("card initialized.");

	// // open the file. note that only one file can be open at a time,
	// // so you have to close this one before opening another.
	// File dataFile = SD.open("datalog.txt");

	// // if the file is available, write to it:
	// if (dataFile) {
	// 	while (dataFile.available()) {
	// 		Serial.write(dataFile.read());
	// 	}
	// 	dataFile.close();
	// }
	// // if the file isn't open, pop up an error:
	// else {
	// 	Serial.println("error opening datalog.txt");
	// }

	u8g2.begin();
	
	// --------------------------------
	digitalWrite(LED_0, HIGH); 
	delay(333);
	digitalWrite(LED_1, HIGH); 
	delay(333);
	digitalWrite(LED_2, HIGH); 
	delay(333);
	digitalWrite(LED_3, HIGH); 
	delay(1000);
	// turn off all leds
	digitalWrite(LED_0, LOW); 
	digitalWrite(LED_1, LOW); 
	digitalWrite(LED_2, LOW); 
	digitalWrite(LED_3, LOW); 
}

//-----------------------------------------------------------------------------

void loop() {
	sw0 = digitalRead(SW_0);
	digitalWrite(LED_1, sw0 ? HIGH : LOW); 
	sw1 = digitalRead(SW_1);
	digitalWrite(LED_2, sw1 ? HIGH : LOW); 

	char key = keypad.getKey();
	if (key){
		Serial.println(key);
	}

	u8g2.firstPage();
	do {
		u8g2.setFont(u8g2_font_profont11_mf  );
		u8g2.drawStr(0,20,"Hello World!");
		u8g2.drawStr(0,48,"012345678901234567890");
	} while (u8g2.nextPage());

	while (gps.available( gpsPort )) {
		fix = gps.read();

		DEBUG_PORT.print( F("Location: ") );
		if (fix.valid.location) {
			DEBUG_PORT.print( fix.latitude(), 6 );
			DEBUG_PORT.print( ',' );
			DEBUG_PORT.print( fix.longitude(), 6 );
		}

		DEBUG_PORT.print( F(", Altitude: ") );
		if (fix.valid.altitude)
			DEBUG_PORT.print( fix.altitude() );

		DEBUG_PORT.println();
	}
}
