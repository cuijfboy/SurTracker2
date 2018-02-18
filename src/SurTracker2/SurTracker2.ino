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
#include <GPX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define LED_0	PC7
#define LED_1	PC9
#define LED_2	PC6
#define LED_3	PC8

#define SERIAL_BR	9600

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
#define GPS_SERAIL_BR	38400

#define SD_CS 	PA4
#define SD_SCK	PA5
#define SD_MISO	PA6
#define SD_MOSI	PA7

#define CH_H	10
#define CH_W	6
#define LAT_LON_SCALE	10000000

uint8 KEYPAD_ROW_PINS[KEYPAD_ROW_CNT] = {KEYPAD_ROW_0, KEYPAD_ROW_1, KEYPAD_ROW_2, KEYPAD_ROW_3};
uint8 KEYPAD_COL_PINS[KEYPAD_COL_CNT] = {KEYPAD_COL_0, KEYPAD_COL_1, KEYPAD_COL_2, KEYPAD_COL_3};
char KEYPAD_KEYS[KEYPAD_ROW_CNT][KEYPAD_COL_CNT] = {
  {KEY_0, KEY_1, KEY_2, KEY_3},
  {KEY_4, KEY_5, KEY_6, KEY_7},
  {KEY_8, KEY_9, KEY_A, KEY_B},
  {KEY_C, KEY_D, KEY_E, KEY_F}
};
Keypad keypad = Keypad(makeKeymap(KEYPAD_KEYS), KEYPAD_ROW_PINS, KEYPAD_COL_PINS, KEYPAD_ROW_CNT, KEYPAD_COL_CNT);

U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, LCD_CLK, LCD_DAT, LCD_CS, LCD_RST);

NMEAGPS  gps;
gps_fix  fix;
GPX gpx;

char str[32], dir;
int32 value;

uint8 sw0, sw1, lock, rec;
File file;
char name[12];

Adafruit_BMP280 baro;
uint8  baro_ready;

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

	// step 1
	digitalWrite(LED_0, HIGH); 

	Serial.begin(SERIAL_BR);
	if (Serial) {
		Serial.print("Serial initialized: ");
		Serial.print(SERIAL_BR);
		Serial.println("bps");
	}

	u8g2.begin();
	if (Serial) {
		Serial.println("LCD initialized");
	}

	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_profont11_mf);
	u8g2.drawStr(0, CH_H * 1, "--- Sur Tracker 2 ---");
	u8g2.drawStr(0, CH_H * 2, "Serial init........OK");
	u8g2.sendBuffer();

	// init sw pins
	pinMode(SW_0, INPUT);
	pinMode(SW_1, INPUT);

	// read sw states
	sw0 = digitalRead(SW_0);
	sw1 = digitalRead(SW_1);

	if (Serial) {
		Serial.print("Switch initialized: sw0 = ");
		Serial.print(sw0);
		Serial.print(", sw1 = ");
		Serial.println(sw1);
	}
	u8g2.drawStr(0, CH_H * 3, "Switch init........OK");
	u8g2.sendBuffer();

	// init gps module enable pin
	pinMode(GPS_EN, OUTPUT);
	if (Serial) {
		Serial.println("GPS_EN pin initialized");
	}

	// init gps module serial port
	gpsPort.begin(GPS_SERAIL_BR);
	while (!gpsPort);
	if (Serial) {
		Serial.print("GSP serial port initialized: ");
		Serial.print(GPS_SERAIL_BR);
		Serial.println("bps");
	}
	u8g2.drawStr(0, CH_H * 4, "GPS init...........OK");
	u8g2.sendBuffer();

	// step 2
	digitalWrite(LED_1, HIGH); 

	// init sd card
	if (Serial) {
		Serial.print("SDCard init... ");
	}
	u8g2.drawStr(0, CH_H * 5, "SDCard init........");
 	if (!SD.begin(SD_CS)) {
 		if (Serial) {
 			Serial.println("Failed, or not present");
		}
 		u8g2.drawStr(CH_W * 17, CH_H * 5, "FAIL");
	} else {
		if (Serial) {
			Serial.println("OK");
		}
		u8g2.drawStr(CH_W * 19, CH_H * 5, "OK");
	}
	u8g2.sendBuffer();

	// step 3
	digitalWrite(LED_2, HIGH); 

	// init barometer module
	if (Serial) {
		Serial.print("Barometer init... ");
	}
	u8g2.drawStr(0, CH_H * 6, "Barometer init.....");
	baro_ready = baro.begin();
	if (!baro_ready) {
		if (Serial) {
 			Serial.println("Could not find a valid BMP280 sensor, check wiring!");
		}
		u8g2.drawStr(CH_W * 17, CH_H * 6, "FAIL");
	} else {
		if (Serial) {
			Serial.println("OK");
		}
		u8g2.drawStr(CH_W * 19, CH_H * 6, "OK");
	}
	u8g2.sendBuffer();

 	// step 4
	digitalWrite(LED_3, HIGH);

	// --------------------------------

	delay(1000 * 3);

	// turn off all leds
	digitalWrite(LED_0, LOW); 
	digitalWrite(LED_1, LOW); 
	digitalWrite(LED_2, LOW); 
	digitalWrite(LED_3, LOW); 

	// init ui
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_profont11_mf);

	// speed
	u8g2.setFont(u8g2_font_profont17_mf);
	u8g2.drawStr(4, 16, "--.--");
	u8g2.setFont(u8g2_font_profont11_mf);
	u8g2.drawStr(CH_W * 1, CH_H * 3, "km/h");
	
	// latitude
	u8g2.drawStr(CH_W * 9, CH_H * 1, "---.---- LAT");
	
	// longitude
	u8g2.drawStr(CH_W * 9, CH_H * 2, "---.---- LON");
	
	// altitude
	u8g2.drawStr(CH_W * 14, CH_H * 3, "--- ALT");

	// heading
	u8g2.drawStr(CH_W * 6, CH_H * 3, "---- HD");

	// satellites
	u8g2.drawStr(CH_W * 0, CH_H * 4, " 0*");

	// date
	u8g2.drawStr(CH_W * 4, CH_H * 4, "YY-MM-DD");

	// time
	u8g2.drawStr(CH_W * 13, CH_H * 4, "hh:mm:ss");

	// temperature
	u8g2.drawStr(CH_W * 1, CH_H * 5, "---- \xb0\x43");
	
	// barometer
	u8g2.drawStr(CH_W * 10, CH_H * 5, "-----.-- Pa");

	// refresh ui
	u8g2.sendBuffer();
	if (Serial) {
		Serial.println("UI inited");
	}

	// enable gps module
	digitalWrite(GPS_EN, HIGH); 
	if (Serial) {
		Serial.println("Enable GPS module");
	}

	gpx.setMetaDesc("SurTracker2 GPX");
	gpx.setName("SurTracker2 Track Name");
	gpx.setDesc("SurTracker2 Track Desc");
	gpx.setSrc("SurTracker2 Track Src");
	
	if (Serial) {
		Serial.println(gpx.getOpen());
		Serial.println(gpx.getMetaData());
		Serial.println(gpx.getTrakOpen());
		Serial.println(gpx.getInfo());
		Serial.println(gpx.getTrakSegOpen());
	}

}

//-----------------------------------------------------------------------------

void loop() {
	sw0 = digitalRead(SW_0);
	sw1 = digitalRead(SW_1);

	if (lock != sw0) {
		lock = sw0;
		if (lock == HIGH) {
			u8g2.drawStr(0, CH_H * 6, "LOCK");
		} else {
			u8g2.drawStr(0, CH_H * 6, "    ");
		}
	}

	if (rec != sw1) {
		rec = sw1;
		memset(name, 0, sizeof(name));
		if (rec == HIGH) {
			u8g2.drawStr(CH_W * 5, CH_H * 6, "REC");
		} else {
			u8g2.drawStr(CH_W * 5, CH_H * 6, "   ");
			if (file) {
				file.println(gpx.getTrakSegClose());
				file.println(gpx.getTrakClose());
				file.println(gpx.getClose());
				file.flush();
				file.close();		
			}
			
		}
	}

	char key = keypad.getKey();
	if (key) {
		if (Serial) {
			Serial.println(key);
		}
	}

	while (gps.available(gpsPort)) {
		fix = gps.read();

		digitalWrite(LED_1, HIGH);

		if (1) {
			if (Serial) {
				Serial.print("Satellites: ");
				Serial.println(fix.satellites);
			}
			sprintf(str, "%2d*", fix.satellites);
			str[3] = 0;
			u8g2.drawStr(CH_W * 0, CH_H * 4, str);
		}

		if (fix.valid.date) {
			if (Serial) {
				Serial.print("Date: ");
				Serial.print(fix.dateTime.year);
				Serial.print("-");
				Serial.print(fix.dateTime.month);
				Serial.print("-");
				Serial.println(fix.dateTime.date);
			}
			sprintf(str, "%02d-%02d-%02d", fix.dateTime.year, fix.dateTime.month, fix.dateTime.date);
			str[8] = 0;
			u8g2.drawStr(CH_W * 4, CH_H * 4, str);
		} else {
			u8g2.drawStr(CH_W * 4, CH_H * 4, "YY-MM-DD");
		}

		if (fix.valid.time) {
			if (Serial) {
				Serial.print("Time: ");
				Serial.print(fix.dateTime.hours);
				Serial.print(":");
				Serial.print(fix.dateTime.minutes);
				Serial.print(":");
				Serial.println(fix.dateTime.seconds);
			}
			sprintf(str, "%02d:%02d:%02d", fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
			str[8] = 0;
			u8g2.drawStr(CH_W * 13, CH_H * 4, str);
		} else {
			u8g2.drawStr(CH_W * 13, CH_H * 4, "hh:mm:ss");
		}

		if (fix.valid.location) {
			if (Serial) {
				Serial.print("Latitude: ");
				Serial.print(fix.latitude());
				Serial.print(", ");
				Serial.println(fix.latitudeL());
				Serial.print("Longitude: ");
				Serial.print(fix.longitude());
				Serial.print(", ");
				Serial.println(fix.longitudeL());
			}

			value = fix.latitudeL();
			dir = value < 0 ? 'S' : 'N';
			value = value < 0 ? (-1 * value) : value;
			sprintf(str, "%3d.%07d%c", value / LAT_LON_SCALE, value % LAT_LON_SCALE, dir);
			str[12] = 0;
			u8g2.drawStr(CH_W * 9, CH_H * 1, str);
			
			value = fix.longitudeL();
			dir = value < 0 ? 'W' : 'E';
			value = value < 0 ? (-1 * value) : value;
			sprintf(str, "%3d.%07d%c", value / LAT_LON_SCALE, value % LAT_LON_SCALE, dir);
			str[12] = 0;
			u8g2.drawStr(CH_W * 9, CH_H * 2, str);

			if (file) {
				digitalWrite(LED_2, HIGH);

				int32 lat_val, lon_val;
				char lat_str[13], lon_str[13];

				lat_val = fix.latitudeL();
				sprintf(lat_str, "%d.%07d", (lat_val / LAT_LON_SCALE), (lat_val < 0 ? (-1 * lat_val) : lat_val) % LAT_LON_SCALE);
				lat_str[12] = 0;

				lon_val = fix.longitudeL();
				sprintf(lon_str, "%d.%07d", (lon_val / LAT_LON_SCALE), (lon_val < 0 ? (-1 * lon_val) : lon_val) % LAT_LON_SCALE);
				lon_str[12] = 0;

				String loc_str = gpx.getPt(GPX_TRKPT, String(lon_str), String(lat_str));
				file.println(loc_str);
				file.flush();
				if (Serial) {
					Serial.println(loc_str);
				}

				digitalWrite(LED_2, LOW);
			}
		} else {
			u8g2.drawStr(CH_W * 9, CH_H * 1, "---.---- LAT");
			u8g2.drawStr(CH_W * 9, CH_H * 2, "---.---- LON");
		}

		if (fix.valid.altitude) {
			if (Serial) {
				Serial.print("Altitude: ");
				Serial.print(fix.altitude());
				Serial.print(", ");
				Serial.println(fix.altitude_cm());
			}
			value = fix.altitude_cm();
			sprintf(str, "%3d.%02dm", value / 100, value % 100);
			str[7] = 0;
			u8g2.drawStr(CH_W * 14, CH_H * 3, str);
		} else {
			u8g2.drawStr(CH_W * 14, CH_H * 3, "--- ALT");
		}

		if (fix.valid.heading) {
			if (Serial) {
				Serial.print("Heading: ");
				Serial.print(fix.heading());
				Serial.print(", ");
				Serial.println(fix.heading_cd());
			}
			value = fix.heading_cd();
			sprintf(str, "%3d.%02d\xb0", value / 100, value % 100);
			str[7] = 0;
			u8g2.drawStr(CH_W * 6, CH_H * 3, str);
		} else {
			u8g2.drawStr(CH_W * 6, CH_H * 3, "---- HD");
		}

		if (fix.valid.speed) {
			if (Serial) {
				Serial.print("Speed: ");
				Serial.println(fix.speed_kph());
			}
			value = fix.speed_kph() * 100;
			sprintf(str, "%2d.%02d", value / 100, value % 100);
			str[5] = 0;
			u8g2.setFont(u8g2_font_profont17_mf);
			u8g2.drawStr(4, 16, str);
			u8g2.setFont(u8g2_font_profont11_mf);
		} else {
			u8g2.setFont(u8g2_font_profont17_mf);
			u8g2.drawStr(4, 16, "--.--");
			u8g2.setFont(u8g2_font_profont11_mf);
		}

		if (baro_ready) {
			value = baro.readTemperature() * 100;
			sprintf(str, "%2d.%02d\xb0\x43", value / 100, value % 100);
			str[8] = 0;
			u8g2.drawStr(CH_W * 1, CH_H * 5, str);

			value = baro.readPressure() * 100;
			sprintf(str, "%6d.%02dPa", value / 100, value % 100);
			str[11] = 0;
			u8g2.drawStr(CH_W * 10, CH_H * 5, str);
		} else {
			u8g2.drawStr(CH_W * 1, CH_H * 5, "---- \xb0\x43");
			u8g2.drawStr(CH_W * 10, CH_H * 5, "-----.-- Pa");
		}

		u8g2.sendBuffer();
		if (Serial) {
			Serial.println("UI refreshed");
			Serial.println("-");
		}

		if (rec == HIGH) {
			if (name[0] == 0 && fix.valid.date && fix.valid.time) {
				sprintf(name, "%02d%02d%02d.gpx", 
					fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
				Serial.println(name);
				file = SD.open(name, FILE_WRITE);
				if (file) {
					file.println(gpx.getOpen());
					file.println(gpx.getMetaData());
					file.println(gpx.getTrakOpen());
					file.println(gpx.getInfo());
					file.println(gpx.getTrakSegOpen());
					file.flush();
				}
			}
		}

		digitalWrite(LED_1, LOW);
	}
}
