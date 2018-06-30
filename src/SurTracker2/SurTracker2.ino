#include <io.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <GPX.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>

#define LED_0	PA15
#define LED_1	PB3
#define LED_2	PB4
#define LED_3	PB5

#define KEY_0 PA0
#define KEY_1 PA1
#define KEY_2 PA2
#define KEY_3 PA3

#define SW_0  PA14
#define SW_1  PA14

#define LCD_CLK	PB8
#define LCD_DAT	PB9
//#define LCD_CS	PA13
//#define LCD_DC  PB12
//#define LCD_RST	PB13		

#define MAIN_SERIAL Serial
#define MAIN_SERIAL_BR  38400

#define GPS_EN  PA8
#define GPS_TX	PA9
#define GPS_RX	PA10
#define GPS_SERIAL Serial1
#define GPS_SERAIL_BR	38400

#define SD_CS 	PA4
#define SD_SCK	PA5
#define SD_MISO	PA6
#define SD_MOSI	PA7

#define CH_H	10
#define CH_W	6
#define LAT_LON_SCALE	10000000

//U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, LCD_CLK, LCD_DAT, LCD_CS, LCD_DC, LCD_RST);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, LCD_CLK, LCD_DAT); 

NMEAGPS  gps;
gps_fix  fix;
GPX gpx;

char str[32], dir;
int32 value;

uint8 sw0, sw1, key0, key1, key2, key3, lock, rec;
File file;
char name[12];

//Adafruit_BMP280 baro;
//uint8  baro_ready;
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

	MAIN_SERIAL.begin(MAIN_SERIAL_BR);
	if (MAIN_SERIAL) {
		MAIN_SERIAL.print("Serial initialized: ");
		MAIN_SERIAL.print(MAIN_SERIAL_BR);
		MAIN_SERIAL.println("bps");
	}

	u8g2.begin();
	if (MAIN_SERIAL) {
		MAIN_SERIAL.println("LCD initialized");
	}

	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_profont11_mf);
	u8g2.drawStr(0, CH_H * 1, "--- Sur Tracker 2 ---");
	u8g2.drawStr(0, CH_H * 2, "Serial init........OK");
	u8g2.sendBuffer();

	// init key pins
  pinMode(KEY_0, INPUT);
  pinMode(KEY_1, INPUT);
  pinMode(KEY_2, INPUT);
  pinMode(KEY_3, INPUT);
  pinMode(SW_0, INPUT);
	pinMode(SW_1, INPUT);

	// read key states
  key0 = digitalRead(KEY_0);
  key1 = digitalRead(KEY_1);
  key2 = digitalRead(KEY_2);
  key3 = digitalRead(KEY_3);
  sw0 = digitalRead(SW_0);
	sw1 = digitalRead(SW_1);

	if (MAIN_SERIAL) {
		MAIN_SERIAL.print("Switch initialized: sw0 = ");
		MAIN_SERIAL.print(sw0);
		MAIN_SERIAL.print(", sw1 = ");
		MAIN_SERIAL.println(sw1);
	}
	u8g2.drawStr(0, CH_H * 3, "Keys init..........OK");
  u8g2.sendBuffer();

	// init gps module enable pin
	pinMode(GPS_EN, OUTPUT);
	if (MAIN_SERIAL) {
		MAIN_SERIAL.println("GPS_EN pin initialized");
	}

	// init gps module serial port
	gpsPort.begin(GPS_SERAIL_BR);
	while (!gpsPort);
	if (MAIN_SERIAL) {
		MAIN_SERIAL.print("GSP serial port initialized: ");
		MAIN_SERIAL.print(GPS_SERAIL_BR);
		MAIN_SERIAL.println("bps");
	}
	u8g2.drawStr(0, CH_H * 4, "GPS init...........OK");
	u8g2.sendBuffer();

	// step 2
	digitalWrite(LED_1, HIGH); 

	// init sd card
	if (MAIN_SERIAL) {
		MAIN_SERIAL.print("SDCard init... ");
	}
	u8g2.drawStr(0, CH_H * 5, "SDCard init........");
 	if (!SD.begin(SD_CS)) {
 		if (MAIN_SERIAL) {
 			MAIN_SERIAL.println("Failed, or not present");
		}
 		u8g2.drawStr(CH_W * 17, CH_H * 5, "FAIL");
	} else {
		if (MAIN_SERIAL) {
			MAIN_SERIAL.println("OK");
		}
		u8g2.drawStr(CH_W * 19, CH_H * 5, "OK");
	}
	u8g2.sendBuffer();

	// step 3
	digitalWrite(LED_2, HIGH); 

	// init barometer module
	if (MAIN_SERIAL) {
		MAIN_SERIAL.print("Barometer init... ");
    MAIN_SERIAL.println("Could not find a valid BMP280 sensor, check wiring!");
	}
	u8g2.drawStr(0, CH_H * 6, "Barometer init.....");
  u8g2.drawStr(CH_W * 17, CH_H * 6, "FAIL");
//	baro_ready = baro.begin();
//	if (!baro_ready) {
//		if (MAIN_SERIAL) {
// 			MAIN_SERIAL.println("Could not find a valid BMP280 sensor, check wiring!");
//		}
//		u8g2.drawStr(CH_W * 17, CH_H * 6, "FAIL");
//	} else {
//		if (MAIN_SERIAL) {
//			MAIN_SERIAL.println("OK");
//		}
//		u8g2.drawStr(CH_W * 19, CH_H * 6, "OK");
//	}
//	u8g2.sendBuffer();

 	// step 4
	digitalWrite(LED_3, HIGH);

	// --------------------------------

	delay(1000 * 3);

	// turn off all leds
	digitalWrite(LED_0, HIGH); 
	digitalWrite(LED_1, HIGH); 
	digitalWrite(LED_2, HIGH); 
	digitalWrite(LED_3, HIGH); 

	// init ui
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_profont11_mf);

	// speed
	u8g2.setFont(u8g2_font_profont11_mf);
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
	if (MAIN_SERIAL) {
		MAIN_SERIAL.println("UI inited");
	}

	// enable gps module
	digitalWrite(GPS_EN, HIGH); 
	if (MAIN_SERIAL) {
		MAIN_SERIAL.println("Enable GPS module");
	}

	gpx.setMetaDesc("SurTracker2 GPX");
	gpx.setName("SurTracker2 Track Name");
	gpx.setDesc("SurTracker2 Track Desc");
	gpx.setSrc("SurTracker2 Track Src");
	
	if (MAIN_SERIAL) {
		MAIN_SERIAL.println(gpx.getOpen());
		MAIN_SERIAL.println(gpx.getMetaData());
		MAIN_SERIAL.println(gpx.getTrakOpen());
		MAIN_SERIAL.println(gpx.getInfo());
		MAIN_SERIAL.println(gpx.getTrakSegOpen());
	}

}

//-----------------------------------------------------------------------------

void loop() {
  key0 = digitalRead(KEY_0);
  key1 = digitalRead(KEY_1);
  key2 = digitalRead(KEY_2);
  key3 = digitalRead(KEY_3);
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

	while (gps.available(gpsPort)) {
		fix = gps.read();

		digitalWrite(LED_1, HIGH);

		if (1) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Satellites: ");
				MAIN_SERIAL.println(fix.satellites);
			}
			sprintf(str, "%2d*", fix.satellites);
			str[3] = 0;
			u8g2.drawStr(CH_W * 0, CH_H * 4, str);
		}

		if (fix.valid.date) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Date: ");
				MAIN_SERIAL.print(fix.dateTime.year);
				MAIN_SERIAL.print("-");
				MAIN_SERIAL.print(fix.dateTime.month);
				MAIN_SERIAL.print("-");
				MAIN_SERIAL.println(fix.dateTime.date);
			}
			sprintf(str, "%02d-%02d-%02d", fix.dateTime.year, fix.dateTime.month, fix.dateTime.date);
			str[8] = 0;
			u8g2.drawStr(CH_W * 4, CH_H * 4, str);
		} else {
			u8g2.drawStr(CH_W * 4, CH_H * 4, "YY-MM-DD");
		}

		if (fix.valid.time) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Time: ");
				MAIN_SERIAL.print(fix.dateTime.hours);
				MAIN_SERIAL.print(":");
				MAIN_SERIAL.print(fix.dateTime.minutes);
				MAIN_SERIAL.print(":");
				MAIN_SERIAL.println(fix.dateTime.seconds);
			}
			sprintf(str, "%02d:%02d:%02d", fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
			str[8] = 0;
			u8g2.drawStr(CH_W * 13, CH_H * 4, str);
		} else {
			u8g2.drawStr(CH_W * 13, CH_H * 4, "hh:mm:ss");
		}

		if (fix.valid.location) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Latitude: ");
				MAIN_SERIAL.print(fix.latitude());
				MAIN_SERIAL.print(", ");
				MAIN_SERIAL.println(fix.latitudeL());
				MAIN_SERIAL.print("Longitude: ");
				MAIN_SERIAL.print(fix.longitude());
				MAIN_SERIAL.print(", ");
				MAIN_SERIAL.println(fix.longitudeL());
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
				if (MAIN_SERIAL) {
					MAIN_SERIAL.println(loc_str);
				}

				digitalWrite(LED_2, LOW);
			}
		} else {
			u8g2.drawStr(CH_W * 9, CH_H * 1, "---.---- LAT");
			u8g2.drawStr(CH_W * 9, CH_H * 2, "---.---- LON");
		}

		if (fix.valid.altitude) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Altitude: ");
				MAIN_SERIAL.print(fix.altitude());
				MAIN_SERIAL.print(", ");
				MAIN_SERIAL.println(fix.altitude_cm());
			}
			value = fix.altitude_cm();
			sprintf(str, "%3d.%02dm", value / 100, value % 100);
			str[7] = 0;
			u8g2.drawStr(CH_W * 14, CH_H * 3, str);
		} else {
			u8g2.drawStr(CH_W * 14, CH_H * 3, "--- ALT");
		}

		if (fix.valid.heading) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Heading: ");
				MAIN_SERIAL.print(fix.heading());
				MAIN_SERIAL.print(", ");
				MAIN_SERIAL.println(fix.heading_cd());
			}
			value = fix.heading_cd();
			sprintf(str, "%3d.%02d\xb0", value / 100, value % 100);
			str[7] = 0;
			u8g2.drawStr(CH_W * 6, CH_H * 3, str);
		} else {
			u8g2.drawStr(CH_W * 6, CH_H * 3, "---- HD");
		}

		if (fix.valid.speed) {
			if (MAIN_SERIAL) {
				MAIN_SERIAL.print("Speed: ");
				MAIN_SERIAL.println(fix.speed_kph());
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

    u8g2.drawStr(CH_W * 1, CH_H * 5, "---- \xb0\x43");
    u8g2.drawStr(CH_W * 10, CH_H * 5, "-----.-- Pa");
//		if (baro_ready) {
//			value = baro.readTemperature() * 100;
//			sprintf(str, "%2d.%02d\xb0\x43", value / 100, value % 100);
//			str[8] = 0;
//			u8g2.drawStr(CH_W * 1, CH_H * 5, str);
//
//			value = baro.readPressure() * 100;
//			sprintf(str, "%6d.%02dPa", value / 100, value % 100);
//			str[11] = 0;
//			u8g2.drawStr(CH_W * 10, CH_H * 5, str);
//		} else {
//			u8g2.drawStr(CH_W * 1, CH_H * 5, "---- \xb0\x43");
//			u8g2.drawStr(CH_W * 10, CH_H * 5, "-----.-- Pa");
//		}

		u8g2.sendBuffer();
		if (MAIN_SERIAL) {
			MAIN_SERIAL.println("UI refreshed");
			MAIN_SERIAL.println("-");
		}

		if (rec == HIGH) {
			if (name[0] == 0 && fix.valid.date && fix.valid.time) {
				sprintf(name, "%02d%02d%02d.gpx", 
					fix.dateTime.hours, fix.dateTime.minutes, fix.dateTime.seconds);
				MAIN_SERIAL.println(name);
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
