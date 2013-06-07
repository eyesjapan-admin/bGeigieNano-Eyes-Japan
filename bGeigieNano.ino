/*
   The bGeigie-nano
   A device for car-borne radiation measurement (aka Radiation War-driving).

   Copyright (c) 2012, Lionel Bergeret
   Copyright (c) 2011, Robin Scheibler aka FakuFaku, Christopher Wang aka Akiba
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <limits.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "TinyGPS.h"

#include "NanoSetup.h"
#include "NanoConfig.h"
#include "NanoDebug.h"

// OLED settings --------------------------------------------------------------
#if ENABLE_SSD1306
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifdef OLED_SPI_MODE
Adafruit_SSD1306 display(OLED_DATA, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
#else
Adafruit_SSD1306 display(OLED_RESET);
#endif

#if (SSD1306_LCDHEIGHT != 32)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// For distance computation
bool gps_fix_first = true;
float gps_last_lon = 0, gps_last_lat = 0;
unsigned long int gps_distance = 0;
#endif

// Geiger settings ------------------------------------------------------------
#define TIME_INTERVAL 5000
#define LINE_SZ 100
#define BUFFER_SZ 12
#define STRBUFFER_SZ 32
#define NX 12
#define AVAILABLE 'A'  // indicates geiger data are ready (available)
#define VOID      'V'  // indicates geiger data not ready (void)


// log file headers
#define LOGFILE_HEADER "# NEW LOG\n# format="
char logfile_name[13];  // placeholder for filename
bool logfile_ready = false;

// geiger statistics
unsigned long shift_reg[NX] = {0};
unsigned long reg_index = 0;
unsigned long total_count = 0;
unsigned long max_count = 0;
unsigned long uptime = 0;
int uphour = 0;
int upminute = 0;
int CO_read=10;
int NOX_read=20;
int str_count = 0;
char geiger_status = VOID;

// the line buffer for serial receive and send
static char line[LINE_SZ];
static char strbuffer[STRBUFFER_SZ];

// Pulse counter --------------------------------------------------------------
#if ENABLE_HARDWARE_COUNTER
	// Hardware counter
	#include "HardwareCounter.h"
	HardwareCounter hwc(HARDWARE_COUNTER_TIMER1, TIME_INTERVAL);
	#else
	// Interrupt counter
	#include "InterruptCounter.h"
#endif

#define IS_READY (interruptCounterAvailable())


// OpenLog settings -----------------------------------------------------------
#if ENABLE_OPENLOG
	#define OPENLOG_RETRY 200
	SoftwareSerial OpenLog(OPENLOG_RX_PIN, OPENLOG_TX_PIN);
	static const int resetOpenLog = OPENLOG_RST_PIN;
#endif
bool openlog_ready = false;

// Gps settings ------------------------------------------------------------
TinyGPS gps(true);
#define GPS_INTERVAL 1000
char gps_status = VOID;

#if ENABLE_SOFTGPS
	SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
#endif

// Gps data buffers
static char lat[BUFFER_SZ];
static char lon[BUFFER_SZ];

// MTK33x9 chipset
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define SBAS_ENABLE "$PMTK313,1*2E\r\n"
#define DGPS_WAAS_ON "$PMTK301,2*2E\r\n"

#if ENABLE_STATIC_GPS
	#include <avr/pgmspace.h>
	// GPS test sentences
	char strGPRMC[] PROGMEM = "$GPRMC,201547.000,A,3014.5527,N,09749.5808,W,0.24,163.05,040109,,*1A";
	char strGPGGA[] PROGMEM = "$GPGGA,201548.000,3014.5529,N,09749.5808,W,1,07,1.5,225.6,M,-22.5,M,18.8,0000*78";
	char *teststrs[2] = {strGPRMC, strGPGGA};

	static void sendstring(TinyGPS &gps, const PROGMEM char *str)
	{
	  while (true)
	  {
		char c = pgm_read_byte_near(str++);
		if (!c) break;
		gps.encode(c);
	  }
	  gps.encode('\r');
	  gps.encode('\n');
	}
#endif

// SHT1 (sensors temp/humidity settings ------------------------------------------------------------
 #if ENABLE_SHT1
	 #include "SHT1x.h"
	 SHT1x SHT1x(dataPin, clockPin);
	 int temperature_read=-40;
	 int humidity_read=-10;
 #endif
 
 
// Function definitions ---------------------------------------------------------
// Atmel Tips and Tricks: 3.6 Tip #6 – Access types: Static
static unsigned long cpm_gen();
static bool gps_gen_filename(TinyGPS &gps, char *buf);
static bool gps_gen_timestamp(TinyGPS &gps, char *buf, unsigned long counts, unsigned long cpm, unsigned long cpb);
static char checksum(char *s, int N);
#if ENABLE_OPENLOG
	static void setupOpenLog();
	static bool loadConfig(char *fileName);
	static void createFile(char *fileName);
#endif
static void gps_program_settings();
static float read_voltage(int pin);
static unsigned long elapsedTime(unsigned long startTime);
#if ENABLE_100M_TRUNCATION
	static void truncate_100m(char *latitude, char *longitude);
#endif

// Nano Settings --------------------------------------------------------------
static ConfigType config;
static DoseType dose;
NanoSetup nanoSetup(OpenLog, config, dose, line, LINE_SZ);

// ****************************************************************************
// Setup
// ****************************************************************************
void setup()
{
  pinMode(GPS_LED_PIN, OUTPUT);
  pinMode(GEIGIE_TYPE_PIN, INPUT);

  Serial.begin(9600);


  // Load EEPROM settings
  nanoSetup.initialize();

#if ENABLE_OPENLOG
  OpenLog.begin(9600);
  setupOpenLog();
  if (openlog_ready) {
    nanoSetup.loadFromFile("SAFECAST.TXT");
  }
#endif

#if ENABLE_HARDWARE_COUNTER
  // Start the Pulse Counter!
  hwc.start();
#else
  // Create pulse counter
  interruptCounterSetup(INTERRUPT_COUNTER_PIN, TIME_INTERVAL);

  // And now Start the Pulse Counter!
  interruptCounterReset();
#endif

#if ENABLE_SOFTGPS
  gpsSerial.begin(9600);

  // Put GPS serial in listen mode
  gpsSerial.listen();

  // initialize and program the GPS module
  gps_program_settings();
#endif

  // setup analog reference to read battery and boost voltage
  analogReference(INTERNAL);

#if ENABLE_SSD1306
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // show splashscreen logo
  display.display();
  
  delay(1000);
  // show 3nd splashscreen
  display.clearDisplay();

  display.setTextColor(WHITE);
  display.setTextSize(1);
  sprintf_P(strbuffer, PSTR("Geigie Nano %s"), NANO_VERSION);
  display.setCursor(0,0);
  if (config.type == GEIGIE_TYPE_B) {
    display.print("b");
  } else {
    display.print("x");
  }
  display.print(strbuffer);
  
  display.setCursor(0, 8);
  int battery =((read_voltage(VOLTAGE_PIN)-30)*12.5);
  battery=(battery+20);
    if (battery < 0) battery=1;
    if (battery > 100) battery=100;
  sprintf_P(strbuffer, PSTR("Battery=%02d"), battery); 
  display.print(strbuffer);
  sprintf_P(strbuffer, PSTR("%%"));
  display.print(strbuffer);
      
  display.setCursor(85, 8);
  sprintf_P(strbuffer, PSTR("#%04d"), config.device_id);
  display.print(strbuffer);

  temperature_read = int(SHT1x.readTemperatureC());
  humidity_read = int(SHT1x.readHumidity()); 
    
  display.setCursor(0, 16);
  sprintf_P(strbuffer, PSTR("Temp=%d"), temperature_read);
  display.print(strbuffer);
    sprintf_P(strbuffer, PSTR("C"));
  display.print(strbuffer);
    
  display.setCursor(52, 16);
  sprintf_P(strbuffer, PSTR("Humid=%d"), humidity_read);
  display.print(strbuffer);
  sprintf_P(strbuffer, PSTR("%%"));
  display.print(strbuffer);

  display.setCursor(0, 24);
  CO_read= analogRead(CO_Pin);
  sprintf_P(strbuffer, PSTR("CO=%d"), CO_read);
  display.print(strbuffer);
  
  display.setCursor(40, 24);
  display.print("NOX=");
  dtostrf((float)(analogRead(NOX_Pin)/1000.00), 4, 3, strbuffer);
  display.print(strbuffer);

  
  display.display();
   delay(9000);
  
#endif

}

// ****************************************************************************
// Main loop
// ****************************************************************************
void loop()
{
  bool gpsReady = false;

#if ENABLE_GEIGIE_SWITCH
  // Check geigie mode switch
  if (analogRead(GEIGIE_TYPE_PIN) > GEIGIE_TYPE_THRESHOLD) {
    config.type = GEIGIE_TYPE_B; // bGeigie
  } else {
    config.type = GEIGIE_TYPE_X; // xGeigie
  }
#endif


#if ENABLE_SOFTGPS
  // Put GPS serial in listen mode
  gpsSerial.listen();
#endif

  // For one second we parse GPS sentences
  for (unsigned long start = millis(); (elapsedTime(start) < GPS_INTERVAL) and !IS_READY;)
  {
	#if ENABLE_SOFTGPS
		while (gpsSerial.available())
		{
		  char c = gpsSerial.read();
	#else
		while (Serial.available())
		{
		  char c = Serial.read();
	#endif

	#if ENABLE_GPS_NMEA_LOG
		  Serial.print(c); // uncomment this line if you want to see the GPS data flowing
	#endif
      if (gps.encode(c)) // Did a new valid sentence come in?
        gpsReady = true;
    }
  }


  if ((gpsReady) || (gps_status == AVAILABLE)) {
    digitalWrite(GPS_LED_PIN, HIGH);
  } else {
    digitalWrite(GPS_LED_PIN, LOW);
  }

  // generate CPM every TIME_INTERVAL seconds
  if IS_READY {
      unsigned long cpm=0, cpb=0;

      // obtain the count in the last bin
      cpb = interruptCounterCount();

      // reset the pulse counter
      interruptCounterReset();

      // insert count in sliding window and compute CPM
      shift_reg[reg_index] = cpb;     // put the count in the correct bin
      reg_index = (reg_index+1) % NX; // increment register index
      cpm = cpm_gen();                // compute sum over all bins

      // update the total counter
      total_count += cpb;
      uptime += 5;

      // update max cpm
      if (cpm > max_count) max_count = cpm;


      // set status of Geiger
      if (str_count < NX)
      {
        geiger_status = VOID;
        str_count++;
      } else if (cpm == 0) {
        geiger_status = VOID;
      } else {
        geiger_status = AVAILABLE;
      }

#if ENABLE_WAIT_GPS_FOR_LOG
      if ((!logfile_ready) && (gps_status == AVAILABLE))
#else
      if (!logfile_ready)
#endif
      {
         if (gps_gen_filename(gps, logfile_name)) {
           logfile_ready = true;

#if ENABLE_OPENLOG
           createFile(logfile_name);
           // print header to serial
           sprintf_P(strbuffer, PSTR(LOGFILE_HEADER));
           OpenLog.print(strbuffer);
           sprintf_P(strbuffer, PSTR(NANO_VERSION));
           OpenLog.print(strbuffer);

	#ifdef ENABLE_LND_DEADTIME
			   sprintf_P(strbuffer, PSTR("nano\n# deadtime=on\n"));
	#else
			   sprintf_P(strbuffer, PSTR("nano\n));
	#endif
           OpenLog.print(strbuffer);

#endif
         }
      }

      // generate timestamp. only update the start time if
      // we printed the timestamp. otherwise, the GPS is still
      // updating so wait until its finished and generate timestamp
      memset(line, 0, LINE_SZ);
      gps_gen_timestamp(gps, line, shift_reg[reg_index], cpm, cpb);

      // Printout line
      Serial.println(line);

#if ENABLE_OPENLOG
      if ((logfile_ready) && (GEIGIE_TYPE_B == config.type)) {
        // Put OpenLog serial in listen mode
        OpenLog.listen();
        OpenLog.println(line);

      }
#endif
  }


}

// ****************************************************************************
// Utility functions
// ****************************************************************************

/* calculate elapsed time. this takes into account rollover */
unsigned long elapsedTime(unsigned long startTime) {
  unsigned long stopTime = millis();

  if (startTime >= stopTime) {
    return startTime - stopTime;
  } else {
    return (ULONG_MAX - (startTime - stopTime));
  }
}

#if ENABLE_OPENLOG
/* wait for openlog prompt */
bool waitOpenLog(bool commandMode) {
  int safeguard = 0;
  bool result = false;

  while(safeguard < OPENLOG_RETRY) {
    safeguard++;
    if(OpenLog.available())
      if(OpenLog.read() == (commandMode ? '>':'<')) break;
    delay(10);
  }

  if (safeguard >= OPENLOG_RETRY) {

  } else {

    result = true;
  }

  return result;
}

/* setups up the software serial, resets OpenLog */
void setupOpenLog() {
  pinMode(resetOpenLog, OUTPUT);
  OpenLog.listen();

  // reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);

  if (!waitOpenLog(true)) {
    logfile_ready = true;
  } else {
    openlog_ready = true;
  }
}

/* create a new file */
void createFile(char *fileName) {
  int result = 0;
  int safeguard = 0;

  OpenLog.listen();

  do {
    result = 0;

    do {
#ifndef ENABLE_SLEEPMODE
      // reset the watchdog timer
      wdt_reset();
#endif

      OpenLog.print("append ");
      OpenLog.print(fileName);
      OpenLog.write(13); //This is \r

      // wait for OpenLog to indicate file is open and ready for writing
      if (!waitOpenLog(false)) {
        break;
      }
      result = 1;
    } while (0);

    if (0 == result) {
      // reset OpenLog
      digitalWrite(resetOpenLog, LOW);
      delay(100);
      digitalWrite(resetOpenLog, HIGH);

      // Wait for OpenLog to return to waiting for a command
      waitOpenLog(true);
    }
  } while (0 == result);

  //OpenLog is now waiting for characters and will record them to the new file
}
#endif

/* compute check sum of N bytes in array s */
char checksum(char *s, int N)
{
  int i = 0;
  char chk = s[0];
  for (i=1 ; i < N ; i++)
    chk ^= s[i];

  return chk;
}

/* compute cpm */
unsigned long cpm_gen()
{
   unsigned int i;
   unsigned long c_p_m = 0;

   // sum up
   for (i=0 ; i < NX ; i++)
     c_p_m += shift_reg[i];

#ifdef ENABLE_LND_DEADTIME
   // deadtime compensation (medcom international)
   c_p_m = (unsigned long)((float)c_p_m/(1-(((float)c_p_m*1.8833e-6))));
#endif

   return c_p_m;
}

/* generate log filename */
bool gps_gen_filename(TinyGPS &gps, char *buf) {
  int year = 2012;
  byte month = 0, day = 0, hour = 0, minute = 0, second = 0, hundredths = 0;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (TinyGPS::GPS_INVALID_AGE == age) {
    return false;
  }

  // Create the filename for that drive
  sprintf_P(buf, PSTR("%04d%02d%02d.log"),config.device_id, month, day);

  return true;
}

/* convert long integer from TinyGPS to string "xxxxx.xxxx" */
void get_coordinate_string(bool is_latitude, unsigned long val, char *buf)
{
  unsigned long left = 0;
  unsigned long right = 0;

  left = val/100000.0;
  right = (val - left*100000)/10;
  if (is_latitude) {
    sprintf_P(buf, PSTR("%04ld.%04ld"), left, right);
  } else {
    sprintf_P(buf, PSTR("%05ld.%04ld"), left, right);
  }
}

/* convert long integer from TinyGPS to float WGS84 degrees */
float get_wgs84_coordinate(unsigned long val)
{
  double result = 0.0;
  result = val/10000000.0;
  result = ((result-(int)result)/60.0)*100 + (int)result;
  return (float)result;
}

/* generate log result line */
bool gps_gen_timestamp(TinyGPS &gps, char *buf, unsigned long counts, unsigned long cpm, unsigned long cpb)
{
  int year = 2012;
  byte month = 0, day = 0, hour = 0, minute = 0, second = 0, hundredths = 0;
  long int x = 0, y = 0;
  float faltitude = 0, fspeed = 0;
  unsigned short nbsat = 0;
  unsigned long precission = 0;
  unsigned long age;
  byte len, chk;
  byte len1, chk1;
  char buf1;
  char NS = 'N';
  char WE = 'E';
  static int toggle = 0;

  memset(lat, 0, BUFFER_SZ);
  memset(lon, 0, BUFFER_SZ);
  memset(strbuffer, 0, STRBUFFER_SZ);

  // get GPS date
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (TinyGPS::GPS_INVALID_AGE == age) {
    year = 2012, month = 0, day = 0, hour = 0, minute = 0, second = 0, hundredths = 0;
  }

  // get GPS position, altitude and speed
  gps.get_position(&x, &y, &age);
  if (!gps.status()) {
    gps_status = VOID;
  } else {
    gps_status = AVAILABLE;
  }
  faltitude = gps.f_altitude();
  fspeed = gps.f_speed_kmph();
  nbsat = gps.satellites();
  precission = gps.hdop();

  if (x < 0) { NS = 'S'; x = -x;}
  if (y < 0) { WE = 'W'; y = -y;}
  get_coordinate_string(true, x == TinyGPS::GPS_INVALID_ANGLE ? 0 : x, lat);
  get_coordinate_string(false, y == TinyGPS::GPS_INVALID_ANGLE ? 0 : y, lon);
  dtostrf(faltitude == TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : faltitude, 0, 2, strbuffer);


#if ENABLE_SHT1
	temperature_read = int(SHT1x.readTemperatureC());
	humidity_read = int(SHT1x.readHumidity());
  #else
	 int temperature_read= -40;
     int humidity_read=0;
#endif

//NOX and CO sensors readings
NOX_read= analogRead(NOX_Pin);
CO_read= analogRead(CO_Pin);
  

  // prepare the log entry
  memset(buf, 0, LINE_SZ);
  sprintf_P(buf, PSTR("$%s,%04d,%02d-%02d-%02dT%02d:%02d:%02dZ,%ld,%ld,%ld,%c,%s,%c,%s,%c,%s,%c,%d,%ld"),  \
              NANO_HEADER, \
              config.device_id, \
              year, month, day,  \
              hour, minute, second, \
              cpm, \
              cpb, \
              total_count, \
              geiger_status, \
              lat, NS,\
              lon, WE,\
              strbuffer, \
              gps_status, \
			  nbsat  == TinyGPS::GPS_INVALID_SATELLITES ? 0 : nbsat,\
              precission == TinyGPS::GPS_INVALID_HDOP ? 0 : precission);

  len = strlen(buf);
  buf[len] = '\0';

  // generate checksum
  chk = checksum(buf+1, len);
  
  //quick fix to get NOX data in the string
  dtostrf((float)(analogRead(NOX_Pin)/1000.00), 4, 3, strbuffer);

    sprintf_P(buf + len, PSTR("*%X%s$%s,%04d,%d,%d,%d,%s"), 
              (int)chk, \
              "\n", \
              NANO_HEADER_SENSOR, \
              config.device_id, \
              temperature_read, \
              humidity_read,\
              CO_read, \
     		  strbuffer);
  
  



#if ENABLE_SSD1306
  // compute distance
  if (gps.status()) {
    int trigger_dist = 25;
    float flat = get_wgs84_coordinate(x);
    float flon = get_wgs84_coordinate(y);

    if(fspeed > 5)
      // fpspeed/3.6 * 5s = 6.94 m
      trigger_dist = 5;
    if(fspeed > 10)
      trigger_dist = 10;
    if(fspeed > 15)
      trigger_dist = 20;

    if(gps_fix_first)
    {
      gps_last_lat = flat;
      gps_last_lon = flon;
      gps_fix_first = false;
    }
    else
    {
      // Distance in meters
      unsigned long int dist = (long int)TinyGPS::distance_between(flat, flon, gps_last_lat, gps_last_lon);

      if (dist > trigger_dist)
      {
        gps_distance += dist;
        gps_last_lat = flat;
        gps_last_lon = flon;
      }
    }
  }

  // ready to display the data on screen
  display.clearDisplay();
  int offset = 0;
if (config.type == GEIGIE_TYPE_B) {
    // **********************************************************************
    // bGeigie mode
    // **********************************************************************
    // Display uptime
    uphour = uptime/3600;
    upminute = uptime/60 - uphour*60;
    sprintf_P(strbuffer, PSTR("%02dh%02dm"), uphour, upminute);
    display.setCursor(92, offset+16);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.println(strbuffer);

    // Display CPM (with deadtime compensation)
    display.setCursor(0, offset);
    display.setTextSize(2);
    if (VOID == geiger_status) {
      display.setTextColor(BLACK, WHITE); // 'inverted' text
    } else {
      display.setTextColor(WHITE);
    }
    if (cpm > 1000) {
      dtostrf((float)(cpm/1000.00), 3, 2, strbuffer);
      display.print(strbuffer);
      display.print("k");
    } else {
      dtostrf((float)cpm, 0, 0, strbuffer);
      display.print(strbuffer);
    }
    sprintf_P(strbuffer, PSTR(" CPM"));
    display.print(strbuffer);

    // Display SD, GPS and Geiger states
    if (openlog_ready) {
      display.setTextColor(WHITE);
    } else {
      if (toggle) {
        display.setTextColor(BLACK, WHITE); // 'inverted' text
      } else {
        display.setTextColor(WHITE);
      }
    }
    display.setTextSize(1);
    if (!gps.status()) {
      display.setCursor(92, offset);
      sprintf_P(strbuffer, PSTR("No GPS"));
      display.println(strbuffer);
    } else {
      display.setCursor(110, offset); 
      sprintf(strbuffer,"%2d", nbsat);
      display.print(strbuffer);
      sprintf_P(strbuffer, PSTR("^"));
      display.println(strbuffer);
  
    }

    // Display uSv/h
    display.setTextColor(WHITE);
    display.setCursor(0, offset+16); // textsize*8
    if (config.mode == GEIGIE_MODE_USVH) {
      dtostrf((float)(cpm/config.cpm_factor), 0, 3, strbuffer);
      display.print(strbuffer);
      sprintf_P(strbuffer, PSTR(" uSv/h"));
      display.println(strbuffer);
    } 
    else if (config.mode == GEIGIE_MODE_BQM2) {
      dtostrf((float)(cpm*config.bqm_factor), 0, 3, strbuffer);
      display.print(strbuffer);
      sprintf_P(strbuffer, PSTR(" Bq/m2"));
      display.println(strbuffer);
    }

    if (toggle) {
      // Display distance
      dtostrf((float)(gps_distance/1000.0), 0, 1, strbuffer);
      display.setCursor(116-(strlen(strbuffer)*6), offset+8); // textsize*8
      display.print(strbuffer);
      sprintf_P(strbuffer, PSTR("km"));
      display.println(strbuffer);
    } else {
      // Display altidude
      if (gps.status()) {
        dtostrf(faltitude, 0, 0, strbuffer);
      } else {
        sprintf_P(strbuffer, PSTR("--"));
      }
      display.setCursor(122-(strlen(strbuffer)*6), offset+8); // textsize*8
      display.print(strbuffer);
      display.println("m");
    }
      // Display date
  sprintf_P(strbuffer, PSTR("%02d/%02d %02d:%02d:%02d"),  \
        day, month, \
        hour, minute, second);
  display.setCursor(0, offset+24); // textsize*8
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(strbuffer);
   }
   
  // **********************************************************************
  // XGeigie Mode
  // **********************************************************************

  else if (config.type == GEIGIE_TYPE_X) {
    display.setTextSize(1);
    display.setCursor(0,0); 
    dtostrf((float)(cpm/config.cpm_factor), 0, 2, strbuffer);
    display.print(strbuffer);
    sprintf_P(strbuffer, PSTR(" uS/h"));
    display.print(strbuffer);
    
    display.setCursor(85, 0);
    sprintf_P(strbuffer, PSTR("#%04d"), config.device_id);
    display.print(strbuffer);
  
    display.setCursor(0,8); 
    display.print(config.user_name);
  
    display.setCursor(0, 16);
    sprintf_P(strbuffer, PSTR("Temp=%d"), temperature_read);
    display.print(strbuffer);
    sprintf_P(strbuffer, PSTR("C"));
    display.print(strbuffer);
    
    display.setCursor(52, 16);
    sprintf_P(strbuffer, PSTR("Humid=%d"), humidity_read);
    display.print(strbuffer);
    sprintf_P(strbuffer, PSTR("%%"));
    display.print(strbuffer);
    
    display.setCursor(0, 24);
    sprintf_P(strbuffer, PSTR("CO=%d"), CO_read);
    display.print(strbuffer);
  
    display.setCursor(40, 24);
    display.print("NOX=");
    dtostrf((float)(analogRead(NOX_Pin)/1000.00), 4, 3, strbuffer);
    display.print(strbuffer);


  }

  // **********************************************************************
  // Common display parts
  // **********************************************************************


  // Display battery indicator
  // Range = [3.5v to 4.3v]
  //int battery = ((read_voltage(VOLTAGE_PIN)-3.5)*8/0.8);
  int battery =((read_voltage(VOLTAGE_PIN)-30));
  if (battery < 0) battery = 0;
  if (battery > 8) battery = 8;

display.drawRect(116, offset+24, 12, 7, WHITE);
display.fillRect(118, offset+26, battery, 3, WHITE);
  
  display.display();
#endif

  // Display items toggling
  toggle ^= 1;

  return (gps_status == AVAILABLE);
}

/* setup the GPS module to 1Hz and RMC+GGA messages only */
void gps_program_settings()
{

}

void gps_send_message(const uint8_t *msg, uint16_t len)
{
  uint8_t chk = 0x0;
  // header
  gpsSerial.write(0xA0);
  gpsSerial.write(0xA1);
  // send length
  gpsSerial.write(len >> 8);
  gpsSerial.write(len & 0xff);
  // send message
  for (unsigned int i = 0 ; i < len ; i++)
  {
    gpsSerial.write(msg[i]);
    chk ^= msg[i];
  }
  // checksum
  gpsSerial.write(chk);
  // end of message
  gpsSerial.write(0x0D);
  gpsSerial.write(0x0A);
  gpsSerial.write('\n');
}

/* retrieve battery voltage */
float read_voltage(int pin)
{
  static float voltage_divider = (float)VOLTAGE_R2 / (VOLTAGE_R1 + VOLTAGE_R2);
  float result = (float)analogRead(pin)/1024 *10 / voltage_divider;
  return result;
}