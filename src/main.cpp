#include <Wire.h>
#include <SPI.h>

// TFT display
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"

// BME280
#include <Adafruit_BME280.h>

// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "RTClib.h"

// TFT display
#define TFT_SPI_SCK 6
#define TFT_SPI_SDA 7 // TX

#define TFT_DC 9 // A0
#define TFT_CS 10
#define TFT_RST 11
// Hardware SPI on Feather or other boards
//Adafruit_GC9A01A(int8_t _CS, int8_t _DC, int8_t _MOSI, int8_t _SCLK,
                   //int8_t _RST = -1, int8_t _MISO = -1);
Adafruit_GC9A01A tft(TFT_CS, TFT_DC, TFT_SPI_SDA, TFT_SPI_SCK, TFT_RST);

// BME280
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp;
Adafruit_Sensor *bme_pressure;
Adafruit_Sensor *bme_humidity;

// DS3231
RTC_DS3231 rtc;

DateTime NewYearDate(2025, 1, 1, 0, 0, 0);

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

void setup() {
  Serial.begin(9600);
  Serial.println("RPI pico watches!");

  // ---------- 
  // TFT
  // ---------- 
  tft.begin();
  tft.fillScreen(GC9A01A_BLACK);
  // ---------- 

  // ---------- 
  // BME
  // ---------- 
  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1){ 
      delay(1000);
      Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    }
  }

  bme_temp = bme.getTemperatureSensor();
  bme_pressure = bme.getPressureSensor();
  bme_humidity = bme.getHumiditySensor();

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  // ---------- 

  // ---------- 
  // RTC
  // ---------- 
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // ---------- 
}

void loop(void) {
  DateTime now = rtc.now();
  TimeSpan tillNY = NewYearDate - now;

  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  String temperature_s = String(temp_event.temperature) + " *C";
  String humidity_s = String(humidity_event.relative_humidity) + " %";
  String pressure_s = String(pressure_event.pressure) + " hPa";

  String day_of_week_s = daysOfTheWeek[now.dayOfTheWeek()];
  String date_s = String(now.year()) + "/" + String(now.month()) + "/" + String(now.day());
  String time_s = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());

  String tillNY_1_s = String(tillNY.days()) + " days " + String(tillNY.hours()) + " hr ";
  String tillNY_2_s = String(tillNY.minutes()) + " min " + String(tillNY.seconds()) + " s ";

  //---------------
  // Debug printouts
  //---------------
  Serial.println(temperature_s);
  Serial.println(humidity_s);
  Serial.println(pressure_s);
  Serial.println(day_of_week_s);
  Serial.println(date_s);
  Serial.println(time_s);
  //---------------

  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);  
  tft.setTextSize(2);

  tft.setCursor(40, 40);
  tft.println(temperature_s);
  tft.setCursor(40, 60);
  tft.println(humidity_s);
  tft.setCursor(40, 80);
  tft.println(pressure_s);

  tft.setCursor(40, 100);
  tft.println(day_of_week_s);
  tft.setCursor(40, 120);
  tft.println(date_s);
  tft.setCursor(40, 140);
  tft.println(time_s);

  tft.setCursor(40, 160);
  tft.println(tillNY_1_s);
  tft.setCursor(40, 180);
  tft.println(tillNY_2_s);

  delay(1000);
}
