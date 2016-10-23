
/***************************************************************************
  This is a library for the BME280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME280 Breakout
  ----> http://www.adafruit.com/products/2650

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  This file was modified by Markus Haack (https://github.com/mhaack)
  in order to work with Particle Photon & Core.
  Then further modified by Mac McAlpine https://github.com/macsboost
 ***************************************************************************/
 
//#include "Adafruit_BME280/Adafruit_Sensor.h"
#include "Adafruit_BME280/Adafruit_BME280.h"

#define BME_SCK D4
#define BME_MISO D3
#define BME_MOSI D2
#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

//This Sketch measure differential pressure across the air filter in the house HVAC

/*                                    +-----+
 *                          +----------| USB |----------+
 *                          |          +-----+       *  |
 *                          | [ ] VIN           3V3 [ ] |
 *                          | [ ] GND           RST [ ] |
 *                          | [ ] TX           VBAT [ ] |
 *                          | [ ] RX  [S]   [R] GND [ ] |
 *                          | [ ] WKP            D7 [*] |
 *                          | [ ] DAC +-------+  D6 [*] |
 *                          | [ ] A5  |   *   |  D5 [*] |
 *                          | [ ] A4  |Photon |  D4 [*] |P1 PWR
 *                          | [ ] A3  |       |  D3 [*] |P2 PWR
 *                          | [ ] A2  +-------+  D2 [*] |
 *                      P1  | [*] A1             D1 [*] |SCL BME 0x76
 *                      P2  | [*] A0             D0 [*] |SDA BME 0x76
 *                          |                           |
 *                           \    []         [______]  /
 *                            \_______________________/
 *
 *
 */

/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   230400-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/


Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

Timer timer(14232, checkEnvironment); // call this function every 10 seconds
Timer timer2(9901, serialDebug); // call this function every 10 seconds
Timer timer3(10123, checkEnvironment2);

int caladdress = 0;
double atticTemp = 0;
double baro = 0;
double humidity = 0;


//ApplicationWatchdog wd(5000, System.reset); //5 second application watchdog
SYSTEM_MODE(SEMI_AUTOMATIC);



ApplicationWatchdog wd(60000, System.reset);


void setup() {
  Serial.begin(230400);
  Serial.println(F("BME280 test"));

  /*if (!bme.begin(0x76)) {  //0x76 is the i2c address of the sensor
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }*/
  
  
  Particle.publish("DEBUG", "starting...");

  if (bme.begin(0x76)) {  //0x76 is the i2c address of the sensor
    Particle.publish("DEBUG", "starting the environment timer...");
    timer.start(); //initialize timer
    timer2.start();//initialize timer2
    timer3.start();//initialize timer2
  }
  else {
    Particle.publish("WARN", "Could not find a valid BMP280 sensor, check wiring!");
  }

  Particle.publish("DEBUG", "started!");
 
  Particle.connect(); 
  Particle.variable("bme280temp", atticTemp);
  Particle.variable("bme280baro", baro);
  Particle.variable("bme280humid", humidity);
}

void loop() {
    digitalWrite(D7,!digitalRead(D7)); //blink D7
    wd.checkin(); //watchdog checkin
    
    if (System.buttonPushed() > 2000) { //trigger a wifi disconnect if you hold for over 2 seconds.
        //RGB.color(255, 255, 0); // YELLOW
        if (Particle.connected() == false) {  //if not connected, delay, 5000ms before attempting reconnect.  without delay was causing gps to fail.
            Serial.println("Connecting to wifi");
		    Particle.connect();
		    delay(1000);
        } else {
            //Particle.disconnect();
            WiFi.off();
            delay(1000);
        }   
    }
}



void serialDebug() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
}

void checkEnvironment() {

  Particle.publish("RSSI", String(WiFi.RSSI()));
  //Particle.publish("DEBUG", "checking the environment...");
  atticTemp = cToF(bme.readTemperature());
  Particle.publish("environment/temperature", String(atticTemp));
  
}

void checkEnvironment2() {

  baro = pToHg(bme.readPressure());
  Particle.publish("environment/pressure", String(baro));
  Particle.publish("environment/altitude", String(bme.readAltitude(SEALEVELPRESSURE_HPA)));
  humidity = bme.readHumidity();
  Particle.publish("environment/humidity", String(humidity));
}

float cToF(float c) {
  return c * 9/5 + 32;
}

float pToHg(float p) {
  return p/3389.39;
}