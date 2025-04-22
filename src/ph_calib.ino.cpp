# 1 "C:\\Users\\BGLB\\AppData\\Local\\Temp\\tmp70w5bokc"
#include <Arduino.h>
# 1 "E:/Octa/Documents/PlatformIO/Projects/pH-calib-UNO/src/ph_calib.ino"
#include <Wire.h>
#include <DFRobot_SHT20.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include <SoftwareSerial.h>

SoftwareSerial outSerial(10,11);


DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);


LiquidCrystal_I2C lcd(0x27, 20, 4);


#define TdsSensorPin A1
#define MotorPin 8
GravityTDS gravityTds;


float temperature = 25;
float tdsValue = 0;
float ecValue = 0;
float ec25Value = 0;


SimpleTimer timer;
float calibration_value = 26.34 - 0.5;
int buffer_arr[10], temp;
unsigned long int avgval;
float ph_act;
String outString;


float phThreshold = 6.0;
float ecThreshold = 2.0;
float tempThreshold = 35;
# 48 "E:/Octa/Documents/PlatformIO/Projects/pH-calib-UNO/src/ph_calib.ino"
void setup();
void loop();
void sendData();
void displayData();
#line 48 "E:/Octa/Documents/PlatformIO/Projects/pH-calib-UNO/src/ph_calib.ino"
void setup() {
    outString.reserve(200);
    Serial.begin(9600);
    outSerial.begin(19200);
    Wire.begin();


    lcd.init();
    lcd.backlight();
    lcd.setCursor(5, 1);
    lcd.print("HM&M SYSTEM");
    delay(1000);
    lcd.clear();


    sht20.initSHT20();
    delay(100);
    Serial.println("SHT20 Sensor Ready");


    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);
    gravityTds.setAdcRange(1024);
    gravityTds.begin();


    pinMode(MotorPin, OUTPUT);
    digitalWrite(MotorPin, LOW);

    timer.setInterval(1000L, displayData);
    timer.setInterval(5000L, sendData);
}

void loop() {
    timer.run();
}

void sendData() {
    outString = String(temperature) + ',' + String(tdsValue) + ',' + String(ecValue) + ',' + String(ph_act);
    outSerial.println(outString);
}

void displayData() {

    float humd = sht20.readHumidity();
    temperature = sht20.readTemperature();


    gravityTds.setTemperature(temperature);
    gravityTds.update();
    tdsValue = gravityTds.getTdsValue();
    ecValue = tdsValue * 1.0;






    for (int i = 0; i < 10; i++) {
        buffer_arr[i] = analogRead(A0);
        delay(30);
    }


    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
            if (buffer_arr[i] > buffer_arr[j]) {
                temp = buffer_arr[i];
                buffer_arr[i] = buffer_arr[j];
                buffer_arr[j] = temp;
            }
        }
    }


    avgval = 0;
    for (int i = 2; i < 8; i++) {
        avgval += buffer_arr[i];
    }


    float volt = (float)avgval * 5.0 / 1024.0 / 6.0;
    ph_act = -5.70 * volt + calibration_value;




    Serial.print("Time:");
    Serial.print(millis() / 1000);
    Serial.print(" sec, Temp:");
    Serial.print(temperature, 2);
    Serial.print(" C, Humidity:");
    Serial.print(humd, 2);
    Serial.print("%, pH:");
    Serial.print(ph_act, 2);
    Serial.print(", EC:");
    Serial.print(ecValue, 2);
    Serial.println(" S/m");


    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(temperature, 2);
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Humidity:");
    lcd.print(humd, 2);
    lcd.print("%");

    lcd.setCursor(0, 2);
    lcd.print("pH:");
    lcd.print(ph_act, 2);
    lcd.print("   ");

    lcd.setCursor(0, 3);
    lcd.print("EC:");
    lcd.print(ecValue, 2);
    lcd.print("S/m");

    float t = millis() / 1000.0;




    if ((ph_act < phThreshold || ecValue > ecThreshold || temperature > tempThreshold)) {
      digitalWrite(MotorPin, HIGH);
    } else {
      digitalWrite(MotorPin, LOW);
    }

}