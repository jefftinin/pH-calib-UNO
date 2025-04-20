#include <Wire.h>
#include <DFRobot_SHT20.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleTimer.h>
#include <EEPROM.h>
#include "GravityTDS.h"
#include <SoftwareSerial.h>

SoftwareSerial outSerial(10,11);

// Initialize SHT20 Sensor
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

// Initialize LCD with I2C address 0x27, 20 columns, 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);

// TDS Sensor Pin
#define TdsSensorPin A1
#define MotorPin 8
GravityTDS gravityTds;

// Sensor variables
float temperature = 25;
float tdsValue = 0;
float ecValue = 0;
float ec25Value = 0;

// pH sensor variables
SimpleTimer timer;
float calibration_value = 26.34 - 0.5;
int buffer_arr[10], temp;
unsigned long int avgval;
float ph_act;
String outString;

// User adjustable
float phThreshold = 5.5;
float ecThreshold = 2.0;
float tempThreshold = 35;

float concentrationThreshold = 1.0;

// First-order decay model threshold logic (example model)
// Adjust these coefficients to match the decay characteristics of your herbicide
//float k = 0.1; // Decay constant
//float C0 = 10.0; // Initial concentration

void setup() {
    Serial.begin(9600);
    outSerial.begin(19200);
    Wire.begin();

    // Initialize LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(5, 1);
    lcd.print("HM&M SYSTEM");
    delay(1000);
    lcd.clear();

    // Initialize SHT20 sensor
    sht20.initSHT20();
    delay(100);
    Serial.println("SHT20 Sensor Ready");

    // Initialize TDS sensor
    gravityTds.setPin(TdsSensorPin);
    gravityTds.setAref(5.0);
    gravityTds.setAdcRange(1024);
    gravityTds.begin();

    // Initialize motor control pin
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
    // Read SHT20 Temperature and Humidity
    float humd = sht20.readHumidity();
    temperature = sht20.readTemperature();

    // Read and calculate TDS and EC values
    gravityTds.setTemperature(temperature);
    gravityTds.update();
    tdsValue = gravityTds.getTdsValue();
    ecValue = tdsValue * 1.0; // Simple conversion factor

    ec25Value = ecValue/(1+0.019*(temperature - 25.0));



    // Read pH sensor values
    for (int i = 0; i < 10; i++) {
        buffer_arr[i] = analogRead(A0);
        delay(30);
    }

    // Sort values to get median
    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
            if (buffer_arr[i] > buffer_arr[j]) {
                temp = buffer_arr[i];
                buffer_arr[i] = buffer_arr[j];
                buffer_arr[j] = temp;
            }
        }
    }

    // Average middle 6 values
    avgval = 0;
    for (int i = 2; i < 8; i++) {
        avgval += buffer_arr[i];
    }

    // Calculate pH value
    float volt = (float)avgval * 5.0 / 1024.0 / 6.0;
    ph_act = -5.70 * volt + calibration_value;

    // COncentration
    float conc = ph_act ;
    // Print sensor data to Serial Monitor
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

    // Update LCD display without clearing
    lcd.setCursor(0, 0);
    lcd.print("Temp:");
    lcd.print(temperature, 2);
    lcd.print("C"); // Extra spaces to overwrite old data

    lcd.setCursor(0, 1);
    lcd.print("Humidity:");
    lcd.print(humd, 2);
    lcd.print("%"); // Extra spaces to prevent leftover digits

    lcd.setCursor(0, 2);
    lcd.print("pH:");
    lcd.print(ph_act, 2);
    lcd.print("   "); // Ensures no flickering

    lcd.setCursor(0, 3);
    lcd.print("EC:");
    lcd.print(ecValue, 2);
    lcd.print("S/m");

    float t = millis() / 1000.0;
    //float concentration = C0 * exp(-k * t);
    
    // Simulated threshold using pH, EC, and temp (replace with your actual condition)
    //if ((ph_act < phThreshold || ecValue > ecThreshold || temperature > tempThreshold) && concentration > concentrationThreshold) {
    if ((ph_act < phThreshold || ecValue > ecThreshold || temperature > tempThreshold)) {
      digitalWrite(MotorPin, HIGH);
    } else {
      digitalWrite(MotorPin, LOW);
    }
    
}
