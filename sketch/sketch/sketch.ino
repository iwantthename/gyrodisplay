#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
 
MPU6050 accelgyro;
 
int16_t ax, ay, az;
int16_t gx, gy, gz;
 
#define LED_PIN 13
bool blinkState = false;
 
String str_out;
 
void setup() {
    Wire.begin();
    Serial.begin(9600);
 
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
 
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
}
 
void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
    str_out = String(ax) + "," + String(ay) + "," + String(az) + "," + String(gx) + "," + String(gy) + "," + String(gz);
    Serial.println(str_out);
     
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
