#include <Servo_ESP32.h> //ESP32 Servo Library
#include "BluetoothSerial.h" //ESP32 Bluetooth Library

BluetoothSerial ESP_BT; //initialise BT 
Servo_ESP32 lace_servo; //assign servo to variable

bool bluet_tighten = false;//initialise tightening via Bluetooth
bool bluet_loosen = false;//initialise loosening via Bluetooth

//Force Sensor
int forceSensor = 34; //D34 ESP32
int forceVal = analogRead(forceSensor); //read analog from ADC4

//Battery Monitoring
int batt = 33;
int battAnalog = analogRead(batt);
float battVolt = 0;
float battPercent = 0;

//Servo
int servoPin = 14; //D14 ESP32

//Capacitive Touch
int tighten = 27; //D27 ESP32 
int loosen = 13;  //D13 ESP32

//Capacitive Touch Initial Setup
int threshold = 55;
bool touch2detected = false;
bool touch3detected = false;

void gotTouch2(){//tighten - goes CW
 touch2detected = true;
}

void gotTouch3(){//loosen - goes CCW
 touch3detected = true;
}

void BL_tighten(){
  if (ESP_BT.read() == 5){
    bluet_tighten = true;
  }
}

void BL_loosen(){
  if (ESP_BT.read() == 2){
  bluet_loosen = true;    
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  touchAttachInterrupt(tighten, gotTouch2, threshold);
  touchAttachInterrupt(loosen, gotTouch3, threshold);

  //--bluetooth connection--
  ESP_BT.begin("NF-LEFT");

//force sensor & battery monitoring resolution
  analogReadResolution(10);

  pinMode(servoPin, OUTPUT);
}

void loop(){
  
  //--laces--
  lace_servo.detach();
  //sensors for tightening conditions
  if(touch2detected || ESP_BT.read() == 5 || forceVal > 800 ){
    touch2detected = false;
    //bluet_tighten = false;
    Serial.println("--Tightening--");
    lace_servo.attach(servoPin);
    lace_servo.write(10);
  }
  //sensors for loosening conditions
  if(touch3detected || ESP_BT.read() == 2){
    touch3detected = false;
   // bluet_loosen = false;
    Serial.println("--Loosening--");
    lace_servo.attach(servoPin);
    lace_servo.write(180);
  }
 
  battVolt = (battAnalog/1023)*4.2;
  int forceVal = analogRead(forceSensor); //read analog from ADC4

  Serial.printf("Tighten Val = %d\n",touchRead(tighten));
  Serial.printf("Loosen Val = %d\n",touchRead(loosen));
  Serial.printf("Battery = %dV \n",battAnalog);
  //ESP_BT.write() = battVolt
  Serial.printf("Bluetooth Received = %d\n",ESP_BT.read());
  Serial.println(forceVal);
  delay(200);
  

}
