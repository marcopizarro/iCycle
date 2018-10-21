//
//#include <Wire.h>
//#include <SPI.h>
//#include <Adafruit_LSM9DS1.h>
//#include <Adafruit_Sensor.h>  // not used in this demo but required!
//
//// i2c
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
//
//#define LSM9DS1_SCK A5
//#define LSM9DS1_MISO 12
//#define LSM9DS1_MOSI A4
//#define LSM9DS1_XGCS 6
//#define LSM9DS1_MCS 5

#define SENSORPIN 4
#define SENSORPINTWO 5


// variables will change:
int sensorState = 0, lastState=0, sensorStateTwo = 0, duaSensor=0;
double comp = 5;

//void setupSensor()
//{
//  // 1.) Set the accelerometer range
//  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
//  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
//  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
//  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
//  
//  // 2.) Set the magnetometer sensitivity
//  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
//  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
//  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
//  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
//
//  // 3.) Setup the gyroscope
//  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
//  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
//  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
//}

void setup() {
  // initialize the LED pin as an output:
  pinMode(9, OUTPUT);      
  pinMode(10, OUTPUT); 
  pinMode(11, OUTPUT); 
  // initialize the sensor pin as an input:
  pinMode(SENSORPIN, INPUT);
  pinMode(SENSORPINTWO, INPUT);
  digitalWrite(SENSORPIN, HIGH); // turn on the pullup
  digitalWrite(SENSORPINTWO, HIGH); // turn on the pullup
  Serial.begin(9600);

//  while (!Serial) {
//    delay(1); // will pause Zero, Leonardo, etc until serial console opens
//  }
//  
//  Serial.println("LSM9DS1 data read demo");
//  
//  // Try to initialise and warn if we couldn't detect the chip
//  if (!lsm.begin())
//  {
//    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
//    while (1);
//  }
//  Serial.println("Found LSM9DS1 9DOF");
//
//  // helper to just set the default scaling we want, see above!
//  setupSensor();
//  
}

void loop(){
//  lsm.read();  /* ask it to read in the data */ 
analogWrite(11, 255);
  /* Get a new sensor event */ 
//  sensors_event_t a, m, g, temp;
//
//  lsm.getEvent(&a, &m, &g, &temp); 
  // read the state of the pushbutton value:
  sensorState = digitalRead(SENSORPIN);
  sensorStateTwo = digitalRead(SENSORPINTWO);
  duaSensor = sensorState && sensorStateTwo;
  
  // check if the sensor beam is broken
  // if it is, the sensorState is LOW:
  if (sensorState == HIGH && sensorStateTwo != HIGH) {     
    Serial.println("right");
    turnRight();
  } 
  else if(sensorState == HIGH && sensorStateTwo == HIGH) {
    // turn LED off:
    Serial.println("left");
    turnLeft();
  }
  
  
  if (duaSensor && !lastState || !duaSensor && lastState) {
    Serial.println("changed state");
    comp+=1.7;
  } 
  else{
    if(comp>0){
      comp-=.1;
    }
  }

  if(comp/10 >=0.5){
    Serial.println("emergency");
    comp-=.3;
    emergency();
  }
  
//  Serial.println(a.acceleration.z);
//  
//  if(a.acceleration.z <= .2){
//    analogWrite(11, 255);
//  }
//  else{
//    analogWrite(11, 0);
//  }

  Serial.println(comp);

  lastState = sensorState && sensorStateTwo;
}

void turnLeft()
{
  analogWrite(9, 255);
  delay(500);
  analogWrite(9, 0);
  delay(500);
}

void turnRight()
{
  analogWrite(10, 255);
  delay(500);
  analogWrite(10, 0);
  delay(500);
}
void emergency(){
  analogWrite(9, 127);
  analogWrite(10, 127);
  delay(100);
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(100);
   analogWrite(9, 127);
  analogWrite(10, 127);
  delay(100);
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(100);
   analogWrite(9, 127);
  analogWrite(10, 127);
  delay(100);
  analogWrite(9, 0);
  analogWrite(10, 0);
  delay(100);
}


