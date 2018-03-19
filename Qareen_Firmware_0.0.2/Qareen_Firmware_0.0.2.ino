#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

float gyroscopePitch = 0;
float gyroscopeRoll = 0;
float gyroscopeYaw = 0;
float accPitch = 0;
float accRoll = 0;
float accYaw = 0;
float truePitch = 0.0;
float trueRoll = 0.0;
float trueYaw = 0.0;
unsigned long lastMillis = 0;
unsigned long deltaTime = 0;

//[0] = pitch, [1] = roll, [2] = yaw, [3] = altitude.//////////////////////////////////////////
float inputPRYA[4] = {0, 0, 0, 20};
float errorDomain[3] = {4, 4, 4};
float slowerMotors = 1.87;
float fasterMotors = 0.34;
int maxMotorSpeed = 245;
int minMotorSpeed = 0;


//[1] = O^^^O = [0]
//      ^   ^
//[2] = O^^^O = [3]
//0 is no frequency, 225 is max frequency. Probably don't want to max this out though.
int ESCFreq[4] = {inputPRYA[3], inputPRYA[3], inputPRYA[3], 0};

void tempCompliment();
void findAccelAngle(sensors_event_t &acc);
void findGyroAngle(sensors_event_t &gyr);
void motorControl();
void tempLEDSimulation();

/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

///////TEMP LEDS///////
#define ESCZero     3
#define ESCOne      6
#define ESCTwo      5
#define ESCThree    9

void configureSensor(void){
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

void setup(void) {
  while (!Serial);  // wait for flora/leonardo
  
  Serial.begin(9600);
  //Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  pinMode(ESCZero, OUTPUT);
  pinMode(ESCOne, OUTPUT);
  pinMode(ESCTwo, OUTPUT);
  pinMode(ESCThree, OUTPUT);
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    //Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  //Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  //Serial.println("");
}

void loop(void) {  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  findAccelAngle(accel);
  findGyroAngle(gyro);

  tempCompliment();
  tempLEDSimulation();

}

void findAccelAngle(sensors_event_t &acc){

  accPitch = (atan2(acc.acceleration.y,acc.acceleration.z)) * RAD_TO_DEG;
  accRoll = (atan2(acc.acceleration.x,acc.acceleration.z)) * RAD_TO_DEG;
  
  }

void findGyroAngle(sensors_event_t &gyr){

  deltaTime = millis() - lastMillis;
  lastMillis = millis();
  gyroscopePitch = gyr.gyro.x*deltaTime/1000;
  gyroscopeRoll = gyr.gyro.y*deltaTime/1000;
  gyroscopeYaw = gyr.gyro.z*deltaTime/1000;
  
  }

void tempCompliment(){
  
  truePitch = (.96 * (truePitch + gyroscopePitch)) + (.04 * accPitch);
  trueRoll  = (.92 * (trueRoll  + gyroscopeRoll )) + (.08 * accRoll );
  }

void motorControl(){
  
}

void tempLEDSimulation(){

   ESCFreq[0] = 0;
   ESCFreq[1] = 0;
   ESCFreq[2] = 0;
   ESCFreq[3] = 0;
   trueRoll += .5;
    Serial.println(trueRoll);
   if(truePitch > (inputPRYA[0] + errorDomain[0])){
        ESCFreq[0] = (abs(inputPRYA[0] - truePitch)/fasterMotors;
        ESCFreq[1] = (abs(inputPRYA[0] - truePitch)/fasterMotors;
        ESCFreq[2] = -(abs(inputPRYA[0] - truePitch)/slowerMotors;
        ESCFreq[3] = -(abs(inputPRYA[0] - truePitch)/slowerMotors;
   }else if(truePitch < (inputPRYA[0] - errorDomain[0])){
    //if(ESCFreq[3] <= maxMotorSpeed && ESCFreq[0] >= minMotorSpeed){
        ESCFreq[0] = -(abs(inputPRYA[0] - truePitch)/slowerMotors;
        ESCFreq[1] = -(abs(inputPRYA[0] - truePitch)/slowerMotors;
        ESCFreq[2] = (abs(inputPRYA[0] - truePitch)/fasterMotors;
        ESCFreq[3] = (abs(inputPRYA[0] - truePitch)/fasterMotors;
      //}
   }
   
  if(trueRoll > (inputPRYA[1] + errorDomain[1])){
    //if(ESCFreq[1] <= maxMotorSpeed && ESCFreq[0] >= minMotorSpeed){
        ESCFreq[0] = (abs(inputPRYA[1] - trueRoll)/fasterMotors;
        ESCFreq[1] = -(abs(inputPRYA[1] - trueRoll)/slowerMotors;
        ESCFreq[2] = -(abs(inputPRYA[1] - trueRoll)/slowerMotors;
        ESCFreq[3] = (abs(inputPRYA[1] - trueRoll)/fasterMotors;
     // }
  }else if(trueRoll < (inputPRYA[1] - errorDomain[1])){
    //if(ESCFreq[0] <= maxMotorSpeed && ESCFreq[1] >= minMotorSpeed){
        ESCFreq[0] = -(abs(inputPRYA[1] - trueRoll)/slowerMotors;
        ESCFreq[1] = (abs(inputPRYA[1] - trueRoll)/fasterMotors;
        ESCFreq[2] = (abs(inputPRYA[1] - trueRoll)/fasterMotors;
        ESCFreq[3] = -(abs(inputPRYA[1] - trueRoll)/slowerMotors;
      //}
  }
    analogWrite(ESCTwo,ESCFreq[2] + inputPRYA[3]);
    analogWrite(ESCThree,ESCFreq[3] + inputPRYA[3]);
    analogWrite(ESCZero,ESCFreq[0] + inputPRYA[3]);
    analogWrite(ESCOne,ESCFreq[1] + inputPRYA[3]);
}










