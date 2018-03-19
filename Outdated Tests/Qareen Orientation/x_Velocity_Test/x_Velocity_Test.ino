#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
  
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

double velocity = 0;
int seconds = 0;

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  while (!Serial);  // wait for flora/leonardo
  
  Serial.begin(9600);
  Serial.println(F("Accelerometer Test\n"));
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
}

void loop(void) 
{  

  bool addSecond = false;

  sensors_event_t accel, mag, gyro, temp;;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  if(accel.acceleration.x > 1.50 || accel.acceleration.x < -1.50){

    addSecond = true;
    
    }else{

      seconds = 0;
      
      }

  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");

  Serial.println("**********************\n");

  velocity = velocity + accel.acceleration.x * seconds;

  Serial.print("Velocity = "); Serial.print(velocity); Serial.print("\n");

  if(addSecond){
    seconds++;
    }

  delay(200);
}
