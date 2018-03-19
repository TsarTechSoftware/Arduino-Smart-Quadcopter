double currentLat;
double currentLon;
double initLat;
double initLon;

float directionFacing;
void returnToHome();

void setup() {
  // start gps
  // calibrate quickly
  // delay until ready
  // clock coords currentLat, currentLong
  // initLat = currentLat
  // initLon = curruntLon

}

void loop() {

  // find directionFacing
  

}

void returnToHome(){

  // turn until direction facing = north
  double o = currentLat - initLat;
  double a = currentLon - initLon;
  double h = sqrt(pow(x,2) + pow(y,2));
  // typecast the below statement
  float angle = asin(o/h);
  
  
  }
