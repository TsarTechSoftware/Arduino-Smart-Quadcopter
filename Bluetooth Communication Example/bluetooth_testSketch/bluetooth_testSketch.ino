int incomingByte = 0;   // for incoming serial data
int led = 13;

void aProto();
void bProto();
void cProto();

void setup()
{
  Serial.begin(9600); // // opens serial port, sets data rate to 9600 bps
 
  pinMode(led, OUTPUT);
}
 
void loop(){ 
  digitalWrite(led, HIGH);    // turn the LED off by making the voltage LOW
 
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();//not using this

   if(incomingByte == 'a'){
      aProt();
    } 
    else if(incomingByte == 'b'){
      bProt();
      }
     else if(incomingByte == 'c'){
      cProt();
     }
   delay(500);
  }
}

void aProt(){

    for(int a = 0; a < 10; a++){
     digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
     delay(100);
     digitalWrite(led, HIGH);
     delay(100);
    }
}

void bProt(){

  for(int a = 0; a < 100; a++){

     digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
     delay(10);
     digitalWrite(led, HIGH);
     delay(10);
    
    }
}

void cProt(){
    Serial.write("This is the \"C\" protical.\n");
  }

