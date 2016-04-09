// this example is public domain. enjoy!
// www.ladyada.net/learn/sensors/thermocouple

/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  The pid is designed to output an analog value,
 * but the relay can only be On/Off.
 *
 *   To connect them together we use "time proportioning
 * control"  Tt's essentially a really slow version of PWM.
 * First we decide on a window size (5000mS say.) We then 
 * set the pid to adjust its output between 0 and that window
 * size.  Lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the 
 * window being "Relay Off Time"
 ********************************************************/

#include "max6675.h"
#include <PID_v1.h>

#define RelayPin 13



int thermoDO = 5;
int thermoCS = 6;
int thermoCLK = 7;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
int vccPin = 3;
int gndPin = 2;

//Define Variables we'll be connecting to
double Setpoint, Input, Output, kP, kI, kD;


//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2.5,3,1, DIRECT);

int WindowSize = 1000;
unsigned long windowStartTime;
unsigned long now = millis();
unsigned long tcReadTimer = millis();
  
void setup() {
  //Serial.begin(9600);
  Serial.begin(38400);
  
  // use Arduino pins 
  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);
  pinMode(RelayPin, OUTPUT); digitalWrite(RelayPin, LOW);
  Serial.println("MAX6675 test");
  // wait for MAX chip to stabilize
  delay(500);
  
  windowStartTime = millis();
  

  //initialize the variables we're linked to
  Setpoint = 350;
  
  kP = 150;   // set proportional constant
  kI = .1;   // set integral constant
  kD = 20;   // set derivative constant
  
//  kP = 200;   // set proportional constant
//  kI = .025;   // set integral constant
//  kD = 20;   // set derivative constant

//  kP = 2.5;   // set proportional constant
//  kI = 3;   // set integral constant
//  kD = 5;   // set derivative constant
  
  //set up the PID Tuning Parameters
  myPID.SetTunings(kP,kI,kD);

  //tell the PID to range between 0 and the full window size
  //myPID.SetOutputLimits(0, 255);
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
 
}

void loop() {
  // basic readout test, just print the current temp
  	   //Serial.print("C = "); 
	   //Serial.println(thermocouple.readCelsius());
	   //Serial.print("F = ");
	   //Serial.println(thermocouple.readFarenheit());

  if(now - tcReadTimer > 1000)  // read the thermocouple every 300ms--faster causes problems
  {
    Input = thermocouple.readFarenheit();
    tcReadTimer = now;
    //Serial.println("reading tc");
  }
  
  myPID.Compute();
  


  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  now = millis();
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if(Output > now - windowStartTime) 
  {
    digitalWrite(RelayPin,HIGH);
    Serial.print("Relay HIGH   ");
  }    
  else
  { 
    digitalWrite(RelayPin,LOW);
    Serial.print("Relay LOW    ");
  }  

  Serial.print("Input (Deg F.)= ");
  Serial.print(Input);
  Serial.print("   Set Point = ");
  Serial.print(Setpoint);
  Serial.print("   Output = ");
  Serial.print(Output);
  Serial.print("   wdwStrtTm ");
  Serial.print(windowStartTime);
  Serial.print("   now ");
  Serial.print(now);
  Serial.print("     now-wdwStrtTm=");
  Serial.print(now - windowStartTime);
  Serial.print("     tcReadTimer=");
  Serial.println(tcReadTimer);
  //delay(200);
}

