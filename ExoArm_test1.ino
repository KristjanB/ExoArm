#include <ArduinoTimer.h>
#include <CircularBuffer.h>
#include <CommandHandler.h>
#include <CommandProcessor.h>
#include <DataStore.h>
#include <DeviceAddress.h>
#include <EEPROMStore.h>
#include <Filter.h>
#include <MegunoLink.h>
#include <MessageHeaders.h>

#include "MegunoLink.h" // Helpful functions for communicating with MegunoLink Pro. 
#include "filter.h"
#include "Servo.h"

TimePlot plot;

int potVal;
int angle;
float dtrate;
double last;

ExponentialFilter<float> FilteredMuscleValue(5, 20); // right- weight, left- start value

void moveup(float writemotor){
     //writemotor = map(writemotor, 0, -80, 255, 0);
     digitalWrite(3, HIGH);
     digitalWrite(4, LOW);
     analogWrite(5, writemotor);

}

void stopmove(){
        digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      analogWrite(5, 0);
}

void movedown(float writemotor){

      writemotor = map(writemotor, 0,-255, 0, 255);
      digitalWrite(3, LOW);
      digitalWrite(4, HIGH);
      analogWrite(5, writemotor);
}


float errorSum;
float Kp = 2;
float Ki = 0;

float computePID(int setpoint){

  
  angle = analogRead(2);
  angle = map(angle, 0, 344, 0, 90); // dejanski kot
  Serial.print(angle);
 
  double time_elapsed = micros();
  dtrate = (time_elapsed - last) / 1000.f;
  last = time_elapsed;
  
  float error = setpoint - angle;
  errorSum += error * dtrate;
  errorSum = constrain(errorSum, -255, 255);
//Serial.println(error);
  return (Kp * error) + (Ki * errorSum);



  
//  Serial.println(time_elapsed, 5);
}

int Step = 0;

void setup()
{
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);
  Step = 0;
}

int k = 1;
int previousMillis = 0;
int interval = 50;
void loop()
{
  int program_runtime = millis();
  
//
  int readingUp = digitalRead(8);
  int readingDown = digitalRead(9);
  
  if(readingUp == 1 && readingDown == 0){
    if(program_runtime - previousMillis > interval) {
       previousMillis = program_runtime;  
       Step -= 5;
    }
  
  }
  if(readingDown == 1 && readingUp == 0) {
    if(program_runtime - previousMillis > interval) {
       previousMillis = program_runtime;  
       Step += 5;
    }

  }
  Step = constrain(Step, -150, 150);
  if(readingDown == 0 && readingUp == 0) Step = 0;
  


  //plot.SendData("pid", writemotor);
  Serial.println(Step);
  if(Step > 0) moveup(Step);
  if(readingUp == 0 && readingDown == 0) stopmove();
  if(Step < 0) movedown(Step);
 
}

