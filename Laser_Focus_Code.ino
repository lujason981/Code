#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;

// button setup
const int buttonPin = 8;
const int buttonLED = 10;
unsigned int buttonState = 0;
unsigned int setting = 0;

//rgb led setup
const int redPin = 3;             //pin to control the red LED inside the RGB LED
const int greenPin = 5;           //pin to control the green LED inside the RGB LED
const int bluePin = 6;            //pin to control the blue LED inside the RGB LED    

//Store distance readings to get rolling average
#define Cutting_SIZE 20
int history[Cutting_SIZE];
byte historySpot;
float distance = 0;
float avg = 0;

//store measurement readings for higher accuracy average
#define Measure_SIZE 50
int measure[Measure_SIZE];
byte measureSpot;
int a = 0;
int b = 0;
int c = 0;
int d = 0;

void setup() {

  Serial.begin (9600);                        //set up a serial connection with the computer

  //sensor setup
  Wire.begin();
  distanceSensor.begin();

  //button setup
  pinMode(buttonPin, INPUT);                  //set button pin as input
  pinMode(buttonLED, OUTPUT);

  //set the RGB LED pins to output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  //average function setup
  for (int x = 0; x < Cutting_SIZE; x++){
    history[x] = 0;
  }
  for (int x = 0; x < Measure_SIZE; x++){
    measure[x] = 0;
  }
}

void loop() {
  //reset RGB LED
  analogWrite(redPin, 0);
  analogWrite(greenPin, 0);
  analogWrite(bluePin, 0);

  digitalWrite(buttonLED, HIGH);

  //checks if button has been pressed
  buttonState = digitalRead(buttonPin);

  //if the button has been pressed...
  if (buttonState == HIGH) {
    //light up LED white to indicate that it is on
    digitalWrite(buttonLED, LOW);
    setting = 1;                              //setting = on
    delay(250);                               //delay to prevent accidental double press

    //Setting 1: Lights up with certain colors when at certain distances. Refer to the reference sheet for color of each materal.
    while (setting == 1) {
      buttonState = digitalRead(buttonPin);   //check if the button is pressed again
      distance = average();
      
      analogWrite(redPin, 0);
      analogWrite(greenPin, 0);
      analogWrite(bluePin, 0);

      //swtich to output distance setting
      if (buttonState == HIGH) {
        
        //make LED white
        analogWrite(redPin, 100);
        analogWrite(greenPin, 100);
        analogWrite(bluePin, 100);

        setting = 2;                          //setting = output distance
        delay(250);                           //delay to prevent accidental double press
      }

//MATERIAL SETTINGS

      //FPL Distance Setting (Blue)
      while (distance > 118 && distance < 126 && setting == 1){
        distance = average();
        Serial.print("Distance(mm): ");
        Serial.print(distance);
        Serial.println();
        
        if (distance < 121){
          analogWrite(redPin, 0);
          analogWrite(greenPin, 255);
          analogWrite(bluePin, 0);
        }
        
        else if (distance > 123){
          analogWrite(redPin, 255);
          analogWrite(greenPin, 0);
          analogWrite(bluePin, 0);
        }

        else if (distance > 121 && distance < 123){      
          analogWrite(redPin, 0);
          analogWrite(greenPin, 0);
          analogWrite(bluePin, 255);
        }
        
        buttonState = digitalRead(buttonPin);

        //swtich to output distance setting
        if (buttonState == HIGH) {              
          
          analogWrite(redPin, 100);
          analogWrite(greenPin, 100);
          analogWrite(bluePin, 100);
          delay(250);
          setting = 2;
          
        } 
      }

      //________ Distance Setting (Purple)
      while (distance > 140 && distance < 148 && setting == 1){
        distance = average();
        Serial.print("Distance(mm): ");
        Serial.print(distance);
        Serial.println();
        
        if (distance < 143){
          analogWrite(redPin, 0);
          analogWrite(greenPin, 255);
          analogWrite(bluePin, 0);
        }
        
        else if (distance > 145){
          analogWrite(redPin, 255);
          analogWrite(greenPin, 0);
          analogWrite(bluePin, 0);
        }

        else if (distance > 143 && distance < 145){      
          analogWrite(redPin, 255);
          analogWrite(greenPin, 0);
          analogWrite(bluePin, 255);
        }
        
        buttonState = digitalRead(buttonPin);

        //swtich to output distance setting
        if (buttonState == HIGH) {              
          
          analogWrite(redPin, 100);
          analogWrite(greenPin, 100);
          analogWrite(bluePin, 100);
          delay(250);
          setting = 2;
          
        } 
      }


      //PASTE NEW MATERIAL SETTING ABOVE THIS LINE AND UNCOMMENT.
    }

    //Second setting: Outputs the distance to the serial monitor.
    while (setting == 2) {
      distance = average(); //Get the result of the measurement from the sensor
      a = distance - 4;
      b = distance + 4;
      c = distance - 1;
      d = distance + 1;
      Serial.print("Distance(mm): ");
      Serial.print(distance);
      Serial.print('\t');
      Serial.print("a = ");
      Serial.print(a);
      Serial.print('\t');
      Serial.print("b = ");
      Serial.print(b);
      Serial.print('\t');
      Serial.print("c = ");
      Serial.print(c);
      Serial.print('\t');
      Serial.print("d = ");
      Serial.print(d);
      Serial.print('\t');
      Serial.println();
      
      buttonState = digitalRead(buttonPin);   //check if the button is pressed again

      if (buttonState == HIGH) {
        digitalWrite(buttonLED, HIGH);
        analogWrite(redPin, 0);
        analogWrite(greenPin, 0);
        analogWrite(bluePin, 0);

        setting = 0;
        delay(250);
      }
    }
  }
}


//average distance function
float average()
{
  float avgDistance = 0;
  if (setting == 1){
    distanceSensor.startRanging(); //Write configuration block of 135 bytes to setup a measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    int reading = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    history[historySpot] = reading;
    if (++historySpot == Cutting_SIZE)
      historySpot = 0;

    
    for (int x = 0; x < Cutting_SIZE; x++)
      avgDistance += history[x];

    avgDistance /= Cutting_SIZE;
  }
  else if (setting == 2){
    distanceSensor.startRanging(); //Write configuration block of 135 bytes to setup a measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    int reading = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    measure[measureSpot] = reading;
    if (++measureSpot == Measure_SIZE)
      measureSpot = 0;

    
    for (int x = 0; x < Measure_SIZE; x++)
      avgDistance += measure[x];

    avgDistance /= Measure_SIZE;
  }
  return avgDistance;
  
}

//BLANK MATERIALS TEMPLATE

//Below is the function for new materials. Copy and paste ALL code when adding a new material.
//Fill in the blanks with the material name, distance, and desired color.

//      //______ Distance Setting (*insert color*)
//      while (distance > __a__ && distance < __b__ && setting == 1){
//        distance = average();
//        Serial.print("Distance(mm): ");
//        Serial.print(distance);
//        Serial.println();
//
//        if (distance < __c__){
//          analogWrite(redPin, 0);
//          analogWrite(greenPin, 255);
//          analogWrite(bluePin, 0);
//        }
//        
//        else if (distance > __d__){
//          analogWrite(redPin, 255);
//          analogWrite(greenPin, 0);
//          analogWrite(bluePin, 0);
//        }
//
//        else if (distance > __c__ && distance < __d__){       
//          analogWrite(redPin, ___);
//          analogWrite(greenPin, ___);
//          analogWrite(bluePin, ___);
//        }
//        
//        buttonState = digitalRead(buttonPin);
//
//        //swtich to output distance setting
//        if (buttonState == HIGH) {              
//          
//          analogWrite(redPin, 100);
//          analogWrite(greenPin, 100);
//          analogWrite(bluePin, 100);
//          delay(250);
//          setting = 2;
//          
//        } 
//      }

