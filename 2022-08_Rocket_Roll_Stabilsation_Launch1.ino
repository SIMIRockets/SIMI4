/****************************************************************************
SIMI Rockets
Rocket Roll Stabilization Launch 2
Data Recording - Servos installed
2022-10 
*/

#include "Arduino_NineAxesMotion.h"        //IMU Sensor Library
#include <Wire.h>                          //Serial Library
#include <Servo.h>                         //Servo Library
#include <SPI.h>                           //SPI Library
#include <SD.h>                            //SD Library

Servo myServo;                             //Object for Servo Control

NineAxesMotion mySensor;                  //Object for 9DOF sensor 
unsigned long lastStreamTime = 0;         //To store the last streamed time stamp
const int streamPeriod = 20;              //To stream at 50Hz without using additional timers (time period(ms) =1000/frequency(Hz))
float Orient = 0;                         //roll orientation of the rocket
int LaunchOrient = 0;                     //roll orientation of the rocket at launch event
float PrevOrient = 0;                     //last measured roll orientation
int Deviation = 0;                        //deviation of actual roll orientation from launch orientation
bool Launch = false;                      //variable for launch event
int angle = 90;                           //servo angle control surface in vertical pasition

float angvelY = 0;                        //angular velocity Y axis
float angvelYold = 0;                     //previous angular velocity 
float linaccY = 0;                        //linear accelaration Y axis (negative = upwards)
int Roll = 0;
int milliOld = 0;
int milliNew = 0;
int dt = 0;


void setup()
{
  //Peripheral Initialization
  Serial.begin(9600);                                //Initialize the Serial Port to view information on the Serial Monitor
  Wire.begin();                                      //Initialize I2C communication to the let the library communicate with the sensor.
                                
  mySensor.initSensor();                             //Sensor Initialization
  mySensor.setOperationMode(OPERATION_MODE_NDOF);    //9 DOF mode for IMU sensor 
  mySensor.setUpdateMode(MANUAL);	                   //Setting to MANUAL requires fewer reads to the sensor
  mySensor.updateAccelConfig();
  delay(2000);
  Launch = false;
  myServo.attach(6);                                 //Servo PWM pin 6
  
  Serial.print("Initializing SD card...");           // see if the card is present and can be initialized:
                                                    
  if (!SD.begin(4)) {
    Serial.println("Card failed, or not present");   // don't do anything more:                        
    while (1);
  }
  Serial.println("card initialized."); 
}

void loop()
{
  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();     //Update time stamp for stream time
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGyro();         //Update Gyro data
    PrevOrient = Orient;           //Store previous roll Orientation
    milliOld = milliNew;
    
    Serial.print(" Gyro Y ");
    angvelY = mySensor.readGyroY();
    angvelY = angvelYold * 0.7 + angvelY * 0.3;         //low pass filter for angular velocity 
    Serial.print(angvelY);
    angvelYold = angvelY;
    Roll = angvelY;                                     //roll rate for differential control
    linaccY = mySensor.readLinearAcceleration(Y_AXIS);  //reading linear acceleration Y axis  

    milliNew = millis();
    dt = milliNew - milliOld;                       //time between measurements 
    Orient = PrevOrient + (angvelY * dt / 1000);    //integration of change of orientation angle 
    Serial.print(" Orient: ");
    Serial.print(Orient);
    
    //Roll = Roll - ((LaunchOrient - Orient) / 2)   //roll rate corrected by proportional control is off

    Serial.print(" lY: ");
    Serial.print(linaccY);  //Linear Acceleration Y-Axis data
    Serial.print("m/s2 ");
    if (linaccY < -4) //detecting linear accel. >4m/s2 in flight direction
    {
      Serial.print("launch detected");
      if (Launch == false)          //if launch event not detected yet
      {
        LaunchOrient = Orient;      //store roll orientation of rocket at launch in LaunchOrient
      }
      
      Launch = true;                //store launch event
    }
    if (Launch == true)
    {
      if (Roll < -400)
      {
        Roll = -400;    //limitation of roll rate
      }
      if (Roll > 400)
      {
        Roll = 400;     //limitation of roll rate
      }
      Serial.print("Roll: ");
      Serial.print(Roll);
      angle = map(Roll, -400, 400, 117, 77);  //generating servo command - max 20deg deflection - 97deg = vertical position
      Serial.print(" angle: ");
      Serial.print(angle);
      myServo.write(angle);                   //send servo command

      File dataFile = SD.open("datalog.txt", FILE_WRITE);   //open SD text file 
  
        if (dataFile) {                                     // if the file is available, write to it:
          dataFile.print(milliNew);
          dataFile.print(",");
          dataFile.print(angle);
          dataFile.print(",");
          dataFile.print(linaccY);
          dataFile.print(",");
          dataFile.print(angvelY);
          dataFile.print(",");
          dataFile.println(Orient);
          dataFile.close();    
        }  
        else {
          Serial.println("error opening datalog.txt");      // if the file isn't open, pop up an error:
        }
          }
    Serial.println();
  }
}
