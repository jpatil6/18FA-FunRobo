// TODO:
//  - figure out why the propellers are being actuated even though we aren't actuating them
//  - make propellor speed dependent on how far narwhal is
//  - incorporate different behaviors into boat
//  - experiment with different proportional control constants, or different control loops
//============================================================================================
// Libraries
#include <EasyTransfer.h>
#include <SoftwareSerial.h> //includes software serial library
#include <Servo.h>
//============================================================================================
// Create global variables, objects, and constants
Servo rudder;
Servo prop;
Servo table;
EasyTransfer ET; // this is for arduino communication

// creating data structure to receive from SENSE arduino
struct RECEIVE_DATA_STRUCTURE
{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int error_xValue;
};

//Give struct an actual name
RECEIVE_DATA_STRUCTURE pixydata;

// variables for motor controls
int rudderPin = 5;
int rudderAngle = 0;
int propPin = 6;
int objectDistance = 0;
int tablePin = 12;
int directionAngle = 0;
int tablePWM = 0;

// initialize proportional control constant
int Kp = 5;

// initialize time loop variables
unsigned long oldLoopTime = 0;
unsigned long newLoopTime = 0;
unsigned long cycleTime = 0;
const long controlLoopInterval = 25;
//============================================================================================
// Startup code to configure robot and set mission for launch.
//============================================================================================
void setup() {
  // put your setup code here, to run once:
  //actSerial has same name in think and act arduinos
  //not attach the servos and motors. the motors act like continuous rotation servos
  Serial.begin(9600);
  rudder.attach(rudderPin);
  prop.attach(propPin);
  table.attach(tablePin);
  ET.begin(details(pixydata), &Serial);
  Serial.println("CAREFULL ROBOT IS RUNNING");
}

//============================================================================================
// Flight code to run continuously
//============================================================================================
void loop() {

  // angle inputted determines direction of rudders
  // FIX THIS: direction of rudders does not depend on speed of motors?

  //-------------------------REAL TIME CONTROL LOOP STARTS HERE ------------------------
  // real time clock control
  newLoopTime = millis();              // update time
  //run loop once every control loop interval
  if (newLoopTime - oldLoopTime >= controlLoopInterval) {
    oldLoopTime = newLoopTime;         // update time

    // if data has been received from the SENSE arduino
    if (ET.receiveData()) {

      // turn the boat accordingly
      turnBoat(pixydata.error_xValue,Kp);

      // debugging section -> uncomment line below to make sure x error is being received properly
//Serial.print("X Error Received: "); Serial.println(pixydata.error_xValue);
    }
    else{

      // if no data has been received, do nothing
      turnBoat(0,Kp);
      }
    
  }
  //-------------------------REAL TIME CONTROL LOOP ENDS HERE ------------------------
}

//TODO: Implement Speed Control and rudder control
//setDistanceToSpeed(objectDistance);
//setAngleToRudders(-pixydata.angle);


/////////////// FUNCTIONS ////////////////////////////////////
//void commandRudderMotor (int error_xValue, int Kp) {
//  // setting threshold for appropriate "close enough" error values
//  if (error_xValue > -5 && error_xValue < 5){error_xValue = 0;}
//  
//
//  // calculating proprtional control
//  rudderPWM = Kp * error_xValue + 1500;
//
//  // dealing with corner cases
//  if (rudderPWM > 1750){rudderPWM = 1750;}
//  else if (rudderPWM < 1250){rudderPWM = 1250;}
//
//  Serial.print("motor command PWM: ");
//  Serial.println(rudderPWM);
//
//  // sending table motor PWM
//  rudder.writeMicroseconds(rudderPWM);
//}

void setDistanceToSpeed(int distance) {
  //  if (distance < 0 || distance > 50){
  //    Serial.println("Input angle is outside of the allowed interval");
  //  }
  //  else {
  // propellors speed dependent on distance of object
  // farther the object, faster the propellors; vice versa
  int microSec = map(distance, -50, 50, 1250, 1750); // test speeds of different distances
  prop.writeMicroseconds(microSec);
  //  }
}
 
void turnBoat(int error_xValue, int Kp) {
  // This function commands the rudder motor and table motor simealtaneuosly to give the illusion
  // of the boat turning but on land
  // NOTE: This uses a proportional controller that aims to minimize error_xValue

  // setting threshold for appropriate "close enough" error values
  if (error_xValue > -5 && error_xValue < 5){error_xValue = 0;}
  

  // calculating proprtional control
  tablePWM = Kp * error_xValue + 1500;

  // dealing with corner cases
  if (tablePWM > 1750){tablePWM = 1750;}
  else if (tablePWM < 1250){tablePWM = 1250;}

  Serial.print("motor command PWM: ");
  Serial.println(tablePWM);

  // sending table motor PWM
  table.writeMicroseconds(tablePWM);

  // calculate rudder angle based off tablePWM
  rudderAngle = map(tablePWM,1250,1750,10,80);

  // send angle to rudder
  rudder.write(rudderAngle);
  
  }
