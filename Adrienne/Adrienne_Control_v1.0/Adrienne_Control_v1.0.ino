#include <SPI.h>

/*****************************************************************************************
   Title: 2018 (ENGR3390 final project 1)
   Description: This structure template contains a SENSE-THINK-ACT flow to
   allow a Robot to perform a sequence of meta-behaviors in soft-real-time based on direct
   text commands from a human operator.
   Robot Name: Simple PixyCam sensor
   What does code do: poll open, sense: object recognition data from PixyCam camera every second,
   think: whether the detected object and its location is a target or hole,
   act: print the target/hole status and location of detected objects,
   carry out operator text input, loop indefinitely
   Hardware warnings: Do not wire the six headed arduino I/O pin the wrong way around.
   Created by J.Patil October 2018
 * ***************************************************************************************
*/

//
// TODO:
//      SENSE:
//      THINK:
//      ACT:

//========================================================================================
// Load supporting Arduino Libraries
//========================================================================================
#include <Servo.h>        //ServoMotors library
#include <Pixy2.h>         // Pixy Library
#include <SPI.h>          //
#include <SharpIR.h>      // IR sensors library
#include <TugNeoPixel.h>  //NeoPixel Ring library

//========================================================================================
// Create and initialize global variables, objects and constants (containers for all data)
//========================================================================================
boolean realTimeRunStop = true;   //create a name for real time control loop flag
String command = "stop ";         //create a String object name for operator command string
String loopError = "no error";    //create a String for the real time control loop error system
unsigned long oldLoopTime = 0;    //create a name for past loop time in milliseconds
unsigned long newLoopTime = 0;    //create a name for new loop time in milliseconds
unsigned long cycleTime = 0;      //create a name for elapsed loop cycle time
const long controlLoopInterval = 1000; //create a name for control loop cycle time in milliseconds

TugNeoPixel neo = TugNeoPixel(7, 16);  //initialize NeoPixel object

//Pixy variables

int targetArray[18];
Pixy2 pixy;

//Sonar variables

const int sonar1 = 0;  //sets signal pin for first sonar sensor
const int sonar2 = 1;  //sets signal pin for second sonar sensor
const int sonar3 = 2;  //sets signal pin for third sonar sensor
int trigger = 13;  //sets 1 trigger pin for all 3 sensors
float sonarArray[3] = {0, 0, 0};

//IR variables
SharpIR IR1(SharpIR::GP2Y0A02YK0F, A0);
SharpIR IR2(SharpIR::GP2Y0A02YK0F, A1);
SharpIR IR3(SharpIR::GP2Y0A02YK0F, A2);
SharpIR IR4(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IR5(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IR6(SharpIR::GP2Y0A02YK0F, A5);
int IRarray[6] = {0, 0, 0, 0, 0, 0};

//Think variables

//Move variables
const int rudderPin = 11;
const int propellorPin = 12;
Servo rudder;
Servo propellor;


//=========================================================================================
// Startup code to configure robot and pretest all robot functionality (to run once)
// and code to setup robot mission for launch.
//=========================================================================================
void setup() {      // Step 1)Put your robot setup code here, to run once:
  Serial.begin(9600);                 // start serial communications
  neo.begin();                        // start the NeoPixel
  Serial.println(" Robot Controller Starting Up! Watch your fingers! ");

  // Step 2)Put your robot mission setup code here, to run once:
  // Add mission code here

  //Pixy initializing

   pixy.init();

  //Sonar initializing

  pinMode(trigger, OUTPUT);  //sets trigger pin as output

  //IR initializing

  //Think initializing

  //Move initializing

}

//=============================================================================
// Flight code to run continuously until robot is powered down
//=============================================================================
void loop() {

  // Step 3)Put Operator-Input-to-Robot and Robot-Reports-Back-State code in non-real-time "outer" loop:
  // Put real-time dependant sense-think-act control in the inner loop

  // GET Operator Control Unit (OCU) Input: ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu------------
  command = getOperatorInput();                       // get operator input from serial monitor
  //Serial.println("command 1 is "+command);

  if (command == "stop") realTimeRunStop = false;     // skip real time inner loop
  else realTimeRunStop = true;                        // Set loop flag to run = true

  // 4)Put your main flight code into "inner" soft-real-time while loop structure below, to run repeatedly,
  // at a known fixed "real-time" periodic interval. This "soft real-time" loop timimg structure, runs
  // fast flight control code once every controlLoopInterval.

  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
  while (realTimeRunStop == true) {     // if OCU-Stop not commanded, run control loop
    // Check if operator inputs a command during real-time loop eecution
    if (Serial.available() > 0) {     // check to see if operator typed at OCU
      realTimeRunStop = false;      // if OCU input typed, stop control loop
      command = Serial.readString();      // read command string to clear buffer
      //Serial.println("command 2 is "+command);
      break;      // break out of real-time loop
    }
    else {
      realTimeRunStop = true; // if no operator input, run real-time loop
    }

    // Real-Time clock control. Check to see if one clock cycle has elapesed before running this control code
    newLoopTime = millis();           // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) { // if true run flight code
      oldLoopTime = newLoopTime;      // reset time stamp



      //SENSE-sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense-------

      // TODO add sensor code here

      // THINK think---think---think---think---think---think---think---think---think---think---think---------
      // pick robot behavior based on operator input command typed at console

      if ( command == "stop") {
        Serial.println("Stop Robot");
        realTimeRunStop = false;    //exit real time control loop
        break;
      }
      else if (command == "move") { //Move robot to Operator commanded position
        Serial.println("Move robot ");
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //don't exit loop after running once
      }
      else if (command == "idle") { //Make robot alive with small motions
        Serial.println("Idle Robot");
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else
      {
        Serial.println("***** WARNING *******Invalid Input, Robot Stopped, Please try again!");
        realTimeRunStop = false;
      }

      // ACT-act---act---act---act---act---act---act---act---act---act---act---act---act---act------------


      // Check to see if all code ran successfully in one real-time increment
      cycleTime = millis() - newLoopTime;   // calculate loop execution time
      if ( cycleTime > controlLoopInterval) {
        Serial.println("******************************************");
        Serial.println("error - real time has failed, stop robot!"); // loop took too long to run
        Serial.print(" 1000 ms real-time loop took = ");
        Serial.println(cycleTime);                                   // print loop time
        Serial.println("******************************************");
        break;          // break out of real-time inner loop
      }
    } // end of "if (newLoopTime - oldLoopTime >= controlLoopInterval)" real-time loop structure
  } // end of "inner" "while(realTimeRunStop == true)" real-time control loop
  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
  // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******

  // SEND Robot State to Operator Control Unit-(OCU) ocu--- ocu-- ocu--- ocu--- ocu--- ocu-- -ocu--
  Serial.println("================================================================");
  Serial.println("| Robot control loop stopping to wait for new command ");   // send robot status

} // end of "outer" void loop()

//=============================================================================
// END OF Flight Code
//=============================================================================

//=============================================================================
//=============================================================================
//FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----FUNCTIONS----
// Functions for each section of above code
// Please note: Except for very simple cases, it would be better to place all of these functions in a
// myRobotControlFuctions.h file and #include it at start of program to keep robot flight code brief
// More on creating header *.h files later in the course

//--------------------------------------------------------------------------------------------------------------------------
// Functions for setup code
//-----------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------
// Functions for flight code
//------------------------------------------------------------------------------------------------------------------------------

// OCU functions ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu------------

String getOperatorInput() {
  // This function prints operator command options on the serial console and prompts
  // operator to input desired robot command
  // Serial.println(" ");
  Serial.println("======================================================================================");
  Serial.println("| Robot Behavior-Commands: move(moves robot), stop(e-stops motors), idle(robot idles)|");
  Serial.println("|                                                                                    |");
  //Serial.println("====================================================================================");
  Serial.println("| Please type desired robot behavior in command line at top of this window           |");
  Serial.println("| and then press SEND button.                                                        |");
  Serial.println("======================================================================================");
  while (Serial.available() == 0) {}; // do nothing until operator input typed
  command = Serial.readString();      // read command string
  //command.trim();
  Serial.print("| New robot behavior command is: ");    // give command feedback to operator
  Serial.println(command);
  Serial.println("| Type 'stop' to stop control loop and wait for new command                          |");
  Serial.println("======================================================================================");
  return command;
}

// SENSE functions sense---sense---sense---sense---sense---sense---sense---sense---sense---


// Pixy function

void findTarget()
{

}

// IR function
void readIR() {
  /* Uses the getDistance function from SharpIR library to get distances in cm*/
  IRarray[0] = IR1.getDistance();
  IRarray[1] = IR2.getDistance();
  IRarray[2] = IR3.getDistance();
  IRarray[3] = IR4.getDistance();
  IRarray[4] = IR5.getDistance();
  IRarray[5] = IR6.getDistance();
}
// Sonar function

float mapSonar(float reading)
{
  return 0.0 + (reading - 0.0) * (18.0 - 0.0) / (19.0 - 0.0);
}

void readSonar() {
  /*
    Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
    Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual inches
  */
  digitalWrite(trigger, HIGH);
  delay(1); //triggers the sonars/makes them take a reading
  digitalWrite(trigger, LOW);

  sonarArray[0] = mapSonar(analogRead(sonar1) / 2.0);
  sonarArray[1] = mapSonar(analogRead(sonar2) / 2.0);
  sonarArray[2] = mapSonar(analogRead(sonar3) / 2.0);
}


// THINK functions think---think---think---think---think---think---think---think---think---

// Think function

// ACT functions act---act---act---act---act---act---act---act---act---act---act---act---act---

// Move function

// END of Functions
//=============================================================================
// END OF Robot Control CODE
