/*****************************************************************************************
 * Title: 2018 Fun-Robo Narwhal-tracking autonomous tugboat (ENGR3390 final project 1)
 * Description: This structure template contains a SENSE-THINK-ACT flow to
   allow a robotic tugboat to perform a sequence of meta-behaviors in soft-real-time
   based on direct text commands from a human operator.
 * Robot Name: Adrienne
 * What does code do: poll open, 
   sense: Detect range and bearing of target and obstacles with the Pixycam and IR sensors 
   think: Combine bearing arrays for target and obstacles to optimize bearing of the tugboat. 
          If unobstructed on set bearing, calculate propellor speed proportional to range
   act:   Set table motor and rudder bearing, set propellor speed
   carry out operator text input, loop indefinitely
 * Hardware warnings: Do not wire the Pixycam ICSP arduino I/O pin the wrong way around.
 * XBEE:1. Set shield SerialSelect switch to SW_SER for uploading code. 
        2. Set it back to HW_SER and put the XBee back onto shield.
        3. Remove USB cable. Use XCTU terminal to communicate with arduino
   Created by J.Patil, V.Deturck, E.O'Brien, H.Khoury. Dec 2018
 * ***************************************************************************************
*/

//
// TODO:
//      OCU/COMM- Upon 'stop', set rudder and prop to 0 before terminating loop
//              - Add NeoPixel states and code     
//      SENSE:  - Pending combination of IR+Sonar into objectArray
//              - Pending findTarget function (also include Dock as target)
//      THINK:  - Pending Dead Reckoning + BumbleBee arbiter + find center function
//              - Create Mission Definiton file and structure 
//      ACT:    - Pending moveboat() function

//========================================================================================
// Load supporting Arduino Libraries
//========================================================================================
#include <Servo.h>        // ServoMotors library
#include <Pixy2.h>        // Pixy Library
#include <SPI.h>          //
#include <SharpIR.h>      // IR sensors library
#include <TugNeoPixel.h>  // NeoPixel Ring library

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


// OCU and communication variables

TugNeoPixel neo = TugNeoPixel(8, 16);  //initialize NeoPixel object

//Pixy variables

int targetArray[19];
Pixy2 pixy;

//Sonar and IR variables

int objectArray[19];

const int sonar1 = A3;  //sets signal pin for first sonar sensor
const int sonar2 = A4;  //sets signal pin for second sonar sensor
const int sonar3 = A5;  //sets signal pin for third sonar sensor
int trigger = 13;  //sets 1 trigger pin for all 3 sensors
float sonarArray[3] = {0, 0, 0};

SharpIR IR1(SharpIR::GP2Y0A02YK0F, A8);
SharpIR IR2(SharpIR::GP2Y0A02YK0F, A9);
SharpIR IR3(SharpIR::GP2Y0A02YK0F, A10);
SharpIR IR4(SharpIR::GP2Y0A02YK0F, A11);
SharpIR IR5(SharpIR::GP2Y0A02YK0F, A12);
SharpIR IR6(SharpIR::GP2Y0A02YK0F, A13);
int IRarray[6] = {0, 0, 0, 0, 0, 0};

//Think variables

//Move variables
const int rudderPin = 7;
const int propellorPin = 6;
Servo rudder;
Servo propellor;
int setspeed;
int setdirection;


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
  propellor.attach(propellorPin);
  rudder.attach(rudderPin);
  propellor.writeMicroseconds(1400);
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
      else if (command == "idle") {
        Serial.println("Idle Robot");
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "manual") { //Move robot to Operator commanded position
        manualArbiter();
        realTimeRunStop = false;     // exit loop after running once
      }
      else if (command == "wallfollow") { 
        // Add wallfollow code
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "fig8") { 
        // Add fig8 code
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "fig8dock") { 
        // Add fig8dock code
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "hunt") { 
        // Add hunt code
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else
      {
        Serial.println("***** WARNING *******Invalid Input, Robot Stopped, Please try again!");
        realTimeRunStop = false;
      }

      // ACT-act---act---act---act---act---act---act---act---act---act---act---act---act---act------------

      Serial.println(cycleTime);
      // Check to see if all code ran successfully in one real-time increment
      cycleTime = millis() - newLoopTime;   // calculate loop execution time
      if ( cycleTime > controlLoopInterval) {
        Serial.println(F("******************************************"));
        Serial.println(F("error - real time has failed, stop robot!")); // loop took too long to run
        Serial.print(F(" 1000 ms real-time loop took = "));
        Serial.println(cycleTime);                                   // print loop time
        Serial.println(F("******************************************"));
        setspeed = 0;
        setdirection = 0;
        moveboat();
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

// This function prints operator command options on the serial console and prompts
// operator to input desired robot command
String getOperatorInput()
{
  Serial.println(F("======================================================================================"));
  Serial.println(F("|Robot Behavior-Commands: stop | idle | manual | wallfollow | fig8 | fig8dock | hunt |"));
  Serial.println(F("|                                                                                    |"));
  Serial.println(F("| Please type desired robot behavior in command line and press enter.                |"));
  Serial.println(F("======================================================================================"));
  while (Serial.available()==0) {};   // do nothing until operator input typed
  command = Serial.readString();      // read command string
  //command.trim();
  Serial.print(F("| New robot behavior command is: "));    // give command feedback to operator
  Serial.println(command);
  Serial.println(F("| Type 'stop' to stop control loop and wait for new command                          |"));
  Serial.println(F("======================================================================================"));
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

// Sonar functions
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

// Manual arbiter
// Receives characters from serial to manually navigate the boat by adjusting rudder(in deg) 
// and propellor settings (in % speed). 
// 'wasd' for standard front-left-back-right, 'qe' for slight left-right,
// 'zc' for extreme left-right. 'o' to idle, 'x' to exit manual mode
void manualArbiter(){
  Serial.println(F("Entered manual mode. Type 's' to exit mode"));
  char inbit;
  bool keepManual = true;
  int steptime = 10;
  while(keepManual)
  {
    if(Serial.available())
    {
      inbit = Serial.read();
      switch(inbit)
      {
        case 'w':
          setspeed = 30;
          setdirection = 0;
          break;
        case 's':
          setspeed = -20;
          setdirection = 0;
          break;
        case 'q':
          setspeed = 30;
          setdirection = -20;
          break;
        case 'e':
          setspeed = 30;
          setdirection = 20;
          break;
        case 'a':
          setspeed = 30;
          setdirection = -40;
          break;
        case 'd':
          setspeed = 30;
          setdirection = 40;
          break;
        case 'z':
          setspeed = 30;
          setdirection = -60;
          break;
        case 'c':
          setspeed = 30;
          setdirection = 60;
          break;
        case 'x':
          setspeed = 0;
          setdirection = 0;
          keepManual = false;
          break;
        case 'o':
          setspeed = 0;
          setdirection = 0;
          break;
        default:
          Serial.println("Wrong input! Terminating manual mode");
          keepManual = false;
          break;  
      }//close switch
    moveboat();
    delay(steptime);
    //setspeed = 0;
    //setdirection = 0;
    //moveboat();
    }//close if Serial.available 
  }//close while keepManual
}//close manualArbiter


// ACT functions act---act---act---act---act---act---act---act---act---act---act---act---act---

void centerServos()
{
  bool needtocenter = true;
  char inbit[4];
  while(needtocenter)
  {
    
  }
}
// moveboat function
//Note: Needs calibration of the center. currently rudder center = 85, prop center = 1400 
void moveboat()
{
  rudder.write(map(setdirection,-90,90,-5,175));
  propellor.writeMicroseconds(map(setspeed,-100,100,900,1900));
}

// END of Functions
//=============================================================================
// END OF Robot Control CODE
