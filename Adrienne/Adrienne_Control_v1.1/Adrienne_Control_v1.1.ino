/*****************************************************************************************
   Title: 2018 Fun-Robo Narwhal-tracking autonomous tugboat (ENGR3390 final project 1)
   Description: This structure template contains a SENSE-THINK-ACT flow to
   allow a robotic tugboat to perform a sequence of meta-behaviors in soft-real-time
   based on direct text commands from a human operator.
   Robot Name: Adrienne
   What does code do: poll open,
   sense: Detect range and bearing of target and obstacles with the Pixycam and IR sensors
   think: Combine bearing arrays for target and obstacles to optimize bearing of the tugboat.
          If unobstructed on set bearing, calculate propellor speed proportional to range
   act:   Set table motor and rudder bearing, set propellor speed
   carry out operator text input, loop indefinitely
   Hardware warnings: Do not wire the Pixycam ICSP arduino I/O pin the wrong way around.
   XBEE:1. Set shield SerialSelect switch to SW_SER for uploading code.
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
int wallFollowCounter = 0;

//Pixy variables

int targetArray[19];
Pixy2 pixy;

//Sonar and IR variables

int objectArray[19];

int sensorThreshhold = 20;
const int sonar1 = A3;  //sets signal pin for first sonar sensor
const int sonar2 = A4;  //sets signal pin for second sonar sensor
const int sonar3 = A5;  //sets signal pin for third sonar sensor
int trigger = 12;  //sets 1 trigger pin for all 3 sensors
int sonarArray[6] = {};
void readSonar();

const int IR1 = 8;
const int IR2 = 9;
const int IR3 = 10;
const int IR4 = 11;
const int IR5 = 12;
const int IR6 = 13;
int IRarray[6];
void readIR();
int IRreadingCount = 20;

//Think variables

// these are the values that our voting function changes. They tell the boat where to go.
int theta;
int magnitude;
void manualArbiter();
void moveboat();

//Move variables
const int rudderPin = 7;
const int propellorPin = 6;
Servo rudder;
Servo propellor;
int setspeed; // these directly tell the boat where to go
int setdirection;


//=========================================================================================
// Startup code to configure robot and pretest all robot functionality (to run once)
// and code to setup robot mission for launch.
//=========================================================================================
void setup()
{ // Step 1)Put your robot setup code here, to run once:

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
      setspeed = 0;
      setdirection = 0;
      moveboat();
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
        Serial.println("In wallfollow");
        if (wallFollowCounter == 0) {
          Serial.println("In wallfollow state 0");
          findObjects();
          setHeading(9);
          if ( sonarArray[3] <= 310) {
            wallFollowCounter++;
          }
        }
        if (wallFollowCounter == 1) {
          Serial.println("In wallfollow state 1");
          findObjects();
          swerveAroundIceberg(0);     //need to set side
          if ( IRarray[2] <= 90 || IRarray [3] <= 90) {
            wallFollowCounter++;
          }
        }
        if (wallFollowCounter == 2) {
          Serial.println("In wallfollow state 2");
          findObjects();
          maintainDistance(60, 0);     //need to set side
          if ( sonarArray[3] <= 120) {
            wallFollowCounter++;
          }
        }
        if (wallFollowCounter == 3) {
          Serial.println("In wallfollow state 3");
          findObjects();
          setHeading(13);
          if ( sonarArray[3] >= 120) {
            wallFollowCounter++;
          }
        }
        if (wallFollowCounter == 4) {
          Serial.println("In wallfollow state 4");
          findObjects();
          realTimeRunStop = false;    //exit real time control loop
        }
        votingFunc();
        moveboat();
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "figure 8") {
        // Add fig8 code
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "figure 8 dock") {
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

      // Serial.println(cycleTime);
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
  while (Serial.available() == 0) {}; // do nothing until operator input typed
  command = Serial.readString();      // read command string
  //command.trim();
  Serial.print(F("| New robot behavior command is: "));    // give command feedback to operator
  Serial.println(command);
  Serial.println(F("| Type 'stop' to stop control loop and wait for new command                          |"));
  Serial.println(F("======================================================================================"));
  return command;
}

// SENSE functions sense---sense---sense---sense---sense---sense---sense---sense---sense---

void findObjects()
{
  for (int entry = 0; entry < 19; entry++)
  {
    targetArray[entry] = 50;
  }
  readIR();
  readSonar();
  for (int reading = 0; reading < 6; reading++)
  {
    int objectPos; //b in the gaussian function
    int objectWidth; //c in the gaussian functions, the std deviatiation
    int objectSize; // a in the gaussian function


    if ((IRarray[reading] < 0))
    {
      IRarray[reading] = 0;
    } else if (IRarray[reading] > 120)
    {
      IRarray[reading] = 120;
    }
    objectPos = map(reading, 0, 5, 0, 18); // needs to be rounded
    objectWidth = 1;
    objectSize = map(IRarray[reading], 20, 120, 50, 10);
    int objectArrayTemp[19];
    for (int entry = 0; entry < 19; entry++) // then make a gaussian function with those values
    {
      objectArrayTemp[entry] = objectSize * pow(2.718,-1*(pow((entry - objectPos),2)/(2 * pow(objectWidth,2))));
      // then we populate target array with the values of the gaussian function from 0 to 17
    }
    for (int entry = 0; entry < 19; entry++)
    {
      objectArray[entry] = objectArray[entry] - objectArrayTemp[entry];
    }
  }
  Serial.print("IRarray: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(IRarray[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("Sonararray: ");
  for (int i = 0; i < 6; i++)
  {
    Serial.print(sonarArray[i]);
    Serial.print("\t");
  }
  Serial.println();
}

// Pixy function

void findPixyTarget()
{
  for (int entry = 0; entry = 18; entry++)
  {
    targetArray[entry] = 0;
  }
  int targetPos; //b in the gaussian function
  int targetWidth; //c in the gaussian functions, the std deviatiation
  int targetSize; // a in the gaussian function
  pixy.ccc.getBlocks();
  for (int i = 0; i <= pixy.ccc.numBlocks; i++) //go through all the pixy blocks
  {
    if (pixy.ccc.blocks[i].m_signature = 1) //if it's narwhal colored...
    {
      if (pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height >= 100)//and it's big:
      {
        //this area threshold is arbitrary right now
        targetPos = map(pixy.ccc.blocks[i].m_x, 0, 316, 6, 12); //we find where it is
        targetWidth = map(pixy.ccc.blocks[i].m_width, 1, 316, 1, 3); // and how much of our field of view it takes
        targetSize = map(pixy.ccc.blocks[i].m_height, 1, 208, 50, 100); // and how close it is, based on height

        for (int entry = 0; entry <= 18; entry++) // then make a gaussian function with those values
        {
          targetArray[entry] = targetSize * pow(2.718,-1*(pow((entry - targetPos),2)/(2 * pow(targetWidth,2))));
          // then we populate target array with the values of the gaussian function from 0 to 17
        }
        break;
      }
    }
  }
}


// IR function
void readIR()
{
  /* Uses the getDistance function from SharpIR library to get distances in cm*/
  IRarray[0] = averageOut(IR1);
  IRarray[1] = averageOut(IR2);
  IRarray[2] = averageOut(IR3);
  IRarray[3] = averageOut(IR4);
  IRarray[4] = averageOut(IR5);
  IRarray[5] = averageOut(IR6);
}

int averageOut( uint8_t pin) {
  /* Averages out the IR reading to get a more accurate value */

  int distance = 0;
  int i = 1;
  while (i < IRreadingCount) {
    distance += int(pow(analogRead(pin), -1.02) * 13755); // empirically determined equation to calculate the distance in cm
    i ++;
  }
  distance = int(distance / (i - 1));
  return distance;
}


void readSonar()
{
  /*
    Scale factor is (Vcc/512) per inch. A 5V supply yields ~9.8mV/in
    Arduino analog pin goes from 0 to 1024, so the value has to be divided by 2 to get the actual cm
  */
  // this function alters the sonarZones array. After it runs, 0 in the array means there is nothing there,
  // and 1 means there is something there.
  digitalWrite(trigger, HIGH);
  delay(1); //triggers the sonars/makes them take a reading
  digitalWrite(trigger, LOW);

  float reading1 = analogRead(sonar1) / 2.0;
  float reading2 = analogRead(sonar2) / 2.0;
  float reading3 = analogRead(sonar3) / 2.0;

  sonarArray[0] = reading1;
  sonarArray[1] = reading1;
  sonarArray[2] = reading2;
  sonarArray[3] = reading2;
  sonarArray[4] = reading3;
  sonarArray[5] = reading3;
}


// THINK functions think---think---think---think---think---think---think---think---think---

// Set Heading. Creates a function that tells the robot where to go directly and puts it in targetArray

void setHeading(int heading)
{
  for (int entry = 0; entry <= 18; entry++) // then make a gaussian function with those values
  {
    targetArray[entry] = 100 * exp((entry - heading) ^ 2 / (16));
    // then we populate target array with the values of the gaussian function from 0 to 18
  }
}

void swerveAroundIceberg(int side) {
  setHeading(1);
}

//maintainDistance
void maintainDistance(int dist, int side)
{
  if (side == 0) {
    if (objectArray[0] < 0.9 * dist)
    {
      setHeading(11);
    } else if (objectArray[0] > 1.1 * dist) {
      setHeading(7);
    }
  } else if (side == 1) {
    if (objectArray[18] < 0.9 * dist)
    {
      setHeading(11);
    } else if (objectArray[18] > 1.1 * dist) {
      setHeading(7);
    }
  }
}
// Voting Function
// Takes the gaussian functions from find object and find target and outputs an angle
// and a distance to the point we want to go to.
void votingFunc()
{
  int voteArray[19];
  for (int entry = 0; entry <= 18; entry++)
  {
    voteArray[entry] = targetArray[entry] + objectArray[entry];
  }
  int maximum = 0;
  int maximumIndex = voteArray[19];
  for (int entry = 0; entry <= 18; entry++)
  {
    if (voteArray[entry] > maximum)
    {
      maximum = voteArray[entry];
      maximumIndex = entry;
    }
  }
  theta = maximum * 10; // so that we're giving the boat an angle.
  magnitude = voteArray[maximum]; // A number from 0 to 150. It gets bigger as the target gets farther away
  pickBumblebeeCircle(magnitude, theta);
}

// Manual arbiter
// Receives characters from serial to manually navigate the boat by adjusting rudder(in deg)
// and propellor settings (in % speed).
// 'wasd' for standard front-left-back-right, 'qe' for slight left-right,
// 'zc' for extreme left-right. 'o' to idle, 'x' to exit manual mode
void manualArbiter()
{
  Serial.println(F("Entered manual mode. Type 's' to exit mode"));
  char inbit;
  bool keepManual = true;
  int steptime = 10;
  while (keepManual)
  {
    if (Serial.available())
    {
      inbit = Serial.read();
      switch (inbit)
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
// pickBumblebeeCircle
// this function accepts a radius and theta to select the rudder and propellor settings
//incoming r ranges from 0 to 150, theta goes from 0 to 180 degress
void pickBumblebeeCircle(int r, double theta)
{

  r = r * 120 / 150;
  theta = (180 - theta) * PI / 180;
  //Serial.print("r: ");
  //Serial.println(r);
  //Serial.print("theta: ");
  //Serial.println(theta);

  double zone[] = { -60, -191.5, -347.5, -914, 916, 333.5, 168.5, 60};
  int turningmaxspeed[] = {30, 45, 60, 75, 60, 45, 30};
  int turninglookup[] = { -45, -30, -15, 0, 15, 30, 45};
  int i;
  boolean valid = false;


  for (i = 0; i < 7; i++)
  {
    if (i == 3) {
      if ((r > zone[i]*cos(theta)) && (r > zone[i + 1]*cos(theta)))
      {
        //Serial.print("In zone: ");
        //Serial.println(i+1);
        valid = true;
        break;
      }
    } else {
      if ((r > zone[i]*cos(theta)) && (r < zone[i + 1]*cos(theta)))
      {
        //Serial.print("In zone: ");
        //Serial.println(i+1);
        valid = true;
        break;
      } else if ((r < zone[i]*cos(theta)) && (r > zone[i + 1]*cos(theta))) {
        //Serial.print("In zone: ");
        //Serial.println(i+1);
        valid = true;
        break;
      }//end inner if
    }//end outer if
  }//end for
  if (valid)
  {
    setdirection = turninglookup[i];
    setspeed = (turningmaxspeed[i] * r / 120);
  } else {
    setspeed = 0;
    setdirection = 0;
  }
}


// moveboat function
//Note: Needs calibration of the center. currently rudder center = 85, prop center = 1400
void moveboat()
{
  rudder.write(map(setdirection, -90, 90, -5, 175));
  propellor.writeMicroseconds(map(setspeed, -100, 100, 900, 1900));
}

// END of Functions
//=============================================================================
// END OF Robot Control CODE
