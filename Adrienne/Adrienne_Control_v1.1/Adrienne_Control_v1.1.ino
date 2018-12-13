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
const long controlLoopInterval = 500; //create a name for control loop cycle time in milliseconds

int startupcounter = 0;

// Time loop counters (indicates which stage of the command the boat is doing)
int wallFollowCounter = 0;
int fig8Counter = 0;
int fig8DockCounter = 0;
int huntCounter = 0;

// OCU and communication variables
TugNeoPixel neo = TugNeoPixel(8, 16);  //initialize NeoPixel object


//Sense arrays (array length is 19 ints)

int targetArray[19];
int objectArray[19];


//Pixy variables

Pixy2 pixy;

//Sonar and IR variables

const int sonar1 = A3;  //sets signal pin for first sonar sensor
const int sonar2 = A4;  //sets signal pin for second sonar sensor
const int sonar3 = A5;  //sets signal pin for third sonar sensor
int trigger = 12;  //sets 1 trigger pin for all 3 sensors
int sonarArray[6];      //sets Array length to 6 ints

//Digital pins that IRs are plugged into:
const int IR1 = 8;
const int IR2 = 9;
const int IR3 = 10;
const int IR4 = 11;
const int IR5 = 12;
const int IR6 = 13;
int IRarray[6];      //sets Array length to 6 ints
int IRreadingCount = 20 ;

//Think variables

//Move variables
const int rudderPin = 7;      //Rudders are connected to digital pin 7
const int propellorPin = 6;   //props are connected to digital pin 6
Servo rudder;                 // setting rudder as servo variable
Servo propellor;              // setting props as servo variable
int setspeed;                 // tells boat what the prop speed should be
int setdirection;             // tells boat how to direct rudders


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
  propellor.attach(propellorPin);      //dictates which pin commands prop servo
  rudder.attach(rudderPin);            //dictates which pin commands rudder servo
  propellor.writeMicroseconds(1400);   //tells props to be idle
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

  startupcounter = 0;
  wallFollowCounter = 0;

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
      command = "stop";
      //Serial.println("command 2 is "+command);
      break;      // break out of real-time loop
    }
    else {
      realTimeRunStop = true; // if no operator input, run real-time loop
    }

    // Real-Time clock control. Check to see if one clock cycle has elapesed before running this control code
    newLoopTime = millis();           // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) { // if true run flight code

      //Serial.print("newLoop- oldLoop: ");
      //Serial.println(newLoopTime - oldLoopTime);
      oldLoopTime = newLoopTime;      // reset time stamp

      //SENSE-sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense-------
      findObjects();

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
      else if (command == "wallfollow") {    //command to tell boat to exit the dock and follow the wall 
        if (startupcounter > 5)
        {
          wallfollow();
        } else {
          startupcounter++;
        }
      }
      else if (command == "figure 8") {      //command to tell boat to exit dock and do a figure 8 around the icebergs 3 times
        fig8();
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "figure 8 dock") { //similar to figure 8 code but only does one figure 8 and then returns to the dock
        fig8Dock();
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true;     //run loop continually
      }
      else if (command == "hunt") {          //find and chase narwhale 
        hunt();
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
      //Serial.print("Cycle time: ");
      //Serial.println(cycleTime);
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

//=========================================================================================
// SENSE functions sense---sense---sense---sense---sense---sense---sense---sense---sense---
//=========================================================================================

void findObjects() //function to detect objects using both IR & Sonar
{
  int objectArrayTemp[19];
  for (int entry = 0; entry < 19; entry++)
  {
    objectArray[entry] = 50;
  }
  readIR();
  readSonar();
  for (int reading = 0; reading < 6; reading++)
  {
    int objectPos; //b in the gaussian function
    int objectWidth; //c in the gaussian functions, the std deviatiation
    int objectSize; // a in the gaussian function

    objectPos = map(reading, 0, 5, 0, 18); // needs to be rounded
    objectWidth = 1;
    objectSize = map(IRarray[reading], 20, 120, 50, 10);

    for (int i = 0; i < 19; i++) // then make a gaussian function with those values
    {
      objectArrayTemp[i] = objectSize * exp(-(pow((i - objectPos), 2) / (2 * pow(objectWidth, 2))));
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
  /*
    Serial.print("objectArray: ");
    for (int i = 0; i < 19; i++)
    {
    Serial.print(objectArray[i]);
    Serial.print("\t");
    }
    Serial.println();
  */
}

// Pixy functions

void findPixyTarget(int targetType) // target type is 1 or two. 1 for dock, 2 for narwhal
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
    if (pixy.ccc.blocks[i].m_signature = targetType) //if it's narwhal colored...
    {
      if (pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height >= 100)//and it's big:
      {
        //this area threshold is arbitrary right now
        targetPos = map(pixy.ccc.blocks[i].m_x, 0, 316, 6, 12); //we find where it is
        targetWidth = map(pixy.ccc.blocks[i].m_width, 1, 316, 1, 3); // and how much of our field of view it takes
        targetSize = map(pixy.ccc.blocks[i].m_height, 1, 208, 50, 100); // and how close it is, based on height

        for (int entry = 0; entry <= 18; entry++) // then make a gaussian function with those values
        {
          targetArray[entry] = targetSize * pow(2.718, -1 * (pow((entry - targetPos), 2) / (2 * pow(targetWidth, 2))));
          // then we populate target array with the values of the gaussian function from 0 to 17
        }
        break;
      }
    }
  }
}

bool pixyCheck(int targetType) { // 1 means it looks for the dock, 2 means it looks for the narwhal
  pixy.ccc.getBlocks();
  for (int i = 0; i <= pixy.ccc.numBlocks; i++) //go through all the pixy blocks
  {
    if (pixy.ccc.blocks[i].m_signature = targetType) //if it's the right color...
    {
      if (pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height >= 100)//and it's big:
      {
        return true;
      }
      else
      {
        return false;
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
  for (int i = 0; i < 6; i++) {
    //clear extreme values from IR
    if ((IRarray[i] > 120) || (IRarray[i] < 0))
    {
      IRarray[i] = 120;
    }
  }
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
  // this function alters the sonarArray array. After it runs, 0 in the array means there is nothing there,
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

//=========================================================================================
// THINK functions think---think---think---think---think---think---think---think---think---
//=========================================================================================

//======================
//Behaviors
//======================
void fig8() {                                      //function that commands the boat to leave the dock and do a figure 8 around the icebergs 3 times
  Serial.println("In figure 8"); // leaving dock   //indicate to roboticists that it is currently executing fig8 function
  if (fig8Counter == 0) {                          //if boat was just commanded to do fig8, counter would start in 0 and trigger the initial state
    Serial.println("In figure 8 state 0");         //indicate to rboticists which state the boat is in
    setHeading(9);                                 //boat drives out of dock
    if ( sonarArray[1] < 150) {                    //if sonars see something (iceberg) within 150 cm then
      fig8Counter++;                               //trigger state 1
    }
  }
  if (fig8Counter == 1) {                          //if state 1 has been triggered
    Serial.println("In figure 8 state 1");         //indicate to roboticists which state the boat is in
    setHeading (5);     //need to set side         //tells boat to turn to port side
    if (IRarray[4] <= 100 ) {                      //if/until the boat detects the wall infront of it within 100 cm then
      fig8Counter++;                               //trigger state 2
    }
  }
  if (fig8Counter == 2) {                          //if state 2 has been triggered
    Serial.println("In figure 8 state 2");         //indicate to roboticists which state the boat is in
    setHeading(9);                                 //tell boat to drive straight
    if ( IRarray[1] <= 100 || IRarray [2] <= 100) {//if/until it sees something in front/on its port side within 100 cm then
      fig8Counter++;                               //trigger state 3
    }
  }
  if (fig8Counter == 3) {                          //if state 3 has been triggered
    Serial.println("In figure 8 state 3");         //indicate to roboticists which state the boat is in
    maintainDistance(100, 0);  //need to set side  //tell boat to maintain a distance of 100cm from the wall on it's port side until
    if ( sonarArray[4] <= 130) {                   //if boat detects iceberg within a distance of 130 cm on starboard side then
      fig8Counter++;                               //trigger state 4
    }
  }
  if (fig8Counter == 4) {                          //if state 4 has been triggered
    Serial.println("In figure 8 state 4");         //indicate to roboticists which state the boat is in
    circleIceberg(1);                              //command boat to circle iceberg on starboard side until
    if ( sonarArray[0] <= 160) {                   //if sonar detects something (other iceberg) within 160 cm on port side then
      fig8Counter++;                               //trigger state 4
    }
  }
  if (fig8Counter == 5){                           //if state 5 has been triggered
     Serial.println("In figure 8 state 5");        //indicate to roboticists which state the boat is in
     setHeading(7);                                //command boat to turn slightly to the left until
     if ( sonarArray[0] <= 100){                   //if sonar detects something (iceberg) within 100 cm on the left side then
        fig8Counter++;                             //trigger state 6
     }
  }
  if (fig8Counter == 6) {                          //if state 6 has been triggered
    Serial.println("In figure 8 state 6");         //indicate to roboticists which state the boat is in
    circleIceberg(0);                              //command boat to circle iceberg on port side until
    if ( sonarArray[2] <= 160) {                   //if sonar detects something (other iceberg) infront of it within 160 cm then
      fig8Counter++;                               //trigger state 7
    }
  }
  if (fig8Counter == 7){                           //if state 7 has been triggered
     Serial.println("In figure 8 state 7");        //indicate to roboticists which state the boat is in
     setHeading(11);                                //command boat to turn slightly to the right until
     if ( sonarArray[0] <= 100){                   //if sonar detects something (iceberg) within 100 cm on the left side then
        fig8Counter++;                             //trigger state 8
     }
  }
  //Figure 8 #2 
  if (fig8Counter == 8) {                          //if state 8 has been triggered
    Serial.println("In figure 8 state 8"); // start figure 8 two
    circleIceberg(1);
    if ( sonarArray[0] <= 160) {
      fig8Counter++;
    }
  }
  if (fig8Counter == 9){                           //if state 9 has been triggered
     Serial.println("In figure 8 state 9");        //indicate to roboticists which state the boat is in
     setHeading(7);                                //command boat to turn slightly to the left until
     if ( sonarArray[0] <= 100){                   //if sonar detects something (iceberg) within 100 cm on the left side then
        fig8Counter++;                             //trigger state 10
     }
  }   
  if (fig8Counter == 10) {
    Serial.println("In figure 8 state 10"); // finish figure 8 two
    circleIceberg(0);
    if ( sonarArray[2] <= 160) {
      fig8Counter++;
    }
  }
  if (fig8Counter == 11){                           //if state 11 has been triggered
     Serial.println("In figure 8 state 11");        //indicate to roboticists which state the boat is in
     setHeading(7);                                //command boat to turn slightly to the left until
     if ( sonarArray[0] <= 100){                   //if sonar detects something (iceberg) within 100 cm on the left side then
        fig8Counter++;                             //trigger state 12
     }
  }   
  //Figure 8 #3 
  if (fig8Counter == 12) {
    Serial.println("In figure 8 state 12"); // start figure 8 three
    circleIceberg(1);
    if ( sonarArray[0] <= 160) {
      fig8Counter++;
    }
  }
  if (fig8Counter == 13){                           //if state 13 has been triggered
     Serial.println("In figure 8 state 13");        //indicate to roboticists which state the boat is in
     setHeading(7);                                //command boat to turn slightly to the left until
     if ( sonarArray[0] <= 100){                   //if sonar detects something (iceberg) within 100 cm on the left side then
        fig8Counter++;                             //trigger state 14
     }
  }   
  if (fig8Counter == 14) {
    Serial.println("In figure 8 state 14"); // finish figure 8 three
    circleIceberg(0);
    if ( sonarArray[2] <= 160) {
      fig8Counter++;
    }
  }
  if (fig8Counter == 15) {
    Serial.println("Done");
    realTimeRunStop = false;
  }
  votingFunc();
  moveboat();
  realTimeRunStop = true;     //run loop continually
}

void wallfollow() {                                 //wall following function
  Serial.println("In wallfollow");
  if (wallFollowCounter == 0) {                    //if the boat is commanded to wallFollow, counter would start at 0 (int wallFollowCounter = 0, as per indicated above)
    Serial.println("In wallfollow state 0");       //indicates to roboticists which state the boat is in
    setHeading(9);                                 //boat drives out of the dock
    if ( sonarArray[2] < 330) {                    //until it detects the wall of the pool within 11 feet
      wallFollowCounter++;                         //then it adds to the counter and triggers state 1
    }
  }
  if (wallFollowCounter == 1) {                    //if state 1 has been triggered
    Serial.println("In wallfollow state 1");       //indicate to roboticists which state the boat is in
    setHeading (5);                                //once the boat sees the iceberg, it should turn to the port side
    if (sonarArray[4] <= 150 ) {                   //if sonars detect iceberg on its starboard side then
      wallFollowCounter++;                         //trigger state 2
    }
  }
  if (wallFollowCounter == 2) {                    //if state 2 has been triggered
    Serial.println("In wallfollow state 2");       //indicate to roboticists which state the boat is in
    setHeading(9);                                 //tell the rudders to straighten out and drive straight (to the wall)
    if ( IRarray[1] <= 100 || IRarray [2] <= 100) {//if IRs detect wall on port side and in front then
      wallFollowCounter++;                         //trigger state 3
    }
  }
  if (wallFollowCounter == 3) {                    //if state 3 has been triggered
    Serial.println("In wallfollow state 3");       //indicate to roboticists which state the boat is in
    setHeading(16);                                //tell boat to turn rudders and turn to its starboard side
    if ( IRarray[0] <= 80) {                       //If/once the boat detects the wall on its port side within 80 cm then
      wallFollowCounter++;                         //trigger state 4
    }
  }
  if (wallFollowCounter == 4) {                    //if state 4 has been triggered
    Serial.println("In wallfollow state 4");       //indicate to roboticists which state the boat is in
    maintainDistance(70, 0);   //need to set side  //use maintainDistance function to command boat to maintain a 70 cm gap between it and the wall on the port side
    if ( sonarArray[2] <= 40 && IRarray[3] > 100) {//if sonar detects something in front of it within 40 cm and the IR does not, then the boat has seen the dock and then
      wallFollowCounter++;                         //trigger state 5
    }
  }
  if (wallFollowCounter == 5) {                    //if state 5 has been triggered
    Serial.println("In wallfollow state 5");       //indicate to roboticists which state the boat is in
    setHeading(13);                                //tell boat to turn rudders to turn boat slightly to its starboard direction
    if ( sonarArray[3] > 40) {                     //if the front sonar no longer detects the dock then
      wallFollowCounter++;                         //trigger state 6
    }
  }
  if (wallFollowCounter == 6) {                    //if state 6 has been triggered
    Serial.println("In wallfollow state 6");       //indicate to roboticists which state the boat is in
    setHeading(9);                                 //tell boat to go straight ahead
  }
  votingFunc();
  moveboat();
}

void fig8Dock() {                                        //Figure 8 and self docking function
  Serial.println("In figure 8"); // leaving dock         //indicate to roboticists that the boat is currently attempting to do a figure 8 around the icebergs
  if (fig8DockCounter == 0) {                            //if boat was just commanded to do fig8Dock, counter would start in 0 and trigger the initial state
    Serial.println("In figure 8 state 0");               //indicate to roboticists which state the boat is in
    setHeading(9);
    if ( sonarArray[1] < 150) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 1) {
    Serial.println("In figure 8 state 1"); // turning toward the wall
    swerveAroundIceberg(0);     //need to set side
    if (IRarray[4] <= 100 ) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 2) {
    Serial.println("In figure 8 state 2"); // go straight toward the wall
    setHeading(9);
    if ( IRarray[1] <= 100 || IRarray [2] <= 100) {
      fig8Counter++;
    }
  }
  if (fig8DockCounter == 3) {
    Serial.println("In figure 8 state 3"); // follow the wall until you see the iceberg
    maintainDistance(100, 0);     //need to set side
    if ( sonarArray[2] <= 130) {
      fig8Counter++;
    }
  }
  if (fig8DockCounter == 4) {
    Serial.println("In figure 8 state 4"); // circle around the iceberg until you see the other iceberg
    circleIceberg(1);
    if ( sonarArray[0] <= 160) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 5) {
    Serial.println("In figure 8 state 5"); // switch to circling the second iceberg
    circleIceberg(0);
    if ( sonarArray[2] <= 160) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 6) {
    Serial.println("In figure 8 state 6"); // start figure 8 two
    circleIceberg(1);
    if ( sonarArray[0] <= 160) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 7) {
    Serial.println("In figure 8 state 7"); // finish figure 8 two
    circleIceberg(0);
    if ( sonarArray[2] <= 160) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 8) {
    Serial.println("In figure 8 state 8"); // start figure 8 three
    circleIceberg(1);
    if ( sonarArray[0] <= 160) {
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 9) {
    Serial.println("In figure 8 state 9"); // finish figure 8 three
    circleIceberg(0);
    if (pixyCheck(1)) {//if it can see the dock
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 10) {
    Serial.println("In figure 8 state 9"); // finish figure 8 three
    circleIceberg(0);
    if (pixyCheck(1) == false) { //when it's too close to see the dock, it stops
      fig8DockCounter++;
    }
  }
  if (fig8DockCounter == 11) {
    Serial.println("Done");
    realTimeRunStop = false;
  }
  votingFunc();
  moveboat();
}

void hunt() {
  if (huntCounter == 0) {
    setHeading(9);
    if ( sonarArray[1] < 150) {
      huntCounter++;
    }
  }
  if (huntCounter == 1) {
    setHeading(15); //circling to look for the critter
    if (pixyCheck(2) == true) {
      huntCounter++;
    }
  }
  if (huntCounter = 2) {
    findPixyTarget(2);
    if (pixyCheck(2) == false) {
      huntCounter++;
    }
    votingFunc();
    moveboat();
  }
}
//======================
//States
//======================

// Set Heading. Creates a function that tells the robot where to go directly and puts it in targetArray

void setHeading(int heading)
{
  Serial.print("Target array: ");
  for (int entry = 0; entry < 19; entry++) // then make a gaussian function with those values
  {
    targetArray[entry] = 100 * pow(2.718, -1 * (pow((entry - heading), 2) / 16));
    // then we populate target array with the values of the gaussian function from 0 to 18
    //Serial.print(targetArray[entry]);
    //Serial.print("\t");
  }
  //Serial.println();
}
// Swerve Around Iceberg
void swerveAroundIceberg(int side) {  // side 0 is left, side 1 is right
  if (side == 0) {
    setHeading(5);
  }
}

//maintainDistance
void maintainDistance(int dist, int side)   // distance in cm, side: 0 is left, 1 is right
{
  if (side == 0) {
    if (IRarray[0] < 0.8 * dist)
    {
      setHeading(11);
    } else if (IRarray[0] > 1.1 * dist) {
      setHeading(7);
    }
  } else if (side == 1) {
    if (IRarray[18] < 0.8 * dist)
    {
      setHeading(12);
    } else if (IRarray[18] > 1.1 * dist) {
      setHeading(6);
    }
  }
}

void circleIceberg(int side)
{
  if (side == 0) {
    if (IRarray[0] < 100) {
      setHeading(9);
    }
    else {
      setHeading(5);
    }
  }
  if (side == 1) {
    if (IRarray[5] < 100) {
      setHeading(9);
    }
    else {
      setHeading(13);
    }
  }
}

// Voting Function
// Takes the gaussian functions from find object and find target and outputs an angle
// and a distance to the point we want to go to.
void votingFunc()
{
  int voteArray[19];
  for (int entry = 0; entry < 19; entry++)
  {
    voteArray[entry] = targetArray[entry] + objectArray[entry];
  }
  int maximum = 0;
  int maximumIndex = voteArray[0];
  for (int i = 0; i < 19; i++)
  {
    if (voteArray[i] > maximum)
    {
      maximum = voteArray[i];
      maximumIndex = i;
    }
  }
  int theta = maximumIndex * 10; // so that we're giving the boat an angle.
  int magnitude = maximum; // A number from 0 to 150. It gets bigger as the target gets farther away
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

//=========================================================================================
// ACT functions act---act---act---act---act---act---act---act---act---act---act---act---act---
//=========================================================================================

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
  int turningmaxspeed[] = {20, 30, 30, 30, 30, 30, 20};
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
