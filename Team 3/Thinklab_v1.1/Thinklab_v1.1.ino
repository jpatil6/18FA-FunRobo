/*****************************************************************************************
 * Title: 2018 Fun-Robo Narwhal chasing tugboat (ENGR3390 Think Lab)
 * Description: This structure template contains a SENSE-THINK-ACT flow to
 * allow a robotic tugboat to perform a sequence of meta-behaviors in soft-real-time 
 * based on direct text commands from a human operator.
 * Robot Name: PyCkSiE tEaM 3 tug boat
 * What does code do: poll open, 
 * sense: Detect range and bearing of target and obstacles with the Pixycam and IR sensors 
 * think: Combine bearing arrays for target and obstacles to optimize bearing of the tugboat. 
 *        If unobstructed on set bearing, calculate propellor speed proportional to range
 * act:   Set table motor and rudder bearing, set propellor speed
 * carry out operator text input, loop indefinitely
 * Hardware warnings: Do not wire the Pixycam ICSP arduino I/O pin the wrong way around.
 * Created by PyCkSiE tEaM 3 October 2018
 * ***************************************************************************************
 */

// TODO: 
//      SENSE:
//      THINK: 
//      ACT: 

//========================================================================================
// Load supporting Arduino Libraries
//========================================================================================
#include <Servo.h>    //example of loading ServoMotors library
#include <Pixy.h>
#include <SPI.h>
#include <SenseTarget.h>
#include <SharpIR.h>

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

//int IR1Pin = 1;                  //Analog input pins from 5 IR sensors
//int IR2Pin = 2;
//int IR3Pin = 3;
//int IR4Pin = 4;
//int IR5Pin = 5;
int tablePotPin = 0;             //Analog input pin from table turning potentiometer

int rudderPin = 3;                //Digital output to motors
int turnTablePin = 5;
int propellorPin = 6;
int redPin = 9;                   //RGB behavior light PWM pins
int greenPin = 10;
int bluePin = 11;

SenseTarget sensetarget;
IRObject irobject(60,10);
Pixy pixy;
Servo propellor;
Servo table;
Servo rudder;

const int bearinglen = 15;
int targetbearing[bearinglen];
int targetrange;
int objectbearing[bearinglen];

int bearingcommand=1;
int speedcommand=0;
int yawratecommand = 0;

//=========================================================================================
// Startup code to configure robot and pretest all robot functionality (to run once)
// and code to setup robot mission for launch.
//=========================================================================================
void setup() {
  
  //pinMode(eStopPin, INPUT_PULLUP);    // use internal pull-up on ESTOP switch input pin
  Serial.begin(9600);                 // start serial communications
  Serial.println(F(" Robot Controller Starting Up! Watch your fingers! "));

  pixy.init();

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);

  propellor.attach(propellorPin);
  table.attach(turnTablePin);
  rudder.attach(rudderPin);

  rudder.write(1500);
  propellor.writeMicroseconds(1500);

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

while(realTimeRunStop == true) {      // if OCU-Stop not commanded, run control loop
  // Check if operator inputs a command during real-time loop execution
  if (Serial.available() > 0) {     // check to see if operator typed at OCU
    realTimeRunStop = false;      // if OCU input typed, stop control loop
    command = Serial.readString();      // read command string to clear buffer
    //Serial.println("command 2 is "+command);
    break;      // break out of real-time loop
  }
  else {realTimeRunStop = true;}    // if no operator input, run real-time loop

  // Real-Time clock control. Check to see if one clock cycle has elapesed before running this control code
  newLoopTime = millis();           // get current Arduino time (50 days till wrap)
  if (newLoopTime - oldLoopTime >= controlLoopInterval) { // if true run flight code
    oldLoopTime = newLoopTime;      // reset time stamp

//SENSE-sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense-------

  //IR object detection
  irDistAndBearing(objectbearing,bearinglen);
  
  //Pixy target detection
  int blocks = pixy.getBlocks();
  if(blocks)
  {
    targetrange = sensetarget.getTargetRange(pixy,blocks);
    sensetarget.getTargetBearing(targetbearing,bearinglen,pixy,blocks,targetrange,1);    
  
  //combine Pixycam and IR bearing arrays
    for(int i=0;i<bearinglen;i++)
    {
      targetbearing[i]=targetbearing[i]*objectbearing[i];
    }
  }

  printSenseData();
  
  
// THINK think---think---think---think---think---think---think---think---think---think---think---------
  // pick robot behavior based on operator input command typed at console
    
    if ( command == "stop"){
      Serial.println("Stop Robot");
      setColor(255,0,0);          //LED set to RED
      realTimeRunStop = false;    //exit real time control loop
      break;
    }
    else if (command == "chase"){  //Move robot to Operator commanded position
      if(blocks>0)
      {
        setColor(0,255,0);        //LED set to Green 
      }else{
        setColor(0,0,255);        //LED set to Blue
      }
      //Serial.println("Move robot ");

      calculateSpeedBearingcommands();
      //goToBearing(bearingcommand);
      yawRateFunction(yawratecommand);
      setPropSpeed(speedcommand);
      
      Serial.println("Type stop to stop robot");
      realTimeRunStop = true;     //don't exit loop after running once
    }
    else if (command == "idle"){  //Make robot alive with small motions
      Serial.println("Idle Robot");
      setColor(255/3,255,255);    //LED set to Coral White
      Serial.println("Type stop to stop robot");
      
      printSenseData();
      
      realTimeRunStop = true;     //run loop continually
    }
    else
    {
      Serial.println("***** WARNING *******Invalid Input, Robot Stopped, Please try again!");
      realTimeRunStop = false;
    }

// ACT-act---act---act---act---act---act---act---act---act---act---act---act---act---act------------

    //ESTOP = digitalRead(eStopPin);    // check ESTOP switch

// Check to see if all code ran successfully in one real-time increment
    cycleTime = millis()-newLoopTime;     // calculate loop execution time
    if( cycleTime > controlLoopInterval){
      Serial.println("******************************************");
      Serial.println("error - real time has failed, stop robot!"); // loop took too long to run
      Serial.print(" 1000 ms real-time loop took = ");
      Serial.println(cycleTime);                                   // print loop time
      Serial.println("******************************************");
      propellor.writeMicroseconds(1500);
      break;          // break out of real-time inner loop
      }
      } // end of "if (newLoopTime - oldLoopTime >= controlLoopInterval)" real-time loop structure
    } // end of "inner" "while(realTimeRunStop == true)" real-time control loop
    // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******
    // real-time-loop******real-time-loop******real-time-loop******real-time-loop******real-time-loop******

  // SEND Robot State to Operator Control Unit-(OCU) ocu--- ocu-- ocu--- ocu--- ocu--- ocu-- -ocu--
    Serial.println("================================================================");
    Serial.println("| Robot control loop stopping to wait for new command ");   // send robot status
    //if (ESTOP == true) Serial.println("| Robot motors E-Stopped by external switch");   // send E-Stop

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

// Realtime loop functions loop---loop---loop---loop---loop---loop---loop---loop---loop---loop----

// OCU functions ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu---ocu------------

// This function prints operator command options on the serial console and prompts
// operator to input desired robot command
String getOperatorInput()
{
  Serial.println(F("======================================================================================"));
  Serial.println(F("|Robot Behavior-Commands: chase(moves robot), stop(e-stops motors), idle(robot idles)|"));
  Serial.println(F("|                                                                                    |"));
  Serial.println(F("| Please type desired robot behavior in command line at top of this window           |"));
  Serial.println(F("| and then press SEND button.                                                        |"));
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

// This function prints the sensed bearing arrays and range for target and obstacles
void printSenseData()
{
  Serial.println(F("Target Bearing array: "));
  for(int i=0;i<bearinglen;i++)
  {
    Serial.print(targetbearing[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print(F("Target range: "));
  Serial.println(targetrange);  
}

// SENSE functions sense---sense---sense---sense---sense---sense---sense---sense---sense---

void irDistAndBearing (int expandedBearingArray[], int desiredLength) {
  int irDistArray[5];
  for (int irCounter = 0; irCounter < 5; irCounter++) {
    
    SharpIR sharp(irCounter + 1, 1080);  

    
    irDistArray[irCounter] = sharp.distance();
    
    // Converting distance output into a thresholded negative version of the field
    
       if (irDistArray[irCounter] > 60) { 
        irDistArray[irCounter] = 100; 
       } else if (irDistArray[irCounter] > 10 && irDistArray[irCounter] < 60) {
         irDistArray[irCounter] = map(irDistArray[irCounter], 10, 60, 0, 100);
       } else if (irDistArray[irCounter] < 10){
         irDistArray[irCounter] = 0; 
        } // closes if/else loop
        
       for (int timesValueRepeated = 0; timesValueRepeated < desiredLength/5 ; timesValueRepeated++) { 
          
         expandedBearingArray[timesValueRepeated+3*irCounter] = irDistArray[irCounter]; 

         
         //Serial.print(expandedBearingArray[timesValueRepeated+3*irCounter]); 
         //Serial.print("\t"); 
         
         } //closes duplication for loop   
         
    } // closes surrounding for loop
    
    //Serial.println(); 
  
} // closes irRange

// THINK functions think---think---think---think---think---think---think---think---think---

void calculateSpeedBearingcommands()
{
  //calculate the maximum of the target bearing array
  boolean isMaxArray[bearinglen];
  int globalmax=-99;
  for(int i =0; i<bearinglen;i++)
  {
    isMaxArray[i]=false;
    if (targetbearing[i]>globalmax)
    {
      globalmax = targetbearing[i];
      for (int j = 0;j<i;j++)
      {
        isMaxArray[j]=false;
      }
      isMaxArray[i]=true;
    }else if(targetbearing[i]==globalmax)
    {
      isMaxArray[i]=true;
    }
  }

  
  //find the median value of isMaxArray
  int optima;
  if(isMaxArray[bearinglen/2])
  {
    optima = bearinglen/2;
  }else{
    int optimaright=10000;
    int optimaleft=10000;
    for(int i = bearinglen/2+1;i<bearinglen;i++)
    {
      if(isMaxArray[i])
      {
        optimaright = i-bearinglen/2+1;
        optima = i;
        break;
      }
    }
    for(int i = bearinglen/2-1;i>=0;i--)
    {
      if(isMaxArray[i])
      {
        optimaleft =  bearinglen/2-1-i;
        if(optimaleft<optimaright)
        {
          optima = i;
        }
        break;
      }
    }
  }
  //Serial.println(optima);
  
  if(targetbearing[2]>1)
  {
    //map optima to bearingcommand
    bearingcommand = map(optima,0,bearinglen-1,-90,90);
    Serial.print("Set bearing to in degrees: ");
    Serial.println(bearingcommand);

    yawratecommand = map(optima,0,bearinglen-1,-250,250);
    
    //map amplitude at optima to speedcommand
    speedcommand = map(targetbearing[optima],100,10000,0,100);
    Serial.print("Set speed to in percentage: ");
    Serial.println(speedcommand);
  }else{
    yawratecommand = 0;
  }
}

// ACT functions act---act---act---act---act---act---act---act---act---act---act---act---act---

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}

//function to set propellor speed by percentage 
void setPropSpeed(int speedPercentage){
  int microSec;
  if (speedPercentage>99){          //2000 mm [this is a stand in number]
    microSec= 2000;       //full speed forward
  }
  else if (speedPercentage<=99 && speedPercentage>=0){   //1000 [stand in]
    microSec = map(speedPercentage,0, 100, 1500, 2000); // test speeds of different distances
  }
  else if (speedPercentage < 0){
    microSec=1000;         //back up slowly from narwhale to prevent running it over
  }
  propellor.writeMicroseconds(microSec);
}

//function to set bearing of turn table and rudder.
//int targetBearing: the angle bearing the boat must turn to 
void goToBearing(int targetBearing) //FUNCTION
{

  int potPin=0;
  int voltsPerRevolution=463;
  int straightAheadVoltage=512;
  int potVoltage;
  int difference;
  int pulseWidth;
  int neutralPulseWidth=1500;
  int maxPulseWidth=2000;
  int minPulseWidth=1000;
  int rudderPulseWidth;
  int differenceScale=10;
  int rudderDifferenceScale=4;
  bool checker1;
  bool checker2;
  float targetBearingVoltage;

  float voltsPerDegree=voltsPerRevolution/360.0;
  //Serial.print("Volts Per Degree is ");
  //Serial.println(voltsPerDegree);
  targetBearingVoltage=straightAheadVoltage+targetBearing*voltsPerDegree;//voltage measured on a scale of 0 to 1024
  potVoltage=analogRead(potPin);

  difference=targetBearingVoltage-potVoltage; //difference is positive if target is right of current heading, negative if target is left

  checker1=difference>0;
  //Serial.println(checker1);
  
  do
  {
  //Serial.print("Target voltage is");
  //Serial.println(targetBearingVoltage);
  potVoltage=analogRead(potPin);
  //Serial.println("Pot Voltage is");
  //Serial.println(potVoltage);
  difference=targetBearingVoltage-potVoltage;
  checker2=difference>0;
  //Serial.println(checker2);
  pulseWidth=1500+difference*differenceScale;
  if(pulseWidth>maxPulseWidth){
    pulseWidth=maxPulseWidth;
  }
  if(pulseWidth<minPulseWidth){
    pulseWidth=minPulseWidth;
  }
  table.writeMicroseconds(pulseWidth);
  rudderPulseWidth=(pulseWidth-neutralPulseWidth)*rudderDifferenceScale+neutralPulseWidth;
  rudder.writeMicroseconds(rudderPulseWidth);
  //rudder.writeMicroseconds(-pulseWidth);
  //Serial.print("Pulse Width is");
  //Serial.println(pulseWidth);
  delay(4);
  } while(checker2==checker1); // && bearingInt==0);
  //Serial.println("Target Bearing Reached");
  table.writeMicroseconds(neutralPulseWidth);
}

void yawRateFunction(int yawRate) //MAY need to initialize yawRate in setup--I'm not sure
{
  //Initializing the local user-configurable variables
  int neutralPulseWidth=1500;
  int maxPulseWidth=2000;
  int minPulseWidth=1000;
  float rudderScale=0.6;

  //Initializing the local non user-configurable variables
  int pulseWidth;
  int rudderPulseWidth;

  //Serial.println("!!");
  pulseWidth=neutralPulseWidth+yawRate;
  table.writeMicroseconds(pulseWidth);

  rudderPulseWidth=neutralPulseWidth+rudderScale*yawRate;
  rudder.writeMicroseconds(rudderPulseWidth);
  
}

// END of Functions
//=============================================================================
// END OF Robot Control CODE
