/*
 * please correct what the idle speed is in lines 26, and 92!!!!
 * also, i have the reverse speed set to 1200, I don't know if that is too fast in reverse or not (line 98)
 * -Hanna
 */
#include <Servo.h>

Servo prop;
Servo rudder;
int propPin = 6;          //set pin
int rudderPin = 7;         //set pin
int speedcommand = 0;
int heading = 0;


boolean realTimeRunStop=true;         //create a name for real time control loop flag
String command ="stop    ";           //create a String object name for operator command string
String loopError="no error";          //create a String for the real time control loop error system
unsigned long oldLoopTime=0;          //create a name for past loop time in milliseconds
unsigned long newLoopTime=0;          //create a name for new loop time in milliseconds
unsigned long cycleTime=0;            //create a name for elapsed loop cycle time
const long controlLoopInterval=1000;  //create a name for control loop cycle time in milliseconds

void setup() {
  Serial.begin (9600);
  prop.attach (propPin);
  rudder.attach (rudderPin);
  prop.writeMicroseconds(1300);   // pls correct idle speed
  delay (500);
  Serial.println ("CAREFULL ROBOT IS RUNNING");
}
String getOperatorInput (){
  // This function prints operator command options on the serial console and prompts 
  // operator to input desired robot command
  // Serial.println("  ") 
  
  Serial.println("Enter speedcommand)");
  while (Serial.available() ==0) {};                     // do nothing until operator input typed
  command = Serial.readString();                         // read command string
  command.trim ();
  Serial.print(" New speedcommand is:    ");  // give command feedback to operator 
  Serial.println(command); 
  command = speedcommand;
  Serial.println("Enter heading)");
  while (Serial.available() ==0) {};                     // do nothing until operator input typed
  command = Serial.readString();                         // read command string
  command.trim ();
  Serial.print(" New heading is:    ");  // give command feedback to operator 
  Serial.println(command); 
  command = heading;
  Serial.println("| Type 'stop' to stop control loop and wait for new command                             |"); 
  Serial.println("=========================================================================================");
  return command;
}


void loop() {

    command = getOperatorInput();         //get operator input from serial monitor
  if (command == "stop") realTimeRunStop = false;      //skip real time inner loop
  else realTimeRunStop=true;      //Set loop flag to run = true
  while(realTimeRunStop==true){     //if OCU-Stop not commanded, run control loop
    //Check in operator inputs a command during real-time loop execution
    if (Serial.available()>0){          //check to see in operator typed at OCu
      realTimeRunStop=false;            //if OCU input typed, stop control loop
      command=Serial.readString();      //read command string to clear buffer
      break;                            //break out of real-time loop
    }
    else {realTimeRunStop=true;}        //if no operator input, run real-time loop

    // REal-Time clock control. Check to see in one clock cycle has elapesed before running this control code
    newLoopTime=millis();               //get current Arduino time (50 days till wrap)
    if (newLoopTime -oldLoopTime>= controlLoopInterval) {   //if true run flight code
      oldLoopTime=newLoopTime;      //reset time stamp
    }

    if (command=="stop"){
        Serial.println("Stop Robot");
        realTimeRunStop=false;      //exit real time control loop
        break;
      }
    else { 
      Move(speedcommand, heading);
    }
  }
}

void Move(int speedPercentage, int setDirection){
  int microSec;
  int rudderAngle; 
  if (speedPercentage>99){          
    microSec= 2000;       //full speed forward
  }
  else if (speedPercentage<=99 && speedPercentage>=0){  
    microSec = map(speedPercentage,0, 100, 1300, 2000);     // pls correct idle speed
  }
  else if (speedPercentage < 0){
    microSec=1200;         //Run away!!! 
  }
    
  /*if (setDirection >= 0 && setDirection<= 90){
      rudderAngle = map(setDirection, 0, 90, 180, 90);
  }
  else if (setDirection >=270 && setDirection<= 360){
    rudderAngle = map(setDirection, 270, 360, 180, 90);
  }
  */
  rudderAngle = map(setDirection, -90,90, 180, 0);
  rudder.write (rudderAngle);
  prop.writeMicroseconds(microSec);
  
}
