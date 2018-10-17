// TODO:
//  - filter bad data from pixycam
//  - experiment to see if narwhal can be found more reliably in different lighting
//  - incorporate behaviors, like scanning for narwhal when it isn't found, or an idle
//  - incorporate IR sensing, look to pixyteam 1's code for help here
//============================================================================================
// Load supporting Arduino Libraries
#include <SPI.h>
#include <Pixy.h>
#include <EasyTransfer.h>
//============================================================================================
// Create global variables, objects, constants
Pixy pixy; // This is the main Pixy object
EasyTransfer ET; // Transfer object

// creating data structure to send to ACT arduino
struct SEND_DATA_STRUCTURE
{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int error_xValue;
};
//Give struct an actual name
SEND_DATA_STRUCTURE pixydata;

int xValue;
int angle;
int newLoopTime = 0;
int oldLoopTime = 0;
int controlLoopInterval = 25;
int desired_xValue = 315/2;
int actual_xValue = 0;
//============================================================================================
// Startup code to configure robot and set mission for launch.
//============================================================================================
void setup()
{
  ET.begin(details(pixydata), &Serial);
  pixy.init();
  Serial.begin(9600);
  Serial.print("Starting...\n");
}
//============================================================================================
// Flight code to run continuously
//============================================================================================
void loop()
{
  int i;                                //variable needed for pixy functionality
  uint16_t blocks;                      //variable needed for pixy functionality
  char buf[32];                         //variable needed for pixy functionality


  //------------------------REAL TIME CONTROL LOOP STARTS HERE ---------------------
  // real time clock control
  i++;                                  // needed for pixy functionality
  newLoopTime = millis();               // update time
  //run loop once every control loop interval
  if (newLoopTime - oldLoopTime >= controlLoopInterval)
  {
    // grab blocks!
    blocks = pixy.getBlocks();          // 1 = blocks detected, 0 = no blocks

    // degugging section -> uncomment the print statemments under here and upload to Arduino to help debug
//    Serial.print("Time: ");
//    Serial.println(millis());
//    Serial.print("Blocks? ");
//    Serial.println(blocks);
    
    oldLoopTime = newLoopTime;          // update time
    // if blocks are detected, find the x centerpoint and remap to angle. Update pixydata, send
    if (blocks)
    {
      actual_xValue = pixy.blocks[0].x;          //give x center of object
      pixydata.error_xValue = desired_xValue - actual_xValue; // calculate pixel distance between desired x and actual x pixels
      
      // debugging section -> uncomment this to help debug
//      Serial.print("X actual: ");
//      Serial.println(actual_xValue); // x position of narwhal on camera

      ET.sendData();                             // send data to next Arduino
    }
  }
  // ---------------------REAL TIME CONTROL LOOP ENDS HERE -------------------------

}
