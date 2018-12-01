/*
Arduino library to have easy acces to different light patterns an Adafruit
NeoPixel strip. This has been created to eventually be used on a
self-controlling robot tug boat.

Written by Viktor Deturck for Fundamentals of Robotics - Act Lab
Light patterns created by John Moreland.
@Olin college of Engineering
*/

#include <Arduino.h>
#include <TugNeoPixel.h>
#include <Adafruit_NeoPixel.h>

TugNeoPixel::TugNeoPixel(int pin, int nrPixels):strip(nrPixels, pin, NEO_GRB + NEO_KHZ800)  // strip is instance of Adafruit_NeoPixel, needs to be defined in the class definition
{
  // Declaring the colors
  OFF      = strip.Color(0, 0, 0, 0);
  YELLOW   = strip.Color(255, 150, 0, 50);
  RED      = strip.Color(255, 0, 0, 0);
  ORANGE   = strip.Color(255, 50, 0, 0);
  GREEN    = strip.Color(0, 255, 0, 0);
  BLUE     = strip.Color(0, 0, 255, 0);
  _nrPixels = nrPixels;

}

void TugNeoPixel::begin()
{
  strip.setBrightness(50);  // Begins the working of the strip
  strip.begin();
  strip.show();
}

void TugNeoPixel::setBrightness(int brightness) //Sets the brightness of the strip
{
  strip.setBrightness(brightness);
}

void TugNeoPixel::neoPixelTestLoop() {
  neoPixelWarningStatic();
  delay(2000);

  neoPixelEstopStatic();
  delay(2000);

  neoPixelTargetLeftStatic();
  delay(2000);

  neoPixelTargetRightStatic();
  delay(2000);

  //do 6 loops
  for (int i = 0; i < 6; i++) {
    neoPixelLightLoop();
  }
  delay(100);
}

void TugNeoPixel::neoPixelLightLoop() {
  // this function lights up pairs of neopixels in sequence

  // turn all pixels off
  for (int i = 0; i < _nrPixels; i++) {
    strip.setPixelColor(i, OFF);
  }
  strip.show();

  // loop through turning pixels on and off
  for (int i = 0; i < _nrPixels; i++) {

    // Get index of last pixel (value needs to stay between 0 and 11)
    int lastPixel = (_nrPixels + i - 1) % _nrPixels;

    // update strip
    strip.setPixelColor(lastPixel, 0, 0, 0); //make last pixel dark
    strip.setPixelColor(i, BLUE); //make current pixel blue
    strip.setPixelColor(i + 1, BLUE);
    strip.show();
    delay(35);
  }
}

void TugNeoPixel::neoPixelWarningDynamic() {
  // this function flashes

  // set even index lights to yellow
  for (int i = 0; i < _nrPixels; i = i + 2) {
    strip.setPixelColor(i, YELLOW);
  }
  strip.show();

  // set even index lights to off
  for (int i = 0; i < _nrPixels; i = i + 2) {
    strip.setPixelColor(i, OFF);
  }
  delay (250);
  // set odd index lights to yellow
  for (int i = 1; i < _nrPixels; i = i + 2) {
    strip.setPixelColor(i, YELLOW);
  }
  strip.show();
  delay(250);

  // set odd index lights to off
  for (int i = 1; i < _nrPixels; i = i + 2) {
    strip.setPixelColor(i, OFF);
  }
}
void TugNeoPixel::neoPixelWarningStatic() {
  // this function displays a static warning warning light
  Serial.println("Check");
  //Assign LEDs
  int color1_lights[] = {0, 1, 4, 5, 8, 9, 12, 13};
  int color2_lights[] = {2, 3, 6, 7, 10, 11, 14, 15};
  int color1_lights_length = sizeof(color1_lights) / sizeof(int);
  int color2_lights_length = sizeof(color2_lights) / sizeof(int);

  //Change each LED's color
  for (int i = 0; i < (color1_lights_length); i++) {
    int set_led = color1_lights[i];
    strip.setPixelColor(set_led, YELLOW);
    //delay(1);
  }
  for (int i = 0; i < (color2_lights_length); i++) {
    int set_led = color2_lights[i];
    strip.setPixelColor(set_led, OFF);
    //delay(1);
  }

  //Push LED changes
  strip.show();
}

void TugNeoPixel::neoPixelEstopStatic() {
  //this function displays an "estop" behavior

  //Assign LEDs
  int color1_lights[] = {0, 2, 4, 6, 8, 10, 12, 14};
  int color2_lights[] = {1, 3, 5, 7, 9, 11, 13, 15};
  int color1_lights_length = sizeof(color1_lights) / sizeof(int);
  int color2_lights_length = sizeof(color2_lights) / sizeof(int);
  Serial.println("Check");

  //Change each LED's color
  for (int i = 0; i < (color1_lights_length); i++) {
    int set_led = color1_lights[i];
    strip.setPixelColor(set_led, RED);
    //delay(1);
  }
  for (int i = 0; i < (color2_lights_length); i++) {
    int set_led = color2_lights[i];
    strip.setPixelColor(set_led, ORANGE);
    //delay(1);
  }

  //Push LED changes
  strip.show();
}


void TugNeoPixel::neoPixelTargetLeftStatic() {
  //this function displays a "target seen in left field of view" behavior

  //Assign LEDs
  int color1_lights[] = {2, 3, 4, 5};
  int color2_lights[] = {0, 1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  int color1_lights_length = sizeof(color1_lights) / sizeof(int);
  int color2_lights_length = sizeof(color2_lights) / sizeof(int);

  //Change each LED's color
  for (int i = 0; i < (color1_lights_length); i++) {
    int set_led = color1_lights[i];
    strip.setPixelColor(set_led, GREEN);
    //delay(1);
  }
  for (int i = 0; i < (color2_lights_length); i++) {
    int set_led = color2_lights[i];
    strip.setPixelColor(set_led, OFF);
    //delay(1);
  }

  //Push LED changes
  strip.show();
}


void TugNeoPixel::neoPixelTargetRightStatic() {
  //this function displays a "target seen in right field of view" behavior

  //Assign LEDs
  int color1_lights[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 15};
  int color2_lights[] = {10, 11, 12, 13};
  int color1_lights_length = sizeof(color1_lights) / sizeof(int);
  int color2_lights_length = sizeof(color2_lights) / sizeof(int);

  //Change each LED's color
  for (int i = 0; i < (color1_lights_length); i++) {
    int set_led = color1_lights[i];
    strip.setPixelColor(set_led, OFF);
    //delay(1);
  }
  for (int i = 0; i < (color2_lights_length); i++) {
    int set_led = color2_lights[i];
    strip.setPixelColor(set_led, GREEN);
    //delay(1);
  }

  //Push LED changes
  strip.show();
}

void TugNeoPixel::neoPixelUp() {
  //this function displays a "target seen in right field of view" behavior

  //Assign LEDs
  int color1_lights[] = {2,3,4,5,6,7,8,9,10,11,12,13};
  int color2_lights[] = {14,15,0,1};
  int color1_lights_length = sizeof(color1_lights) / sizeof(int);
  int color2_lights_length = sizeof(color2_lights) / sizeof(int);

  //Change each LED's color
  for (int i = 0; i < (color1_lights_length); i++) {
    int set_led = color1_lights[i];
    strip.setPixelColor(set_led, OFF);
    //delay(1);
  }
  for (int i = 0; i < (color2_lights_length); i++) {
    int set_led = color2_lights[i];
    strip.setPixelColor(set_led, RED);
    //delay(1);
  }

  //Push LED changes
  strip.show();
}

void TugNeoPixel::displayTarget(int degrees) {
  //This displays the target as a green pixel on the NeoPixel Ring
  strip.setPixelColor(_lastObstaclePos, OFF);
  int pixelPos = (int) (degrees/(360/_nrPixels)+0.5);
  strip.setPixelColor(pixelPos, GREEN);
  _lastObstaclePos = pixelPos; // Storing what pixel was last on.
  strip.show();
}

void TugNeoPixel::displayObstacle(int degrees) {
//This displays obstacles as red pixels on the NeoPixel Ring
  strip.setPixelColor(_lastTargetPos, OFF);
  int pixelPos = (int) (degrees/(360/_nrPixels)+0.5);
  strip.setPixelColor(pixelPos, RED);
  _lastTargetPos = pixelPos; // Storing what pixel was last on.
  strip.show();
}

void TugNeoPixel::displayHeading(int degrees) {
//This displays obstacles as red pixels on the NeoPixel Ring
  strip.setPixelColor(_lastHeadingPos, OFF);
  int pixelPos = (int) (degrees/(360/_nrPixels)+0.5);
  strip.setPixelColor(pixelPos, BLUE);
  _lastTargetPos = pixelPos; // Storing what pixel was last on.
  strip.show();
}

