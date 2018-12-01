#ifndef TugNeoPixel_h
#define TugNeoPixel_h

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif


#define NUM_LEDS 12
#define BRIGHTNESS 100


class TugNeoPixel
{
  public:
    TugNeoPixel(int pin, int nrPixels);
    void neoPixelLightLoop();
    void neoPixelWarningDynamic();
    void neoPixelWarningStatic();
    void neoPixelEstopStatic();
    void neoPixelTargetLeftStatic();
    void neoPixelTargetRightStatic();
    void neoPixelTestLoop();
    void neoPixelUp();
    void displayTarget(int degrees);
    void displayObstacle(int degrees);
    void displayHeading(int degrees);
    void begin();
    void setBrightness(int brightness);

  private:
    int _pin;
    int _nrPixels;
    int _lastTargetPos;
    int _lastObstaclePos;
    int _lastHeadingPos;
    Adafruit_NeoPixel strip;
    uint32_t OFF;
    uint32_t YELLOW;
    uint32_t RED;
    uint32_t ORANGE;
    uint32_t GREEN;
    uint32_t BLUE;	



};
#endif

