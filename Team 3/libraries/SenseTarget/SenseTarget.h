/*
  SenseTarget.h - Library for detecting narwhal target using Pixycam
  Created by Jai Patil, October 27, 2018.
  Released for Olin Fun Robo students.
*/
#ifndef SenseTarget_h
#define SenseTarget_h

#include "Arduino.h"
#include "Pixy.h"
#include "SPI.h"


class SenseTarget
{
  public:
    SenseTarget();
    void getTargetBearing(int bearing[],int len, Pixy pixy,int blocks, int range, int curvebroadness);
    int getTargetRange(Pixy pixy, int blocks);
  private:
};
    
#endif
