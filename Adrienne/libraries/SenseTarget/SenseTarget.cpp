/*
  SenseTarget.cpp - Library for detecting narwhal target using Pixycam
  Created by Jai Patil, October 27, 2018.
  Released for Olin Fun Robo students.
*/

#include "Arduino.h"
#include "SenseTarget.h"
#include "Pixy.h"
#include "SPI.h"

/*
  CONSTRUCTOR
*/
SenseTarget::SenseTarget()
{

  
}

/*
  PIXYCAM FUNCTION
  Function that fills the bearing array with a gaussian function mapped from -90deg to 90deg,
  with a peak at location of the target, a set amplitude, and a specified variance. 
*/
void SenseTarget::getTargetBearing(int bearing[],int len, Pixy pixy,int blocks, int range, int curvebroadness)
{
  //Pixy pixy;
  //pixy.init();
  
  //blocks = pixy.getBlocks();
  int amplitude = 0;
  if (range>=5 && range<=150)
  {
    amplitude = map(range,5,150,0,100);
  } else if (range > 1500)
  {
    amplitude = 100;
  }

  if (blocks)
  {
    //Serial.print("In the library: ");
    //Serial.println(blocks);
    for (int i=0;i<blocks;i++)
    {
      Serial.print("The signature: ");
      Serial.println(pixy.blocks[i].signature);
      if(pixy.blocks[i].signature)
      {
         //required array indices
         double arraymidpoint = (len-1)/2.0;
         double arrayhalfwidth = (double)len*37.5/180.0;
         //Serial.print("The arraymidpoint and arrayhalfwidth: ");
         //Serial.println(arraymidpoint);
	 //Serial.println(arrayhalfwidth);

         //use map function to convert pixel values to bearing[] indices
         double targetcenter = (double)(pixy.blocks[i].x)*(2.0*arrayhalfwidth)/(319.0)+(arraymidpoint-arrayhalfwidth);
		
         double targetwidth = (double)(pixy.blocks[i].width)*arrayhalfwidth/(160);
         //Serial.print("The targetcenter and targethalfwidth: ");
         //Serial.println(targetcenter);
	 //Serial.println(targetwidth);
		
         
         //gaussian function = a*exp(-pow(x-b,2)/(2*pow(c,2))) 
         //in bearing[] with amplitude a = amplitude, b = targetcenter, c = targetwidth/curvesharpness
         for(int j=0; j<len; j++)
         {
            bearing[j]= 1.0+(double(amplitude*exp(-pow(j-targetcenter,2)/(2*pow(targetwidth*curvebroadness/1.5,2)))));
         }
      }
    }
  }
}

/*
  PIXYCAM FUNCTION
  Function that returns range of detected target based on calibrated distance measurements (in mm) 
*/
int SenseTarget::getTargetRange(Pixy pixy, int blocks)
{
  //Pixy pixy;
  //pixy.init();
  double range;
  //blocks = pixy.getBlocks();
  if (blocks)
  {
    //Serial.print("In the library: ");
    //Serial.println(blocks);
    for (int i=0;i<blocks;i++)
    {
      double expratio = 1.32;
      double currentratio = (double)pixy.blocks[i].width/(double)pixy.blocks[i].height;
      //Serial.print("Aspect ratio: ");
      //Serial.println(currentratio);
      //Serial.print("The signature: ");
      //Serial.println(pixy.blocks[i].signature);
	//pixy.blocks[i].signature==1 && 
      if(currentratio>=(0.8*expratio) && currentratio<=(1.2*expratio))
      {
	//Serial.println("It has entered computation");        
	//object pixel area
        double pxArea = (double)pixy.blocks[i].height*(double)pixy.blocks[i].width;

        //object range
        range = 5345.5*pow(pxArea,-0.535);
      }
    }
  }
  return (int)range;
}
