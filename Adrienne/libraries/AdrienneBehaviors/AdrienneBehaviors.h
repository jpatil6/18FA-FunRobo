/*
  AdrienneBehaviors.h - Library for describing all the mission behaviors
  Adrienne has
  Created by Eamon, October 27, 2018.
*/
#ifndef AdrienneBehaviors_h
#define AdrienneBehaviors_h

#include "Arduino.h"

class AdrienneBehaviors
{
public:
  AdrienneBehaviors();
  void fig8();
  void wallfollow();
  void fig8Dock();
  void hunt();
private:
};

#endif
