#ifndef _MYLIB_H_
#define _MYLIB_H_

#include "simpletools.h"
#include "ping.h"
#include "abdrive.h"
	
  extern const int DO, CLK, DI, CS;
  extern const float widthOfRobotF;
  extern const int widthOfRobot;
  extern const float distancePerTick;
  extern const float wheelDistance;
  /*typedef struct Waypoint{
    float x_pos;
    float y_pos;
    struct Waypoint * next;
  } Waypoint;*/

  //extern char* itos(int x);
  extern double msin(double x);
  extern double mcos(double x);
  extern double masin(double x);
  extern void calculateIR(int * irLeft, int * irRight, int * irLeftOld, int * irRightOld);
  extern void turnAround(double angle, double radius);
  extern void turnInPlace(double angle);
  extern void turnInPlaceDeg(double angle);
  extern void turnInPlaceNoOvershoot(double angle);
  extern void turnInPlaceDegNoOvershoot(double angle);
  
#endif
