/*  Copyright 2009 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This file is part of libSOC.

    libSOC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libSOC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libSOC.  If not, see <http://www.gnu.org/licenses/> */

#ifndef MT_schunk_h
#define MT_schunk_h

#include "array.h"
#include "process.h"
#include "robot_variables.h"
#include <vector>

//fwd declarations
class CDevice;
namespace SDH{ class cSDH; class cDSA; }
struct q_currentReferenceVar;

void schunkEmergencyShutdown(int);
extern bool schunkShutdown;

//===========================================================================
struct SchunkArmModule:public Process{
  q_currentReferenceVar *var;

  //INPUT
  floatA q_reference;
  //OUTPUT
  floatA q_real;

  //INTERAL
  CDevice* pDev;
  bool isOpen;
  float stepHorizon,maxStep;
  bool sendMotion,readPositions;
  uintA motorIndex;
  
  //essential Process routines!
  SchunkArmModule(q_currentReferenceVar *_var=NULL);
  void open();
  void step();
  void close();
  
  void reportParameters(std::ostream& os);
  int waitForEnd(int iMod);
  void setVel(uint motor,float vel);
  void zeroCurAll();
  void zeroVelAll();
  void stopAll();
  void getPos(floatA& q);
  void getOff(floatA& off);
  void getState(floatA& q,floatA& v,floatA& c);
};

//===========================================================================
struct SchunkHandModule:public Process{
  q_currentReferenceVar *var;

  //INPUT
  arr v_reference;
  //OUTPUT
  arr q_real;

  //INTERNAL
  bool isOpen;
  bool sendMotion;
  uintA motorIndex;
  
  //essential Process routines!
  SchunkHandModule();
  void open();
  void step();
  void close();
  
  void setVelocities(const arr& v,double a);
  void setZeroVelocities(double a);
  void getPos(arr &q);

  //obsolete...
  void stop();
  void move(uint i,double x,bool wait,double v=20.);
  void moveAll(const arr& q,bool wait,double v=20.);
  void setVelocity(uint i,double v,double a);
  void movePinch(double x){  moveAll(ARR(90,x,-.2*x,.5*x,-.1*x,x,-.2*x),true); }

  //-- things to hide..
  SDH::cSDH* hand;
  std::vector<int> fingers;
  static SchunkHandModule *global_this;
  static void signalStop(int);
};

//===========================================================================
struct SchunkSkinModule:public Process,Variable{
  //INTPUT (none)
  MT::Array<uint16> emul_data;

  //OUTPUT
  arr y_real; //6D sensor reading

  //INTERNAL
  SDH::cDSA* ts;
  bool isOpen;
  bool isEmulation;
   
  SchunkSkinModule();
  void open();
  void step();
  void close();
  
  void report();
  void setFramerate(uint framerate);
  void getFrame(MT::Array<short unsigned int>& x);
  void getImage(byteA& img);
  void getIntegrals(arr& y);
};

void testPerformance(SchunkArmModule &schunk, int iMod );
void testCube(SchunkArmModule &schunk, int iMod );

#ifdef MT_IMPLEMENTATION
#  include "schunk.cpp"
#endif

#endif
