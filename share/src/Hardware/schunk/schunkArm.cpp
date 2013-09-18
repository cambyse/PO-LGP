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

#include "schunk.h"

#ifdef MT_SCHUNK //NOTE THIS COMPILER FLAG!

#define NTCAN_CLEAN_NAMESPACE
#define __EXPORTED_HEADERS__
#include <lwa/Device/Device.h>
#undef __EXPORTED_HEADERS__
#undef V

//--- emergency shutdown (call signal)

SchunkArm::SchunkArm():Module("SchunkArm"), pDev(NULL), isOpen(false) {
}

void SchunkArm::open() {
  //get parameters
  stepHorizon=MT::getParameter<float>("schunkStepHorizon", 50);
  maxStep=MT::getParameter<float>("schunkMaxStep", .03);
  sendMotion=MT::getParameter<bool>("schunkSendArmMotion", true);
  readPositions=MT::getParameter<bool>("schunkReadArmPositions", false);
  
  cout <<" -- SchunkArm init .." <<std::flush;
  addShutdown(this, shutdownLWA);
  if (access("/dev/can0", R_OK|W_OK)==-1 || access("/dev/can1", R_OK|W_OK)==-1) {
    HALT("the devices /dev/can0 or /dev/can1 do not exist -- call `mountHardware' on console first");
  }
  static const char* acInitString = "ESD:0, 1000";
  pDev=newDevice(acInitString);
  if (!pDev) HALT("Device unknown");
  int iRetVal = pDev->init(acInitString);
  if (iRetVal!=0) HALT("Device init returned error " <<iRetVal);
  pDev->resetAll();
  uint m;
  //for(m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|0);
  //for(m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|1);
  //MY DEFAULT 64 4 3
  for (m=3; m<=9; m++) pDev->setC0(m, MT::getParameter<uint>("C0-Pgain", 64)); //P-gain (range 12..64, only even values) (previously: 32)
  for (m=3; m<=9; m++) pDev->setDamp(m, MT::getParameter<uint>("D-Igain" , 4)); //I-gain (range 1..4)
  for (m=3; m<=9; m++) pDev->setA0(m, MT::getParameter<uint>("A0-Dgain", 3)); //D-gain (range 1..12) (previously: 3)
  for (m=3; m<=9; m++) pDev->recalcPIDParams(m);
  for (m=3; m<=9; m++) pDev->setMaxVel(m, .1);
  for (m=3; m<=9; m++) pDev->setMaxAcc(m, .1);
  //for(m=3;m<=9;m++){ schunk.pDev->moveRamp(m, q_desired(m-3), .1, .1); //.3, 3.);
  ofstream zconf("z.schunk_mconfig");
  reportParameters(zconf);
  zconf.close();
  
  
  getPos(q_rea);
  q_ref=q_rea;

  q_real.set()=q_rea;
  q_reference.set()=q_ref;
  
  isOpen=true;
  cout <<" done" <<endl;
}

void SchunkArm::close() {
  cout <<" -- SchunkArm close .." <<std::flush;
  //for(uint m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|0);
  if (isOpen) {
    pDev->haltAll();
    pDev->resetAll();
    pDev->exit();
    delete pDev;
    pDev=NULL;
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void SchunkArm::reportParameters(ostream& os) {
  float f;
  long l;
  unsigned long u;
  short s;
  os
  <<"*** Schunk LWA parameters:"
  <<"\n -- device:"
  <<"\n    name=" <<pDev->getName()
  <<"\n    init string=" <<pDev->getInitString()
  // <<"\n    baud rate=" <<pDev->getBaudRate()
  <<"\n    module count=" <<pDev->getModuleCount();
  for (uint m=3; m<=9; m++) {
    os<<"\n -- motor " <<m
    <<"\n    getHomeOffset=" <<(pDev->getHomeOffset(m, &f)?-1:f)
    <<"\n    getDefHomeOffset=" <<(pDev->getDefHomeOffset(m, &f)?-1:f)
    <<"\n    getHomeOffsetInc=" <<(pDev->getHomeOffsetInc(m, &l)?-1:l)
    <<"\n    getIncRatio=" <<(pDev->getIncRatio(m, &f)?-1:f)
    <<"\n    getDioData=" <<(pDev->getDioData(m, &u)?-1:u)
    <<"\n    getC    (P-gain)=" <<(pDev->getC0(m, &s)?-1:s)
    <<"\n    getDamp (I-gain)=" <<(pDev->getDamp(m, &s)?-1:s)
    <<"\n    getA    (D-gain)=" <<(pDev->getA0(m, &s)?-1:s)
    <<"\n    getPos=" <<(pDev->getPos(m, &f)?-1:f)
    <<"\n    getVel=" <<(pDev->getVel(m, &f)?-1:f)
    <<"\n    getCur=" <<(pDev->getCur(m, &f)?-1:f)
    <<"\n    getMinPos=" <<(pDev->getMinPos(m, &f)?-1:f)
    <<"\n    getMaxPos=" <<(pDev->getMaxPos(m, &f)?-1:f)
    <<"\n    getMaxVel=" <<(pDev->getMaxVel(m, &f)?-1:f)
    <<"\n    getMaxAcc=" <<(pDev->getMaxAcc(m, &f)?-1:f)
    <<"\n    getMaxCur=" <<(pDev->getMaxCur(m, &f)?-1:f)
    <<"\n    getDeltaPos=" <<(pDev->getDeltaPos(m, &f)?-1:f)
    <<"\n    getMaxDeltaPos=" <<(pDev->getMaxDeltaPos(m, &f)?-1:f)
    <<"\n    getSavePos=" <<(pDev->getSavePos(m, &f)?-1:f)
    <<"\n    getIPolVel=" <<(pDev->getIPolVel(m, &f)?-1:f)
    <<"\n    getPosCountInc=" <<(pDev->getPosCountInc(m, &l)?-1:l)
    <<"\n    getPosInc=" <<(pDev->getPosInc(m, &l)?-1:l)
    <<"\n    getVelInc=" <<(pDev->getVelInc(m, &l)?-1:l)
    <<"\n    getCurInc=" <<(pDev->getCurInc(m, &s)?-1:s)
    <<"\n    getMinPosInc=" <<(pDev->getMinPosInc(m, &l)?-1:l)
    <<"\n    getMaxPosInc=" <<(pDev->getMaxPosInc(m, &l)?-1:l)
    <<"\n    getMaxVelInc=" <<(pDev->getMaxVelInc(m, &l)?-1:l)
    <<"\n    getMaxAccInc=" <<(pDev->getMaxAccInc(m, &l)?-1:l)
    <<"\n    getDeltaPosInc=" <<(pDev->getDeltaPosInc(m, &l)?-1:l)
    <<"\n    getMaxDeltaPosInc=" <<(pDev->getMaxDeltaPosInc(m, &l)?-1:l)
    <<"\n    getSavedPos=" <<(pDev->getSavedPos(m, &f)?-1:f)
    <<"\n    getHomeVel=" <<(pDev->getHomeVel(m, &f)?-1:f)
    <<"\n    getHomeVelInc=" <<(pDev->getHomeVelInc(m, &l)?-1:l)
    <<std::flush;
  }
}

int SchunkArm::waitForEnd(int iMod) {
  unsigned long uiState;
  do {
    if (pDev->getModuleState(iMod, &uiState) != 0) {
      printf("getModuleState returned error\n");
      return -1;
    }
    if (uiState & STATEID_MOD_ERROR) {
      printf("An error occured in module%d\n", iMod);
      return 0;
    }
  } while (!(uiState & STATEID_MOD_HOME) || !(uiState & STATEID_MOD_RAMP_END));
  return 0;
}

void SchunkArm::setVel(uint motor, float vel) {
  if (motor==9) pDev->moveVel(motor, vel);
  cout <<"setVel " <<motor <<' ' <<vel <<endl;
}

void SchunkArm::zeroCurAll() {
  for (uint m=3; m<=9; m++)  pDev->moveCur(m, .0f);
}
void SchunkArm::zeroVelAll() {
  for (uint m=3; m<=9; m++)  pDev->moveVel(m, .0f);
}
void SchunkArm::stopAll() {
  //pDev->softStopAll();
}

void SchunkArm::getPos(floatA& q) {
  q.resize(7);
  for (uint m=3; m<=9; m++) pDev->getPos(m, &q(m-3));
}

void SchunkArm::step() {
  uint m;
  
  q_rea = q_real.get();
  q_ref = q_reference.get();

  double delta= maxDiff(q_rea, q_ref);
#if 0
  static ofstream logfil;
  static bool logfilOpen=false;
  if (!logfilOpen) { logfil.open("schunk.log"); logfilOpen=true; }
  logfil <<delta <<endl;
#endif
  if (delta>maxStep) {
    MT_MSG(" *** WARNING *** too large step -> making no step,  |dq|=" <<delta);
  } else if (isOpen && sendMotion) {
#if 1 //don't read real positions from robot
    for (m=0; m<7; m++) { pDev->moveStep(m+3, q_ref(m), stepHorizon); }
    q_rea=q_ref;
#else //read real positions from robot
    unsigned long uiState;
    unsigned char uiDio;
    for (m=0; m<7; m++) {
      pDev->moveStepExtended(m+3, q_ref(m), stepHorizon,
                             &uiState, &uiDio, &q_rea(m));
    }
#endif
  } else {
    MT::wait(.001*(6)); //+rnd(2))); //randomized dummy duration
  }
  
  q_real.set() = q_rea;
}

void SchunkArm::getOff(floatA& off) {
  off.resize(7);
  for (uint m=3; m<=9; m++) pDev->getHomeOffset(m, &off(m-3));
}

void SchunkArm::getState(floatA& q, floatA& v, floatA& c) {
  q.resize(7);
  v.resize(7);
  c.resize(7);
  for (uint m=3; m<=9; m++) pDev->getPos(m, &q(m-3));
  for (uint m=3; m<=9; m++) pDev->getVel(m, &c(m-3));
  for (uint m=3; m<=9; m++) pDev->getCur(m, &v(m-3));
}






//===========================================================================



void testPerformance(SchunkArm &schunk, int iMod) {
  //unsigned long uiState;
  //unsigned char uiDio;
  float fPos;
  //float fPos2;
  int iRetVal = 0;
  
  cout <<"\n Performing 1000 getPos & moveStep..." <<endl;
  double time=MT::realTime();
  uint i;
  iRetVal = schunk.pDev->getPos(iMod, &fPos);
  if (iRetVal != 0) printf("Error fetching pos: %d\n", iRetVal);
  for (i = 0; i < 1000; i++) {
    //iRetVal = schunk.pDev->getModuleState( iMod, &uiState );
    //if( iRetVal != 0 ) printf( "Error fetching state: %d\n", iRetVal );
    //iRetVal = schunk.pDev->getPos( iMod, &fPos );
    //if( iRetVal != 0 ) printf( "Error fetching pos: %d\n", iRetVal );
    iRetVal = schunk.pDev->moveStep(iMod, fPos, 100.);
    if (iRetVal != 0) printf("Error move step: %d\n", iRetVal);
//       iRetVal = schunk.pDev->moveStepExtended(iMod, fPos, 100., &uiState, &uiDio, &fPos2 );
//       if( iRetVal != 0 ) printf( "Error move step: %d\n", iRetVal );
  }
  cout <<"-> state+pos time = " <<(MT::realTime()-time)/i <<"sec (typical=0.0011..sec)" <<endl;
}

void testCube(SchunkArm &schunk, int iMod) {
  int iRetVal;
  unsigned long uiState;
  float defMaxPos, defMinPos, defMaxVel, defMaxAcc;
  
  iRetVal = schunk.pDev->getModuleState(iMod, &uiState);
  if (iRetVal == 0) {
    printf("Homing module%d ", iMod);
    if (schunk.pDev->homeModule(iMod) != 0)
      printf("module home returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nSending a Reset to module%d\n", iMod);
    if (schunk.pDev->resetModule(iMod) != 0)
      printf("resetModule returned error\n");
      
    if (schunk.pDev->getDefMinPos(iMod, &defMinPos) != 0)
      printf("getDefMinPos returned error\n");
    else
      printf("DefMinPos: %f\n", defMinPos);
      
    if (schunk.pDev->getDefMaxPos(iMod, &defMaxPos) != 0)
      printf("getDefMaxPos returned error\n");
    else
      printf("DefMaxPos: %f\n", defMaxPos);
      
    if (schunk.pDev->getDefMaxVel(iMod, &defMaxVel) != 0)
      printf("getDefMaxVel returned error\n");
    else
      printf("DefMaxVel: %f\n", defMaxVel);
      
    if (schunk.pDev->getDefMaxAcc(iMod, &defMaxAcc) != 0)
      printf("getDefMaxAcc returned error\n");
    else
      printf("DefMaxAcc: %f\n", defMaxAcc);
    /*
          while( !checkKeyboard() )
          {
          schunk.pDev->moveCur( iMod, 2.0 ) ;
          schunk.pDev->moveCur( iMod, -2.0 ) ;
        }
    
          if( schunk.pDev->haltModule( iMod ) != 0 )
          printf( "haltModule returned error\n" );
    */
    
//*
    printf("Moving module to maxPos");
    
    if (schunk.pDev->moveRamp(iMod, 0.2/*defMaxPos*/, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nMoving module to minPos");
    if (schunk.pDev->moveRamp(iMod, -0.2/*defMinPos*/, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nMoving module back to zero");
    if (schunk.pDev->moveRamp(iMod, 0.0, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
    //*/
    testPerformance(schunk, iMod);
  } else
    printf("getModuleState returned error %d\n", iRetVal);
    
}


#else //ndef MT_SCHUNK

void SchunkArm::open() {}
void SchunkArm::close() {}
void SchunkArm::step() {}

#endif
