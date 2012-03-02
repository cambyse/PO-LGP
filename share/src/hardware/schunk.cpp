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

#include <biros/biros.h>
#include "schunk.h"

//===========================================================================
//
// FIRST: the Process wrappers
//
// BELOW: the actual hardware implementations in sSchunk*
//

SchunkArm::SchunkArm():Process("SchunkArm"){
  s = new sSchunkArm();
  birosInfo.getVariable(hardwareReference, "HardwareReference", this);
}

SchunkArm::~SchunkArm(){ delete s; }

void SchunkArm::open(){
  s->openArm = birosInfo.getParameter<bool>("openArm", this, false);
  if(!s->openArm) return;
  
  GeometricState *geo;
  birosInfo.getVariable(geo, "GeometricState", this);
  geo->readAccess(this);
  s->motorIndex.resize(7);
  s->motorIndex(0) = geo->ors.getBodyByName("m3")->inLinks(0)->index;
  s->motorIndex(1) = geo->ors.getBodyByName("m4")->inLinks(0)->index;
  s->motorIndex(2) = geo->ors.getBodyByName("m5")->inLinks(0)->index;
  s->motorIndex(3) = geo->ors.getBodyByName("m6")->inLinks(0)->index;
  s->motorIndex(4) = geo->ors.getBodyByName("m7")->inLinks(0)->index;
  s->motorIndex(5) = geo->ors.getBodyByName("m8")->inLinks(0)->index;
  s->motorIndex(6) = geo->ors.getBodyByName("m9")->inLinks(0)->index;
  geo->deAccess(this);
  

  s->open();
  hardwareReference->writeAccess(this);
  if(hardwareReference->q_real.N<7) hardwareReference->q_real.resizeCopy(7);
  for(uint m=0; m<7; m++)
    hardwareReference->q_real(s->motorIndex(m)) = (float)s->q_real(m);
  hardwareReference->deAccess(this);
}

void SchunkArm::step(){
  if(!s->openArm) return;
  
  hardwareReference->readAccess(this);
  for(uint m=0; m<7; m++)
    s->q_reference(m) = (float)hardwareReference->q_reference(s->motorIndex(m));
  hardwareReference->deAccess(this);

  s->step();
}

void SchunkArm::close(){
  if(!s->openArm) return;
  s->close();
}

//===========================================================================

SchunkHand::SchunkHand():Process("SchunkHand"){
  s = new sSchunkHand();
  birosInfo.getVariable(hardwareReference, "HardwareReference", this);
}

SchunkHand::~SchunkHand(){
  delete s;
}

void SchunkHand::open(){
  s->open();
  
  s->motorIndex.resize(7);
  for(uint m=0; m<=6; m++) s->motorIndex(m) = m+7;

  hardwareReference->writeAccess(this);
  for(uint m=0; m<7; m++) hardwareReference->q_real(s->motorIndex(m)) = s->q_real(m);
  hardwareReference->deAccess(this);
}

void SchunkHand::step(){
  hardwareReference->readAccess(this);
  if(!s->v_reference.N) s->v_reference.resize(7);
  for(uint m=0; m<7; m++) s->v_reference(m) = hardwareReference->v_reference(s->motorIndex(m));
  hardwareReference->deAccess(this);

  s->step();

  hardwareReference->writeAccess(this);
  for(uint m=0; m<7; m++) hardwareReference->q_real(s->motorIndex(m)) = s->q_real(m);
  hardwareReference->deAccess(this);
}

void SchunkHand::close(){
  s->close();
}

//===========================================================================

SchunkSkin::SchunkSkin():Process("SchunkSkin"){
  s = new sSchunkSkin();
  birosInfo.getVariable(skinPressure, "SkinPressure", this);
}

SchunkSkin::~SchunkSkin(){
  delete s;
}

void SchunkSkin::open(){
  s->open();
}

void SchunkSkin::step(){
  s->step();

  skinPressure->writeAccess(this);
  skinPressure->y_real = s->y_real;
  skinPressure->deAccess(this);
}

void SchunkSkin::close(){
  s->close();
}

//===========================================================================
//
// BELOW: the actual hardware implementations in sSchunk*
//

#ifdef MT_SCHUNK //NOTE THIS COMPILER FLAG!

#define NTCAN_CLEAN_NAMESPACE
#include <sdh/sdh.h>
#include <sdh/dsa.h>
#define __EXPORTED_HEADERS__
#include <lwa/Device/Device.h>
#undef __EXPORTED_HEADERS__
#undef V

//--- emergency shutdown (call signal)
struct shutdownFct { void *classP; void (*call)(void*); };
static MT::Array<shutdownFct> shutdownFunctions; //!< list of shutdown functions
static void addShutdown(void *classP, void (*call)(void*)){ shutdownFunctions.memMove=true; shutdownFct f; f.classP=classP; f.call=call; shutdownFunctions.append(f); }
bool schunkShutdown=false;
void schunkEmergencyShutdown(int){
  MT_MSG("initiating smooth shutdown...");
  schunkShutdown=true;
  for(uint i=shutdownFunctions.N; i--;){ shutdownFunctions(i).call(shutdownFunctions(i).classP); }
  MT_MSG("...done");
}

void shutdownLWA(void* p){ MT_MSG("...");  sSchunkArm *lwa=(sSchunkArm*)p;  lwa->close(); }
void shutdownSDH(void* p){ MT_MSG("...");  sSchunkHand *sdh=(sSchunkHand*)p;  sdh->stop();  sdh->close(); }
void shutdownDSA(void* p){ MT_MSG("...");  sSchunkSkin *dsa=(sSchunkSkin*)p;  dsa->close(); }


//===========================================================================
//
// Arm
//

sSchunkArm::sSchunkArm(){
  pDev=NULL;
  isOpen=false;
}

void sSchunkArm::open(){
  //get parameters
  stepHorizon=birosInfo.getParameter<float>("schunkStepHorizon", 50);
  maxStep=birosInfo.getParameter<float>("schunkMaxStep", .03);
  sendMotion=birosInfo.getParameter<bool>("schunkSendArmMotion", false);
  readPositions=birosInfo.getParameter<bool>("schunkReadArmPositions", false);
  
  cout <<" -- sSchunkArm init .." <<std::flush;
  addShutdown(this, shutdownLWA);
  if(access("/dev/can0", R_OK|W_OK)==-1 || access("/dev/can1", R_OK|W_OK)==-1){
    HALT("the devices /dev/can0 or /dev/can1 do not exist -- call `mountHardware' on console first");
  }
  static const char* acInitString = "ESD:0, 1000";
  pDev=newDevice(acInitString);
  if(!pDev) HALT("Device unknown");
  int iRetVal = pDev->init(acInitString);
  if(iRetVal!=0) HALT("Device init returned error " <<iRetVal);
  pDev->resetAll();
  uint m;
  //for(m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|0);
  //for(m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|1);
  //MY DEFAULT 64 4 3
  for(m=3; m<=9; m++) pDev->setC0(m, MT::getParameter<uint>("C0-Pgain", 64)); //P-gain (range 12..64, only even values) (previously: 32)
  for(m=3; m<=9; m++) pDev->setDamp(m, MT::getParameter<uint>("D-Igain" , 4)); //I-gain (range 1..4)
  for(m=3; m<=9; m++) pDev->setA0(m, MT::getParameter<uint>("A0-Dgain", 3)); //D-gain (range 1..12) (previously: 3)
  for(m=3; m<=9; m++) pDev->recalcPIDParams(m);
  for(m=3; m<=9; m++) pDev->setMaxVel(m, .1);
  for(m=3; m<=9; m++) pDev->setMaxAcc(m, .1);
  //for(m=3;m<=9;m++){ schunk.pDev->moveRamp(m, q_desired(m-3), .1, .1); //.3, 3.);
  ofstream zconf("z.schunk_mconfig");
  reportParameters(zconf);
  zconf.close();
  
  getPos(q_real);
  q_reference=q_real;
  
  isOpen=true;
  cout <<" done" <<endl;
}

void sSchunkArm::close(){
  cout <<" -- sSchunkArm close .." <<std::flush;
  //for(uint m=3;m<=9;m++) pDev->setConfig(m, CONFIGID_MOD_SYNC_MOTION|0);
  if(isOpen){
    pDev->haltAll();
    pDev->resetAll();
    pDev->exit();
    delete pDev;
    pDev=NULL;
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void sSchunkArm::reportParameters(ostream& os){
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
  for(uint m=3; m<=9; m++){
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

int sSchunkArm::waitForEnd(int iMod){
  unsigned long uiState;
  do {
    if(pDev->getModuleState(iMod, &uiState) != 0){
      printf("getModuleState returned error\n");
      return -1;
    }
    if(uiState & STATEID_MOD_ERROR){
      printf("An error occured in module%d\n", iMod);
      return 0;
    }
  } while(!(uiState & STATEID_MOD_HOME) || !(uiState & STATEID_MOD_RAMP_END));
  return 0;
}

void sSchunkArm::setVel(uint motor, float vel){
  if(motor==9) pDev->moveVel(motor, vel);
  cout <<"setVel " <<motor <<' ' <<vel <<endl;
}

void sSchunkArm::zeroCurAll(){
  for(uint m=3; m<=9; m++)  pDev->moveCur(m, .0f);
}
void sSchunkArm::zeroVelAll(){
  for(uint m=3; m<=9; m++)  pDev->moveVel(m, .0f);
}
void sSchunkArm::stopAll(){
  //pDev->softStopAll();
}

void sSchunkArm::getPos(floatA& q){
  q.resize(7);
  for(uint m=3; m<=9; m++) pDev->getPos(m, &q(m-3));
}

void sSchunkArm::step(){
  uint m;
  
  double delta= maxDiff(q_real, q_reference);
#if 0
  static ofstream logfil;
  static bool logfilOpen=false;
  if(!logfilOpen){ logfil.open("schunk.log"); logfilOpen=true; }
  logfil <<delta <<endl;
#endif
  if(delta>maxStep){
    MT_MSG(" *** WARNING *** too large step -> making no step,  |dq|=" <<delta);
  } else if(isOpen && sendMotion){
#if 1 //don't read real positions from robot
    for(m=0; m<7; m++){ pDev->moveStep(m+3, q_reference(m), stepHorizon); }
    q_real=q_reference;
#else //read real positions from robot
    unsigned long uiState;
    unsigned char uiDio;
    for(m=0; m<7; m++){
      pDev->moveStepExtended(m+3, q_reference(m), stepHorizon,
                             &uiState, &uiDio, &q_real(m));
    }
#endif
  }else{
    MT::wait(.001*(6)); //+rnd(2))); //randomized dummy duration
  }
  
#if 0
  if(var){
    var->writeAccess(this);
    for(m=0; m<7; m++) var->q_real(motorIndex(m)) = (float)q_real(m);
    var->deAccess(this);
  } else MT_MSG("Variable pointer not set");
#endif
}

void sSchunkArm::getOff(floatA& off){
  off.resize(7);
  for(uint m=3; m<=9; m++) pDev->getHomeOffset(m, &off(m-3));
}

void sSchunkArm::getState(floatA& q, floatA& v, floatA& c){
  q.resize(7);
  v.resize(7);
  c.resize(7);
  for(uint m=3; m<=9; m++) pDev->getPos(m, &q(m-3));
  for(uint m=3; m<=9; m++) pDev->getVel(m, &c(m-3));
  for(uint m=3; m<=9; m++) pDev->getCur(m, &v(m-3));
}


//===========================================================================
//
// Hand

sSchunkHand *sSchunkHand::global_this=NULL;

sSchunkHand::sSchunkHand(){
  hand=NULL;
  isOpen=false;
}

void sSchunkHand::open(){
  //read parameters
  sendMotion=birosInfo.getParameter<bool>("schunkSendHandMotion", true);
  
  cout <<" -- sSchunkHand init .." <<std::flush;
  addShutdown(this, shutdownSDH);
  hand=new SDH::cSDH(false, false, -1);
  
  int  can_net = 1;
  int  can_baudrate = 1000000;
  int  can_timeout = -1;
  int  can_id_read = 43;
  int  can_id_write = 42;
  //int  rs232_port = 9;
  //int  rs232_baudrate = 115200;
  hand->OpenCAN_ESD(can_net,
                    can_baudrate,
                    can_timeout,
                    can_id_read,
                    can_id_write);
                    
  fingers.push_back(0);
  fingers.push_back(1);
  fingers.push_back(2);
  fingers.push_back(3);
  fingers.push_back(4);
  fingers.push_back(5);
  fingers.push_back(6);
  
  hand->SetController(hand->eCT_VELOCITY_ACCELERATION);
  //hand->SetVelocityProfile(SDH::cSDHBase::eVP_SIN_SQUARE);
  //hand->SetVelocityProfile(SDH::cSDHBase::eVP_RAMP);
  
  hand->SetAxisEnable(hand->All, true);
  
  getPos(q_real);
  
  isOpen=true;
  cout <<" done" <<endl;
}

void sSchunkHand::close(){
  cout <<" -- sSchunkHand close .." <<std::flush;
  if(isOpen){
    hand->Close();
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void sSchunkHand::stop(){ hand->Stop(); }

void sSchunkHand::move(uint i, double x, bool wait, double v){
  hand->SetAxisTargetVelocity(i, v);
  hand->SetAxisTargetAngle(i, x);
  hand->MoveAxis(i, wait);
}

void sSchunkHand::moveAll(const arr& q, bool wait, double v){
  if(q.N!=7){ stop(); HALT("SDH q wrong dim"); }
  for(uint i=0; i<7; i++){
    hand->SetAxisTargetVelocity(i, v);
    hand->SetAxisTargetAngle(i, q(i));
  }
  hand->MoveAxis(fingers, wait);
}

void sSchunkHand::setVelocity(uint i, double v, double a){
  //sdh.hand->SetAxisEnable( axis_index, true );
  CHECK(a>0., "always assume positive acceleration");
  //if(v < hand->GetAxisReferenceVelocity(i)) a*=-1.;
  hand->SetAxisTargetAcceleration(i, a);
  hand->SetAxisTargetVelocity(i, v);
}

void sSchunkHand::setVelocities(const arr& v, double a){
#if 1
  CHECK(a>0., "always assume positive acceleration");
  uint i;
  std::vector<double> v_reference = hand->GetAxisReferenceVelocity(fingers);
  std::vector<double> vel(7);  for(i=0; i<7; i++) vel[i]=v(i)/MT_PI*180.;
  std::vector<double> acc(7);  for(i=0; i<7; i++) acc[i]=a;
  try {
    hand->SetAxisTargetAcceleration(fingers, acc);
    hand->SetAxisTargetVelocity(fingers, vel);
  } catch (SDH::cSDHLibraryException* e){
    cerr <<"\ndemo-dsa main(): Caught exception from SDHLibrary: " <<e->what() <<". Giving up!\n";
    delete e;
  } catch (...){
    cerr <<"\ncaught unknown exception, giving up\n";
  }
#else
  for(uint i=0; i<7; i++) setVelocity(i, v(i)/MT_PI*180., a);
#endif
}

void sSchunkHand::setZeroVelocities(double a){
  arr v(7);
  v.setZero();
  setVelocities(v, a);
}

void sSchunkHand::getPos(arr &q){
  std::vector<double> handq = hand->GetAxisActualAngle(fingers);
  q.resize(7);
  for(uint i=0; i<7; i++) q(i) = handq[i]/180.*MT_PI;
}

void sSchunkHand::step(){
  
  if(isOpen && sendMotion){
    setVelocities(v_reference, 360.);
    getPos(q_real);
  }else{
    MT::wait(.001*(40)); //+rnd(2))); //randomized dummy duration
  }
  
}


//===========================================================================
//
// Skin
//

sSchunkSkin::sSchunkSkin(){
  ts=NULL;
  isOpen=false;
  isEmulation=false;
}

void sSchunkSkin::open(){

  if(isEmulation){ // without hardware
    emul_data.resize(27*18);
    emul_data.setZero();
    return;
  }
  
  // below: with hardware
  
  cout <<" -- sSchunkSkin init .." <<std::flush;
  addShutdown(this, shutdownDSA);
  //SDH_ASSERT_TYPESIZES();
  
  int debug_level = -1;
  
  //int  iFinger = 2;
  
  //bool can_use = true;
  //int  can_net = 1;
  //int  can_baudrate = 1000000;
  //int  can_timeout = -1;
  //int  can_id_read = 43;
  //int  can_id_write = 42;
  //int  rs232_port = 9;
  //int  rs232_baudrate = 115200;
  int  rs232_dsaport = 9;
  
  //int framerate = 10;
  //bool do_RLE = true;
  //bool fullframe = true;
  
  try {
    ts = new SDH::cDSA(debug_level, rs232_dsaport);
  } catch (SDH::cSDHLibraryException* e){
    cerr <<"\ndemo-dsa main(): Caught exception from SDHLibrary: " <<e->what() <<". Giving up!\n";
    delete e;
  } catch (...){
    cerr <<"\ncaught unknown exception, giving up\n";
  }
  ts->SetFramerate(30, true);
  cout <<" done" <<endl;
  isOpen=true;
}

void sSchunkSkin::report(){
  if(isEmulation){
    cout <<"Skin feedback is emulated!" <<endl;
    return;
  }
  
  cout <<ts->GetControllerInfo() <<endl;
  
  const SDH::cDSA::sSensorInfo &sinfo=ts->GetSensorInfo();
  cout <<sinfo <<endl;
  uint M=sinfo.nb_matrices;
  for(uint m=0; m<M; m++){
    const SDH::cDSA::sMatrixInfo &minfo=ts->GetMatrixInfo(m);
    cout <<"Matrix " <<m <<"   " <<minfo <<endl;
  }
}

void sSchunkSkin::setFramerate(uint framerate){
  if(isEmulation) return;
  ts->SetFramerate(framerate, true);
}

void sSchunkSkin::close(){
  cout <<" -- sSchunkSkin close .." <<std::flush;
  if(isOpen){
    ts->Close();
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void sSchunkSkin::getFrame(MT::Array<uint16>& x){
  uint16* f;
  
  if(isEmulation){
    f = emul_data.p;
  }else{
    try {
      f = ts->UpdateFrame().texel;
    } catch (SDH::cSDHLibraryException* e){
      //HALT("schunk exception: " <<e->what());
      MT_MSG("* * * schunk exception: " <<e->what());
      delete e;
      return;
    }
  }
  
  uint M=6, X=6, Y=14;
  x.resize(M, Y, X);
  x.setZero();
  memmove(&x(1, 0, 0), f+ 0*6, 6*14*sizeof(uint16));
  memmove(&x(0, 1, 0), f+14*6, 6*13*sizeof(uint16));
  memmove(&x(3, 0, 0), f+27*6, 6*14*sizeof(uint16));
  memmove(&x(2, 1, 0), f+41*6, 6*13*sizeof(uint16));
  memmove(&x(5, 0, 0), f+54*6, 6*14*sizeof(uint16));
  memmove(&x(4, 1, 0), f+68*6, 6*13*sizeof(uint16));
}

void sSchunkSkin::getImage(byteA& img){
  MT::Array<uint16> x, display;
  getFrame(x);
  display.resize(29, 20);
  display.setZero();
  display.setMatrixBlock(x[0], 0, 0);
  display.setMatrixBlock(x[1], 15, 0);
  display.setMatrixBlock(x[2], 0, 7);
  display.setMatrixBlock(x[3], 15, 7);
  display.setMatrixBlock(x[4], 0, 14);
  display.setMatrixBlock(x[5], 15, 14);
  
  img.resize(29, 20);
  for(uint i=0; i<img.N; i++) img.elem(i) = (byte)pow(display.elem(i), 0.67);
}

void sSchunkSkin::getIntegrals(arr& y){
  MT::Array<uint16> x;
  getFrame(x);
  x.reshape(6, x.N/6);
  y.resize(6);
  y.setZero();
  uint i, j;
  for(i=0; i<6; i++) for(j=0; j<x.d1; j++) y(i) += (double)x(i, j);
  y/=250000.;
}

void sSchunkSkin::step(){
  getIntegrals(y_real);
}


//===========================================================================



void testPerformance(sSchunkArm &schunk, int iMod){
  //unsigned long uiState;
  //unsigned char uiDio;
  float fPos;
  //float fPos2;
  int iRetVal = 0;
  
  cout <<"\n Performing 1000 getPos & moveStep..." <<endl;
  double time=MT::realTime();
  uint i;
  iRetVal = schunk.pDev->getPos(iMod, &fPos);
  if(iRetVal != 0) printf("Error fetching pos: %d\n", iRetVal);
  for(i = 0; i < 1000; i++){
    //iRetVal = schunk.pDev->getModuleState( iMod, &uiState );
    //if( iRetVal != 0 ) printf( "Error fetching state: %d\n", iRetVal );
    //iRetVal = schunk.pDev->getPos( iMod, &fPos );
    //if( iRetVal != 0 ) printf( "Error fetching pos: %d\n", iRetVal );
    iRetVal = schunk.pDev->moveStep(iMod, fPos, 100.);
    if(iRetVal != 0) printf("Error move step: %d\n", iRetVal);
//       iRetVal = schunk.pDev->moveStepExtended(iMod, fPos, 100., &uiState, &uiDio, &fPos2 );
//       if( iRetVal != 0 ) printf( "Error move step: %d\n", iRetVal );
  }
  cout <<"-> state+pos time = " <<(MT::realTime()-time)/i <<"sec (typical=0.0011..sec)" <<endl;
}

void testCube(sSchunkArm &schunk, int iMod){
  int iRetVal;
  unsigned long uiState;
  float defMaxPos, defMinPos, defMaxVel, defMaxAcc;
  
  iRetVal = schunk.pDev->getModuleState(iMod, &uiState);
  if(iRetVal == 0){
    printf("Homing module%d ", iMod);
    if(schunk.pDev->homeModule(iMod) != 0)
      printf("module home returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nSending a Reset to module%d\n", iMod);
    if(schunk.pDev->resetModule(iMod) != 0)
      printf("resetModule returned error\n");
      
    if(schunk.pDev->getDefMinPos(iMod, &defMinPos) != 0)
      printf("getDefMinPos returned error\n");
    else
      printf("DefMinPos: %f\n", defMinPos);
      
    if(schunk.pDev->getDefMaxPos(iMod, &defMaxPos) != 0)
      printf("getDefMaxPos returned error\n");
    else
      printf("DefMaxPos: %f\n", defMaxPos);
      
    if(schunk.pDev->getDefMaxVel(iMod, &defMaxVel) != 0)
      printf("getDefMaxVel returned error\n");
    else
      printf("DefMaxVel: %f\n", defMaxVel);
      
    if(schunk.pDev->getDefMaxAcc(iMod, &defMaxAcc) != 0)
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
    
    if(schunk.pDev->moveRamp(iMod, 0.2/*defMaxPos*/, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nMoving module to minPos");
    if(schunk.pDev->moveRamp(iMod, -0.2/*defMinPos*/, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
      
    printf("\nMoving module back to zero");
    if(schunk.pDev->moveRamp(iMod, 0.0, defMaxVel/2.0, defMaxAcc/2.0) != 0)
      printf("MoveRamp returned error\n");
    else
      schunk.waitForEnd(iMod);
    //*/
    testPerformance(schunk, iMod);
  } else
    printf("getModuleState returned error %d\n", iRetVal);
    
}


#else //ndef MT_SCHUNK

#include "schunk_internal.h"

bool schunkShutdown=false;
sSchunkArm::sSchunkArm(q_currentReferenceVar *_var):Process("SchunkArmProcess"){}
void sSchunkArm::open(){}
void sSchunkArm::close(){}
void sSchunkArm::step(){}
sSchunkHand::sSchunkHand():Process("SchunkHandProcess"){}
void sSchunkHand::open(){}
void sSchunkHand::close(){}
void sSchunkHand::step(){}
sSchunkSkin::sSchunkSkin():Process("SchunkSkinProcess"){}
void sSchunkSkin::open(){}
void sSchunkSkin::close(){}
void sSchunkSkin::step(){}
void sSchunkSkin::getImage(byteA&){}
void schunkEmergencyShutdown(int){}

#endif
