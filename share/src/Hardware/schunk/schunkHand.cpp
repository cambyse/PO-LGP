#include "schunk.h"

#ifdef MT_SCHUNK //NOTE THIS COMPILER FLAG!

#define NTCAN_CLEAN_NAMESPACE
#include <sdh/sdh.h>

SchunkHand *SchunkHand::global_this=NULL;

SchunkHand::SchunkHand():Module("SchunkHand"), isOpen(false), hand(NULL) {
}

void SchunkHand::open() {
  //read parameters
  sendMotion=MT::getParameter<bool>("schunkSendHandMotion", true);
  
  cout <<" -- SchunkHand init .." <<std::flush;
  addShutdown(this, shutdownSDH);
  hand=new SDH::cSDH(false, false, -1);
  
  int  can_net = 1;
  int  can_baudrate = 1000000;
  int  can_timeout = -1;
  int  can_id_read = 43;
  int  can_id_write = 42;
  //int  rs232_port = 9;
  //int  rs232_baudrate = 115200;
  try {
    hand->OpenCAN_ESD(can_net,
		      can_baudrate,
		      can_timeout,
		      can_id_read,
		      can_id_write);
  } catch (SDH::cSDHLibraryException* e) {
    cerr <<"\ndemo-dsa main(): Caught exception from SDHLibrary: " <<e->what() <<". Giving up!\n";
    delete e;
  } catch (...) {
    cerr <<"\ncaught unknown exception, giving up\n";
  }
                    
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
  
  getPos(q_real.set());
  getVel(v_real.set());
  
  isOpen=true;
  cout <<" done" <<endl;
}

void SchunkHand::close() {
  cout <<" -- SchunkHand close .." <<std::flush;
  if (isOpen) {
    hand->Close();
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void SchunkHand::stop() { hand->Stop(); }

void SchunkHand::move(uint i, double x, bool wait, double v) {
  hand->SetAxisTargetVelocity(i, v);
  hand->SetAxisTargetAngle(i, x);
  hand->MoveAxis(i, wait);
}

void SchunkHand::moveAll(const arr& q, bool wait, double v) {
  if (q.N!=7) { stop(); HALT("SDH q wrong dim"); }
  for (uint i=0; i<7; i++) {
    hand->SetAxisTargetVelocity(i, v);
    hand->SetAxisTargetAngle(i, q(i));
  }
  hand->MoveAxis(fingers, wait);
}

void SchunkHand::setVelocity(uint i, double v, double a) {
  //sdh.hand->SetAxisEnable( axis_index, true );
  CHECK(a>0., "always assume positive acceleration");
  //if(v < hand->GetAxisReferenceVelocity(i)) a*=-1.;
  hand->SetAxisTargetAcceleration(i, a);
  hand->SetAxisTargetVelocity(i, v);
}

void SchunkHand::setVelocities(const arr& v, double a) {
#if 1
  CHECK(a>0., "always assume positive acceleration");
  uint i;
  std::vector<double> v_reference = hand->GetAxisReferenceVelocity(fingers);
  std::vector<double> vel(7);  for (i=0; i<7; i++) vel[i]=v(i)/MT_PI*180.;
  std::vector<double> acc(7);  for (i=0; i<7; i++) acc[i]=a;
  try {
    hand->SetAxisTargetAcceleration(fingers, acc);
    hand->SetAxisTargetVelocity(fingers, vel);
  } catch (SDH::cSDHLibraryException* e) {
    cerr <<"\ndemo-dsa main(): Caught exception from SDHLibrary: " <<e->what() <<". Giving up!\n";
    delete e;
  } catch (...) {
    cerr <<"\ncaught unknown exception, giving up\n";
  }
#else
  for (uint i=0; i<7; i++) setVelocity(i, v(i)/MT_PI*180., a);
#endif
}

void SchunkHand::setZeroVelocities(double a) {
  arr v(7);
  v.setZero();
  setVelocities(v, a);
}

void SchunkHand::getPos(arr &q) {
  std::vector<double> handq = hand->GetAxisActualAngle(fingers);
  q.resize(7);
  for (uint i=0; i<7; i++) q(i) = handq[i]/180.*MT_PI;
}

void SchunkHand::getVel(arr &v) {
  std::vector<double> handv = hand->GetAxisActualVelocity(fingers);
  v.resize(7);
  for (uint i=0; i<7; i++) v(i) = handv[i]*MT_PI/180.;
}

void SchunkHand::step() {
  if (isOpen && sendMotion) {
    setVelocities(v_reference.get(), 360.);
    getPos(q_real.set());
    getVel(v_real.set());
  } else {
    MT::wait(.001*(40)); //+rnd(2))); //randomized dummy duration
  }
  
}

#else //ndef MT_SCHUNK

SchunkHand::SchunkHand() {}
void SchunkHand::open() {}
void SchunkHand::close() {}
void SchunkHand::step() {}

#endif
