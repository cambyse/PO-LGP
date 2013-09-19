#include "schunk.h"

#ifdef MT_SCHUNK //NOTE THIS COMPILER FLAG!

#define NTCAN_CLEAN_NAMESPACE
#include <sdh/dsa.h>

SchunkSkin::SchunkSkin():Module("SchunkSkin"), ts(NULL), isOpen(false), isEmulation(false) {
}

void SchunkSkin::open() {

  if (isEmulation) { // without hardware
    emul_data.resize(27*18);
    emul_data.setZero();
    return;
  }
  
  // below: with hardware
  
  cout <<" -- SchunkSkin init .." <<std::flush;
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
  } catch (SDH::cSDHLibraryException* e) {
    cerr <<"\ndemo-dsa main(): Caught exception from SDHLibrary: " <<e->what() <<". Giving up!\n";
    delete e;
  } catch (...) {
    cerr <<"\ncaught unknown exception, giving up\n";
  }
  ts->SetFramerate(30, true);
  cout <<" done" <<endl;
  isOpen=true;
}

void SchunkSkin::report() {
  if (isEmulation) {
    cout <<"Skin feedback is emulated!" <<endl;
    return;
  }
  
  cout <<ts->GetControllerInfo() <<endl;
  
  const SDH::cDSA::sSensorInfo &sinfo=ts->GetSensorInfo();
  cout <<sinfo <<endl;
  uint M=sinfo.nb_matrices;
  for (uint m=0; m<M; m++) {
    const SDH::cDSA::sMatrixInfo &minfo=ts->GetMatrixInfo(m);
    cout <<"Matrix " <<m <<"   " <<minfo <<endl;
  }
}

void SchunkSkin::setFramerate(uint framerate) {
  if (isEmulation) return;
  ts->SetFramerate(framerate, true);
}

void SchunkSkin::close() {
  cout <<" -- SchunkSkin close .." <<std::flush;
  if (isOpen) {
    ts->Close();
    isOpen=false;
  }
  cout <<" done" <<endl;
}

void SchunkSkin::getFrame(MT::Array<uint16>& x) {
  uint16* f;
  
  if (isEmulation) {
    f = emul_data.p;
  } else {
    try {
      f = ts->UpdateFrame().texel;
    } catch (SDH::cSDHLibraryException* e) {
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

void SchunkSkin::getImage(byteA& img) {
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
  for (uint i=0; i<img.N; i++) img.elem(i) = (byte)pow(display.elem(i), 0.67);
}

void SchunkSkin::getIntegrals(arr& y) {
  MT::Array<uint16> x;
  getFrame(x);
  x.reshape(6, x.N/6);
  y.resize(6);
  y.setZero();
  uint i, j;
  for (i=0; i<6; i++) for (j=0; j<x.d1; j++) y(i) += (double)x(i, j);
  y/=250000.;
}

void SchunkSkin::step() {
  getIntegrals(y_real.set());
}

#else //ndef MT_SCHUNK

SchunkSkin::SchunkSkin() {}
void SchunkSkin::open() {}
void SchunkSkin::close() {}
void SchunkSkin::step() {}
void SchunkSkin::getImage(byteA&) {}
void SchunkSkin::getIntegrals(arr&) {}
void SchunkSkin::report() { NICO }

#endif
