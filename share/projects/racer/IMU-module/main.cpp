#include <Core/util.h>
#include <System/engine.h>

#include "modules.h"

void testIMU(){
  struct MySystem:System{
    ACCESS(arr, imuData)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", ModuleThread::loopFull);
      addModule<KalmanFilter>("KalmanFilter", ModuleThread::listenFirst);
      addModule<RacerDisplay>("RacerDisplay", ModuleThread::loopWithBeat, 0.1);
      connect();
    }
  } S;

  cout <<S <<endl;

  //    engine().enableAccessLog();
  engine().open(S);


  for(;;){
    S.imuData.var->waitForNextWriteAccess();
    if(engine().shutdown.getValue()) break;
//    if(S.imuData.get()()(0)>10.) break;
  }

  //    engine().shutdown.waitForSignal();

  engine().close(S);

  cout <<"bye bye" <<endl;
}

void testMotors(){
  struct MySystem:System{
    ACCESS(arr, controls)
    MySystem(){
      addModule<Motors>("Motors", ModuleThread::loopFull);
      connect();
    }
  } S;

  cout <<S <<endl;

  engine().open(S);

  S.controls.set()() = ARR(5.,5.,10.);
  MT::wait(3);
  S.controls.set()() = ARR(128.,128.,10.);
  MT::wait(3);
  S.controls.set()() = ARR(0.,0.,10.);
  MT::wait(3);
  S.controls.set()() = ARR(128.,128.,1.);
  MT::wait(3);
  S.controls.set()() = ARR(0.,0.,1.);
  MT::wait(3);

  engine().close(S);

  cout <<"bye bye" <<endl;
}

void testBalance(){
  struct MySystem:System{
    ACCESS(arr, imuData)
    ACCESS(arr, stateEstimate);
    ACCESS(arr, encoderData)
    ACCESS(arr, controls)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", ModuleThread::loopFull);
      addModule<KalmanFilter>("KalmanFilter", ModuleThread::listenFirst);
      addModule<RacerDisplay>("RacerDisplay", ModuleThread::loopWithBeat, 0.1);
      addModule<Motors>("Motors", ModuleThread::loopFull);
      connect();
    }
  } S;

  cout <<S <<endl;

  double zeroTh=MT::getParameter<double>("zeroTh", 0.);
  double k_th=MT::getParameter<double>("k_th", 0.);
  double k_thDot=MT::getParameter<double>("k_thDot", 0.);

  //    engine().enableAccessLog();
  engine().open(S);

  for(int i = 0;; ++i){
    S.stateEstimate.var->waitForNextWriteAccess();
    arr x = S.stateEstimate.get();
    arr enc = S.encoderData.get();

    double u = k_th*(zeroTh - x(1)) + k_thDot * (0. - x(3));
//    u=5.;
    cout <<"\r state = " <<x <<std::flush;
//    cout <<"enc= " <<enc/MT_2PI <<std::endl;

    S.controls.set()() = ARR(u, u, 10.);

    if(engine().shutdown.getValue()) break;
  }

  engine().close(S);

  cout <<"bye bye" <<endl;

}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  testIMU();
//  testMotors();
//  testBalance();
//  findBalancePoint();

  return 0;
}


// -k_th 400 -k_thDot 30
//./x.exe -IMU_tilt 0.22 -IMU_gyroNoise .1 -IMU_accelNoise 1 -k_th 400 -k_thDot 30
