#include <Core/util.h>
#include <System/engine.h>

#include <Hardware/racer/modules.h>

void testIMU(){
  struct MySystem:System{
    ACCESS(arr, imuData);
    ACCESS(arr, stateEstimate);
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
    S.stateEstimate.var->waitForNextWriteAccess();
    arr x = S.stateEstimate.get();
//    arr enc = S.encoderData.get();
    cout <<"\r state = " <<x <<std::flush;
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
      //      addModule<RacerDisplay>("RacerDisplay", ModuleThread::loopWithBeat, 0.1);
      addModule<Motors>("Motors", ModuleThread::loopFull);
      connect();
    }
  } S;

  cout <<S <<endl;

  double zeroTh=MT::getParameter<double>("zeroTh", 0.);
  double k_th=MT::getParameter<double>("k_th", 0.);
  double k_thDot=MT::getParameter<double>("k_thDot", 0.);
  double k_acc=MT::getParameter<double>("k_acc", 0.);

  //    engine().enableAccessLog();
  engine().open(S);
  double motor_vel=0.;

  for(int i = 0;; ++i){
    S.stateEstimate.var->waitForNextWriteAccess();
    arr x = S.stateEstimate.get();
    arr enc = S.encoderData.get();

    double x_ref = 0.;
    double th_ref = 0.1 * (x_ref-x(0)) + 0.1 * (0.-x(2));
    double u = k_th*(th_ref - x(1)) + k_thDot * (0.-x(3));

    motor_vel += k_acc*u;
//    cout <<"\r state = " <<x <<std::flush;
//    cout <<"enc= " <<enc/MT_2PI <<std::endl;

    S.controls.set()() = ARR(motor_vel, motor_vel, 10.);

    if(engine().shutdown.getValue()) break;
  }

  engine().close(S);

  cout <<"bye bye" <<endl;

}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

//  testIMU();
//  testMotors();
  testBalance();

  return 0;
}


// -k_th 400 -k_thDot 30
//./x.exe -IMU_tilt 0.22 -IMU_gyroNoise .1 -IMU_accelNoise 1 -k_th 400 -k_thDot 30
