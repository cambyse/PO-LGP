#include <Core/util.h>
//#include <System/engine.h>

#include <Hardware/racer/modules.h>

void TEST(IMU){
  struct MySystem{
    ACCESS(arr, imuData);
    ACCESS(arr, stateEstimate);
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", Module::loopFull);
      addModule<KalmanFilter>("KalmanFilter" /*,Module::listenFirst*/ );
      addModule<RacerDisplay>("RacerDisplay", /*Module::loopWithBeat,*/ 0.1);
      //connect();
    }
  } S;

  cout <<S <<endl;

  //    //engine().enableAccessLog();
  threadOpenModules(true);


  for(;;){
    S.stateEstimate.var->waitForNextRevision();
    arr x = S.stateEstimate.get();
//    arr enc = S.encoderData.get();
    cout <<"\r state = " <<x <<std::flush;
    if(moduleShutdown().getValue()) break;
//    if(S.imuData.get()->(0)>10.) break;
  }

  //    moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  cout <<"bye bye" <<endl;
}

void TEST(Motors){
  struct MySystem{
    ACCESS(arr, controls)
    MySystem(){
      addModule<Motors>("Motors", Module::loopFull);
      //connect();
    }
  } S;

  cout <<S <<endl;

  threadOpenModules(true);

  S.controls.set() = ARR(5.,5.,10.);
  mlr::wait(3);
  S.controls.set() = ARR(128.,128.,10.);
  mlr::wait(3);
  S.controls.set() = ARR(0.,0.,10.);
  mlr::wait(3);
  S.controls.set() = ARR(128.,128.,1.);
  mlr::wait(3);
  S.controls.set() = ARR(0.,0.,1.);
  mlr::wait(3);

  threadCloseModules();

  cout <<"bye bye" <<endl;
}

void TEST(Balance){
  struct MySystem{
    ACCESS(arr, imuData)
    ACCESS(arr, stateEstimate);
    ACCESS(arr, encoderData)
    ACCESS(arr, controls)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", Module::loopFull);
      addModule<KalmanFilter>("KalmanFilter" /*,Module::listenFirst*/ );
      //      addModule<RacerDisplay>("RacerDisplay", /*Module::loopWithBeat,*/ 0.1);
      addModule<Motors>("Motors", Module::loopFull);
      //connect();
    }
  } S;

  cout <<S <<endl;

  double zeroTh=mlr::getParameter<double>("zeroTh", 0.);
  double k_th=mlr::getParameter<double>("k_th", 0.);
  double k_thDot=mlr::getParameter<double>("k_thDot", 0.);
  double k_acc=mlr::getParameter<double>("k_acc", 0.);

  //    //engine().enableAccessLog();
  threadOpenModules(true);
  double motor_vel=0.;

  for(int i = 0;; ++i){
    S.stateEstimate.var->waitForNextRevision();
    arr x = S.stateEstimate.get();
    arr enc = S.encoderData.get();

    double x_ref = 0.;
    double th_ref = 0.1 * (x_ref-x(0)) + 0.1 * (0.-x(2));
    double u = k_th*(th_ref - x(1)) + k_thDot * (0.-x(3));

    motor_vel += k_acc*u;
//    cout <<"\r state = " <<x <<std::flush;
//    cout <<"enc= " <<enc/MLR_2PI <<std::endl;

    S.controls.set() = ARR(motor_vel, motor_vel, 10.);

    if(moduleShutdown().getValue()) break;
  }

  threadCloseModules();

  cout <<"bye bye" <<endl;

}

int main(int argc, char **argv) {
  mlr::initCmdLine(argc, argv);

//  testIMU();
//  testMotors();
  testBalance();

  return 0;
}


// -k_th 400 -k_thDot 30
//./x.exe -IMU_tilt 0.22 -IMU_gyroNoise .1 -IMU_accelNoise 1 -k_th 400 -k_thDot 30
