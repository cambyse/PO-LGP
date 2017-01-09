#include <Core/util.h>
//#include <System/engine.h>

#include "modules.h"

void testIMU(){
  struct MySystem{
    ACCESS(arr, imuData)
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
    S.imuData.data->waitForNextRevision();
    if(S.imuData.get()->(0)>10.) break;
  }

  //    moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  cout <<"bye bye" <<endl;
}

void findBalancePoint(){
  struct MySystem{
    ACCESS(arr, imuData)
    ACCESS(arr, stateEstimate)
    ACCESS(arr, controls)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", Module::loopFull);
      addModule<KalmanFilter>("KalmanFilter" /*,Module::listenFirst*/ );
      addModule<RacerDisplay>("RacerDisplay", /*Module::loopWithBeat,*/ 0.1);
      addModule<Motors>("Motors", Module::loopFull);
      //connect();
    }
    double get_time() {
      return imuData.get()->(0);
    }
  } S;

  cout <<S <<endl;

  //    //engine().enableAccessLog();
  threadOpenModules(true);


  double start_time = -1;
  for(;;){
    //S.imuData.data->waitForNextRevision();
    S.stateEstimate.data->waitForNextRevision();
    if (start_time == -1) start_time = S.get_time();

    S.controls.set() = ARR(0., 0., 0.);

    double current_time = S.get_time();
    if (current_time - start_time > 0.25) {
      arr x = S.stateEstimate.get();
      cout << "size(s): " << x.N << endl;
      cout << "[" << current_time << "] angle: " << x(1) << endl;
      start_time = current_time;
    }
    //if(S.imuData.get()->(0)>10.) break;
  }

  //    moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  cout <<"bye bye" <<endl;
}

void testMotors(){
  struct MySystem{
    ACCESS(arr, controls)
    MySystem(){
      addModule<Motors>("Motors", Module::loopFull);
      //connect();
    }
  } S;

  cout <<S <<endl;

  threadOpenModules(true);

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

class Sigmoid {
public:
  Sigmoid(double alpha=0.): alpha_(alpha) {}
  double basic_sigmoid(double x) const {
    if (x > 0) {
      return 1./(1. + exp(-alpha_ * x));
    } else {
      double expx = exp(alpha_ * x);
      return expx / (1. + expx);
    }
  }
  double operator()(double x) const {
    return 2*basic_sigmoid(x) - 1.;
  }
protected:
  double alpha_;
};

void testBalance(){
  struct MySystem{
    ACCESS(arr, imuData)
    ACCESS(arr, stateEstimate)
    ACCESS(arr, controls)
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", Module::loopFull);
      addModule<KalmanFilter>("KalmanFilter" /*,Module::listenFirst*/ );
      addModule<RacerDisplay>("RacerDisplay", /*Module::loopWithBeat,*/ 0.1);
      addModule<Motors>("Motors", Module::loopFull);
      //connect();
    }
    double get_time() {
      return imuData.get()->(0);
    }
  } S;

  cout <<S <<endl;

  //    //engine().enableAccessLog();
  threadOpenModules(true);

  double integral_angular_deviation = 0.;

  Sigmoid sigmoid(10.);
  double prev_time = -1;
  for(int i = 0;; ++i){
    S.stateEstimate.data->waitForNextRevision();
    arr x = S.stateEstimate.get();

    double current_time = S.get_time();
    if (prev_time == -1) prev_time = current_time;
    double elapse_time = current_time - prev_time;
    prev_time = current_time;
//    if (start_time == -1 || i % 100 == 0) {
//      cout << "elapse time: " << elapse_time/100 << endl;
//    }

    //double u = 1000.*(.0-x(1));
    //double u = 200. * (.0 - x(1));
    double set_pt = 0.03;
    double angle = x(1);

    //double gamma = elapse_time/T;
    //integral_angular_deviation += elapse_time * (angle - set_pt);
    //double T = 1.0;
    //set_pt -= elapse_time/T * integral_angular_deviation;
    double u = 128. * sigmoid(set_pt - angle);

    if (u > 128) {
      u = 128;
      cerr << "ERROR: + limit hit" << endl;
    }
    if (u < -127) {
      cerr << "ERROR: - limit hit" << endl;
      u = -127;
    }
    S.controls.set() = ARR(u, u, 10.);

    //if(S.imuData.get()->(0)>10.) break;
  }

  //    moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  cout <<"bye bye" <<endl;

}

int main() {
//  testIMU();
//  testMotors();
  testBalance();
//  findBalancePoint();

  return 0;
}


