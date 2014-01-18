#include "modules.h"

#include <Algo/kalman.h>

#include <Hardware/racer/racer.h>
#include <Hardware/racer/MD25.h>
#include <Hardware/racer/MPU9150.h>

//===========================================================================
//
// IMU_Poller
//

struct sIMU_Poller{
  IMUInterface* imu;
  ofstream fil;
};

void IMU_Poller::open(){
  s = new sIMU_Poller;
  s->imu = new MPU9150();
  if(s->imu->open()) std::cout << "Open connection successfully" << std::endl;
  MT::open(s->fil, "nogit-data/IMU_Poller.dat");
  s->fil <<"time acc0 acc1 acc2 gyro0 gyro1 gyro2" <<endl;
  MT::arrayBrackets="  ";
}

void IMU_Poller::close(){
  s->imu->close();
  s->fil.close();
  delete s;
  s=NULL;
}

void IMU_Poller::step(){
  double time=MT::realTime();
  s->imu->step();
  imuData.writeAccess();
  imuData().resize(7);
  imuData()(0) = time;
  for(uint i=0;i<3;i++) imuData()(1+i) = s->imu->_accel[i]*(1./(1<<14));
  for(uint i=0;i<3;i++) imuData()(4+i) = s->imu->_gyro[i]*(1./(1<<13));
  s->fil <<imuData() <<endl;
  imuData.deAccess();
};


//===========================================================================
//
// KalmanFilter
//

struct sKalmanFilter{
  Racer R;
  Kalman K;
  double time;
  ofstream fil;
};

void KalmanFilter::open(){
  s = new sKalmanFilter;
  s->R.q(1)=MT_PI/2.;
  s->K.initialize(cat(s->R.q, s->R.q_dot),1.*eye(4));
  s->time=0.;
  MT::open(s->fil, "nogit-data/KalmanFilter.dat");
  s->fil <<"time x th x_dot th_dot y0_pred y1_pred y2_pred y3_pred y0_true y1_true y2_true y3_true" <<endl;
  MT::arrayBrackets="  ";
}

void KalmanFilter::step(){
  //-- get data
  arr imu = imuData.get();
  arr enc = encoderData.get();
  arr y_true(4);
  y_true(0)=imu(1);
  y_true(1)=imu(3);
  y_true(2)=imu(5);
  if(enc.N==0) y_true(3) = 0.;
  if(enc.N==1) y_true(3) = enc(0);
  if(enc.N==2) y_true(3) = 0.5*(enc(0)+enc(1));

  double tau=imu(0)-s->time;
  s->time=imu(0);

  //-- get model
  arr y_pred, C, c, W;
  s->R.getObservation(y_pred, C, c, W);
  arr A,a,B;
  s->R.getDynamicsAB(A,a,B);

  //-- Kalman update
  s->K.stepPredict(eye(4)+tau*A, tau*a, diag(ARR(1e-6, 1e-6, 1., 1.)));
  s->K.stepObserve(y_true, C, c, W);

  //-- write results
  stateEstimate.set()()=s->K.b_mean;

  s->R.q = s->K.b_mean.sub(0,1);
  s->R.q_dot = s->K.b_mean.sub(2,3);

  s->fil <<s->time <<' '
      <<s->K.b_mean
     <<y_pred
    <<y_true
   <<endl;

}

void KalmanFilter::close(){
  s->fil.close();
  delete s;
  s=NULL;
}

//===========================================================================
//
// RacerDisplay
//

struct sRacerDisplay{
  Racer R;
};

void RacerDisplay::open(){
  s = new sRacerDisplay;
}

void RacerDisplay::step(){
  arr x = stateEstimate.get();
  if(!x.N) return;

  s->R.q = x.sub(0,1);
  s->R.q_dot = x.sub(2,3);
  s->R.gl.update();
}

void RacerDisplay::close(){
  delete s;
  s=NULL;
}

//===========================================================================
//
// MotorController
//

struct sMotorController{
  Racer R;
};

void MotorController::open(){
  s = new sMotorController;
}

void MotorController::step(){
  arr x = stateEstimate.get();
  if(!x.N) return;

  s->R.q = x.sub(0,1);
  s->R.q_dot = x.sub(2,3);
  s->R.gl.update();
}

void MotorController::close(){
  delete s;
  s=NULL;
}

