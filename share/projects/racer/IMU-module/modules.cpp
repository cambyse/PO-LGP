#include "modules.h"

#include <Algo/kalman.h>
#include <Core/thread.h>

#include <Hardware/racer/racer.h>
#include <Hardware/racer/MD25.h>
#include <Hardware/racer/MPU9150.h>

Mutex i2cmutex;

static const double counts_per_motorTurn = 360.;
static const double motorTurn_per_wheelTurn = 30.;
static const double rad_per_count = MT_2PI/(counts_per_motorTurn);

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
  i2cmutex.lock();
  if(s->imu->open()) std::cout << "Open connection successfully" << std::endl;
  i2cmutex.unlock();
  MT::open(s->fil, "nogit-data/IMU_Poller.dat");
  s->fil <<"time acc0 acc1 acc2 gyro0 gyro1 gyro2" <<endl;
  MT::arrayBrackets="  ";
}

void IMU_Poller::close(){
  i2cmutex.lock();
  s->imu->close();
  i2cmutex.unlock();
  s->fil.close();
  delete s;
  s=NULL;
}

void IMU_Poller::step(){
  double time=MT::realTime();
  i2cmutex.lock();
  s->imu->step();
  i2cmutex.unlock();
  imuData.writeAccess();
  imuData().resize(7);
  imuData()(0) = time;
  //scale to double, and flip coordinate axes
  imuData()(1) =  s->imu->_accel[2]*(1./(1<<14));
  imuData()(2) = -s->imu->_accel[1]*(1./(1<<14));
  imuData()(3) =  s->imu->_accel[0]*(1./(1<<14));
  imuData()(4) =  s->imu->_gyro[2]*(1./(1<<13));
  imuData()(5) = -s->imu->_gyro[1]*(1./(1<<13));
  imuData()(6) =  s->imu->_gyro[0]*(1./(1<<13));
  s->fil <<imuData() <<endl;
  imuData.deAccess();
};


//===========================================================================
//
// Motors interface
//

struct sMotors{
  MD25* motor;
  ofstream fil;
  double lastTime;
  arr lastEnc;
};

void Motors::open(){
  s = new sMotors;
  s->motor = new MD25();
  i2cmutex.lock();
  if(s->motor->open()) std::cout << "Open connection to MD25 successfully ... " <<std::flush;
  byte volts;
  s->motor->readVoltage(volts);
  std::cout <<"VOLTAGE = " <<double(volts)/10 <<'V' <<endl;
  i2cmutex.unlock();
  MT::open(s->fil, "nogit-data/Motors.dat");
  s->fil <<"time enc0 enc1 vel0 vel1 acc dtime denc0 denc1" <<endl;
  MT::arrayBrackets="  ";
}

void Motors::step(){
  double time=MT::realTime();
  arr u = controls.get();
  if(!u.N) return;
  CHECK(u.N==3," need u=(vel1, vel2, acc)");

  if(u(2)<0) u*=-1.;
  int vel0=128 + int(u(0));  if(vel0<0) vel0=0;  if(vel0>255) vel0=255;
  int vel1=128 + int(u(1));  if(vel1<0) vel1=0;  if(vel1>255) vel1=255;
  int acc=u(2);
  int32_t encoder1 = 0;
  int32_t encoder2 = 0;
  i2cmutex.lock();
  s->motor->setMotorSpeedAndAcceleration("", vel0, vel1, acc);
  s->motor->readEncoder1(encoder1);
  s->motor->readEncoder2(encoder2);
  i2cmutex.unlock();

  encoderData.writeAccess();
  encoderData().resize(3);
  encoderData()(0) = time;
  encoderData()(1) = rad_per_count*encoder1; //0.11
  encoderData()(2) = rad_per_count*encoder2;
  s->fil <<encoderData() <<' ' <<u;
  if(s->lastEnc.N) s->fil <<(encoderData()-s->lastEnc)/(time-s->lastTime) <<endl;
  else s->fil <<"0 0 0" <<endl;
  s->lastEnc=encoderData();
  s->lastTime=time;
  encoderData.deAccess();
}

void Motors::close(){
  i2cmutex.lock();
  s->motor->close();
  i2cmutex.unlock();
  s->fil.close();
  delete s;
  s=NULL;
}


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
//  s->R.q(1)=MT_PI/2.;
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
  if(enc.N==3) y_true(3) = 0.5*(enc(1)+enc(2));

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
  s->R.gl().update();
}

void RacerDisplay::close(){
  delete s;
  s=NULL;
}

