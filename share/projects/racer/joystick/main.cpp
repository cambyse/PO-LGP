#include <Core/util.h>
#include <System/engine.h>

#include <Hardware/racer/modules.h>
#include <Hardware/joystick/joystick.h>

void testJoystick(){
  struct MySystem:System{
    ACCESS(arr, joystickState);
    MySystem(){
      addModule<JoystickInterface>("JoystickInterface", ModuleThread::loopWithBeat, 0.01);
      connect();
    }
  } S;

  engine().open(S);

  for(int i = 0;; ++i){
    S.joystickState.var->waitForNextWriteAccess();
    arr J = S.joystickState.get();
    cout <<"\r joy=" <<J <<std::flush;
    if(engine().shutdown.getValue()) break;
  }

  engine().close(S);

  cout <<"bye bye" <<endl;
}


void run(){
  struct MySystem:System{
    ACCESS(arr, imuData)
    ACCESS(arr, stateEstimate);
    ACCESS(arr, encoderData)
    ACCESS(arr, controls);
    ACCESS(arr, joystickState);
    MySystem(){
      addModule<IMU_Poller>("IMU_Poller", ModuleThread::loopFull);
      addModule<KalmanFilter>("KalmanFilter", ModuleThread::listenFirst);
      //addModule<RacerDisplay>("RacerDisplay", ModuleThread::loopWithBeat, 0.1);
      addModule<Motors>("Motors", ModuleThread::loopFull);
      addModule<JoystickInterface>("JoystickInterface", ModuleThread::loopWithBeat, 0.01);
      connect();
    }
  } S;

  //double zeroTh=MT::getParameter<double>("zeroTh", 0.);
  double k_th=MT::getParameter<double>("k_th", 0.);
  double k_thDot=MT::getParameter<double>("k_thDot", 0.);
  double k_acc=MT::getParameter<double>("k_acc", 0.);
  double joy_gain = MT::getParameter<double>("joy_gain");
  double joy_gain_vel = MT::getParameter<double>("joy_gain_vel");

  //    engine().enableAccessLog();
  engine().open(S);
  double motor_vel=0.;
  double x_ref = 0.;

  for(int i = 0;; ++i){
    S.stateEstimate.var->waitForNextWriteAccess();
    arr x = S.stateEstimate.get();
    arr J = S.joystickState.get();

    x_ref -= joy_gain*J(4);
    double th_ref = 0.1 * (x_ref-x(0)) + 0.1 * (0.-x(2));
    double u = k_th*(th_ref - x(1)) + k_thDot * (0.-x(3));

    motor_vel += k_acc*u;
//    cout <<"\r state = " <<x <<std::flush;
//    cout <<"enc= " <<enc/MT_2PI <<std::endl;

    double turn = joy_gain_vel*J(1);
    S.controls.set()() = ARR(motor_vel+turn, motor_vel-turn, 10.);

    uint mode = uint(J(0));
    if(mode&0x10 || mode&0x20 || mode&0x40 || mode&0x80){
      S.controls.set()() = ARR(0, 0, 10.);
      S.encoderData.var->waitForNextWriteAccess();
      break;
    }

    if(engine().shutdown.getValue()) break;
  }

  engine().close(S);

  cout <<"bye bye" <<endl;

}

int main(int argc, char **argv) {
  MT::initCmdLine(argc, argv);

  //testJoystick();

  run();

  return 0;
}

