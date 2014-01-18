#include <Core/module.h>

struct IMU_Poller : Module {
  struct sIMU_Poller *s;

  ACCESS(arr, imuData)

  IMU_Poller():s(NULL){}
  virtual ~IMU_Poller(){}

  void open();
  void step();
  void close();
};


struct KalmanFilter : Module{
  struct sKalmanFilter *s;

  ACCESS(arr, imuData)
  ACCESS(arr, encoderData)
  ACCESS(arr, stateEstimate)

  KalmanFilter():s(NULL){}
  virtual ~KalmanFilter(){}

  void open();
  void step();
  void close();
};


struct RacerDisplay : Module{
  struct sRacerDisplay *s;

  ACCESS(arr, stateEstimate)

  RacerDisplay():s(NULL){}
  virtual ~RacerDisplay(){}

  void open();
  void step();
  void close();
};


struct MotorController : Module{
  struct sMotorController *s;

  ACCESS(arr, stateEstimate)
  ACCESS(arr, encoderData)

  MotorController():s(NULL){}
  virtual ~MotorController(){}

  void open();
  void step();
  void close();
};
