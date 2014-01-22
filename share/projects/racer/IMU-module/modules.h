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


struct Motors : Module{
  struct sMotors *s;

  ACCESS(arr, controls)
  ACCESS(arr, encoderData)

  Motors():s(NULL){}
  virtual ~Motors(){}

  void open();
  void step();
  void close();
};
