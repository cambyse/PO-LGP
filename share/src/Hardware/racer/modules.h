#include <Core/thread.h>

struct IMU_Poller : Thread {
  struct sIMU_Poller *s;

  ACCESS(arr, imuData)

  IMU_Poller() : Thread("IMU_Poller"), s(NULL){}
  virtual ~IMU_Poller(){}

  void open();
  void step();
  void close();
};


struct KalmanFilter : Thread {
  struct sKalmanFilter *s;

  ACCESS(arr, imuData)
  ACCESS(arr, encoderData)
  ACCESS(arr, stateEstimate)

  KalmanFilter() : Thread("KalmanFilter"),s(NULL){}
  virtual ~KalmanFilter(){}

  void open();
  void step();
  void close();
};


struct RacerDisplay : Thread {
  struct sRacerDisplay *s;

  ACCESS(arr, stateEstimate)

  RacerDisplay() : Thread("RacerDisplay"),s(NULL){}
  virtual ~RacerDisplay(){}

  void open();
  void step();
  void close();
};


struct Motors : Thread {
  struct sMotors *s;

  ACCESS(arr, controls)
  ACCESS(arr, encoderData)

  Motors() : Thread("Motors"),s(NULL){}
  virtual ~Motors(){}

  void open();
  void step();
  void close();
};
