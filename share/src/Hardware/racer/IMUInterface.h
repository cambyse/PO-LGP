#ifndef IMUINTERFACE_H
#define IMUINTERFACE_H


class IMUInterface {

  public:

    int16_t _gyro[3];
    int16_t _accel[3];
    
    virtual bool open() = 0;
    virtual void step() = 0;
    virtual void close() = 0;
  
    virtual ~IMUInterface() {}


};

#endif //IMUINTERFACE_H
