#ifndef MPU9150_H
#define MPU9150_H

#include <stdint.h>

#include "IMUInterface.h"
#include "MPU9150Controller.h"

class MPU9150 : public IMUInterface {

  public:
  
    bool open();
    void step();
    void close();
  
 
  private: 

    MPU9150Controller _controller;
    
    


};




#endif //MPU9150_H
