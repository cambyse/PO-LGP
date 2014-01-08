#ifndef MPU9150CONTROLLER_H
#define MPU9150CONTROLLER_H

#include <stdint.h>
#include <iomanip>
#include <math.h>

#include "Controller.h"
#include "Constants.h"

class MPU9150Controller : public Controller {

  public:

    bool wakeUpMPU9150();

    bool getStartInclination();
  
    bool readGyro(int16_t gyro[]);
    
    bool readAccel(int16_t accel[]);
   
    
};

  


#endif //MPU9150CONTROLLER_H
