#include "MPU9150.h"

bool MPU9150::open() {

  bool success = false;
 
  _controller.getConnection(); 
  
    
    if (_controller.wakeUpMPU9150()) {
    
      

      success = true;
     
    }

  

  return success;
}


void MPU9150::step() {


  if (!_controller.readGyro(_gyro)) {

      std::cout << "Failed to read gyro data!   " << std::endl;
      
  }
   
  if (!_controller.readAccel(_accel)) {

      std::cout << "Failed to read accel data! " << std::endl;

  }

}


void MPU9150::close() {

  _controller.closeConnection();

}



