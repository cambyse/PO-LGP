#include <Core/util.h>

#include <Hardware/racer/MD25.h>
#include <Hardware/racer/MPU9150.h>


int main() {
  IMUInterface* imu = new MPU9150();
  
  ofstream fil("nogit-data/imu.dat");
  fil <<"time acc0 acc1 acc2 gyro0 gyro1 gyro2" <<endl;
  
  if(imu->open()) std::cout << "Open connection successfully" << std::endl;

  for(;;){
    imu->step();
    
    double time=MT::realTime();
    fil <<time
       <<' ' << imu->_accel[0] << ' ' <<imu->_accel[1] << ' ' <<imu->_accel[2] << ' '
      <<imu->_gyro[0] << ' ' << imu->_gyro[1] << ' ' << imu->_gyro[2] << ' ' << std::endl;

    if(time>30.) break;
  }
  
  fil.close();
  imu->close();
  return 0;
}


