#include "MPU9150Controller.h"


bool MPU9150Controller::wakeUpMPU9150() {

  bool success = false;

  unsigned char buffer[5];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_WRITE;
  buffer[2] = MPU_PWR_MGMT_1;
  buffer[3] = 0x01;
  buffer[4] = 0x00;

  if (writeRegister(buffer, "MPU_PWR_MGMT", 5, 0)) {
   
    std::cout << "Successfully wake up IMU" << std::endl;
    success = true;

  }
  else {

    std::cout << "Failed to wake up IMU" << std::endl;

  } 
  usleep(5000);
  return success;
}


bool MPU9150Controller::getStartInclination() {

  bool success = false;
  
  int16_t startInclinationAccel[3];
  double tmp[3];
  
  if (readAccel(startInclinationAccel)) {

    
    //Attention: depends on the full sacle range
    // set in register 28 ACCEL_CONFIG
    for (int i = 0; i < 3; i++) {
      tmp[i] = (double)startInclinationAccel[i];
      std::cout << tmp[i] << std::endl; 
      
    }

    for (int i = 0; i < 3; i++) {

      tmp[i] *= (1.0 / 16384.0);
      std::cout << tmp[i] << std::endl; 
      
    }


 
  
    double forceVectorLength = sqrt(pow(tmp[0], 2) + pow(tmp[1], 2) + pow(tmp[2], 2));
    std::cout << "R: " << forceVectorLength << std::endl; 

    double inclinationX = acos(tmp[0] / forceVectorLength) * (180 / M_PI);
    
  std::cout << "Inclination x-direction: " << inclinationX << std::endl; 

  success = true;

  }

  return success;

}
/*
bool MPU9150Controller::readGyro(int16_t gyro[]) {

  
  unsigned char buffer[4];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_GYRO_XOUT_H;
  buffer[3] = 0x02;
 
  if (!read16BitIntRegisterMSB(buffer, gyro[0], "GyroX")) {

    return false;
      
  }

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_GYRO_YOUT_H;
  buffer[3] = 0x02;

  
  if (!read16BitIntRegisterMSB(buffer, gyro[1], "GyroY")) {

    return false;
      
  }

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_GYRO_ZOUT_H;
  buffer[3] = 0x02;

  if (!read16BitIntRegisterMSB(buffer, gyro[2], "GyroZ")) {

    return false;
      
  }
  
  return true;

}

bool MPU9150Controller::readAccel(int16_t accel[]) {

  
  unsigned char buffer[4];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_ACCEL_XOUT_H;
  buffer[3] = 0x02;
 
  if (!read16BitIntRegisterMSB(buffer, accel[0], "AccelX")) {

    return false;
      
  }

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_ACCEL_YOUT_H;
  buffer[3] = 0x02;

  
  if (!read16BitIntRegisterMSB(buffer, accel[1], "AccelY")) {

    return false;
      
  }

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_ACCEL_ZOUT_H;
  buffer[3] = 0x02;

  if (!read16BitIntRegisterMSB(buffer, accel[2], "AccelZ")) {

    return false;
      
  }
  
  return true;

}
*/
bool MPU9150Controller::readGyro(int16_t gyro[]) {

  bool success = false;
  
  unsigned char buffer[6];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_GYRO_XOUT_H;
  buffer[3] = 0x06;
  

  if (readRegister(buffer, "read Gyro", 6)) {
    
    /*std::cout << "read1: " <<(int) buffer[0] << "  " <<(int) buffer[1] << std::endl;
    std::cout << "read2: " <<(int) buffer[2] << "  " << (int)buffer[3] << std::endl;
    std::cout << "read3: " << (int)buffer[4] << "  " << (int)buffer[5] << std::endl;
    */

    gyro[0] = (int16_t)((buffer[0] << 8) + buffer[1]);
    gyro[1] = (int16_t)((buffer[2] << 8) + buffer[3]);
    gyro[2] = (int16_t)((buffer[4] << 8) + buffer[5]);

    success = true;

  } 
  return success;
}


bool MPU9150Controller::readAccel(int16_t accel[]) {

  bool success = false;
  int16_t tmpBuffer[3];
  
  unsigned char buffer[6];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MPU_9150_READ;
  buffer[2] = MPU_ACCEL_XOUT_H;
  buffer[3] = 0x06;
 

  if (readRegister(buffer, "read Accel", 6)) {
    /*  std::cout << "read1: " <<(int) buffer[0] << "  " <<(int) buffer[1] << std::endl;
    std::cout << "read2: " <<(int) buffer[2] << "  " << (int)buffer[3] << std::endl;
    std::cout << "read3: " << (int)buffer[4] << "  " << (int)buffer[5] << std::endl;
    */    

    tmpBuffer[0] = (int16_t)((buffer[0] << 8) + buffer[1]);
    tmpBuffer[1] = (int16_t)((buffer[2] << 8) + buffer[3]);
    tmpBuffer[2] = (int16_t)((buffer[4] << 8) + buffer[5]);

    if (tmpBuffer[0] != -1){

       accel[0] = tmpBuffer[0];
       accel[1] = tmpBuffer[1];
       accel[2] = tmpBuffer[2];
       success = true;
    }
    else{

        success = false;
    }
  }

  return success;
}

