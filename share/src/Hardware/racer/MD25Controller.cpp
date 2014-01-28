#include "MD25Controller.h"

bool MD25Controller::setMD25Mode(Mode mode) {
  
  bool isTransmissionSuccessful = true;

  unsigned char buffer[5];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_WRITE;
  buffer[2] = MD_25_MODE;
  buffer[3] = 0x01;
  buffer[4] = mode;
  
  if (!_stream->registerInteraction(buffer, "Set mode at register 0x0F", 5, 0)) {
  
    isTransmissionSuccessful = false;  

  }

  return isTransmissionSuccessful;

}

bool MD25Controller::getMD25Mode(Mode mode) {
  
  bool isTransmissionSuccessful = true;
  unsigned char buffer[4];
  
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_READ;
  buffer[2] = MD_25_MODE;
  buffer[3] = 0x01;
  
  if (_stream->registerInteraction(buffer, " Read Mode ", 4, 1)) {
  
  mode = (Mode)buffer[0];

  }
  else {
  
    isTransmissionSuccessful = false;
  
  }

  return isTransmissionSuccessful;

}


bool MD25Controller::setMotorSpeed(const std::string& description, Motor motor, unsigned char speed) {

  bool isTransmissionSuccessful = true;
  unsigned char buffer[5];

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_WRITE;
  buffer[2] = motor;
  buffer[3] = 0x01;
  buffer[4] = speed;
  

  if (!_stream->registerInteraction(buffer, description, 5, 0)) {

    isTransmissionSuccessful = false;  

  } 

  return isTransmissionSuccessful;

}

  bool MD25Controller::setMotorSpeed(const std::string& description, unsigned char speed_1, unsigned char speed_2) {

  bool isTransmissionSuccessful = true;
  unsigned char buffer[6];

  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_WRITE;
  buffer[2] = MD_25_MOTOR_1;
  buffer[3] = 0x02;
  buffer[4] = speed_1;
  buffer[5] = speed_2;
  

  if (!_stream->registerInteraction(buffer, description, 6, 0)) {

    isTransmissionSuccessful = false;  

  } 

  return isTransmissionSuccessful;

}


bool MD25Controller::readSpeed(unsigned char& speed1, unsigned char& speed2) {
  
  bool success = false;

  unsigned char buffer[4];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_READ;
  buffer[2] = 0x00;
  buffer[3] = 0x02;

  if (readRegister(buffer, "read speed", 2)) {

    speed1 = buffer[0];
    speed2 = buffer[1];
    success = true;

  }
 

  return success;
  
}


bool MD25Controller::readVoltage(unsigned char& voltage) {

  bool success = false;  

  unsigned char buffer[4];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_READ;
  buffer[2] = MD_25_VOLTAGE;
  buffer[3] = 0x01;
  
  if (_stream->registerInteraction(buffer, "read voltage", 4, 1)) {

    voltage = buffer[0];
    success = true;

  }

  return success;

}

bool MD25Controller::disableSpeedRegulation() {

  bool success = false;

  unsigned char buffer[5];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_WRITE;
  buffer[2] = MD_25_COMMAND;
  buffer[3] = 0x01;
  buffer[4] = 0x30;

  if (_stream->registerInteraction(buffer, "disable Speed Regulation", 5, 0)) {

      success = true;

  }

  return success;

}

bool MD25Controller::setAcceleration(int acceleration) {

  bool success = false;

  unsigned char buffer[5];
  buffer[0] = I2C_INTERFACE;
  buffer[1] = MD_25_WRITE;
  buffer[2] = MD_25_ACCELERATION;
  buffer[3] = 0x01;

  if(acceleration < 1){
    acceleration = 1;
  }
  else {

    if (acceleration > 10) {
      acceleration = 10;
    }
  }

  buffer[4] = (unsigned char)acceleration;

  if (_stream->registerInteraction(buffer, "set accel", 5, 0)) {

      success = true;

  }

  return success;

}

bool MD25Controller::readEncoder1(int32_t& result) {

    bool success = false;

    unsigned char buffer[4];
    buffer[0] = I2C_INTERFACE;
    buffer[1] = MD_25_READ;
    buffer[2] = MD_25_ENCODER_1_MSB;
    buffer[3] = 0x04;

    if (read32BitIntRegisterMSB(buffer, result, "read encoder 1")) {

        success = true;

    }

    return success;
}

bool MD25Controller::readEncoder2(int32_t& result) {

    bool success = false;

    unsigned char buffer[4];
    buffer[0] = I2C_INTERFACE;
    buffer[1] = MD_25_READ;
    buffer[2] = MD_25_ENCODER_2_MSB;
    buffer[3] = 0x04;

    if (read32BitIntRegisterMSB(buffer, result, "read encoder 2")) {

        success = true;

    }

    return success;
}

bool MD25Controller::resetEncoders() {

    bool success = false;

    unsigned char buffer[5];
    buffer[0] = I2C_INTERFACE;
    buffer[1] = MD_25_WRITE;
    buffer[2] = MD_25_COMMAND;
    buffer[3] = 0x01;
    buffer[4] = MD_25_RESET_ENCODER;

    if (_stream->registerInteraction(buffer, "reset encoder", 5, 0)) {

        success = true;

    }

    return success;
}

