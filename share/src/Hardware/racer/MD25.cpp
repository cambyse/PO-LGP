#include "MD25.h"


bool MD25::open() {

  bool success = false;

  _controller.getConnection();


  if (_controller.setMD25Mode(SEPERATE_0_TO_255) && _controller.resetEncoders()  && _controller.setAcceleration(10)) {

       success = true;

    }
    
  return success;

}


void MD25::step() {


  if (!_controller.readSpeed(_speed1, _speed2)) {
    
    std::cout << "Failed to read speed registers" << std::endl;

  }

  // std::cout << "speed1 " << (int)_speed1 << "   " << (int)_speed2 << std::endl;

  //  _controller.setMotorSpeed("blub", 0x43, 0x79);
 
}


bool MD25::setMotorSpeed(const std::string& description, Motor motor, unsigned char speed) {

  bool success = false;

  if (_controller.setMotorSpeed(description, motor, speed)) {

    success = true;
  }

  return success;
}

bool MD25::setMotorSpeed(const std::string& description, unsigned char speed_1, unsigned char speed_2) {
 
  bool success = false;

  if (_controller.setMotorSpeed(description, speed_1, speed_2)) {

    success = true;

  }

  return success;
}


bool MD25::setMD25Mode(Mode mode) {

  bool success = false;

  if (_controller.setMD25Mode(mode)) {

    success = true;

  }

  return success;
}


bool MD25::getMD25Mode(Mode mode) {

  bool success = false;

  if (_controller.getMD25Mode(mode)) {

    success = true;
  
  }

  return success;
}


bool MD25::readEncoder1(int32_t& result) {

    bool success = false;

    if (_controller.readEncoder1(result)) {

        success = true;

    }

    return success;
}


bool MD25::readEncoder2(int32_t& result) {

    bool success = false;

    if (_controller.readEncoder2(result)) {

        success = true;

    }

    return success;
}

bool MD25::readVoltage(unsigned char& volts) {

    bool success = false;

    if (_controller.readVoltage(volts)) {

        success = true;

    }

    return success;
}

bool MD25::resetEncoders() {

    bool success = false;

    if (_controller.resetEncoders()) {

        success = true;

    }

    return success;
}

void MD25::close() {

  _controller.closeConnection();

}

MD25::~MD25() {

  _controller.setMotorSpeed("stop motor", 128, 128);

}

bool MD25::setMotorSpeedAndAcceleration(const std::string& description, unsigned char speed_1, unsigned char speed_2, int acceleration) {

  bool success = false;

  if (_controller.setMotorSpeed(description, speed_1, speed_2) && _controller.setAcceleration(acceleration)) {

      success = true;

  }

  return success;

}
