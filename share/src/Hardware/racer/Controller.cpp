#include "Controller.h"



bool Controller::read8BitIntRegister(unsigned char buffer[], int8_t& result, std::string description) {

  bool isTransmissionSuccessful = true;
  
  if (_stream->registerInteraction(buffer, description, 4, 1)) {
  
  result = (int8_t)buffer[0];

  }

  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 

}
 
bool Controller::readUnsigned8BitIntRegister(unsigned char buffer[], uint8_t& result, std::string description) {

  bool isTransmissionSuccessful = true;
  
  if (_stream->registerInteraction(buffer, description, 4, 1)) {
  
  result = (uint8_t)buffer[0];

  }

  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 
  
}
    
bool Controller::read16BitIntRegisterMSB(unsigned char buffer[], int16_t& result, std::string description) {
 
  bool isTransmissionSuccessful = true;
  

  if (_stream->registerInteraction(buffer, description, 4, 2)) {
  
    result = (int16_t)((buffer[0] << 8) + buffer[1]);
  
  }
  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 
  
}
 
bool Controller::readUnsigned16BitIntRegisterMSB(unsigned char buffer[], uint16_t& result, std::string description) {

  bool isTransmissionSuccessful = true;
  
  if (_stream->registerInteraction(buffer, description, 4, 2)) {
  
    result = (uint16_t)((buffer[0] << 8) + buffer[1]);

  }

  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 
 
}
    
bool Controller::read32BitIntRegisterMSB(unsigned char buffer[], int32_t& result, std::string description) {

 bool isTransmissionSuccessful = true;
  
  if (_stream->registerInteraction(buffer, description, 4, 4)) {
  
    result = (int32_t)((buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3]);

  }

  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 
 
 }
 

bool Controller::readUnsigned32BitIntRegisterMSB(unsigned char buffer[], uint32_t& result, std::string description) {

   bool isTransmissionSuccessful = true;
  
  if (_stream->registerInteraction(buffer, description, 4, 4)) {
  
    result = (uint32_t)((buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3]);

  }

  else {
    
    isTransmissionSuccessful = false;
 
  }

  return isTransmissionSuccessful; 
  
}




bool Controller::readRegister(unsigned char buffer[], std::string description, int bytesToRead) {

  bool isTransmissionSuccessful = true;

  if (!_stream->registerInteraction(buffer, description, 4, bytesToRead)) {

    isTransmissionSuccessful = false;

  }

  return isTransmissionSuccessful;

}
 
bool Controller::writeRegister(unsigned char buffer[], std::string description, int bytesToWrite, int bytesToRead) {


  bool isTransmissionSuccessful = true;

  if (!_stream->registerInteraction(buffer, description, bytesToWrite, bytesToRead)) {

      isTransmissionSuccessful = false;

  }

  return isTransmissionSuccessful;

}

void Controller::getConnection() {

  _stream = Connection::getConnection();

 
}


bool Controller::isDeviceConnected(unsigned char address) {

  bool isConnected = true;

    
  if (!_stream->isDeviceConnected(address)) {
 
    isConnected = false;
  
  }

  return isConnected;

}

void Controller::closeConnection() {

  _stream->closeConnection();

}

