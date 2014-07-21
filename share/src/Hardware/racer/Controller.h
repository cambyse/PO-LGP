#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <iostream>
#include <string>

#include "Connection.h"
#include "Constants.h"



class Controller {

  public:
       
    bool read8BitIntRegister(unsigned char buffer[], int8_t& result, std::string description);
    bool readUnsigned8BitIntRegister(unsigned char buffer[], uint8_t& result, std::string description);
    
    bool read16BitIntRegisterMSB(unsigned char buffer[], int16_t& result, std::string description);
    bool readUnsigned16BitIntRegisterMSB(unsigned char buffer[], uint16_t& result, std::string description);
    
    bool read32BitIntRegisterMSB(unsigned char buffer[], int32_t& result, std::string description);
    bool readUnsigned32BitIntRegisterMSB(unsigned char buffer[], uint32_t& result, std::string description);

    bool readRegister(unsigned char buffer[], std::string description, int bytesToRead);
    bool writeRegister(unsigned char buffer[], std::string description, int bytesToWrite, int bytesToRead);

    void getConnection();
   
    bool isDeviceConnected(unsigned char address);

    void closeConnection();
   
    
  
  protected:
    
    Connection* _stream;
    
};


#endif //CONTROLLER_H
