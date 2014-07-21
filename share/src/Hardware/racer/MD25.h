#ifndef MD25_H
#define MD25_H

#include <iostream>

#include "MotorInterface.h"
#include "MD25Controller.h"
#include "Constants.h"



class MD25  {


  public:
  
    ~MD25();
    bool open();
    void step();
    bool setMotorSpeed(const std::string &description, Motor motor, unsigned char speed);
    bool setMotorSpeed(const std::string &description, unsigned char speed_1, unsigned char speed_2);
    bool setMD25Mode(Mode mode);
    bool getMD25Mode(Mode mode);
    bool readEncoder1(int32_t& result);
    bool readEncoder2(int32_t& result);
    bool setMotorSpeedAndAcceleration(const std::string& description, unsigned char speed_1, unsigned char speed_2, int acceleration);
    bool resetEncoders();
    bool readVoltage(unsigned char& volts);

    
    void close();

    unsigned char _speed1;
    unsigned char _speed2;        
    
  private:
  
    MD25Controller _controller;  

    


    
  
};

#endif //MD25_H
