#ifndef MD25CONTROLLER_H
#define MD25CONTROLLER_H


#include "Controller.h"
#include "Constants.h"

class MD25Controller : public Controller {

  public:

    bool setMD25Mode(Mode mode);
    bool getMD25Mode(Mode mode); 

    bool setMotorSpeed(const std::string &description, 
                       Motor motor, 
                       unsigned char speed
                      );
    
    bool setMotorSpeed(const std::string &description,  
                       unsigned char speed_1, 
                       unsigned char speed_2
                      );   

    bool readSpeed(unsigned char& speed1, unsigned char& speed2);    

    bool readVoltage(unsigned char& voltage);

    bool disableSpeedRegulation();

    bool setAcceleration(int acceleration);

    bool readEncoder1(int32_t& result);

    bool readEncoder2(int32_t& result);

    bool resetEncoders();

  private:

    Mode _mode;
    



};





#endif //MD25CONTROLLER_H
