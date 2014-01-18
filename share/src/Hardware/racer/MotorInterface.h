#ifndef MOTORINTERFACE_H
#define MOTORINTERFACE_H


class MotorInterface {

  public:

    unsigned char _speed1;
    unsigned char _speed2;    
    unsigned char _voltage;

    
    virtual bool open() = 0;
    virtual void step() = 0;
    virtual void close() = 0;


    virtual ~MotorInterface() {} 
  
};

#endif //MOTORINTERFACE
