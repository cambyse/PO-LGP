#ifndef MT_robot_processes_h
#define MT_robot_processes_h

struct CameraModule;

struct LEDtracker:public Process, Variable {
  CameraModule *var;
  
  //OUTPUT
  floatA center;
  
  byteA rgbL, rgbR, green;
  floatA theta, tmp;
  
  LEDtracker():Process("LEDtracker"), Variable("LEDtracker"){}
  
  void open(){};
  void close(){};
  void step();
};
#endif
