#include "spinner.h"

#ifdef MLR_ROS

#include "roscom.h"

RosCom_Spinner::RosCom_Spinner(const char* nodeName) : Thread("RosCom_Spinner", .001){
  useRos = mlr::getParameter<bool>("useRos", false);
  if(useRos) rosCheckInit(nodeName);
  threadLoop();
}

RosCom_Spinner::~RosCom_Spinner(){
  threadClose();
}


void RosCom_Spinner::step(){
  if(useRos) ros::spinOnce();
}

#else

RosCom_Spinner::RosCom_Spinner(const char* nodeName) : Thread("RosCom_Spinner", -1){}
RosCom_Spinner::~RosCom_Spinner(){}

void RosCom_Spinner::step(){}

#endif
