#include <ros/ros.h>
#include <mlr_srv/StringString.h>
#include <Core/util.h>


//===========================================================================

int query(const char* cmd){
  mlr_srv::StringString com;
  com.request.str = cmd;
  if(ros::service::call("/RAP/service", com)){
    cout <<cmd <<": " <<com.response.str <<endl;
  }else cout <<"getState failed" <<endl;

  return 0;
}

//===========================================================================

int main(int argc, char** argv) {
  setLogLevels(0,0);
  mlr::initCmdLine(argc, argv);
  ros::init(mlr::argc, mlr::argv, "RAPshell");

  if(argc<2) return query("getState");
  if(!strcmp(argv[1],"st")) return query("getState");
  if(!strcmp(argv[1],"sy")) return query("getSymbols");

  //-- send a fact
  mlr::String fact;
  fact <<"( ";
  for(int i=1;i<argc;i++) fact <<argv[i] <<' ';
  fact <<")";
  return query(fact);

  return 0;
}
