#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pr2/roscom.h>

//===========================================================================

int printState(){
  cout <<"state = " <<std::endl;
  return 0;
}

//===========================================================================

int sendFact(mlr::String& effect){
  cout <<"sending '" <<effect <<"'" <<std::flush;

  ros::init(mlr::argc, mlr::argv, "RAPshell");
  ros::NodeHandle nh;
  ros::Publisher pub_effect = nh.advertise<std_msgs::String>("/RAP/effect", 1, true);

  if(ros::ok()){
    for(uint i=0;i<50;i++){ //wait until a subscriber has subscribed
      if(pub_effect.getNumSubscribers()>0) break;
      mlr::wait(.02);
      ros::spinOnce();
    }
    if(!pub_effect.getNumSubscribers()){
      cout <<"there is no subscriber to my message -- won't have any effect if I send it.." <<endl;
    }

    pub_effect.publish(conv_string2string(effect));

    ros::spinOnce();
    mlr::wait(.05); //this is necessary to ensure that the subscriber has spinned once before the publisher is killed
  }

  cout <<" -- done" <<endl;

  return 0;
}

//===========================================================================

struct GetSymbols{
  ros::NodeHandle nh;
  ros::Subscriber sub_symbols;
  mlr::String symbols;
  ConditionVariable hasSymbols;

  GetSymbols(){
    hasSymbols.setValue(0);
    sub_symbols = nh.subscribe("/RAP/symbols", 1, &GetSymbols::cb_symbols, this);
    for(uint i=0;i<50;i++){ //wait until a publisher is found
      if(sub_symbols.getNumPublishers()>0) break;
      mlr::wait(.02);
      ros::spinOnce();
    }
    if(!sub_symbols.getNumPublishers()){
      cout <<"there is no publisher of symbols -- won't have any effect if subscribe it.." <<endl;
    }
    if(!hasSymbols.waitForValueEq(1, false, 1.0)){
      cout <<"waiting for the topic timed out" <<endl;
    }
  }

  void cb_symbols(const std_msgs::String::ConstPtr &msg) {
    symbols = conv_string2string(*msg);
    hasSymbols.setValue(1);
  }

};

int printSymbols(){
  cout <<"symbols=" <<endl;
  ros::init(mlr::argc, mlr::argv, "RAPshell");
  GetSymbols S;
  cout <<S.symbols <<endl;
  return 0;
}


//===========================================================================


int main(int argc, char** argv) {
  setLogLevels(0,0);
  mlr::initCmdLine(argc, argv);

  if(argc<2) return printState();

  if(!strcmp(argv[1],"st")) return printState();

  if(!strcmp(argv[1],"sy")) return printSymbols();

  //-- send a fact
  mlr::String fact;
  fact <<"( ";
  for(int i=1;i<argc;i++) fact <<argv[i] <<' ';
  fact <<")";
  return sendFact(fact);

  return 0;
}
