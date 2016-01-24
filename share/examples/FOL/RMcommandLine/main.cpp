#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pr2/roscom.h>

//===========================================================================


//===========================================================================

void sendTheTopic(mlr::String& effect){
  ros::NodeHandle nh;
  ros::Publisher pub_effect = nh.advertise<std_msgs::String>("/RelationalMachine/effect", 1, true);

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
}

//===========================================================================

struct GetSymbols{
  ros::NodeHandle nh;
  ros::Subscriber sub_symbols;
  mlr::String symbols;
  ConditionVariable hasSymbols;

  GetSymbols(){
    hasSymbols.setValue(0);
    sub_symbols = nh.subscribe("/RelationalMachine/symbols", 1, &GetSymbols::cb_symbols, this);
    for(uint i=0;i<50;i++){ //wait until a publisher is found
      if(sub_symbols.getNumPublishers()>0) break;
      mlr::wait(.02);
      ros::spinOnce();
    }
    if(!sub_symbols.getNumPublishers()){
      cout <<"there is no publisher of symbols -- won't have any effect if I send it.." <<endl;
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



//===========================================================================


int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);
  ros::init(mlr::argc, mlr::argv, "RelationalMachineCommandLine");

//  GetSymbols sym;
//  cout <<"Symbols = " <<sym.symbols <<endl;

  if(argc<2) return 0;

  mlr::String effect;
  effect <<"( ";
  for(int i=1;i<argc;i++) effect <<argv[i] <<' ';
  effect <<")";

  cout <<"sending effect '" <<effect <<"'" <<endl;

  sendTheTopic(effect);

  cout <<"BYE BYE" <<endl;

  return 0;
}
