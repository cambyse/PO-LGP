#include <ros/ros.h>
#include <std_msgs/String.h>
#include <FOL/relationalMachine.h>
#include <Core/thread.h>
#include <pr2/roscom.h>
#include <Actions/RelationalMachineModule.h>

struct RelationalMachineNode{
  RWLock RM_lock;
  RelationalMachine RM;

  ros::NodeHandle nh;
  ros::Publisher pub_state;
  ros::Publisher pub_command;
  ros::Publisher pub_symbols;
  ros::Subscriber sub_newEffect;

  RelationalMachineNode():RM("machine.fol"){
    pub_state     = nh.advertise<std_msgs::String>("/RAP/state", 10, true);
    pub_command   = nh.advertise<std_msgs::String>("/RAP/command", 10, true);
    pub_symbols   = nh.advertise<std_msgs::String>("/RAP/symbols", 10, true);
    sub_newEffect = nh.subscribe("/RAP/effect", 1, &RelationalMachineNode::cb_newEffect, this);

    mlr::wait(.1);
    publishSymbols();
    publishState();
    fwdChainRules();
  }

  void publishState(){
    RM_lock.readLock();
    pub_state.publish(conv_string2string(RM.getState()));
    RM_lock.unlock();
  }

  void publishSymbols(){
    RM_lock.readLock();
    mlr::String str;
    str <<RM.getSymbols();
    pub_symbols.publish(conv_string2string(str));
    RM_lock.unlock();
  }

  void publishCommand(){
    RM_lock.readLock();
    mlr::String str;
    RM.tmp->write(str," ");
    pub_command.publish(conv_string2string(str));
    RM_lock.unlock();
  }

  void fwdChainRules(){
    RM_lock.writeLock();
    RM.fwdChainRules();
    RM_lock.unlock();
    publishCommand();
    publishState();
  }

  void cb_newEffect(const std_msgs::String::ConstPtr& msg){
    mlr::String effect = msg->data.c_str();
    cout <<"received new effect '" <<effect <<"'" <<endl;
    if(!effect.N) return;
    RM_lock.writeLock();
    RM.applyEffect(effect);
    RM_lock.unlock();
    fwdChainRules();
  }

};

//===========================================================================

void testIntegratedRM(){
  RelationalMachineModule RM;
  RosCom_Spinner spinner;

  SubscriberConvNoHeader<std_msgs::String, mlr::String, &conv_string2string>("/RAP/effects", RM.effects);
//  PublisherConv<std_msgs::String, mlr::String, &conv_string2string>("/RAP/state", RM.state);

  threadOpenModules(true);
  moduleShutdown().waitForValueGreaterThan(0);
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "RelationalMachineNode");

#if 1
  RelationalMachineNode RMnode;
  ros::spin();
#else
  testIntegratedRM();
#endif

  return 0;
}

