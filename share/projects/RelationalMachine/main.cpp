#include <ros/ros.h>
#include <std_msgs/String.h>
#include <FOL/relationalMachine.h>
#include <Core/thread.h>

inline std_msgs::String MSG(const MT::String& str){
  std_msgs::String msg;
  if(str.N) msg.data = str.p;
  return msg;
}

struct RelationalMachineNode{
  RWLock RM_lock;
  RelationalMachine RM;

  ros::NodeHandle nh;
  ros::Publisher pub_knowledge;
  ros::Publisher pub_state;
  ros::Publisher pub_command;
  ros::Publisher pub_symbols;
  ros::Subscriber sub_newEffect;

  RelationalMachineNode():RM("assembly_chair.fol"), nh("ThirdHand/RelationalMachine"){
    RM.verbose=true;

    pub_state     = nh.advertise<std_msgs::String>("RelationalState", 10, true);
    pub_knowledge = nh.advertise<std_msgs::String>("RelationalKnowledge", 10, true);
    pub_command   = nh.advertise<std_msgs::String>("RelationalCommand", 10, true);
    pub_symbols   = nh.advertise<std_msgs::String>("RelationalSymbols", 10, true);
    sub_newEffect = nh.subscribe("RelationalEffect", 1, &RelationalMachineNode::cb_newEffect, this);

    MT::wait(.1);
    publishSymbols();
    publishKnowledge();
    publishState();
    fwdChainRules();
  }

  void publishState(){
    RM_lock.readLock();
    pub_state.publish(MSG(RM.getState()));
    RM_lock.unlock();
  }

  void publishKnowledge(){
    RM_lock.readLock();
    pub_knowledge.publish(MSG(RM.getKB()));
    RM_lock.unlock();
  }

  void publishSymbols(){
    RM_lock.readLock();
    MT::String str;
    str <<RM.getSymbols();
    pub_symbols.publish(MSG(str));
    RM_lock.unlock();
  }

  void publishCommand(){
    RM_lock.readLock();
    MT::String str;
    RM.tmp->write(str," ");
    pub_command.publish(MSG(str));
    RM_lock.unlock();
  }

  void fwdChainRules(){
    RM_lock.writeLock();
    RM.fwdChainRules();
    RM_lock.unlock();
    publishCommand();
    publishKnowledge();
    publishState();
  }

  void cb_newEffect(const std_msgs::String::ConstPtr& msg);

};

//===========================================================================

void RelationalMachineNode::cb_newEffect(const std_msgs::String::ConstPtr& msg){
  MT::String effect = msg->data.c_str();
  if(!effect.N) return;
  RM_lock.writeLock();
  RM.applyEffect(effect);
  RM_lock.unlock();
  fwdChainRules();
}

//===========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "RelationalMachineNode");

  RelationalMachineNode RMnode;
  ros::spin();

  return 0;
}

