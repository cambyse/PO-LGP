#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>
#include <pr2/roscom.h>

struct RelationalMachineModule : Module{
  ACCESSlisten(mlr::String, effects)
  ACCESSnew(ActivityL, A)
  ACCESSnew(mlr::String, state)
  ACCESSnew(StringA, symbols)
  ACCESSnew(RelationalMachine, RM)

  Log _log;

  RelationalMachineModule();
  ~RelationalMachineModule();

  void open();
  void step();
  void close();
};

struct RAP_roscom{
  RelationalMachineModule &RMM;

  PublisherConv<std_msgs::String, StringA, conv_stringA2string> pub_symbols;
  PublisherConv<std_msgs::String, mlr::String, conv_string2string> pub_state;
  PublisherConv<std_msgs::String, mlr::String, conv_string2string> pub_effectsProcessed;

  RAP_roscom(RelationalMachineModule &RMM):
    RMM(RMM),
    pub_symbols("/RAP/symbols",RMM.symbols),
    pub_state("/RAP/state",RMM.state),
    pub_effectsProcessed("/RAP/effectsProcessed",RMM.effects){}
};
