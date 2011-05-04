#ifndef SD_graspISF_h
#define SD_graspISF_h

#include <MT/array.h>
#include <MT/robot.h>
#include<MT/specialTaskVariables.h>

#include "utils.h"

struct GraspISFTask:public TaskAbstraction{
  PotentialField *graspobj;
  GraspISFTask();
  double phiAtFrame(ors::Transformation& X, arr &grad);
  void plot_append_data(ControllerModule*);
  void plot_all();

  MT::Array<ors::Shape*> tipsN;
  MT::Array<ors::Shape*> fingsN;
  arr  skins;
  ors::Shape *palm; // palm center wrt wrist body

  double skin_prec;
  bool open_skin;
  arr skin_state;

  bool  grip;

  uint t, T; // time index and max steps

  // plot data
  arr plot_data; // here we collect the data to plot

  /* ------- Task Vars -------- */
  TaskVariableList TVs_all; 
  TaskVariable *TV_tipAlign;
  TaskVariable *TV_fingAlign;
  TaskVariable *TV_palm;
  TaskVariable *TV_oppose;
  PotentialValuesTaskVariable *TV_zeroLevel;

  /* ------ TAsk Abstraction ----- */

  virtual void updateTaskVariables(ControllerModule*); //overloading the virtual
  virtual void initTaskVariables(ControllerModule*);
};


#include "graspISF.cpp"


#endif// header ifdef
