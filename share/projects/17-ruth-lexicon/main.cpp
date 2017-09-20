#include "learn_task.h"
//#include "test_task.h"
//#include "test_place_task.h"
#include <Core/util.h>
#include <iostream>

// HAI 2017 teaching a robot words for object placement on a table
// learning phase: #include "learn_task.h" and learn_task task;
// test phase 1 (point to objects): #include "test_task.h" and test_task task;
// test phase 2 (place object): #include "test_place_task.h" and test_place_task task;
// set objects and table size in model.g
// also set table size in lexicon_TTSR.cpp

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  learn_task task;
//  test_task task;
//  test_place_task task;
  task.start();
  return 0;
}
