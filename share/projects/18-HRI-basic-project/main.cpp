#include "BasicTask.h"
#include <Core/util.h>
#include <iostream>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  int mode = 0;
  BasicTask task(mode);
  task.start();
  return 0;
}
