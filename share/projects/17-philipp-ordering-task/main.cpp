#include "OrderingTask.h"
#include <Core/util.h>
#include <iostream>

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  //cout << "What interaction mode do you want to use?" << endl;
  //cout << "0=proactive, 1=autonomous,2=request,3=commands,4=reactive" << endl;
  int mode=1;
  //std::cin >> mode;
  OrderingTask task(mode);
  //HRITask task;
  task.start();
  return 0;
}
