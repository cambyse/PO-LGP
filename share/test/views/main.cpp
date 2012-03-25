#include <perception/perception.h>
#include <views/views.h>
#include "../../src/views/views.h"

struct IntVar:Variable{
  FIELD( int, i );
  IntVar():Variable("IntVar"){ reg_i(); }
};

int main(int argn,char** argv){
  MT::initCmdLine(argn, argv);
  
  dumpViews(); //before anything has been done!
  
  IntVar v;
  View *view = newView(v,0);

  v.set_i(1, NULL);
  view->write(cout);  cout <<endl;
  v.set_i(2, NULL);
  view->write(cout);  cout <<endl;
  
  
  return 0;
}
