#define MT_IMPLEMENT_TEMPLATES
#include <MT/util.h>
#include <biros/biros.h>
//#include "process_monitor.h"

#include "win.h"

using namespace std;

QApplication *app;

//===========================================================================
//
// test excessive access to Variables
//

struct IntVar:public Variable{
  //BIR_VARIABLE;
  
  FIELD(int, x);

  //BIR_FIELD(bool, mybool);
  
  IntVar():Variable("IntVar"){ x=rnd(1000); reg_x(); }
};

//int IntVar::bir_typeId=-1;

uint PC=0;

struct Maxxer:public Process{
  IntVar *a,*b;

  Maxxer():Process("Maxxer"){};
  
  void open (){}
  void close(){}
  void step (){
    int xa=a->get_x(this);
    int xb=b->get_x(this);
    if(xa>xb) b->set_x(xa, this);
    else a->set_x(xb, this);
  }
};

void testMultiAccess(Gui& gui){
  uint n=MT::getParameter<uint>("n",100);
  MT::Array<IntVar> vars(n);
  MT::Array<Maxxer> procs(2*n);

  gui.add();
  app->processEvents();
  
  for(uint i=0;i<procs.N;i++){
    procs(i).a = &vars.rndElem();
    procs(i).b = &vars.rndElem();
  }

  for(uint i=0;i<procs.N;i++) procs(i).threadLoopWithBeat(rnd.uni(.001,.01));
  MT::wait(1.);
  for(uint i=0;i<procs.N;i++) procs(i).threadClose();

  for(uint i=0;i<vars.N;i++) cout <<vars(i).x <<' ';
  cout <<endl;
  
  gui.exec();
}


int main(int argc,char **argv){
  QApplication app(argc, argv);
  
  Gui gui;
  gui.show();

  testMultiAccess(gui);
  
  return 0;
}
