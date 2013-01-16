#include <biros/biros.h>
#include <MT/util.h>
#include <MT/array_t.cxx>

//===========================================================================
//
// test looping multiple threads on beat or sync.
// Processes only print out some information. Noting else happens.
//

struct TestThread:public Process{
  const char* name;
  double sec;
  int x;

  TestThread(const char* _name,double _sec):Process(_name){
    name=_name;
    sec=_sec;
    x=0;
    threadOpen();
  }
  ~TestThread(){ threadClose(); }

  void open (){ cout <<name <<" is opening,  x=" <<x <<endl; }
  void close(){ cout <<name <<" is closing,  x=" <<x <<endl; }
  void step (){ if(sec) MT::wait(sec); x++; cout <<name <<" has stepped, x=" <<x <<endl; }
};


void testLoop(){
  // create processes
  TestThread A("A loop (self=.08)",.08);
  TestThread E("E slave of B (self=.0)",.0);
  TestThread B("B loop beat (self=.02)",.02);
  TestThread C("C step (self=.07)",.07);
  TestThread D("D step (self=.03)",.03);
  //TestThread E("E loop slave of A (self=.0)",.0);

  // execute processes with different methods
  A.threadLoop();
  B.threadLoopWithBeat(.11);
  E.threadLoopWithBeat(.12); //SyncWithDone(B);

  Metronome ticcer("ticcer (self=.1)",100);

  for(uint t=0;t<20;t++){
    C.threadStep();
    D.threadStep();
    cout <<"*** main process loop iteration " <<t <<endl;
    //win.threadStep();//globalThreadInfoWin.step();
    ticcer.waitForTic();
  }

  // close processes
  A.threadClose();
  B.threadClose();
  C.threadClose();
  D.threadClose();
  E.threadClose();

  MT::wait();

//   win.threadClose();
}


//===========================================================================
//
// test listening to variables
//

struct Int:Variable{
  FIELD( int, x );
  Int():Variable("Integer"), x(0){ reg_x(); }
};

struct Adder:public Process{
  Int *int1;
  Int *int2;
  Adder(const char* name, Int *i1, Int *i2):Process(name), int1(i1), int2(i2){}
  ~Adder(){ }

  void open (){ }
  void close(){ }
  void step (){
    int x=int1->get_x(this);
    cout <<name <<": reading   " <<x <<endl;
    if(int2){
      int2->set_x(x, this);
      cout <<name <<": writing   " <<x <<endl;
    }else{
      int1->set_x(x+1, this);
      cout <<name <<": rewriting " <<x+1 <<endl;
    }
  }
};

void testListening(){
  biros().enableAccessLog();
  // create and open the biros EventControlView
  EventControlView v;

  // create Adder processes with access to the Variables
  Int i1,i2,i3;
  Adder a1("adder1", &i1, NULL);
  Adder a2("adder2", &i1, &i2);
  Adder a3("adder3", &i2, &i3);
  
  a1.threadLoopWithBeat(.5);
  a2.listenTo(LIST<Variable>(i1));
  a3.listenTo(LIST<Variable>(i2));
  
  // run for 20 sec and close everything
  MT::wait(20.);
  
  a1.threadClose();
  a2.threadClose();
  a3.threadClose();

  biros().dumpAccessLog();
}



//===========================================================================
//
// test excessive access to Variables
//

struct ExampleVar:public Variable{
  //BIR_VARIABLE;
  FIELD(int, x);
  //BIR_FIELD(bool, mybool);
  
  ExampleVar():Variable("IntVar"){ x=rnd(1000); reg_x(); }
};

//int IntVar::bir_typeId=-1;

//uint PC=0;

struct Maxxer:public Process{
  ExampleVar *a,*b;

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

void testMultiAccess(){
  uint n=MT::getParameter<uint>("n",100);
  MT::Array<ExampleVar> vars(n);
  MT::Array<Maxxer> procs(2*n);
  //Fl::lock();
  // ThreadInfoWin win;
  // win.threadLoopWithBeat(.1);

  for(uint i=0;i<procs.N;i++){
    procs(i).a = &vars.rndElem();
    procs(i).b = &vars.rndElem();
  }

  for(uint i=0;i<procs.N;i++) procs(i).threadLoopWithBeat(rnd.uni(.001,.01));
  MT::wait(1.);
  for(uint i=0;i<procs.N;i++) procs(i).threadClose();

  for(uint i=0;i<vars.N;i++) cout <<vars(i).x <<' ';
  cout <<endl;
}


//===========================================================================
//
//
//

int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);

  uint mode = MT::getParameter("mode",1);
  switch(mode){
  case 0: testLoop(); break;
  case 1: testListening(); break;
  case 2: testMultiAccess(); break;
  }

  return 0;
}

