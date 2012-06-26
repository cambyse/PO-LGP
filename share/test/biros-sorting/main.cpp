#include <biros/biros.h>
#include <MT/util.h>
#include <biros/biros_logger.h>
#include <biros/control.h>
#include <gtk/gtk.h>

//standard Variable containing only an integer
struct Integer:public Variable {
  FIELD(int, x);
  
  Integer():Variable("IntVar") { x=rnd(100); }
};

//declaration of a Process
Process *newPairSorter(Integer& a, Integer& b);


int main(int argn, char **argv) {
  uint N=200;

  //dumpViews(); //this is possible because ViewInfos are static entities that are created before main
  
  MT::Array<Integer> ints(N);

  cout <<"*** Before sorting:";
  for(uint i=0; i<ints.N; i++)  cout <<ints(i).x <<' ';
  cout << endl;

  for(uint i=1; i<N; i++) newPairSorter(ints(i-1), ints(i));

  b::openInsideOut();
  
  //run
  //loopSerialized(P);
  step(birosInfo.processes);
  
#if 1
  MT::wait(1.); //let them work for a second
  close(birosInfo.processes);
#else
  if(!logService.getReplay()) {
    MT::wait(10.);
    close(P);
  } else {
    Process *p;
    uint i;
    uint step = 0;
    bool allClosed = false;
    while(!allClosed) {
      MT::wait(1.);
      allClosed = true;
      for_list(i, p, P)  if(!p->threadIsClosed())
          allClosed = false;
      cout << step << endl;
      step++;
      MT::wait(1.);
    }
  }
#endif

  cout <<"*** After sorting:";
  for(uint i=0; i<ints.N; i++){
    cout <<ints(i).x <<' ';
    if(i) CHECK(ints(i).x>=ints(i-1).x,"not sorted!");
  }
  cout << endl;

  b::updateInsideOut();
  gtk_main();
  
  return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// the following would usually be implemented in a cpp file, where processes are implemented
//

//===========================================================================
//
// VariableReference
//

template<class T>
struct Ref{
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  Process *p;         ///< pointer to the Process that might want to access the Variable
  uint last_revision; ///< last revision of a read/write access

  struct ReadAccess{
    Ref *r;
    ReadAccess(Ref *_r){ r=_r; r->var->readAccess(r->p); }
    ~ReadAccess(){ r->var->deAccess(r->p); }
    T& operator()(){ return *r->var; }
  };

  Ref(T& _var, Process *_p){ var=&_var; p=_p; p->threadListenTo(var); }
  T& get(){ return ReadAccess(this)(); }
  T& operator()(){ return *var; } //TODO ensure that it is locked
  T& writeAccess(){ var->writeAccess(p); }
  T& deAccess(){ var->deAccess(p); }
};

struct PairSorter:public Process {
  Ref<Integer> a;
  Ref<Integer> b;
  
  PairSorter(Integer& _a, Integer& _b):Process("PairSorter"), a(_a, this), b(_b, this) {
    //a = &_a;
    //b = &_b;
  };
  
  void open() {}
  void close() {}
  void step() {
    int xa = a.get().x;
    int xb = b.get().x;//->get_x(this);
    if(xa>xb){  //swap numbers
      a.writeAccess(); // we want to lock both to ensure no number is lost!
      b.writeAccess();
      xa = a().x;
      xb = b().x;
      if(xa>xb){  //still?
        a().x = xb;
	b().x = xa;
      }
      b.deAccess();
      a.deAccess();
    }
  }
};

Process *newPairSorter(Integer& a, Integer& b){
  return new PairSorter(a, b);
}
