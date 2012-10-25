#include <biros/biros.h>
#include <MT/util.h>
#include <MT/gtk.h>
#include <biros/biros_views.h>
#include <gtk/gtk.h>

//-- standard Variable containing only an integer
struct Integer:public Variable {
  FIELD(int, x);
  
  Integer():Variable("IntVar") { reg_x(); x=rnd(100); }
};

//-- process creator
Process *newPairSorter(Integer& a, Integer& b);


int main(int argn, char **argv) {
  uint N=20;

  MT::Array<Integer> ints(N);

  cout <<"*** Before sorting:";
  for(uint i=0; i<ints.N; i++)  cout <<ints(i).x <<' ';
  cout << endl;

  for(uint i=1; i<N; i++) newPairSorter(ints(i-1), ints(i));

  biros().dump();
  new InsideOut;
  MT::wait(1.);
  
  //run
  //loopSerialized(P);
  step(biros().processes);
  
#if 1
  MT::wait(1.);
  close(biros().processes);
#else
  if(!logService.getReplay()) {
    MT::wait(1.);
    close(P);
  }else{
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
  cout <<endl;

  //b::updateInsideOut();
  
  MT::wait();

  return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// the following would usually be implemented in a cpp file, where processes are implemented
//

//===========================================================================
//
// simple access tokens - for permanent reference access (via get() and set())
//

template<class T> struct ReadToken{
  T *v;
  Process *p;
  ReadToken(T *_v,Process *_p):v(_v),p(_p){ v->readAccess(p); }
  ~ReadToken(){ v->deAccess(p); }
  const T& operator()(){ return *v; }
};

template<class T> struct WriteToken{
  T *v;
  Process *p;
  WriteToken(T *_v,Process *_p):v(_v),p(_p){ v->writeAccess(p); }
  ~WriteToken(){ v->deAccess(p); }
  T& operator()(){ return *v; }
};


//===========================================================================
//
// VariableReference
//

template<class T>
struct Ref{
  T *var;             ///< pointer to the Variable (T must be derived from Variable)
  Process *p;         ///< pointer to the Process that might want to access the Variable
  uint last_revision; ///< last revision of a read/write access


  Ref(T& _var, Process *_p){ var=&_var; p=_p; p->threadListenTo(var); }
  const T& get(){ return ReadToken<T>(var,p)(); }
  T& operator()(){ return *var; } //TODO ensure that it is locked
  void writeAccess(){ var->writeAccess(p); }
  void deAccess(){ var->deAccess(p); }
};

struct PairSorter:public Process {
  Ref<Integer> a;
  Ref<Integer> b;
  double delay;
  
  PairSorter(Integer& _a, Integer& _b):Process("PairSorter"), a(_a, this), b(_b, this) {
    //a = &_a;
    //b = &_b;
    delay = biros().getParameter<double>("stepDelay", this);
  };
  
  void open() {}
  void close() {}
  void step() {
    if(delay)  MT::wait(delay);
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
