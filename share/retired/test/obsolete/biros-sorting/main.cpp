#include <System/biros.h>
#include <Core/util.h>
#include <MT/gtk.h>
#include <biros/biros_views.h>
#include <gtk/gtk.h>

//-- standard VariableData containing only an integer
struct Integer:public VariableData {
  int x;
  
  Integer():VariableData("IntVar") { reg_x(); x=rnd(100); }
};

//-- process creator
Process *newPairSorter(Integer& a, Integer& b);


int main(int argc, char **argv) {
  uint N=20;

  mlr::Array<Integer> ints(N);

  cout <<"*** Before sorting:";
  for(uint i=0; i<ints.N; i++)  cout <<ints(i).x <<' ';
  cout << endl;

  for(uint i=1; i<N; i++) newPairSorter(ints(i-1), ints(i));

  biros().dump();
  new InsideOut;
  mlr::wait(1.);
  
  //run
  //loopSerialized(P);
  step(biros().processes);
  
#if 1
  mlr::wait(1.);
  close(biros().processes);
#else
  if(!logService.getReplay()) {
    mlr::wait(1.);
    close(P);
  }else{
    Process *p;
    uint i;
    uint step = 0;
    bool allClosed = false;
    while(!allClosed) {
      mlr::wait(1.);
      allClosed = true;
      for_list(Type,  p,  P)  if(!p->isClosed())
          allClosed = false;
      cout << step << endl;
      step++;
      mlr::wait(1.);
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
  
  mlr::wait();

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
  T *var;             ///< pointer to the VariableData (T must be derived from VariableData)
  Process *p;         ///< pointer to the Process that might want to access the VariableData
  uint last_revision; ///< last revision of a read/write access


  Ref(T& _var, Process *_p){ var=&_var; p=_p; p->listenTo(var); }
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
    if(delay)  mlr::wait(delay);
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
