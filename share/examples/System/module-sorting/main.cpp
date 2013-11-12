#include <Core/module.h>
#include <System/engine.h>
#include <Gui/graphview.h>
//#include <biros/biros_views.h>

BEGIN_MODULE(PairSorter)
ACCESS(int, a)
ACCESS(int, b)
END_MODULE()

REGISTER_MODULE(PairSorter)

//==============================================================================

void TEST(ModuleSorter){
  uint N=20;

  engine().enableAccessLog();
  //engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  cout <<registry() <<endl <<"----------------------------" <<endl;
  System S;
#if 0
  for(uint i=0;i<N;i++) S.addVariable<int>(STRING("int"<<i));
  for(uint i=1;i<N;i++) S.addModule("PairSorter", STRING("S"<<i-1), ARRAY<uint>(i-1, i));
#else
  for(uint i=1;i<N;i++) S.addModule("PairSorter", STRING("S"<<i-1), ARRAY<MT::String>(STRING("int"<<i-1), STRING("int"<<i)));
  S.connect();
#endif

  cout <<S <<endl;

  engine().open(S);

  //new InsideOut();                 //create an explicit view


  for(uint i=0;i<N;i++) S.getVar<int>(i) = MT::rnd(100);

  for(uint k=0;k<20;k++){
    if(engine().shutdown.getValue()) break;
    for(uint i=0;i<N;i++) cout <<S.getVar<int>(i) <<' ';  cout <<endl;
    engine().step(S);
    MT::wait(.1);
  }

  engine().close(S);

  KeyValueGraph g = S.graph();
  GraphView gv(g);
  gv.watch();
}

//==============================================================================

int MAIN(int argc, char **argv) {
  testModuleSorter();
  return 0;
}

//==============================================================================

void PairSorter::open(){}
void PairSorter::close(){}

void PairSorter::step(){
  int xa = a.get();
  int xb = b.get();//->get_x(this);
  if(xa>xb){  //swap numbers
    a.writeAccess(); // we want to lock both to ensure no number is lost!
    b.writeAccess();
    xa = a();
    xb = b();
    if(xa>xb){  //still?
      a() = xb;
      b() = xa;
    }
    b.deAccess();
    a.deAccess();
  }
}
