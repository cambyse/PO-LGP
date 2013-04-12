#include <system/module.h>
#include <system/engine.h>
#include <MT/graphview.h>
#include <biros/biros_views.h>

BEGIN_MODULE(PairSorter)
ACCESS(int, a)
ACCESS(int, b)
END_MODULE()

int main(int argn, char **argv) {
  uint N=20;

  engine().enableAccessLog();
  //engine().mode=Engine::serial;
  engine().mode=Engine::threaded;

  cout <<registry() <<endl <<"----------------------------" <<endl;
  SystemDescription S;
  for(uint i=0;i<N;i++) S.addVar<int>(STRING("int"<<i));
  for(uint i=1;i<N;i++) S.addModule("PairSorter", STRING("S"<<i-1), ARRAY(S.getVar(i-1), S.getVar(i)));
  S.report();

  engine().create(S);

  new InsideOut();                 //create an explicit view


  for(uint i=0;i<N;i++) S.getValue<int>(i) = MT::rnd(100);

  for(uint k=0;k<20;k++){
    for(uint i=0;i<N;i++) cout <<S.getValue<int>(i) <<' ';  cout <<endl;
    engine().step(S);
    MT::wait(.1);
  }

  GraphView gv(S.system);
  gv.watch();
  return 0;
}

struct PairSorter:PairSorter_Base{
  void step(){
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
};
