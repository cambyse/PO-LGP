#include <Core/module.h>
#include <Gui/graphview.h>


struct PairSorter:Module{
  Access_typed<int> a;
  Access_typed<int> b;
  PairSorter(const char *a_name, const char* b_name)
    : Module(STRING("S_"<<a_name<<"_"<<b_name)),
      a(this, a_name),
      b(this, b_name){}
  PairSorter():Module("S"), a(this, "a"), b(this, "b"){}
  void open(){}
  void close(){}
  void step();
};


REGISTER_MODULE(PairSorter)

//==============================================================================

void TEST(ModuleSorter){
  uint N=20;

  //engine().enableAccessLog();
  ////engine().mode=Engine::serial;
  //engine().mode=Engine::threaded;

  cout <<registry() <<endl <<"----------------------------" <<endl;
#if 0
  for(uint i=0;i<N;i++) S.addVariable<int>(STRING("int"<<i));
  for(uint i=1;i<N;i++) S.addModule("PairSorter", STRING("S"<<i-1), {i-1, i});
#else
  for(uint i=1;i<N;i++) new PairSorter( STRING("int"<<i-1), STRING("int"<<i) );
#endif
  cout <<registry() <<endl <<"----------------------------" <<endl;
  auto vars = registry().getTypedValues<Variable<int> >();

  threadOpenModules(true);

  //new InsideOut();                 //create an explicit view


  for(uint i=0;i<N;i++) vars(i)->set() = mlr::rnd(100);

  for(uint k=0;k<20;k++){
    if(shutdown().getValue()) break;
    for(uint i=0;i<N;i++) cout <<vars(i)->get() <<' ';  cout <<endl;
    stepModules();
    mlr::wait(.1);
  }

  threadCloseModules();

//  Graph g = S.graph();
//  GraphView gv(g);
//  gv.watch();
}

//==============================================================================

void TEST(ModuleSorter2){
  uint N=20;

  mlr::Array<PairSorter*> ps;

  for(uint i=0;i<N-1;i++)
    ps.append(new PairSorter(STRING("int"<<i), STRING("int"<<i+1)) );

  cout <<registry() <<endl;

  for(uint i=0;i<N-1;i++) ps(i)->a.set() = mlr::rnd(100);
  ps.last()->b.set() = mlr::rnd(100);

  openModules();

  for(uint k=0;k<20;k++){
    for(uint i=0;i<N-1;i++) cout <<ps(i)->a.get() <<' ';  cout <<endl;
    stepModules();
    mlr::wait(.1);
  }

  closeModules();
}

//==============================================================================

int MAIN(int argc, char **argv) {
//  testModuleSorter();
  testModuleSorter2();
  return 0;
}

//==============================================================================


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
