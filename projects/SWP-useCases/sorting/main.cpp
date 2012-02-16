#include <MT/process.h>
#include <MT/util.h>

struct RandomNumber:public Variable{
  MT::Rnd rnd;
  uint getRnd(uint modulo){ writeAccess(NULL); uint i=rnd(modulo); deAccess(NULL); return i; }
  void seed(uint s){ rnd.seed(s); }
  RandomNumber():Variable("RandomNumber"){}
};

struct Integer:public Variable{
  FIELD(int, x);
  
  void init(RandomNumber &rnd){ x=rnd.getRnd(1000); }
  Integer():Variable("IntVar"){ x=0; reg_x(); }
};

struct PairSorter:public Process{
  MT::Array<Integer*> vars;
  RandomNumber *rnd;

  PairSorter():Process("PairSorter"){};
  
  void open (){}
  void close(){}
  void step (){
    uint i=rnd->getRnd(vars.N);
    uint j=rnd->getRnd(vars.N);
    vars(i)->writeAccess(this);
    vars(j)->writeAccess(this);
    if((i<j) ^ (vars(i)->x > vars(j)->x)){ //XOR
      //swap numbers
      uint tmp=vars(i)->x;
      vars(i)->x=vars(j)->x;
      vars(j)->x=tmp;
    }
    vars(i)->deAccess(this);
    vars(j)->deAccess(this);
  }
};

int main(int argn, char **argv){
  uint nV=100;
  uint nP=20;
  uint nR=10;

  MT::Array<Integer> ints(nV);
  MT::Array<RandomNumber> rnds(nR);
  MT::Array<PairSorter> sorts(nP);

  //initialize stuff
  ProcessL P;
  for(uint i=0;i<nR;i++) rnds(i).seed(i);
  for(uint i=0;i<nV;i++) ints(i).init(rnds(i%nR));
  for(uint i=0;i<nP;i++){
    sorts(i).vars = LIST(ints);
    sorts(i).rnd = &rnds(i%nR);
    P.append(&sorts(i));
  }

  //run
  loop(P);
  MT::wait();
  stop(P);

  return 0;
}
