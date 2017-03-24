#include <biros/biros.h>
#include <MT/util.h>

struct RandomNumber:public VariableData{
  mlr::Rnd rnd;
  uint getRnd(uint modulo){ writeAccess(NULL); uint i=rnd(modulo); deAccess(NULL); return i; }
  void seed(uint s){ rnd.seed(s); }
  RandomNumber():VariableData("RandomNumber"){}
};

struct Integer:public VariableData{
  int x;
  
  void init(RandomNumber &rnd){ x=rnd.getRnd(100); }
  Integer():VariableData("IntVar"){ x=0; reg_x(); }
};

struct PairSorter:public Process{
  mlr::Array<Integer*> vars;
  RandomNumber *rnd;

  PairSorter():Process("PairSorter"){};
  
  void open (){}
  void close(){}
  void step (){
    uint i=rnd->getRnd(vars.N);
    uint j=rnd->getRnd(vars.N);
    if(i==j) return;
    if(i>j){ uint k=i; i=j; j=k; }
    vars(i)->writeAccess(this); //WARNING: locking two mutexes can lead to a deadlock!!
    vars(j)->writeAccess(this); // ... only the strict ordering of i<j prevents this here!!
    if(vars(i)->x > vars(j)->x){ //swap numbers
      uint tmp=vars(i)->x;
      vars(i)->x=vars(j)->x;
      vars(j)->x=tmp;
    }
    vars(i)->deAccess(this);
    vars(j)->deAccess(this);
  }
};

int main(int argc, char **argv){
  uint nV=20;
  uint nP=10;
  uint nR=10;

  mlr::Array<Integer> ints(nV);
  mlr::Array<RandomNumber> rnds(nR);
  mlr::Array<PairSorter> sorts(nP);

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
  mlr::wait(.01);
  close(P);

  for(uint i=0;i<ints.N;i++) cout <<ints(i).get_x(NULL) <<' ';
  cout <<endl;
  
  return 0;
}
