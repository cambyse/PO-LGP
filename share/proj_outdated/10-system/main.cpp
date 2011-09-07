#define MT_IMPLEMENTATION

#include<MT/isystem.h>

struct Scalar:public Representation{
  double x;
  Scalar(const char* name):Representation(name){ x=MT::rnd.uni(0.,1.); };

  void _read (istream& is){ is >>x; }
  void _write(ostream& os){ os <<x; }
};

struct Factor:public Process{
  double x,y,w;
  bool listen;
  Factor(Scalar &_x,Scalar &_y,const char *name,bool _listen):Process(name){
    R=LIST<Representation>(_x,_y);
    w=MT::rnd.uni(0.,1.);
    w=.1;
    listen=_listen;
  };

  void _open (){ cout <<name <<" is opening, w=" <<w <<endl; }
  void _close(){ cout <<name <<" is closing " <<endl; }

  double _f(){
    pullCopies();
    return x*x + (y-1.)*(y-1.) + w*x*y;
  }
  void _step (){
    if(!listen){
      MT::wait(.01);
    }else{
      R(0)->waitForConditionSignal(); //wait for signal
    }
    pullCopies();
    //a gradient step
    x -= .1 * (2.*x + w*y);
    y -= .1 * (2.*(y-1.) + w*x);
    pushCopies();
    if(!listen) R(1)->setCondition(1); //send a signal
  }
  void pullCopies(){
    R(0)->readAccess(this); x=((Scalar*)R(0))->x; R(0)->deAccess(this);
    R(1)->readAccess(this); y=((Scalar*)R(1))->x; R(1)->deAccess(this);
  }
  void pushCopies(){
    R(0)->writeAccess(this); ((Scalar*)R(0))->x=x; R(0)->deAccess(this);
    R(1)->writeAccess(this); ((Scalar*)R(1))->x=y; R(1)->deAccess(this);
  }
};

int main(int argc, char *argv[]){
  Scalar x("var x"),y("var y"),z("var z");
  Factor f1(x,y,"f1",false),f2(y,z,"f2",true);
  Group G;
  G.set(LIST<Representation>(x,y,z), LIST<Process>(f1,f2));
  G.loop();

  MT::wait(10);

  G.close();

  return 0;
}

