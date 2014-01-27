#if 0
struct Simulator{
  static bool checkValid(doubleA& s){
    static Parameter<MT::Array<int> > maze("maze");
    double x=.5+.5*s(0),y=.5+.5*s(1);
    return !maze()((int)(x*maze().d0),(int)(y*maze().d1));
  }
  static void step(const doubleA& in,const doubleA& goal,doubleA& out){
    doubleA del=goal-in;
    del*=.1/length(del);
    out=in+del;
  }
  static void stepBack(const doubleA& in,const doubleA& goal,doubleA& out){
    doubleA del=goal-in;
    del*=.1/length(del);
    out=in+del;
  }
};

void planSpace(){
  Planner p;
  doubleA a(2),b(2);
  do{ rndUni(a,-1,1); }while(!Simulator::checkValid(a));
  do{ rndUni(b,-1,1); }while(!Simulator::checkValid(a));
  a=.8; b=-.8;
  p.plan(a,b,Simulator());
}
#endif

void planLine(bool line=false){
  Planner p;
  Planner::node a,b;
  if(line){
    p.G <<FILE("line.graph");
    a=p.G.first; b=p.G.last;
  }else{
    p.G <<FILE("maze.graph");
    a=p.G.rndNode(); b=p.G.rndNode();
  }
  p.plan(a,b);
  p.browsePlans(a,b);
}
