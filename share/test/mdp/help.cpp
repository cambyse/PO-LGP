//#include <MT/image.h>

#if 0
struct CopyStruct{
  static void copy(Planner::Node& nn,const NodeNG& n){ nn.p=n.weights; }
  static void copy(Planner::Edge& nn,const EdgeNG& n){}
};
void makeMazeGraph(){
  Parameter<MT::Array<int> > maze("maze");
  Parameter<uint> mazeN("mazeN"),growtime("growtime");;
  NeuralGas ng;
  MT::Array<double> s; s.resize(2);
  Planner p;
  OpenGL gl; gl.add(ng);
  OpenGL gl2; gl2.add(p.G);
  uint t;
  for(t=0;t<growtime;t++){
    rndUni(s);
    if(!maze()((int)(s(0)*maze().d0),(int)(s(1)*maze().d1))){
      ng.learn(2.*s-1.);
      ng.layout();
      gl.display();
    }
  }
  p.G.setForeignGraph(ng,CopyStruct());
  p.layout();
  gl2.watch();
  MT::save(p.G,"maze.graph");
}

void makeLineGraph(){
  Planner p;
  Planner::node n;
  for(uint i=0;i<20;i++){
    n=p.G.new_node();
    n->p.resize(2);
    n->p(0)=(double)i/10.-1.; n->p(1)=0.;
    if(i){
      p.G.new_edge(n,n->prev);
      p.G.new_edge(n->prev,n);
    }
  }
  OpenGL gl; gl.add(p.G);
  p.layout();
  gl.watch();
  MT::save(p.G,"line.graph");
}
#endif

static double mazeDX;

void imageToGraph(char* filename,Graph<Planner::Node,Planner::Edge>& G){
  byteA img0,img;
  read_ppm(img0,filename);
  img=img0;
  make_grey(img);
  //img.display(10.f); MT::wait();
  G.clear();
  Graph<Planner::Node,Planner::Edge>::node n,ns;
  Graph<Planner::Node,Planner::Edge>::edge e; //,e2;
  uint i,x,y,X=img.d1,Y=img.d0,M=(X>Y?X:Y); mazeDX=1./(X-1);
  double Z;
  //uniform maze with self-links
  for(y=0;y<Y;y++) for(x=0;x<X;x++){
    n=G.new_node();
    n->disp().x=x;
    n->disp().y=y;
    G.new_edge(n,n);
    if(x){
      G.new_edge(n,n->prev);
      G.new_edge(n->prev,n);
    }
    if(y){
      G.new_edge(n,G.nodes((y-1)*X+x));
      G.new_edge(  G.nodes((y-1)*X+x),n);
    }
  }
  //make dark locations traps
  forNodes_save(n,ns,G){
    if(img((uint)n->disp().x,(uint)n->disp().y)<128){
      G.del_out_edges(n); G.new_edge(n,n);
      n->disp().hide=true;
      for_in_edges(e,n){
	e->disp().hide=true;
	//for_out_edges(e2,e->from) e2->npi=1./(1+e->from->Nout); //*=.5;
	//e->npi=2./(1+e->from->Nout);
      }
    }
  }
  //set noise transition probs
  forEdges(i,e,G){
    e->npi=1./e->from->Nout;
  }
  //check normalization
  forNodes(n,G){
    Z=0.;
    for_out_edges(e,n) Z+=e->npi;
    CHECK(-1e-6<Z-1. && Z-1.<1e-6,"not normalized:" <<Z);
  }
  forNodes(n,G){
    n->disp().x=-1.+2.*n->disp().x/(M-1);
    n->disp().y=-1.+2.*n->disp().y/(M-1);
  }
  //OpenGL gl; gl.add(G); gl.text="loaded maze"; gl.watch();
}

