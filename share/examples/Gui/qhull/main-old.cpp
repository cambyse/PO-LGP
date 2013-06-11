#ifdef OLD_CODE
struct Node{ MT::Array<double> point; arr& data(){ return point; } };
istream& operator>>(istream& is,Node& n){ return is >>n.point; }
ostream& operator<<(ostream& os,const Node& n){ return os <<n.point; }

int dummy;
struct Edge{ int& data(){ return dummy; } };
istream& operator>>(istream& is,Edge& e){ return is; }
ostream& operator<<(ostream& os,const Edge& e){ return os; }
void testDelaunay(){
  Parameter<uint> dim("dimensionality"),Npoints("Npoints");
  uint i;

  //an empty graph
  Graph<Node,Edge> g;
  Graph<Node,Edge>::node n;

  //generate Npoints nodes
  for(i=0;i<Npoints;i++){
    n=g.new_node();
    n->point.resize(dim);
    rndUni(n->point,-1.,1.,false);
    n->disp().x=n->point(0); n->disp().y=n->point(1); if(dim>2) n->disp().z=n->point(2);
    n->disp().size=.01;
  }

  cout <<"starting triangulation... ";  cout.flush(); MT::resetTimer();

  //triangulate
  delaunay(g,dim);

  cout <<MT::getTimer() <<"sec " <<g.E <<"edges" <<endl;

  OpenGL gl; gl.add(g); gl.watch();

  MT::save(g,"graph");
}
#endif
