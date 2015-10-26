#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>
#include <Gui/graphview.h>
#include <Core/graph.h>

double lambda = .02;
double lambda_reg = 1e-10;
uint width=30, height=30;

std::pair<arr,arr> generateRandomData(uint n=10, double sig=.03){
  struct RndPlane{
    double xlo, xhi, ylo, yhi, b0, b1, b2;
    RndPlane(){
      xlo = rnd.uni(-1.,1.);
      xhi = xlo + 1.*rnd.uni();
      if(height){
        ylo = rnd.uni(-1.,1.);
        yhi = ylo + 1.*rnd.uni();
      }else{
        ylo = -10.; yhi = 10.;
      }
      b0 = rnd.uni(-1.,1.);
      b1 = rnd.uni(-2.,2.);
      b2 = rnd.uni(-2.,2.);
    }
  };
  mlr::Array<RndPlane> planes(n);
  arr X = grid({-1.,-1.}, {1.,1.}, {height,width});
  arr Y(X.d0);
  for(uint i=0;i<Y.N;i++){
    double y=X(i,0), x=X(i,1);
    double z=0.,zp;
    for(RndPlane& p:planes){
      if(x>=p.xlo && x<=p.xhi && y>=p.ylo && y<=p.yhi){
        zp = p.b0 + p.b1*x + p.b2*y;
        if(zp>z) z=zp;
      }
    }
    Y(i)=z;
  }
  rndGauss(Y, sig, true);
  Y.reshape(height+1,width+1);
  return {X,Y};
}

struct Segment{
  //pixels
  uintA pix;
  //statistics
  double n;
  arr X, mu;
  //minimal eigenvector
  arr beta;
  double beta_len;

  void collect(const arr& Phi){
    n=0.;
    mu = zeros(Phi.d1);
    X  = lambda_reg * eye(Phi.d1);
    for(uint &i:pix){
      n++;
      mu+=Phi[i];
      X +=Phi[i]*~Phi[i];
    }
  }

  void comBeta(){
    arr Sig, Beta;
    lapack_EigenDecomp(X - mu*~(mu/n), Sig, Beta);
    beta_len = Sig(0);
    if(beta_len>1e-10) beta = Beta[0];
    else{ beta = zeros(mu.N); beta.last()=1.; }
    //      cout <<"compEig: " <<id <<':' <<Sig <<':' <<Beta <<endl;
    if(beta_len>1e-2) CHECK_ZERO(beta_len - length((X - mu*~(mu/n))*beta), 1e-6, "");
  }

  double f(const arr& phi){
    if(!beta.N) return 0.;
//    double y_pred = -(~phi.sub(0,-2)*s->beta.sub(0,-2)).scalar()/s->beta.last();
    arr mean=mu/n;
    double y_pred = mean.last();
    if(mu.N>1 && beta_len>1e-10 && beta.last()>1e-10) y_pred -= (~(phi.sub(0,-2)-mean.sub(0,-2))*beta.sub(0,-2))/beta.last();
    return y_pred;
  }

  void write(ostream& os) const{ os <<"Segment"; }
};
stdOutPipe(Segment)
\
typedef mlr::Array<Segment*> SegmentL;

double delta_E_fuse(Segment& a, Segment& b){
  arr cX = a.X + b.X;
  arr cmu = a.mu + b.mu;
  double cn = a.n + b.n;

#if 0
  arr betac = a.n*a.beta + b.n*b.beta;
  betac /= length(betac);
  double betac_len = length( (Xc - muc*~(muc/nc)) * betac);
#else
  arr Sig, Beta;
  lapack_EigenDecomp(cX - cmu*~(cmu/cn), Sig, Beta);
  double cbeta_len = Sig(0);
#endif

  return cbeta_len - a.beta_len - b.beta_len;
}

template<class V, class E> void makeGridGraph(Graph& G, uint h, uint w){
  for(uint i=0;i<h;i++) for(uint j=0;j<w;j++){
    new Node_typed<Segment>(G, {STRING(i<<'_'<<j)}, {}, new V(), true);
  }
  NodeL verts = G.list();
  verts.reshape(h,w);
  for(uint i=0;i<h;i++) for(uint j=0;j<w;j++){
    if(i) new Node_typed<E>(G, {}, {verts(i-1,j), verts(i,j)}, new E(), true);
    if(j) new Node_typed<E>(G, {}, {verts(i,j-1), verts(i,j)}, new E(), true);
  }
}

double cost(Graph& S){
  double E=0.;
  for(Node *s:S.getTypedNodes<Segment>()){
    E += s->V<Segment>().beta_len;
  }
  for(Node *e:S.getTypedNodes<uint>()){
    E += lambda*e->V<uint>();
  }
  return E;
}

void fuse(Graph& S, Node *ita, Node *itb){
//  Node *e=S.getChild(ita,itb);
  Segment &a=ita->V<Segment>();
  Segment &b=itb->V<Segment>();
  a.X += b.X;
  a.n += b.n;
  a.mu += b.mu;
  a.pix.append(b.pix);
  a.comBeta();
  for(Node *itc:neighbors(itb)) if(itc!=ita){
    uint nbc = S.getChild(itb,itc)->V<uint>();
    Node *ea=S.getChild(ita,itc);
    if(ea) ea->V<uint>() += nbc;
    else new Node_typed<uint>(S, {}, {ita, itc}, new uint(nbc), true);
  }
  while(itb->parentOf.N) delete itb->parentOf.last();
  delete itb;
}

void planes(){
  auto data = generateRandomData();
  arr &X=data.first, &Y=data.second;
  Y *= .01;
  FILE("z.X") <<X;
  FILE("z.data") <<Y;
  gnuplot(STRING("splot 'z.data' matrix us ($1/15-1):($2/15-1):3 w l"), false, true);
//  Y.reshape(Y.N);
//  X.reshape(Y.d0,Y.d1,X.d1);
//  Y.reshape(Y.d0,Y.d1,1);

  Graph S;
  makeGridGraph<Segment, uint>(S, Y.d0, Y.d1);
  cout <<S <<endl;
  S.writeDot(FILE("z.dot"), false, true);

  for(uint i=0;i<X.d0;i++) S(i)->V<Segment>().pix.append(i);
  for(uint *e:S.getTypedValues<uint>()) *e=1;

  arr Phi;
  for(uint i=0;i<X.d0;i++){
    arr phi;
    if(height){ phi=X[i]; phi.append(Y.elem(i)); }
    else phi={X(i,1), Y.elem(i)};
    Phi.append(phi);
  }
  Phi.reshape(X.d0 , Phi.N/X.d0);

  for(Segment *s:S.getTypedValues<Segment>()){ s->collect(Phi); s->comBeta(); }
  cout <<"E=" <<cost(S) <<endl;


  for(uint k=0;k<10000;k++){
    double Eold = cost(S);
    Node *e = S.getTypedNodes<uint>().rndElem();
    double deltaE = delta_E_fuse(e->parents(0)->V<Segment>(), e->parents(1)->V<Segment>());
    deltaE -= lambda*e->V<uint>();
    if(deltaE<0.){
      fuse(S, e->parents(0), e->parents(1));
      double Enew = cost(S);
      cout <<"fuse: dE=" <<deltaE <<" err=" <<Eold+deltaE-Enew <<endl;
      for(Segment *s:S.getTypedValues<Segment>()){ s->collect(Phi); s->comBeta(); }
      cout <<"E=" <<cost(S) <<endl;
    }

    arr img(X.d0);  img=-1.;
    uintA seg(X.d0);
    for(Node *sit:S.getTypedNodes<Segment>()){
      Segment *s = sit->getValue<Segment>();
      for(uint p:s->pix){
        img(p) = s->f(Phi[p]);
        seg(p) = sit->index;
      }
    }

    if(!(k%100)){
      FILE("z.model") <<img.reshape(Y.d0,Y.d1);
      FILE("z.seg") <<seg.reshape(Y.d0,Y.d1);
      gnuplot("splot 'z.model' matrix w l lw 2, 'z.data' matrix w l", false, true);
      mlr::wait();
    }

//    , 'z.data' matrix us ($1/15-1):($2/15-1):3 w l, 'z.model' matrix us ($1/15-1):($2/15-1):3 w l
  }
  S.writeDot(FILE("z.dot"), false, true);

  return;

#if 0
  CellA cells(Y.d0, Y.d1);
  for(uint i=0;i<cells.d0;i++) for(uint j=0;j<cells.d1;j++){
    arr phi;
    if(height) phi=cat(X.subDim(i,j), Y.subDim(i,j));
    else phi={X(i,j,1), Y(i,j,0)};
    CellL neighbors;
    if(i) neighbors.append(&cells(i-1,j));
    if(j) neighbors.append(&cells(i,j-1));
    if(i<Y.d0-1) neighbors.append(&cells(i+1,j));
    if(j<Y.d1-1) neighbors.append(&cells(i,j+1));
    cells(i,j).init(cells.d1*i+j, phi, neighbors);
  }

//  for(Cell &c:cells) c.s=&cells.elem(13);
  for(Cell &c:cells){ c.resetStatistics(); }
  for(Cell &c:cells){ c.resubmitStatistics(); }
  for(Cell &c:cells){ c.recomputeEig(); }

  cout <<"avg=" <<sum(Y)/(double)Y.N <<endl;
  arr x(Y.d0,Y.d1), e(Y.d0,Y.d1);
  uintA s(Y.d0,Y.d1);
  for(uint i=0;i<cells.N;i++) x.elem(i) = cells.elem(i).f();
  cout <<x <<endl;

  for(uint k=0;k<15;k++){
    for(Cell &c:cells){ c.resetStatistics(); }
    for(Cell &c:cells){ c.resubmitStatistics(); }
    for(Cell &c:cells){ c.recomputeEig(); }

    for(uint i=0;i<cells.N;i++) { x.elem(i) = cells.elem(i).f();  e.elem(i) = cells.elem(i).E(); s.elem(i) = cells.elem(i).s->id; }
//    cout <<"x=" <<x <<endl;
//    cout <<"e=" <<e <<endl;
    cout <<sum(e) <<endl;
//    cout <<"s=" <<s <<endl;
//    for(Cell &c:cells) if(c.s==&c){
//      cout <<c.id <<':' <<c.X <<':' <<c.sig <<endl;
//    }
    if(cells.d0>1){
      FILE("z.model") <<x;
      FILE("z.seg") <<s;
      gnuplot(STRING("splot [:][:][-0:2] 'z.seg' matrix us ($1/15-1):($2/15-1):($3/30) with image, 'z.data' matrix us ($1/15-1):($2/15-1):3 w l, 'z.model' matrix us ($1/15-1):($2/15-1):3 w l"), false, true);
    }else{
      plotClear();
      plotFunction(Y.reshape(Y.N));
      plotFunction(x.reshape(x.N));
      plot(false);
    }
    mlr::wait(1.5);
//    mlr::wait();

    for(uint i=0;i<cells.N;i++) if(!((i+k)%3)) cells.elem(i).step_decide(k);
//    for(uint i=0;i<cells.N;i++) cells.elem(i).step_decide(k);
  }
#endif

}

int main(int argc,char **argv){
  rnd.clockSeed();
  planes();
}
