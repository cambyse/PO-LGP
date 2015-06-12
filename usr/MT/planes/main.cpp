#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

double lambda = .05;
double lambda_reg = 1e-10;
uint width=100, height=0;

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
  MT::Array<RndPlane> planes(n);
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

struct Cell;
typedef MT::Array<Cell> CellA;
typedef MT::Array<Cell*> CellL;

struct Cell{
  uint id;
  Cell *s;

  arr phi; //location features
  arr X_loc; //local statistics
  arr X; //global statistics
  arr X_mean;
  double X_n;
  arr beta; //min eig vec
  double sig; //min eig value

  CellL neighbors;
  Cell *eq;



  Cell():eq(NULL){}

  void init(uint _id, const arr& _phi, const CellL& _neighbors){
    id=_id;
    s = this;
    neighbors = _neighbors;
    phi = _phi;
    X_loc = phi*~phi;
    resetStatistics();
    X += X_loc;
    X_mean = phi;
    X_n = 1.;
    sig=0.;
  }

  double E(){
    int cuts=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=s) cuts++;
    }
//    return MT::sqr(scalarProduct(s->beta,phi));// + lambda*cuts;
    return MT::sqr(f() - phi.last());// + lambda*cuts;
  }
  double f(){
    if(!s->beta.N) return 0.;
//    double y_pred = -(~phi.sub(0,-2)*s->beta.sub(0,-2)).scalar()/s->beta.last();
    arr mean=s->X_mean/s->X_n;
    double y_pred = mean.last();
    if(mean.N>1) y_pred -= (~(phi.sub(0,-2)-mean.sub(0,-2))*s->beta.sub(0,-2))/s->beta.last();
    return y_pred;
  }

  arr dE_dbeta(){
    return 2.*scalarProduct(s->beta,phi)*phi;
  }

  double delta_E_switch(Cell *s_new){
//    if(s_new->s == s) return 0.; TODO: check!

    arr delta_X_new = X_loc;
    arr delta_X_old = -X_loc;

    int cuts_old=0, cuts_new=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=this->s) cuts_old++;
      if(n->s!=s_new->s) cuts_new++;
    }

    double deltaE=0.;
    deltaE += MT::sqr(scalarProduct(phi-s_new->s->X_mean/s_new->s->X_n, s_new->s->beta)); //adding the cell to the new segment
    deltaE -= MT::sqr(scalarProduct(phi-this ->s->X_mean/this ->s->X_n, this ->s->beta)); //removing the cell from the old segment
    deltaE += lambda * (cuts_new - cuts_old);
    return deltaE;
  }

  double delta_E_fuse(Cell *b){
//    if(s_new->s == s) return 0.; TODO: check!

//    if(b->X_n<2 || s->X_n<2) return 1.;

    arr Xc = s->X + b->X;
    arr muc = s->X_mean + b->X_mean;
    double nc = s->X_n + b->X_n;

    arr betac = s->X_n*s->beta + b->X_n*b->beta;
    betac /= length(betac);
    double lambdac = length( (Xc - nc*(muc*~muc)) * betac);

    double lambdaa = length( (s->X - s->X_n*(s->X_mean*~s->X_mean)) * s->beta);
    double lambdab = length( (b->X - b->X_n*(b->X_mean*~b->X_mean)) * b->beta);

    double errold = lambdaa+lambdab;
    return lambdac - errold - lambda*10;
  }

  void step_decide(uint t){
#if 0
    arr deltaE = zeros(neighbors.N);
    for(uint i=0;i<neighbors.N;i++) if(neighbors(i)) deltaE(i) = delta_E(neighbors(i));
    Cell *s_new = neighbors(argmin(deltaE));
#endif
    Cell *s_new = neighbors(t%neighbors.N);
    if(!s_new || s_new->s==s) return;

    double deltaE = delta_E_switch(s_new);
    if(deltaE<0.){ //we switch!
      s->X -= X_loc;
      s = s_new->s;
      s->X += X_loc;
      return;
    }

    deltaE = delta_E_fuse(s_new->s);
    if(deltaE<0.){ //we fuse!
      cout <<"BLA" <<endl;
      s->X -= X_loc;
      s = s_new->s;
      s->X += X_loc;
    }
  }

  void recomputeEig(){
    if(s==this){
      arr Sig, Beta;
      lapack_EigenDecomp(X - X_mean*~(X_mean/X_n), Sig, Beta);
      sig = Sig(0);
      if(sig>1e-10) beta = Beta[0];
      else{ beta = zeros(phi.N); beta.last()=1.; }
//      cout <<"compEig: " <<id <<':' <<Sig <<':' <<Beta <<endl;
    }
  }

  void resetStatistics(){
    X_mean = zeros(phi.N);
    X_n = 0.;
    X = lambda_reg * eye(phi.N);
    X(0,0) = X(phi.N-1,phi.N-1) = 0.;
  }

  void resubmitStatistics(){
    s->X_n += 1.;
    s->X_mean += phi;
    s->X += X_loc;
  }

};

void planes(){
  auto data = generateRandomData();
  arr &X=data.first, &Y=data.second;
  FILE("z.X") <<X;
  FILE("z.data") <<Y;
  gnuplot(STRING("splot 'z.data' matrix us ($1/15-1):($2/15-1):3 w l"), false, true);
//  Y.reshape(Y.N);
  X.reshape(Y.d0,Y.d1,X.d1);
  Y.reshape(Y.d0,Y.d1,1);

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
    MT::wait(1.5);
//    MT::wait();

    for(uint i=0;i<cells.N;i++) if(!((i+k)%3)) cells.elem(i).step_decide(k);
//    for(uint i=0;i<cells.N;i++) cells.elem(i).step_decide(k);
  }


}

int main(int argc,char **argv){
  rnd.clockSeed();
  planes();
}
