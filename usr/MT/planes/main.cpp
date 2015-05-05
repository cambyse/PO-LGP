#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

double lambda = .05;
double lambda_reg = 1e-10;
uint width=100;

arr generateRandomData(uint n=20, double sig=.03){
  struct RndPlane{
    double lo, hi, b0, b1;
    RndPlane(){
      lo = rnd.uni();
      hi = lo + .2*rnd.uni();
      b0 = rnd.uni(-1.,1.);
      b1 = rnd.uni(-10.,10.);
      b0-= rnd.uni()*b1;
    }
  };
  MT::Array<RndPlane> planes(n);
  arr Y(width+1);
  for(uint i=0;i<Y.N;i++){
    double x=(double)i/(Y.N-1);
    double y=0.,yp;
    for(RndPlane& p:planes){
      if(x>p.lo && x<=p.hi){
        yp = p.b0 + p.b1*x;
        if(yp>y) y=yp;
      }
    }
    Y(i)=y;
  }
  rndGauss(Y, sig, true);
  Y.reshape(Y.N,1);
  return Y;
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
  arr beta; //min eig vec
  double sig; //min eig value

  CellL neighbors;
  Cell *eq;



  Cell():eq(NULL){}

  void init(uint _id, const arr& _phi, const CellL& _neighbors){
    id=_id;
    s = this;
    neighbors = _neighbors;

//    phi = cat(ARR(1.), y); //features for constant modelling
    phi = _phi;
    X = X_loc = phi*~phi;
    arr reg = eye(phi.N);
    reg(0,0) = reg(phi.N-1,phi.N-1) = 0.;
    X += lambda_reg * reg;
    sig=0.;
  }

  double E(){
    int cuts=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=s) cuts++;
    }
    return MT::sqr(scalarProduct(s->beta,phi)) + lambda*cuts;
  }
  double f(){
    if(!s->beta.N) return 0.;
    double y_pred = -(~phi.sub(0,-2)*s->beta.sub(0,-2)).scalar()/s->beta.last();
    return y_pred;
  }

  arr dE_dbeta(){
    return 2.*scalarProduct(s->beta,phi)*phi;
  }

//  void comp_invxx(){  invxx = inverse_SymPosDef(xx[s]);  }
//  arr f(){      yf = phi * invxx * xy[s]; return yf; }
//  arr Jf_xx(){  return - phi * invxx * invxx * xy[s];  }
//  arr Jf_xy(){  return phi * invxx;  }
//  double d(){   return sumOfSqr(f() - y); }
//  arr d_xx(){   return 2.*(f() - y)*Jf_xx(); }
//  arr d_xy(){   return 2.*(f() - y)*Jf_xy(); }
//  double costs(){
//    int cuts=0;
//    for(Cell *n:neighbors) if(n){
//      if(n->s!=s) cuts++;
//    }
//    return d() + lambda*cuts;
//  }

  double delta_E(Cell *s_new){
//    if(s_new->s == s) return 0.; TODO: check!

    arr delta_X_new = X_loc;
    arr delta_X_old = -X_loc;

    int cuts_old=0, cuts_new=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=this->s) cuts_old++;
      if(n->s!=s_new->s) cuts_new++;
    }

    double deltaE=0.;
    deltaE += MT::sqr(scalarProduct(phi, s_new->s->beta)); //adding the cell to the new segment
    deltaE -= MT::sqr(scalarProduct(phi, this ->s->beta)); //removing the cell from the old segment
    deltaE += lambda * (cuts_new - cuts_old);
    return deltaE;
  }

  void step_decide(uint t){
#if 0
    arr deltaE = zeros(neighbors.N);
    for(uint i=0;i<neighbors.N;i++) if(neighbors(i)) deltaE(i) = delta_E(neighbors(i));
    Cell *s_new = neighbors(argmin(deltaE));
#endif
    Cell *s_new = neighbors(t%neighbors.N);
    if(!s_new || s_new->s==s) return;
    double deltaE = delta_E(s_new);
    if(deltaE<0.){ //we switch!
      s->X -= X_loc;
      s = s_new->s;
      s->X += X_loc;
    }
  }

  void recomputeEig(){
    if(s==this){
      arr Sig, Beta;
      lapack_EigenDecomp(s->X, Sig, Beta);
      sig = Sig(0);
      beta = Beta[0];
    }
  }

  void report(){
    cout <<id <<'[' <<(eq?eq->id:0) <<']'
//        <<" x=" <<y
        <<" xm=" <<s->beta
//       <<" dEdbeta=" <<dEdbeta
//      <<" beta_sum=" <<beta_sum
//     <<" dEdbeta_sum=" <<dEdbeta_sum
    <<endl;
  }
};

void planes(){
  arr data = generateRandomData();
  plotFunction(data);
  plotGnuplot();
  plot(false);
//  FILE("z.data") <<data;
//  gnuplot("plot 'z.data' us 0:1", true);


  CellA cells(data.N);
  for(uint i=0;i<cells.N;i++){
    arr phi=cat({1.},{double(i)/(cells.N-1)}, data[i]);
    if(i && i<cells.N-1) cells(i).init(i, phi, {&cells(i-1), &cells(i+1)});
    else if(i)  cells(i).init(i, phi, {&cells(i-1), NULL});
    else        cells(i).init(i, phi, {NULL, &cells(i+1)});
  }

  cout <<"avg=" <<sum(data)/(double)data.N <<endl;
  arr x(cells.N), e(cells.N);
  uintA s(cells.N);
  for(uint i=0;i<cells.N;i++) x(i) = cells(i).f();
  cout <<x <<endl;

  for(uint k=0;k<15;k++){
//    for(Cell &c:cells) c.report();
    //  MT::wait();
//    for(Cell &c:cells) c.step_collectCumulates();
//    for(Cell &c:cells) c.step_propagateBeta();

//    for(Cell &c:cells) c.step_average();
//    for(Cell &c:cells) c.step_decide();
    for(uint i=0;i<cells.N;i++) { cells(i).recomputeEig();  x(i) = cells(i).f();  e(i) = cells(i).E(); s(i) = cells(i).s->id; }
    cout <<"x=" <<x <<endl;
    cout <<"e=" <<e <<endl;
    cout <<sum(e) <<endl;
    cout <<"s=" <<s <<endl;
    plotClear();
    plotFunction(data);
    plotFunction(x);
    plot(false);
    MT::wait();

    for(Cell &c:cells) c.step_decide(k);

    //    for(Cell &c:cells) c.report();
//    for(Cell &c:cells) c.step_collectCumulates();
  }
  //  MT::wait();


}

int main(int argc,char **argv){
  rnd.clockSeed();
  planes();
}
