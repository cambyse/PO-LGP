#include <Core/array.h>
#include <Core/util.h>
#include <Gui/plot.h>

double lambda = .5;
uint width=100;

arr generateRandomData(uint n=20, double sig=.03){
  struct RndPlane{
    double lo, hi, b0, b1;
    RndPlane(){
      lo = rnd.uni();
      hi = lo + .2*rnd.uni();
      b0 = rnd.uni(-1.,1.);
      b1 = 0.; //rnd.uni(-10.,10.);
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

arr xx, xy, g_xx, g_xy;

struct Cell{
  uint id;
  uint s;
  arr y;
  arr yf;
  arr xn;
  arr xhat;
  arr xm;

  arr phi; //location features
  arr xx_loc, xy_loc; //local statistics
  arr g_xx_loc, g_xy_loc; //local gradient
  arr g;
  arr invxx;
  arr msg;

  boolA equal;
  arr dEdbeta;
  arr beta, beta_sum, dEdbeta_sum;
  CellL neighbors;
  Cell *eq;



  Cell():eq(NULL){}

  void init(uint _id, const arr& _x, const CellL& _neighbors){
    id=_id;
    s = id;
    y = _x;
    neighbors = _neighbors;
    xn = zeros(2);
    xhat = zeros(2,y.N);
    xm = y;
    equal.resize(neighbors.N)=false;
    beta = cat({1.}, y);
    dEdbeta = zeros(beta.N);

    phi = ARR(1.); //features for constant modelling
    xx_loc = phi*~phi;
    xy_loc = phi*~y; //local contribution to the statistics
    msg = zeros(2,2*(xx_loc.N+xy_loc.N));
  }

  double E(){
    return sumOfSqr(beta-y);
  }
  arr dE_dbeta(){
    return 2.*(beta-y);
  }

  void comp_invxx(){  invxx = inverse_SymPosDef(xx[s]);  }
  arr f(){      yf = phi * invxx * xy[s]; return yf; }
  arr Jf_xx(){  return - phi * invxx * invxx * xy[s];  }
  arr Jf_xy(){  return phi * invxx;  }
  double d(){   return sumOfSqr(f() - y); }
  arr d_xx(){   return 2.*(f() - y)*Jf_xx(); }
  arr d_xy(){   return 2.*(f() - y)*Jf_xy(); }
  double costs(){
    int cuts=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=s) cuts++;
    }
    return d() + lambda*cuts;
  }

  double delta_E(Cell *s_new){
//    if(s_new->s == s) return 0.; TODO: check!

    uint r = s_new->s;

    arr delta_xx_new = xx_loc;
    arr delta_xy_new = xy_loc;

    arr delta_xx_old = -xx_loc;
    arr delta_xy_old = -xy_loc;

    int cuts_old=0, cuts_new=0;
    for(Cell *n:neighbors) if(n){
      if(n->s!=s) cuts_old++;
      if(n->s!=r) cuts_new++;
    }

    double deltaE=0.;
    deltaE += g_xx[r] * delta_xx_new + g_xy[r] * delta_xy_new; //adding the cell to the new segment
    deltaE += g_xx[s] * delta_xx_old + g_xy[s] * delta_xy_old; //removing the cell from the old segment
    deltaE += sumOfSqr(s_new->f()-y)  - sumOfSqr(f() - y); //change in cost arising by switching THIS cell alone;  TODO:this is only approximate!
    deltaE += lambda * (cuts_new - cuts_old);
    return deltaE;
  }

  void step_average(){
//    xm = zeros(x.N);
    double N=0.;
    for(uint i=0;i<neighbors.N;i++){
      Cell *n=neighbors(i);
      if(n){
        n->xhat[i] = xhat[i] + y;
        n->xn(i) = xn(i) + 1.;
        //      xm += xhat[i];
        //      N += xn(i);
      }
      if(n && equal(i)){
        n->xm = xm = .5*(xm + n->xm);
      }
    }
//    xm += x;
//    N += 1.;
//    xm /= N;
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
      xx[s]() -= xx_loc;
      xy[s]() -= xy_loc;
      s = s_new->s;
      xx[s]() += xx_loc;
      xy[s]() += xy_loc;
    }
  }

  void reaccumulate_grads(){
    comp_invxx();
    g_xx[s]() += d_xx();
    g_xy[s]() += d_xy();
  }



  void step_collectCumulates(){
    if(!eq) return;
    eq->beta_sum = cat({-1.},-y);
    eq->dEdbeta_sum = dE_dbeta();
    for(Cell *n:neighbors) if(n->eq==this){
      eq->beta_sum += n-> beta_sum;
      dEdbeta_sum += n->dEdbeta_sum;
    }
  }

  void step_propagateBeta(){
    if(!eq){ //root!
      beta = beta_sum;
      dEdbeta = dEdbeta_sum;
    }else{
      beta = eq->beta;
      dEdbeta = eq->dEdbeta;
    }
  }

  void step_switchNeighbor(){
    double dEzero=-lambda;
    arr dbeta;
    if(eq){
      //cell is linked to neighbor -> compute deletion
      dbeta = cat({-1.},-y);
      dEzero = ~eq->dEdbeta * dbeta;
    }
    arr dE = zeros(neighbors.N);
    dbeta = cat({1.},y);
    for(uint i=0;i<neighbors.N;i++){
      Cell *n = neighbors(i);
      dE(i) = ~n->dEdbeta * dbeta;
    }
    //choose best:
    uint j = argmin(dE);
    if(dEzero + dE(j) < 0.){ //energy would decrease
      if(eq) eq->beta -= dbeta;
      eq=neighbors(j);
      eq->beta += dbeta;
    }
  }

  void report(){
    cout <<id <<'[' <<(eq?eq->id:0) <<']'
        <<" x=" <<y
        <<" xm=" <<beta
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
    if(i && i<cells.N-1) cells(i).init(i, data[i], {&cells(i-1), &cells(i+1)});
    else if(i)  cells(i).init(i, data[i], {&cells(i-1), NULL});
    else        cells(i).init(i, data[i], {NULL, &cells(i+1)});
  }

  cout <<"avg=" <<sum(data)/(double)data.N <<endl;
  arr x(cells.N), e(cells.N);
  uintA s(cells.N);
  for(uint i=0;i<cells.N;i++) x(i) = cells(i).xm;
  cout <<x <<endl;

  xx = g_xx = zeros(cells.N, cells(0).xx_loc.d0,  cells(0).xx_loc.d1 );
  xy = g_xy = zeros(cells.N, cells(0).xy_loc.d0,  cells(0).xy_loc.d1 );
  for(uint i=0;i<cells.N;i++){ xx[i]() += cells(i).xx_loc; xy[i]() += cells(i).xy_loc; }


  for(uint k=0;k<15;k++){
//    for(Cell &c:cells) c.report();
    //  MT::wait();
//    for(Cell &c:cells) c.step_collectCumulates();
//    for(Cell &c:cells) c.step_propagateBeta();

//    for(Cell &c:cells) c.step_average();
//    for(Cell &c:cells) c.step_decide();
    for(uint i=0;i<cells.N;i++) { cells(i).comp_invxx();  x(i) = cells(i).f();  e(i) = cells(i).costs(); s(i) = cells(i).s; }
    cout <<x <<endl;
    cout <<e <<endl;
    cout <<sum(e) <<endl;
    cout <<s <<endl;
    plotClear();
    plotFunction(data);
    plotFunction(x);
    plot(false);
    MT::wait();

    g_xx.setZero();  g_xy.setZero();
    for(Cell &c:cells) c.reaccumulate_grads();
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
