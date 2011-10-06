#include "MinSumGaussNewton.h"

double MinSumGaussNewton::f(uint i, uint j, const arr& x_i, const arr& x_j){
  arr psi, psiI, psiJ;
  if(j<=i) Psi(psi, psiI, psiJ, i, j, x_i, x_j);
  else     Psi(psi, psiJ, psiI, j, i, x_j, x_i);
  return sumOfSqr(psi);
}

void MinSumGaussNewton::reapproxPotentials(uint i, const arr& x_i){
  uint k, m, j;
  arr psi, psiI, psiJ;
  VERBOSE(2, cout <<"reapproximating potentials at node " <<i <<" at " <<x_i <<endl);
  for(k=0; k<del(i).N; k++){
    m=del(i)(k);
    CHECK(E(m, 1)==i, "");
    j=E(m, 0);
    if(j==i){ //node potential
      Psi(psi, psiI, psiJ, i, j, x_i, x[j]);
      fij(m).A=~psiI * psiI;
      fij(m).a=~psiI * (psiI*x_i - psi);
      fij(m).hata=sumOfSqr(psiI*x_i - psi);
      //cout <<fij(m).A <<fij(m).a <<fij(m).hata <<endl;
      //cout <<sumOfSqr(psi) <<endl;
      //cout <<(~x_i * fij(m).A * x_i - 2.*~(fij(m).a)*x_i)(0) + fij(m).hata <<endl;
      fij(m).B.clear();  fij(m).C.clear();  fij(m).b.clear();
    }else{ //pair potential
      /* INEFFICIENCY: when you update fij, you should also update 'fji' because that's
         just the transpose of the other */
      if(j<i) Psi(psi, psiI, psiJ, i, j, x_i, x[j]);
      else    Psi(psi, psiJ, psiI, j, i, x[j], x_i);
      //desribe the potential from j to i:
      fij(m).A=~psiI*psiI;
      fij(m).B=~psiJ*psiJ;
      fij(m).C=~psiI*psiJ;
      fij(m).a=~psiI*(psiI*x_i + psiJ*x[j] - psi);
      fij(m).b=~psiJ*(psiI*x_i + psiJ*x[j] - psi);
      fij(m).hata=sumOfSqr(psiI*x_i + psiJ*x[j] - psi);
      //cout <<fij(m).A <<endl;
      /*cout <<sumOfSqr(psi) <<endl;
      cout <<(~x[i] * fij(m).A * x[i] + ~x[j] * fij(m).B * x[j] + 2. * ~x[i] * fij(m).C * x[j]
      - 2. * ~(fij(m).a)*x[i] - 2.*~(fij(m).b)*x[j])(0) + fij(m).hata <<endl;
      */
    }
  }
}

void MinSumGaussNewton::updateMessage(uint m){
  uint i, j, k, n, mm;
  n=x.d1;
  j=E(m, 0);
  i=E(m, 1);
  VERBOSE(3, cout <<"  updating message " <<m <<":" <<j <<"->" <<i <<endl);
  if(j==i){ //node potential
    mu(m).M   =fij(m).A;
    mu(m).m   =fij(m).a;
    mu(m).hatm=fij(m).hata; //phi(i, hat_x_i);
  }else{
    arr Abar, abar, tmp;
    double hatabar;
    Abar.resize(n, n);  Abar.setZero();
    abar.resize(n);    abar.setZero();
    hatabar=0.;
    if(!clamped.N || !clamped(j)){
      for(k=0; k<del(j).N; k++){ //collect all messages k->j to j (excluding i->j)
        //recall: this includes also node potentials since we index them as j->j
        mm=del(j)(k);
        CHECK(E(mm, 1)==j, "");
        if(E(mm, 0)==i) continue; //(exclude i->j)
        VERBOSE(3, cout <<"    collecting message " <<mm <<":" <<E(mm, 0) <<"->" <<j <<endl);
        Abar    += mu(mm).M;
        abar    += mu(mm).m;
        hatabar += mu(mm).hatm;
      }
      Abar    += fij(m).B;
      abar    += fij(m).b;
      hatabar += fij(m).hata;
      inverse_SymPosDef(tmp, Abar);
      mu(m).hatm = hatabar - (~abar * tmp * abar)(0);
      tmp     = fij(m).C*tmp;
      mu(m).m = fij(m).a - tmp * abar;
      mu(m).M = fij(m).A - tmp * (~fij(m).C);
    }else{
      mu(m).hatm = (~x[j]*fij(m).B*x[j] - 2.*~fij(m).b*x[j])(0);
      mu(m).m = fij(m).a - fij(m).C*x[j];
      mu(m).M = fij(m).A;
    }
#if 0
  }else{ //old version where fij was not stored for both directions
    Abar    += fij(m).A;
    abar    += fij(m).a;
    hatabar += fij(m).hata;
    inverse_SymPosDef(tmp, Abar);
    //if(clamped(j)) tmp.setZero();
    mu(m).hatm = hatabar - (~abar * tmp * abar)(0);
    tmp     = (~fij(m).C)*tmp;
    //if(!clamped(j))
    mu(m).m = fij(m).b - tmp * abar;
    //else            mu(m).m = fij(m).b - ~fij(m).C*x[j];
    mu(m).M =  fij(m).B - tmp * (fij(m).C);
  }
#endif
  VERBOSE(4, cout <<"  Abar=" <<Abar <<"abar=" <<abar <<"hatabar=" <<hatabar <<endl);
}
VERBOSE(4, cout <<"  new message = " <<mu(m).M <<mu(m).m <<mu(m).hatm <<endl);
}

void MinSumGaussNewton::updateMessagesToNode(uint i){
  uint k, m;
  VERBOSE(2, cout <<"updating all messages to node " <<i <<endl);
  for(k=0; k<del(i).N; k++){
    m=del(i)(k);
    CHECK(E(m, 1)==i, "");
    updateMessage(m);
  }
}

double MinSumGaussNewton::totalCost(bool verbose){
  double Fnode=0., Fpair=0.;
  uint m, i, j;
  for(m=0; m<E.d0; m++){
    i=E(m, 0);  j=E(m, 1);
    if(j==i) Fnode += f(i, j, x[i], x[j]);
    if(j< i) Fpair += f(i, j, x[i], x[j]);
    //cout <<"i" <<i <<" j" <<j <<" f=" <<f(i, j, x[i], x[j]) <<endl;
    //the case i>j is excluded to not count couplings twice
  }
#if 1//check consistency of cost terms!
  double F2=0.;
  for(m=0; m<E.d0; m++){
    i=E(m, 0);  j=E(m, 1);
    if(i==j){ //node potential
      F2 += (~x[i] * fij(m).A * x[i] - 2.*~(fij(m).a)*x[i])(0) + fij(m).hata;
    } else if(j<i){
      F2 += (~x[i] * fij(m).A * x[i] + ~x[j] * fij(m).B * x[j] + 2. * ~x[i] * fij(m).C * x[j]
             - 2. * ~(fij(m).a)*x[i] - 2.*~(fij(m).b)*x[j])(0) + fij(m).hata;
    }
  }
  double F=Fnode+Fpair;
  CHECK(fabs((F-F2)/(F+F2+1.))<1e-10, F <<"!=" <<F2);
#endif
  VERBOSE(1, cout <<"costs: nodes=" <<Fnode <<" pairs=" <<Fpair <<" total=" <<Fnode+Fpair <<endl);
  return Fnode+Fpair;
}

void MinSumGaussNewton::init(){
  //uint N=x.d0, n=x.d1;
  uint i, j, m;
  //double fx, fy;
  //arr A, a, Delta, y;
  //double alpha=1.;
  
  //init potentials
  fij.resize(E.d0);
  for(i=0; i<x.d0; i++) reapproxPotentials(i, x[i]);
  
  //init messages zero
  mu.resize(E.d0);
  for(m=0; m<mu.N; m++){
    mu(m).m.resize(x.d1);       mu(m).m.setZero();
    mu(m).M.resize(x.d1, x.d1); mu(m).M.setDiag(1e-6);
    i=E(m, 0); j=E(m, 1);
    if(i==j)     mu(m).hatm=0.; //phi(i, x[i]);
    else if(i<j) mu(m).hatm=0.; //psi(i, j, x[i], x[j]);
    else         mu(m).hatm=0.; //psi(j, i, x[j], x[i]);
  }
  
  MT::open(fil, "z.MinSum");
}

void MinSumGaussNewton::step(uint steps){
  uint N=x.d0, n=x.d1;
  uint i, k, m;
  double fx, fy;
  arr A, a, Delta, y;
  double alpha=1.;
  
  //iterate
  bool fwd=false;
  i=0;
  for(uint sweep=0; sweep<2*steps; sweep++){ //iterate over nodes
    fwd ^= true;
    double cost = totalCost();
    cout <<"** Sweep " <<sweep <<" before-cost=" <<cost <<" fwd=" <<fwd <<endl;
    if(N<2) fwd=true;
    for(i=fwd?0:N-2; fwd?i<N:i>0; i+=fwd?1:-1){
      if(clamped.N && clamped(i)) continue;
      //double cost=0.; //= totalCost();
      fil <<sweep <<' ' <<i <<' ' <<totalCost() <<' ';
      x.write(fil, " ", "", " \n");
      //cout <<"* node " <<i <<" F=" <<cost <<endl;
      
      for(;;){ //iterate optimizing at node i
        reapproxPotentials(i, x[i]);
        updateMessagesToNode(i);
        fx=0.;
        A.resize(n, n);  A.setZero();
        a.resize(n);     a.setZero();
        for(k=0; k<del(i).N; k++){
          m=del(i)(k);
          fx += mu(m).hatm + (~x[i]*mu(m).M*x[i] -2.*~mu(m).m*x[i])(0);
          A  += mu(m).M;
          a  += mu(m).m;
        }
        lapack_Ainv_b_sym(Delta, A, a);
        Delta -= x[i];
        VERBOSE(1, cout <<"optimizing over node " <<i <<": x=" <<x[i] <<" f(x)=" <<fx <<" Delta=" <<Delta <<endl);
        
        //x[i]() += Delta;  break;   VERBOSE(2, cout <<" - FORCE" <<endl);
        double len=norm(Delta);
        if(len>maxStep) Delta*=maxStep/len;
        
        //stopping criterion
        if(len<tolerance) break;
        
        for(;;){ //iterate over step sizes
          y = x[i] + alpha*Delta;
          fy=0.;
          reapproxPotentials(i, y);
          updateMessagesToNode(i);
          for(k=0; k<del(i).N; k++){
            m=del(i)(k);
            fy += mu(m).hatm + (~y*mu(m).M*y -2.*~mu(m).m*y)(0);
          }
          VERBOSE(1, cout /* <<evals*/ <<" \tprobing y=" <<y <<" \tf(y)=" <<fy <<" \t|Delta|=" <<norm(Delta) <<" \talpha=" <<alpha <<std::flush);
          CHECK(fy==fy, "cost seems to be NAN: f(y)=" <<fy);
          if(fy <= fx) break;
          //if(evals>maxEvals) break; //WARNING: this may lead to non-monotonicity -> make evals high!
          //decrease stepsize
          if(alpha<1e-10) break;
          alpha = .5*alpha;
          VERBOSE(1, cout <<" - reject and revise" <<endl);
        }
        if(fy<=fx){
          VERBOSE(1, cout <<" - ACCEPT" <<endl);
          //adopt new point and adapt stepsize
          x[i] = y;
          fx = fy;
          alpha  = pow(alpha, 0.5);
        }else{
          VERBOSE(1, cout <<" - FINAL REJECT" <<endl);
          break;
        }
      }
      
      cout <<" * node " <<i <<" fx=" <<fx <<endl;
      
    }
    cout <<"** Sweep " <<sweep <<" after-cost=" <<totalCost() <<" fwd=" <<fwd <<endl;
  }
}

#include "array_t.cpp"
template MT::Array<Fij>::Array();
template MT::Array<Fij>::~Array();
template MT::Array<Mu>::Array();
template MT::Array<Mu>::~Array();
