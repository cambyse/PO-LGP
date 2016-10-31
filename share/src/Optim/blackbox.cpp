/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#include <iomanip>
#include "blackbox.h"

//===========================================================================

LocalModelBasedOptim::LocalModelBasedOptim(arr& _x, const ScalarFunction& _f,  OptOptions _o)
  : x_init(_x), f(_f), best(NULL), o(_o), it(0), evals(0), numTinySteps(0){
  alpha = o.initStep;
}

LocalModelBasedOptim::~LocalModelBasedOptim(){
  listDelete(D);
}

void LocalModelBasedOptim::step(){
  if(!D.N){ //no data yet
    if(o.verbose>1) cout <<"*** LocalModelBasedOptim:" <<endl;
    evaluate(x_init);

    //startup verbose
//    if(o.verbose>0) fil.open("z.opt");
//    if(o.verbose>0) fil <<0 <<' ' <<eval_cost <<' ' <<best->f <<' ' <<alpha;
//    if(o.verbose>2) fil <<' ' <<x;
//    if(o.verbose>0) fil <<endl;

    return;
  }
  if(D.N==1){ //only 1 data point
    arr x = x_init;
    rndGauss(x, alpha, true);
    evaluate(x);
    return;
  }

  arr X,y; // construct data set
  for(uint i=0;i<D.N;i++){
//    if(i>=2*x_init.N) break; //do not add more points than needed...but how many are needed?
    X.append(D(i)->x);
    y.append(D(i)->f);
  }

  X.reshape(y.N, x_init.N);
  X = catCol(ones(y.N), X); //add bias term
  arr beta = inverse_SymPosDef(~X* X + 1e-1*eye(X.d1))* ~X * y;
  arr grad = beta.sub(1,-1); //remove bias term

  arr delta = grad / length(grad); //always normalize gradient
  //add 'exploration' (determinante component...)

  arr x = best->x - alpha*delta;

  evaluate(x);
}

void LocalModelBasedOptim::run(uint maxIt){
  numTinySteps=0;
  for(uint i=0;i<maxIt;i++){
    step();
//    if(stopCriterion==stopStepFailed) continue;
//    if(stopCriterion==stopCritLineSteps){ reinit();   continue; }
//    if(stopCriterion>=stopCrit1) break;
  }
//  if(o.verbose>1) gnuplot("plot 'z.opt' us 1:3 w l", NULL, false);
//  if(o.fmin_return) *o.fmin_return= fx;
//  return stopCriterion;
}

bool DatumSortCompare(LocalModelBasedOptim::Datum* const& a, LocalModelBasedOptim::Datum* const& b){
  return a->distToBest <= b->distToBest;
}

void LocalModelBasedOptim::evaluate(const arr& x, bool sort){
  if(o.verbose>2) cout <<" \tprobing y=" <<x <<flush;
  double dist = best ? euclideanDistance(x, best->x) : 0.;
  double fx = f(NoArr, NoArr, x);  evals++;
  if(o.verbose>1) cout <<" \tevals=" <<std::setw(4) <<evals <<" \talpha=" <<std::setw(11) <<alpha <<" \tf(y)=" <<fx <<endl;
  Datum d = {x, fx, dist};
  D.append(new Datum(d));
  if(!best) best = D.last(); //first data point
  if(d.f<best->f){ //a new best...
    best = D.last();
    for(Datum *d:D) d->distToBest=euclideanDistance(d->x, best->x);
  }
//  cout <<D <<endl;
//  if(sort) D.sort(DatumSortCompare);
}

//===========================================================================

extern "C"{
#include "CMA/cmaes_interface.h" //by Nikolaus Hansen
}

struct sSearchCMA{
  cmaes_t evo;
};

SearchCMA::SearchCMA(){
  s = new sSearchCMA;
}

SearchCMA::~SearchCMA(){
  delete s;
}

void SearchCMA::init(uint D, int mu, int lambda, const arr &startPoint, const arr &startDev){
  cmaes_init(&s->evo, NULL, D, startPoint.p, startDev.p, 1, lambda, mu, NULL);
}


void SearchCMA::init(uint D, int mu, int lambda, const arr &startPoint, double _startDev){
  CHECK_EQ(startPoint.N,D,"");
  arr startDev(D);
  startDev=_startDev;
  cmaes_init(&s->evo, NULL, D, startPoint.p, startDev.p, 1, lambda, mu, NULL);
}

void SearchCMA::init(uint D, int mu, int lambda, double lo, double hi){
  arr startPoint(D);
  rndUniform(startPoint, lo, hi, false);
  init(D, mu, lambda, startPoint, hi-lo);
}

void SearchCMA::step(arr& samples, arr& costs){
  if(costs.N){
    cmaes_ReestimateDistribution(&s->evo, costs.p);
  }else{ //first iteration: initialize arrays:
    samples.resize(s->evo.sp.lambda, s->evo.sp.N);
    costs.resize(s->evo.sp.lambda).setZero();
  }

  //generate samples
  double *const*rgx = cmaes_SampleDistribution(&s->evo, NULL);
  for(uint i=0;i<samples.d0;i++) samples[i].setCarray(rgx[i], samples.d1);
}

void SearchCMA::getBestSample(arr &sample) {
  sample.resize(s->evo.sp.N);
  sample.setCarray(s->evo.rgxbestever,sample.N);
}

void SearchCMA::getMean(arr &mean) {
  mean.resize(s->evo.sp.N);
  mean.setCarray(s->evo.rgxmean,mean.N);
}
