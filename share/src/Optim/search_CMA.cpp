#include "search.h"

//uint optCMA(arr& x, ScalarFunction& f,
//	    OptOptions opt, CMA_Options cma_opt, arr *fx_user=NULL){


//}

extern "C"{
#include "cmaes_interface.h" //by Nikolaus Hansen
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

void SearchCMA::init(uint D, int mu, int lambda, arr &startPoint, double _startDev){
  CHECK(startPoint.N==D,"");
  arr startDev(D);
  startDev=_startDev;
  cmaes_init(&s->evo, NULL, D, startPoint.p, startDev.p, 1, lambda, mu, NULL);
}

void SearchCMA::init(uint D, int mu, int lambda, double lo, double hi){
  arr startPoint(D);
  rndUniform(startPoint, lo, hi, false);
  init(D, mu, lambda, startPoint, hi-lo);
}

void SearchCMA::step(arr& samples, arr& values){
  if(values.N){
    cmaes_ReestimateDistribution(&s->evo, values.p);
  }else{ //first iteration: initialize arrays:
    samples.resize(s->evo.sp.lambda, s->evo.sp.N);
    values.resize(s->evo.sp.lambda).setZero();
  }

  //generate samples
  double *const*rgx = cmaes_SampleDistribution(&s->evo, NULL);
  for(uint i=0;i<samples.d0;i++) samples[i].setCarray(rgx[i], samples.d1);
}
