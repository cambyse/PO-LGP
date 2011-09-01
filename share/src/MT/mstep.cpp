#include "mstep.h"

void standardMstep(arr& param, const arr& post, uint left, double noise){
  param = post;
  tensorCondNormalize(param, left);
  if(noise){
    rndUniform(param, noise, 2.*noise, true);
    tensorCondNormalize(param, left);
  }
}

void noisyPowerMaxMstep(arr& param, const arr& post, uint left, double power=1., double noise=0.){
  // multiply with a power of the likelihood
  checkNan(param);
  checkNan(post);
  arr like;
  like = post;
  like /= param;
  checkNan(like);
  double Lmin=like.min(), Lmax=like.max();
  Lmin -= .1;
  for(uint i=0; i<like.N; i++) like.elem(i) = ::pow((like.elem(i)-Lmin)/(Lmax-Lmin), power);
  param *= like;
  tensorCondNormalize(param, left);
  if(noise){
    rndUniform(param, noise, 2.*noise, true);
    tensorCondNormalize(param, left);
  }
  checkNan(param);
}

void noisyMaxMstep2(arr& param, const arr& likelihood, uint left, double noise){
  // compute a multiplicative update of the parameter from the posterior
  checkNan(param);
  checkNan(likelihood);
  param = likelihood;
  tensorCondMax(param, left);
  if(noise){
    rndUniform(param, noise, 2.*noise, true);
    tensorCondNormalize(param, left);
  }
}

void noisyMaxMstep(arr& param, const arr& post, uint left, double rate, double noise){
  // compute a multiplicative update of the parameter from the posterior
  checkNan(param);
  checkNan(post);
  arr factor;
  factor = post;
  factor /= param;
  tensorCondMax(factor, left);
  factor *= rate;   //factor = rate*max + (1-rate)
  factor += (1.-rate);
  //rndUniform(factor, .0, noise, true);
  //rndUniform(param, .0001, .00011, true);
  param *= factor;
  tensorCondNormalize(param, left);
  if(noise){
    uint N=1; for(uint l=0; l<left; l++) N*=param.dim(l);
    noise /= N;
    rndUniform(param, noise, 2.*noise, true);
    tensorCondNormalize(param, left);
  }
  checkNan(param);
}

void mstep_11rule(arr& param, const arr& post, uint left, double rate, double noise){
  checkNan(param);
  checkNan(post);
  arr factor;
  factor = post;
  for(uint i=0; i<param.N; i++) if(param.elem(i)<1e-10) param.elem(i)=1e-10;
  factor /= param;
  checkNan(factor);
  tensorCond11Rule(factor, left, rate);
  checkNan(factor);
  param *= factor;
  tensorCondNormalize(param, left);
  if(noise){
    uint N=1; for(uint l=0; l<left; l++) N*=param.dim(l);
    noise /= N;
    rndUniform(param, noise, 2.*noise, true);
    tensorCondNormalize(param, left);
  }
  checkNan(param);
}

void noisyMaxMstep_old(arr& param, const arr& post, uint left){
  // compute a multiplicative update of the parameter from the posterior
  checkNan(param);
  checkNan(post);
  arr factor;
  factor = post;
  factor /= param;
  tensorCondMax(factor, left);
  factor += 3.;
  rndUniform(factor, .0, 1e-3, true);
  rndUniform(param, .0001, .00011, true);
  param *= factor;
  tensorCondNormalize(param, left);
  checkNan(param);
}
