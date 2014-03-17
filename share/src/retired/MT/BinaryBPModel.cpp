/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


#include "BinaryBPModel.h"

void BinaryBPNetModel::initLayered(Data *_data){
  data=_data;
  
  layers = MT::getParameter<uintA>("layers");
  if(!layers(0)) layers(0)=data->Xdim();
  if(!layers.last()) layers.last()=data->Ydim();
  if(layers(0)!=data->Xdim() || layers(layers.N-1)!=data->Ydim()){
    MT_MSG("considering only part of the input/output!");
    data->reduce(layers(0), layers(layers.N-1));
  }
  bool lateral=MT::getParameter<bool>("lateralConnections");
  graphLayered(net.nodes, net.edges, layers, lateral);
  
  net.randomizeWeightsUniform(.1, .1, false);
  ITER = MT::getParameter<uint>("ITER");
  regJ = MT::getParameter<double>("regJ");
  regT = MT::getParameter<double>("regT");
}

void BinaryBPNetModel::randomInit(){
  uint N=MT::getParameter<uint>("N");
  uint D=MT::getParameter<uint>("D");
  if(D==0) graphRandomUndirected(net.nodes, net.edges, N, -1.); //unconnected!
  if(D==1) graphRandomLinear(net.nodes, net.edges, N);     //just a chain!
  //graphRandomTree(net.nodes, net.edges, N, 1);
  if(D>1)  graphRandomFixedDegree(net.nodes, net.edges, N, D);
  net.randomizeWeightsUniform(1., 1., MT::getParameter<bool>("positive"));
  //net.report(cout);
  ITER   = MT::getParameter<uint>("ITER");
}

void BinaryBPNetModel::beliefs(arr& b, const arr& w){
  net.setTandJ(w);
  net.zeroMessages();
  for(uint i=0; i<ITER; i++) net.stepBP();
  net.getNodeBeliefs(b);
}

void BinaryBPNetModel::d_beliefs(arr& dbdw, const arr& w){
  net.setTandJ(w);
  net.zeroMessages();
  net.setNodeBeliefDeltas();
  for(uint i=0; i<ITER; i++) net.stepBP();
  for(uint i=0; i<ITER; i++) net.stepGradBP();
  net.getGradTandJ(dbdw);
}

double BinaryBPNetModel::bethe(){
  net.zeroMessages();
  for(uint i=0; i<ITER; i++) net.stepBP();
  return net.Bethe();
}

void BinaryBPNetModel::d_bethe(arr& grad){
  net.zeroDeltas(0);
  net.addBetheGradientDeltas();
  for(uint i=0; i<ITER; i++) net.stepGradBP();
  net.getGradTandJ(grad);
}

void BinaryBPNetModel::model(arr& output, const arr& input, const arr& w){
  uint N=net.nodes.N, i;
  CHECK(output.N, "");
  //-- set input - iterate BP - read ouput
  net.setTandJ(w);
  net.zeroMessages();
  net.addInputEvidence(input);
  for(i=0; i<ITER; i++)     net.stepBP();
  for(i=0; i<output.N; i++) output(i) = net.nodeBelief(net.nodes(N-output.N+i));
}

void BinaryBPNetModel::model(Data& data, const arr& w){
  data.Z.resizeAs(data.Y);
  uint i;
  for(i=0; i<data.N(); i++)  model(data.Z[i](), data.X[i], w);
}

double BinaryBPNetModel::logLikelihood(const arr& w, const Data& data, arr* grad){
  double logL=0.;
  double logZ_base, logZ_Y, logZ_i;
  arr grad_base, grad_i;
  
  net.setTandJ(w);
  logZ_base = -bethe();
  if(grad){
    grad->resizeAs(w);
    grad->setZero();
    d_bethe(grad_base);
  }
  for(uint i=0; i<data.N(); i++){
    net.setTandJ(w);
    logZ_Y  = -net.addInputEvidence(data.X[i]);
    logZ_Y += -net.addOutputEvidence(data.Y[i]);
    logZ_i = -bethe();
    logL += logZ_i - logZ_base - logZ_Y;
    if(grad){
      d_bethe(grad_i);
      (*grad) += grad_i - grad_base;
    }
  }
  logL/=data.N();
  if(grad)(*grad)/=(double)data.N();
  return logL;
}

double BinaryBPNetModel::loss(arr& output, const arr& target, const arr& input, const arr& w, arr *grad){
  uint i;
  BinaryBPNet::node *n;  BinaryBPNet::edge *e;
  
  output.resize(target.N);
  model(output, input, w);
  
  //-- compute costs (including regularization)
  double C=0.;
#if 1
  for(i=0; i<output.N; i++) C -= (2.*target(i)-1.) * output(i) - log(2.*cosh(output(i))); //NEG log-likelihood
#else
  for(i=0; i<output.N; i++) if(target(i)<.5){ if(output(i)>-.1) C+=1.; }else{ if(output(i)<.1) C+=1.; } //classification error
#endif
  //C += sqrDistance(target, output);
  for_list(Type,  n,  net.nodes) C += regT*fabs(n->theta); //diff
  for_list(Type,  e,  net.edges) C += regJ*fabs(e->J);
  
  if(grad){
    //reinit everything
    /*net.init();
    net.setTandJ(w);
    for(i=0;i<input.N;i++)  net.nodes(i)->theta = input(i); //multiplicative input
    */
    
    //-- compute deltas
    net.zeroDeltas(0);
    
    //compute output deltas
    arr delta(target.N);
#if 1
    for(i=0; i<output.N; i++) delta(i) = - ((2.*target(i)-1.) - tanh(output(i))); //NEG log-likelihood
#else
    for(i=0; i<output.N; i++) if(target(i)<.5){ if(output(i)>-.1) delta(i)=1.; }else{ if(output(i)<.1) delta(i)+=-1.; } //classification error
#endif
    //delta = output-target;  delta *= 2.;
    arr deltaT(net.nodes.N); deltaT.setZero();
    for(i=0; i<delta.N; i++) deltaT(deltaT.N-delta.N+i) = delta(i);
    net.addBeliefDeltas(deltaT);
    
    //compute regularization deltas
    delta.resize(net.nodes.N);
    for_list(Type,  n,  net.nodes) delta(i) = regT*MT::sign(n->theta); //diff
    net.addThetaDeltas(delta);
    delta.resize(net.edges.N);
    for_list(Type,  e,  net.edges) delta(i) = regJ*MT::sign(e->J);
    net.addJDeltas(delta);
    
    for(i=0; i<ITER; i++) net.stepGradBP();
    net.getGradTandJ(*grad);
    //for(i=0;i<input.N;i++) (*grad)(i) =0; //*= input(i); //multiplicative input -> modify gradient //diff
  }
  return C;
}

double BinaryBPNetModel::loss(const arr& w, const uintA& range, const Data& data, arr *grad, double *err){
  arr output;
  double l = 0., errAll=0.;
  arr gradAll(w.N);  gradAll.setZero();
  uint i, j;
  for(i=0; i<range.N; i++){
    j = range(i);
    l += loss(output, data.Y[j], data.X[j], w, grad);
    if(grad) gradAll += *grad;
    if(err){ if(output.maxIndex()!=data.Y[j].maxIndex()) errAll+=1.; else errAll+=0.; }
  }
  l /= double(range.N);
  if(grad)(*grad) = gradAll / double(range.N);
  if(err)(*err)  = errAll  / double(range.N);
  return l;
}

void BinaryBPNetModel::EMtrain(const arr& w, const Data& data){
  BinaryBPNet::node *n;  BinaryBPNet::edge *e;
  arr b_t, b_J;
  uint i, j;
  //cout <<"** before training"; net.report(cout);
  for(i=0; i<data.N(); i++){
    net.setTandJ(w);
    net.zeroMessages();
    net.addInputEvidence(data.X[i]);  //data.sub(i, i, 0, 0));
    net.addOutputEvidence(data.Y[i]); //data.sub(i, i, 1, -1));
    //cout <<"** after conditioning"; net.report(cout);
    for(j=0; j<ITER; j++) net.stepBP();
    //cout <<"** after inference";    net.report(cout);
    net.getNodeBeliefTables(b_t, true);
    net.getPairBeliefTables(b_J, true);
  }
  //cout <<"** expectations: b_t=\n" <<b_t <<"\n  b_J=\n" <<b_J <<endl;
  //-- M-step:
  for_list(Type,  n,  net.nodes) n->theta=0.;
  for_list(Type,  e,  net.edges){
    //marginals:
    /*
    double pf0, pf1, pt0, pt1;
    pf0=b_J(i, 0, 0)+b_J(i, 0, 1);  pf1=b_J(i, 1, 0)+b_J(i, 1, 1);
    pt0=b_J(i, 0, 0)+b_J(i, 1, 0);  pt1=b_J(i, 0, 1)+b_J(i, 1, 1);
    cout <<pf0 <<' ' <<pf1 <<b_t[e->ifrom] <<endl;
    cout <<pt0 <<' ' <<pt1 <<b_t[e->ito] <<endl;
    CHECK(fabs(pf0-b_t(e->ifrom, 0))<1e-5 && fabs(pf1-b_t(e->ifrom, 1))<1e-5, "");
    CHECK(fabs(pt0-b_t(e->ito  , 0))<1e-5 && fabs(pt1-b_t(e->ito  , 1))<1e-5, "");
    */
    double t1, t2;
    table_to_pairExp(e->J, t1, t2, b_J[i]);
    e->from->theta += t1;
    e->to  ->theta += t2;
  }
  //cout <<"** after J updates"; net.report(cout);
  for_list(Type,  n,  net.nodes){
    double p=table_to_nodeExp(b_t[i]);
    if(n->edges.N>1) n->theta -= (n->edges.N-1)*p;
    if(!n->edges.N) n->theta += p;
  }
  //cout <<"** after theta updates"; net.report(cout);
  /* --OLD CODE
    //same as the 'collect' routine:
  double x=0.;
  for_list(Type,  e,  n->edges){
  CHECK(e->to==n || e->from==n, "something's wrong...");
  if(e->to==n) x += atanh( tanh(e->J) * tanh( hatb_t(e->ifrom) - atanh(tanh(e->J)*tanh(hatb_t(i))) ) );
  else         x += atanh( tanh(e->J) * tanh( hatb_t(e->ito  ) - atanh(tanh(e->J)*tanh(hatb_t(i))) ) );
  }
  n->theta = hatb_t(i) - x;
  */
}

double BinaryBPNetModel::totalLoss(const arr& w, const Data& data, arr *grad, double *err){
  double l = 0., errAll=0.;
  arr gradAll(w.N);  gradAll.setZero();
  uint i;
  for(i=0; i<data.N(); i++){
    l += loss(w, TUP(i), data, grad, err);
    if(grad) gradAll += *grad;
    if(err)  errAll  += *err;
  }
  l /= double(data.N());
  if(grad)(*grad) = gradAll / double(data.N());
  if(err)(*err)  = errAll  / double(data.N());
  return l;
}

