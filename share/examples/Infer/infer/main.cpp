#define MT_IMPLEMENT_TEMPLATES

#include <iostream>
#include <map>
#include <Infer/infer.h>
#ifdef MT_DAI
#  include <Infer/daiModule.h>
#endif
#include <stdlib.h>

#define CALC_JT 1
#define CALC_LBP 2

#define MC 1
#define HMM 2

//===========================================================================
/*

How to build a model:

1) new variables

2) new factors using the vars id's
-- set the factor probability tables

DONE

How to do inference

1. option:

a) create message pairs by hand
b) specify a propagation (sweeping) order
c) propagate them

2. option:

constructJunctionTree:
-- create junction tree and do inference

3. option:

loopy_...:
-- create bi-partite factor graph (factors and variables) and iterate parallel loopy BP

 */


//===========================================================================

void testTensorOperations(){
  infer::Variable
    x(3, "x"),
    y(4, "y"),
    z(2, "z"),
    y_(7, "y_"),
    z_(5, "z_");
  
  infer::Factor
    a(LIST(x,y,z)),
    b(LIST(y,z_,y_)),
    c;
  
  a.setRandom();
  b.setRandom();
  
  a.write(cout);
  b.write(cout);
  
  tensorProductMarginal(c,a,b,LIST(z_,y));
  c.write(cout);
  
  tensorProduct(c,a,b);
  c.write(cout);
  
  tensorMarginal(a,c,LIST(z_,y));
  a.write(cout);
  
  tensorMaxMarginal(a,c,LIST(z_,y));
  a.write(cout);
  
  tensorMultiply(c,a);
  c.write(cout);
}


//===========================================================================

void testBurglary() {
  infer::Variable
    bl(2, "burglary"),
    eq(2, "earthquake"),
    al(2, "alarm"),
    ma(2, "maryCalls"),
    jo(2, "johnCalls");
  
  infer::VariableList variables = LIST(bl,eq,al,ma,jo);

  infer::Factor
    p_bl(LIST(bl), ARR( .999, .001)),
    p_eq(LIST(eq), ARR( .998, .002 )),
    p_al(LIST(al, bl, eq), ARR(0.999, 0.71, 0.06, 0.05, 0.001, 0.29, 0.94, 0.95)),
    p_ma(LIST(ma, al), ARR(0.99, 0.3, 0.01, 0.7)),
    p_jo(LIST(jo, al), ARR(0.95, 0.1, 0.05, 0.9));
  
  infer::Factor evid(LIST(jo), ARR( 0, 1.0 ));
  
  p_bl.checkCondNormalization();
  p_eq.checkCondNormalization();
  p_al.checkCondNormalization();
  p_ma.checkCondNormalization();
  p_jo.checkCondNormalization();
  evid.checkCondNormalization();
  

  arr p1,p2,p3,p4,p5;
  //*** naive inference:
  //first construct big joint, then marginalize
  
  infer::Factor joint(LIST(eq, bl, al, ma, jo));
  joint.setOne();
  tensorMultiply(joint, p_eq);
  tensorMultiply(joint, p_bl);
  tensorMultiply(joint, p_al);
  tensorMultiply(joint, p_ma);
  tensorMultiply(joint, p_jo);
  tensorMultiply(joint, evid);
  
  infer::Factor post_bl;
  tensorMarginal(post_bl, joint, LIST(bl));
  cout <<"joint marginal posterior = " <<post_bl <<endl;
  post_bl.getP(p1);

  //-- list of all factors for the remaining algorithms
  infer::FactorList factors=LIST(p_bl, p_eq, p_al, p_ma, p_jo, evid);

  //*** elimination
  eliminationAlgorithm(post_bl,factors,LIST(bl));
  cout <<"elimination algorithm posterior = " <<post_bl <<endl;
  post_bl.getP(p2);

  //*** JTA inference
  //-- version 1
  infer::FactorGraph fg;
  infer::JunctionTree::junctionTreeInference(fg, factors, variables);
  getMarginal(post_bl, LIST(bl), fg);
  cout <<"JTP bl posterior = " <<post_bl <<endl;
  post_bl.getP(p3);
  
  //-- version 2
  infer::FactorGraph fg2;
  infer::JunctionTree::constructJunctionTree(fg2, factors, variables);
  treeInference(fg2.F(0),true);
  fg2.computeBeliefs();
  getMarginal(post_bl, LIST(bl), fg2);
  cout <<"JTP bl posterior = " <<post_bl <<endl;
  post_bl.getP(p4);

  //** LoopyBP
  infer::LoopyBP lbp;
  lbp.initBipartite(variables,factors);
  for(uint t=0;t<10;t++) lbp.step();
  lbp.getVarBelief(post_bl,&bl,false);
  cout <<"BP bl posterior = " <<post_bl <<endl;
  post_bl.getP(p5);

  //-- check
  CHECK(maxDiff(p1,p2,0)<1e-10 && maxDiff(p1,p3,0)<1e-10 && maxDiff(p1,p4,0)<1e-10 && maxDiff(p1,p5,0)<1e-10,"");

  cout <<"*** inference methods consistent" <<endl;
  cout <<"\nThe following warnings indicate that automatic destroctors are not called in a sensible order (first destroy factors, then variables) - that's ok in this test..." <<endl;
}


// creates HMM
void createHMM(infer::FactorList& factors, infer::VariableList& states, infer::VariableList& obs, uint timesteps) {
  factors.clear();
  states.clear();
  obs.clear();
  
  uint i;

  //create two lists of variables
  for(i=0;i<timesteps;i++){
    states.append(new infer::Variable(2, STRING("state"<<i)));
    obs   .append(new infer::Variable(2, STRING("obs"<<i)));
  }


  infer::Factor* f;
  // t=0
  f = new infer::Factor(LIST(*states(0)), ARR(0.5, 0.5));
  factors.append(f);
  // t>0
  for (i=0; i<timesteps-1; i++) {
    f = new infer::Factor(LIST(*states(i+1), *states(i)), ARR( 0.9, 0.2, 0.1, 0.8 ));
    factors.append(f);
  }
  for (i=0; i<timesteps; i++) {
    f = new infer::Factor(LIST(*obs(i), *states(i)), ARR( 0.95, 0.3, 0.05, 0.7 ));
    factors.append(f);
  }
}


// Test JunctionTree or BeliefPropagation on MarkovModel oder HiddenMarkovModel.
void testHMMInference(uint T) {
  infer::FactorList factors;
  infer::VariableList states;
  infer::VariableList obs;
  createHMM(factors, states, obs, T);

  infer::VariableList vars;
  vars.append(states);
  vars.append(obs);
  
  // Specifying some evidence
  factors.append(new infer::Factor(ARRAY(states(0)), ARR(0.0, 1.0)));
  factors.append(new infer::Factor(ARRAY(states(1)), ARR(1.0, 0.0)));
  factors.append(new infer::Factor(ARRAY(states(5)), ARR(1.0, 0.0)));

  arr p1,p2;

  //JTA
  infer::Factor post;
  infer::FactorGraph fg;
  infer::JunctionTree::junctionTreeInference(fg, factors, vars);
  getMarginal(post, LIST(*states(2)), fg);
  cout <<"JTA posterior = " <<post <<endl;
  post.getP(p1);

  //BP
  infer::FactorGraph fg2;
  infer::LoopyBP_obsolete::loopy_belief_propagation(fg2, factors);
  getMarginal(post, LIST(*states(2)), fg2);
  cout <<"BP posterior = " <<post <<endl;
  post.getP(p2);

  //-- check
  CHECK(maxDiff(p1,p2,0)<1e-10,"");

  cout <<"*** HMM inference methods consistent" <<endl;
}


// loopy graphical model tested with JunctionTree und LoopyBP
void testLoop(){
  infer::Variable A(2, "A");
  infer::Variable B(2, "B");
  infer::Variable C(2, "C");
  infer::VariableList vars = LIST(A,B,C);
  
  arr coupling = ARR(.75, .25, .25, .75);
  coupling.reshape(2,2);
  //coupling.setText("[ 1 0 ; 0 1]");
  
  infer::Factor f_ab(LIST(A,B), coupling);
  infer::Factor f_bc(LIST(B,C), coupling);
  infer::Factor f_ca(LIST(C,A), coupling);
  infer::FactorList facs = LIST(f_ab, f_bc, f_ca);
  
  arr p_evid = ARR(.2,.8);
  infer::Factor evid(LIST(A), p_evid);
  facs.append(&evid);
  
  //writeEdges(facs);
  //write(facs);
  
  infer::FactorGraph fg;
  infer::LoopyBP lbp;
  lbp.initPairwise(vars,facs);
  for(uint t=0;t<3;t++) lbp.step();
  MT::Array<infer::Factor> b;
  lbp.getVarBeliefs(b);  cout <<b <<endl;
}

#include "gridBP.inc"
            
void testGridBP(){
  uint i,j,k;
  
  //Gaussian weigh list
  arr weights(256);
  for(i=0;i<weights.N;i++) weights(i) = exp(-.5*i*i/(3.*3.)) + .01;
  
  //image evidence
  byteA image;
  read_ppm(image,"image.ppm");
  make_grey(image);
  arr evid(image.d0,image.d1,256);
  for(i=0;i<image.d0;i++) for(j=0;j<image.d1;j++){
      for(k=0;k<256;k++) evid(i,j,k) = weights(abs((int)k-(int)image(i,j)));
  }

  //coupling matrix
  arr coupling(256,256);
  for(i=0;i<256;i++) for(j=0;j<256;j++) coupling(i,j) = weights(abs((int)i-(int)j));

  //inference
  arr post;
  gridBP(post, evid, coupling);
}

void testGridBP2(){
  arr evid(2,2,2);
  evid = .5;
  evid(0,0,0)=.2;
  evid(0,0,1)=.8;

  arr coupling = ARR(.75, .25, .25, .75);
  coupling.reshape(2,2);
  
  arr post;
  gridBP(post, evid, coupling);
}


#include "rndNetBP.inc"
void testRndNetBP(){
  uint order=MT::Parameter<uint>("order");
  uint N    =MT::Parameter<uint>("N");
  double conn =MT::Parameter<double>("conn");
  rndNetBPwithExcludeEchoMessages(N,2,conn,order);
}

void testPairBP(){
  infer::FactorList facs;
  infer::VariableList vars;
  randomPairNet(vars,facs,3,2,1);
  listWrite(vars,cout,"\n  ");  cout <<endl;
  listWrite(facs,cout,"\n  ");  cout <<endl;
  infer::LoopyBP lbp;
  MT::Array<infer::Factor> b;
  lbp.initPairwise(vars,facs);
  for(uint t=0;t<3;t++) lbp.step();
  lbp.getVarBeliefs(b);  cout <<b <<endl;
  lbp.clear();
  lbp.initBipartite(vars,facs);
  for(uint t=0;t<6;t++) lbp.step();
  lbp.getVarBeliefs(b);  cout <<b <<endl;
}

#ifdef MT_DAI
void testDai(){
  infer::FactorList facs;
  infer::VariableList vars;
  randomPairNet(vars,facs,10,2,1);
  listWrite(vars,cout,"\n  ");  cout <<endl;
  listWrite(facs,cout,"\n  ");  cout <<endl;
  DaiModule dai;
  createDai(dai,vars,facs);
  inference(dai);
}
#endif

int main(int argc, char** argv){
  MT::initCmdLine(argc,argv);

  switch(MT::getParameter<int>("mode")){
  case 0:  testTensorOperations();  break;
  case 1:  testBurglary();  break;
  case 2:  testHMMInference(10);  break;
  case 3:  testLoop();  break;
  case 4:  testGridBP();  break;
  case 5:  testRndNetBP();  break;
  case 6:  testPairBP();  break;
    //case 8:  testDai();  break;
  }
  
  return 0;
}



