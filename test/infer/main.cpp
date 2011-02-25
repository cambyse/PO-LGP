#define MT_IMPLEMENTATION

#include <iostream>
#include <map>
#include <MT/infer.h>
#include <MT/daiModule.h>

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
  Variable
    x(3, "x"),
    y(4, "y"),
    z(2, "z"),
    y_(7, "y_"),
    z_(5, "z_");
  
  Factor
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
  Variable
    bl(2, "burglary"),
    eq(2, "earthquake"),
    al(2, "alarm"),
    ma(2, "maryCalls"),
    jo(2, "johnCalls");
  
  VariableList variables = LIST(bl,eq,al,ma,jo);

  Factor
    p_bl(LIST(bl), "[ .999 .001]"),
    p_eq(LIST(eq), "[ .998 .002 ]"),
    p_al(LIST(al, bl, eq), "[0.999, 0.71, 0.06, 0.05, 0.001, 0.29, 0.94, 0.95]" ),
    p_ma(LIST(ma, al), "[0.99, 0.3, 0.01, 0.7  ]"),
    p_jo(LIST(jo, al), "[ 0.95, 0.1, 0.05, 0.9]");
  
  Factor evid(LIST(jo), "[ 0, 1.0 ]");
  
  p_bl.checkCondNormalization();
  p_eq.checkCondNormalization();
  p_al.checkCondNormalization();
  p_ma.checkCondNormalization();
  p_jo.checkCondNormalization();
  evid.checkCondNormalization();
  

  arr p1,p2,p3,p4;
  //*** naive inference:
  //first construct big joint, then marginalize
  
  Factor joint(LIST(eq, bl, al, ma, jo));
  joint.setOne();
  tensorMultiply(joint, p_eq);
  tensorMultiply(joint, p_bl);
  tensorMultiply(joint, p_al);
  tensorMultiply(joint, p_ma);
  tensorMultiply(joint, p_jo);
  tensorMultiply(joint, evid);
  
  Factor post_bl;
  tensorMarginal(post_bl, joint, LIST(bl));
  cout <<"joint marginal posterior = " <<post_bl <<endl;
  post_bl.getP(p1);

  //-- get a list of factors for the remaining algorithms
  FactorList factors;
  factors.append(&p_bl);
  factors.append(&p_eq);
  factors.append(&p_al);
  factors.append(&p_ma);
  factors.append(&p_jo);
  factors.append(&evid);

  //*** elimination
  eliminationAlgorithm(post_bl,factors,LIST(bl));
  cout <<"elimination algorithm posterior = " <<post_bl <<endl;
  post_bl.getP(p2);

  //*** JTA inference
  //-- version 1
  FactorGraph fg;
  JunctionTree::junctionTreeInference(fg, factors, variables);
  getMarginal(post_bl, LIST(bl), fg);
  cout <<"JTP bl posterior = " <<post_bl <<endl;
  post_bl.getP(p3);
  
  //-- version 2
  FactorGraph fg2;
  JunctionTree::constructJunctionTree(fg2, factors, variables);
  treeInference(fg2.F(0),true);
  fg2.computeBeliefs();
  getMarginal(post_bl, LIST(bl), fg2);
  cout <<"JTP bl posterior = " <<post_bl <<endl;
  post_bl.getP(p4);

  //-- check
  CHECK(maxDiff(p1,p2,0)<1e-10 && maxDiff(p1,p3,0)<1e-10,"");

  cout <<"*** inference methods consistent" <<endl;
}


// creates HMM
void createHMM(FactorList& factors, VariableList& states, VariableList& obs, uint timesteps) {
  factors.clear();
  states.clear();
  obs.clear();
  
  uint i;

  //create two lists of variables
  for(i=0;i<timesteps;i++){
    states.append(new Variable(2, STRING("state"<<i)));
    obs   .append(new Variable(2, STRING("obs"<<i)));
  }


  Factor* f;
  // t=0
  f = new Factor(LIST(*states(0)), "[  0.5   0.5 ]");
  factors.append(f);
  // t>0
  for (i=0; i<timesteps-1; i++) {
    f = new Factor(LIST(*states(i+1), *states(i)), "[  0.9  0.2   0.1   0.8 ]");
    factors.append(f);
  }
  for (i=0; i<timesteps; i++) {
    f = new Factor(LIST(*obs(i), *states(i)), "[  0.95  0.3   0.05   0.7 ]");
    factors.append(f);
  }
}


// Test JunctionTree or BeliefPropagation on MarkovModel oder HiddenMarkovModel.
void testHMMInference(uint T) {
  FactorList factors;
  VariableList states;
  VariableList obs;
  createHMM(factors, states, obs, T);

  VariableList vars;
  vars.append(states);
  vars.append(obs);
  
  // Specifying some evidence
  factors.append(new Factor(LIST(*states(0)), "[  0.0   1.0 ]"));
  factors.append(new Factor(LIST(*states(1)), "[  1.0   0.0 ]"));
  factors.append(new Factor(LIST(*states(5)), "[  1.0   0.0 ]"));

  arr p1,p2;

  //JTA
  Factor post;
  FactorGraph fg;
  JunctionTree::junctionTreeInference(fg, factors, vars);
  getMarginal(post, LIST(*states(2)), fg);
  cout <<"JTA posterior = " <<post <<endl;
  post.getP(p1);

  //BP
  FactorGraph fg2;
  LoopyBP_obsolete::loopy_belief_propagation(fg2, factors);
  getMarginal(post, LIST(*states(2)), fg2);
  cout <<"BP posterior = " <<post <<endl;
  post.getP(p2);

  //-- check
  CHECK(maxDiff(p1,p2,0)<1e-10,"");

  cout <<"*** inference methods consistent" <<endl;
}



// loopy graphical model tested with JunctionTree und LoopyBP
void testLoop(){
  Variable A(2, "A");
  Variable B(2, "B");
  Variable C(2, "C");
  VariableList vars = LIST(A,B,C);
  
  arr coupling;
  coupling.setText("[ .75 .25 ; .25 .75]");
  //coupling.setText("[ 1 0 ; 0 1]");
  
  Factor f_ab(LIST(A,B), coupling);
  Factor f_bc(LIST(B,C), coupling);
  Factor f_ca(LIST(C,A), coupling);
  FactorList facs = LIST(f_ab, f_bc, f_ca);
  
  arr p_evid;
  p_evid.setText("[.2 .8]"); 
  Factor evid(LIST(A), p_evid);
  facs.append(&evid);
  
  writeEdges(facs);
  write(facs);
  
  FactorGraph fg;
  LoopyBP_obsolete::loopy_belief_propagation(fg, facs);
  Factor marginal;
  uint i;
  for(i=0;i<vars.N;i++){
    getMarginal(marginal, LIST(*vars(i)), fg);
    cout <<marginal <<endl;
  }
}

// implement messages `a-to-b-without-c'
// works but only for up to 3 variables...
void testLoopEcho() {
  Variable A(2, "A");
  Variable B(2, "B");
  Variable C(2, "C");
  VariableList vars = LIST(A,B,C);
  
  arr coupling;
  coupling.setText("[ .75 .25 ; .25 .75]");
  //coupling.setText("[1 0 ; 0 1]");
  
  Factor f_ab(LIST(A,B), coupling);
  Factor f_bc(LIST(B,C), coupling);
  Factor f_ca(LIST(C,A), coupling);
  FactorList facs = LIST(f_ab, f_bc, f_ca);
  
  arr p_evid;
  p_evid.setText("[.2 .8]");
  Factor f_a(LIST(A), p_evid);
  Factor f_b(LIST(B));
  Factor f_c(LIST(C));
  facs.append(LIST(f_a, f_b, f_c));

  //messages
  Factor f,b;
  MT::Array<Factor> mu(3,3,3);
  uint i,j,k;
  for(i=0;i<3;i++) for(j=0;j<3;j++) for(k=0;k<3;k++){ initFactor(mu(i,j,k),TUPLE(vars(j))); mu(i,j,k).setOne(); }

  //exact solution
  eliminationAlgorithm(f,facs,LIST(A));  cout <<"exact marginal A=" <<f <<endl;
  eliminationAlgorithm(f,facs,LIST(B));  cout <<"exact marginal B=" <<f <<endl;
  eliminationAlgorithm(f,facs,LIST(C));  cout <<"exact marginal C=" <<f <<endl;
  
  //loop
  for(uint k=0;k<10;k++){
    //A->B
    f=f_a;  tensorMultiply(f,mu(C,A,B));  tensorProductMarginal(mu(A,B,B), f_ab, f, LIST(A));
    f=f_a;  tensorMultiply(f,mu(C,A,C));  tensorProductMarginal(mu(A,B,C), f_ab, f, LIST(A));
    b=f_b;  tensorMultiply(b,mu(A,B,B));  tensorMultiply(b,mu(C,B,B));
    cout <<"\nA->B: b(b) = " <<b <<endl;
    cout <<"\n  mu_AB/B=" <<mu(A,B,B) <<"\n  mu_CB/B=" <<mu(C,B,B) <<"\n  mu_AB/C=" <<mu(A,B,C) <<endl;
    
    //B->C
    f=f_b;  tensorMultiply(f,mu(A,B,A));  tensorProductMarginal(mu(B,C,A), f_bc, f, LIST(B));
    f=f_b;  tensorMultiply(f,mu(A,B,C));  tensorProductMarginal(mu(B,C,C), f_bc, f, LIST(B));
    b=f_c;  tensorMultiply(b,mu(B,C,C));  tensorMultiply(b,mu(A,C,C));
    cout <<"\nB->C: b(c) = " <<b <<endl;
    cout <<"\n  mu_AC/C=" <<mu(A,C,C) <<"\n  mu_BC/C=" <<mu(B,C,C) <<"\n  mu_BC/A=" <<mu(B,C,A)  <<endl;
    
    //C->A
    f=f_c;  tensorMultiply(f,mu(B,C,A));  tensorProductMarginal(mu(C,A,A), f_ca, f, LIST(C));
    f=f_c;  tensorMultiply(f,mu(B,C,B));  tensorProductMarginal(mu(C,A,B), f_ca, f, LIST(C));
    b=f_a;  tensorMultiply(b,mu(B,A,A));  tensorMultiply(b,mu(C,A,A));
    cout <<"\nC->A: b(a) = " <<b <<endl;
    cout <<"\n  mu_BA/A=" <<mu(B,A,A) <<"\n  mu_CA/A=" <<mu(C,A,A) <<"\n  mu_CA/B=" <<mu(C,A,B)  <<endl;

    MT::wait();
    
    //A->C
    f=f_a;  tensorMultiply(f,mu(B,A,C));  tensorProductMarginal(mu(A,C,C), f_ca, f, LIST(A));
    f=f_a;  tensorMultiply(f,mu(B,A,B));  tensorProductMarginal(mu(A,C,B), f_ca, f, LIST(A));
    b=f_c; tensorMultiply(b,mu(A,C,C));  tensorMultiply(b,mu(B,C,C));
    cout <<"\nA->C: b(c) = " <<b <<endl;
    cout <<"\n  mu_AC/C=" <<mu(A,C,C) <<"\n  mu_BC/C=" <<mu(B,C,C) <<"\n  mu_AC/B=" <<mu(A,C,B)  <<endl;

    //C->B
    f=f_c;  tensorMultiply(f,mu(A,C,A));  tensorProductMarginal(mu(C,B,A), f_bc, f, LIST(C));
    f=f_c;  tensorMultiply(f,mu(A,C,B));  tensorProductMarginal(mu(C,B,B), f_bc, f, LIST(C));
    b=f_b;  tensorMultiply(b,mu(A,B,B));  tensorMultiply(b,mu(C,B,B));
    cout <<"\nC->B: b(b) = " <<b <<endl;
    cout <<"\n  mu_AB/B=" <<mu(A,B,B) <<"\n  mu_CB/B=" <<mu(C,B,B) <<"\n  mu_CB/A=" <<mu(C,B,A)  <<endl;
    
    //B->A
    f=f_b;  tensorMultiply(f,mu(C,B,A));  tensorProductMarginal(mu(B,A,A), f_ab, f, LIST(B));
    f=f_b;  tensorMultiply(f,mu(C,B,C));  tensorProductMarginal(mu(B,A,C), f_ab, f, LIST(B));
    b=f_a;  tensorMultiply(b,mu(B,A,A));  tensorMultiply(b,mu(C,A,A));
    cout <<"\nB->A: b(a) = " <<b <<endl;
    cout <<"\n  mu_BA/A=" <<mu(B,A,A) <<"\n  mu_CA/A=" <<mu(C,A,A) <<"\n  mu_BA/C=" <<mu(B,A,C) <<endl;

    MT::wait();
  }
}

#include "gridBP.cpp"
            
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

  arr coupling;
  coupling.setText("[ .75 .25 ; .25 .75]");
  
  arr post;
  gridBP(post, evid, coupling);
}


#include "rndNetBP.cpp"
void testRndNetBP(){
  uint order=MT::Parameter<uint>("order");
  uint N    =MT::Parameter<uint>("N");
  double conn =MT::Parameter<double>("conn");
  rndNetBPwithExcludeEchoMessages(N,2,conn,order);
}

void testPairBP(){
  FactorList facs;
  VariableList vars;
  randomPairNet(vars,facs,3,2,1);
  listWrite(vars,cout,"\n  ");  cout <<endl;
  listWrite(facs,cout,"\n  ");  cout <<endl;
  loopyBP_pairwise(vars,facs,10);
  loopyBP_bipartite(vars,facs,10);
  //meanField_pairwise(vars,facs,10);
}

#ifdef MT_DAI
void testDai(){
  FactorList facs;
  VariableList vars;
  randomPairNet(vars,facs,10,2,1);
  listWrite(vars,cout,"\n  ");  cout <<endl;
  listWrite(facs,cout,"\n  ");  cout <<endl;
  DaiModule dai;
  createDai(dai,vars,facs);
  inference(dai);
}
#endif

int main(int argc, char** argv){
  //testBurglary();
  //testHMMInference(10);
  testLoop();
  //testLoopEcho();
  //testGridBP();
  //testRndNetBP();
  //testPairBP();
  //testDai();
  
  return 0;
}



