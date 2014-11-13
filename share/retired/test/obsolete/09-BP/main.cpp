#define MT_IMPLEMENTATION

#include <MT/BinaryBP.h>

#include <Core/array.h>
//#include <Algo/algos.h>
#include <MT/infer.h>

void convert(LoopyBP& lbp,BinaryBPNet& net){
  BinaryBPNet::node *n;
  BinaryBPNet::edge *e;
  Factor *f;
  Variable *v;
  VariableList vars;
  FactorList facs;
  uint i;
  for_list(Type, n, net.nodes) vars.append(new Variable(2,STRING("var"<<i)));
  for_list(Type, e, net.edges){
    f=new Factor(TUPLE(vars(e->ifrom),vars(e->ito)));
    facs.append(f);
    double g=exp(e->J);
    f->setP(ARR(g,1./g,1./g,g));
  }
  connectThemUp(vars,facs);
  //-- generate evidence factors
  Factor evid;
  for_list(Type, n, net.nodes){
    v=vars(i);
    CHECK(v->factors.N,"");
    f=v->factors(0);
    initFactor(evid,TUPLE(v));
    double g=exp(n->evidence);
    evid.setP(ARR(g,1./g));
    tensorMultiply(*f,evid);
  }

  lbp.initBipartite(vars,facs);
}

//================================================================================

void TEST(EvidenceDiscounting){
  BinaryBPNet net;
  net.setRandom(8,.5,.2,1.);
  net.init();
  arr phi,J,b,M;
  uint i,t;
  for(t=0;t<100;t++) net.stepBP();

  net.getInfo(phi,J,b,M);
  cout <<"** random network:" <<endl;
  cout <<"  evidences = " <<phi <<endl;
  cout <<"  couplings =\n" <<J <<endl;
  cout <<"  messages =\n" <<M <<endl;
  cout <<"  beliefs = " <<b <<endl;

  arr b0=b;

  //Rprop rprop;
  //rprop.dMax=.01;
  //rprop.incr=1.1;
  //rprop.init(.01);
  arr gamma(net.nodes.N); gamma=1.;
  arr grad,gradDiag;
  arr diff(net.nodes.N);
  
  uint k;
  BinaryBPNet::node *n;
  for(k=0;k<10;k++){
    cout <<"** iteration " <<k <<endl;
    // relax to get reference
    for(t=0;t<100;t++) net.stepBP();
    net.getNodeBeliefs(b0);
    cout <<"  beliefs = " <<b0 <<endl;

    // test perturbation
    double eps=1e-6;
    for_list(Type, n, net.nodes){
      n->evidence += eps;
      for(t=0;t<100;t++) net.stepBP();
      net.getNodeBeliefs(b);
      //cout <<"delta: " <<b-b0 <<endl;
      diff(i) = b(i)-b0(i);
      n->evidence -= eps;
    }
    
    // relax again
    for(t=0;t<100;t++) net.stepBP();
    net.getNodeBeliefs(b);
    cout <<"error: " <<length(b-b0) <<endl;
    diff /= eps;
    cout <<"perturbation grad = " <<diff <<endl;
    //cout <<"diff norm: " <<length(diff) <<endl;

    //compute gradient:
    net.initNodeBeliefGradients();
    for(t=0;t<100;t++) net.stepGradBP();
    net.getGradT(grad);
    getDiag(gradDiag,grad);
    cout <<"computed grad     = " <<gradDiag <<endl;
    //cout <<"grad norm: " <<length(grad) <<endl;
                   
    //adapt discounts...
#if 0
    grad -= 1.;
    rprop.step(gamma,grad);
#else
    gamma /= gradDiag;
#endif
    for_list(Type, n, net.nodes) n->discount=gamma(i);
    cout <<"adapted gamma= " <<gamma <<endl;
  }

  /////////////////////////////////////////////////////////////////
  // my standard loopy BP:
  LoopyBP lbp;
  convert(lbp,net);
  MT::Array<Factor> bb;
  for(t=0;t<100;t++) lbp.step();
  lbp.getVarBeliefs(bb);
  for(i=0;i<bb.N;i++) b(i) = .5 * log(bb(i).P(1)/bb(i).P(0));
  cout <<"\n** beliefs:" <<endl;
  cout <<"plain BP     = " <<b <<endl;
  /////////////////////////////////////////////////////////////////
  //elimination:
  Factor post;
  for(i=0;i<b.N;i++){
    eliminationAlgorithm(post,lbp.facs,TUPLE(i));
    b(i) = .5 * log(post.P(1)/post.P(0));
  }
  cout <<"exact        = " <<b <<endl;

  cout <<"corrected BP = " <<-b0 <<endl;
}

//================================================================================

void TEST(Gradient){
  uint t,i;
  
  BinaryBPNet net;
  net.setRandom(8,.5,.2,1.);
  net.initNodeBeliefGradients();
  for(t=0;t<100;t++) net.stepBP();
  for(t=0;t<100;t++) net.stepGradBP();

  arr Gt,GJ,Gt2,GJ2,b,b0,bb,bb0,delta_b,delta_bb;
  net.getNodeBeliefs(b0);
  net.getPairBeliefs(bb0);
  net.getGradT(Gt);
  net.getGradJ(GJ);
  cout <<"node beliefs = " <<b0 <<endl;
  cout <<"pair beliefs = " <<bb0 <<endl;
  
  net.initPairBeliefGradients();
  for(t=0;t<100;t++) net.stepGradBP();
  net.getGradT(Gt2);
  net.getGradJ(GJ2);

  //-- test evidence perturbation
  BinaryBPNet::node *n;
  double eps=1e-6;
  delta_b.resize(net.nodes.N);
  delta_bb.resize(net.nodes.N);
  for_list(Type, n, net.nodes){
    n->evidence += eps;
    for(t=0;t<100;t++) net.stepBP();
    net.getNodeBeliefs(b);
    net.getPairBeliefs(bb);
    delta_b(i) = (b(0)-b0(0))/eps;
    delta_bb(i) = (bb(0)-bb0(0))/eps;
    n->evidence -= eps;
  }
  cout <<"gradBP  node/theta = " <<(~Gt)[0] <<endl;
  cout <<"perturb node/theta = " <<delta_b <<endl;
  cout <<"gradBP  pair/theta = " <<(~Gt2)[0] <<endl;
  cout <<"perturb pair/theta = " <<delta_bb <<endl;
    
  // relax again
  for(t=0;t<100;t++) net.stepBP();
  net.getNodeBeliefs(b);
  cout <<"error: " <<length(b-b0) <<endl;

  //-- test coupling perturbation
  BinaryBPNet::edge *e;
  eps=1e-6;
  delta_b.resize(net.edges.N);
  delta_bb.resize(net.edges.N);
  for_list(Type, e, net.edges){
    e->J += eps;
    for(t=0;t<100;t++) net.stepBP();
    net.getNodeBeliefs(b);
    net.getPairBeliefs(bb);
    delta_b(i) = (b(0)-b0(0))/eps;
    delta_bb(i) = (bb(0)-bb0(0))/eps;
    e->J -= eps;
  }
  cout <<"gradBP  node/J = " <<(~GJ)[0] <<endl;
  cout <<"perturb node/J = " <<delta_b <<endl;
  cout <<"gradBP  pair/J = " <<(~GJ2)[0] <<endl;
  cout <<"perturb pair/J = " <<delta_bb <<endl;

  // relax again
  for(t=0;t<100;t++) net.stepBP();
  net.getNodeBeliefs(b);
  cout <<"error: " <<length(b-b0) <<endl;

}

//================================================================================

void TEST(ParameterLearning){
  /*BinaryBPNet net;
  net.setRandom(8,.5,.2,1.);

  uintA samples;
  net.getSamples(samples,100);
  cout <<samples <<endl;

  BinaryBPNet net2;
  net2.setRandom(8,1.,.01,.01);
  arr grad;

  for(s=0;s<S;s++){
    
  }*/
  
}

//================================================================================

int main(int argc, char** argv){
  testEvidenceDiscounting();
  //testGradient();
  //testParameterLearning();
  return 0;
}
