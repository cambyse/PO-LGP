void testEvidenceDiscounting(){
  BinaryBPNet net;
  net.setRandom(8,.5,.2,1.);
  net.init();
  arr phi,J,b,M;
  uint i,t;
  for(t=0;t<100;t++) net.stepBP();

  net.getInfo(phi,J,b,M);
  cout <<"evidences: " <<phi <<endl;
  cout <<"couplings:\n" <<J <<endl;
  cout <<"messages:\n" <<M <<endl;
  cout <<"beliefs: " <<b <<endl;

  for(t=0;t<100;t++) net.stepBP();
  net.getNodeBeliefs(b);
  cout <<"beliefs: " <<b <<endl;
  
  arr b0=b;

  Rprop rprop;
  //rprop.dMax=.01;
  //rprop.incr=1.1;
  rprop.init(.01);
  arr gamma(net.nodes.N); gamma=1.;
  arr grad(net.nodes.N);
  arr diff(net.nodes.N);
  
  uint k;
  BinaryBPNet::node *n;
  for(k=0;k<10;k++){
    cout <<"iteration " <<k <<endl;
    // relax to get reference
    for(t=0;t<100;t++) net.stepBP();
    net.getNodeBeliefs(b0);
    cout <<"beliefs: " <<b0 <<endl;

    // test perturbation
    double eps=1e-6;
    for_list(i,n,net.nodes){
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
    cout <<"diffs: " <<diff <<endl;
    //cout <<"diff norm: " <<length(diff) <<endl;

    //compute gradient:
    for(t=0;t<100;t++) net.stepGradBP();
    net.getGradT(grad);
    cout <<"grads: " <<grad <<endl;
    //cout <<"grad norm: " <<length(grad) <<endl;
                   
    //adapt discounts...
#if 1
    grad -= 1.;
    rprop.step(gamma,grad);
#else
    gamma /= grad;
#endif
    for_list(i,n,net.nodes) n->discount=gamma(i);
    cout <<"gamma= " <<gamma <<endl;
  }

  /////////////////////////////////////////////////////////////////
  // my standard loopy BP:
  LoopyBP lbp;
  convert(lbp,net);
  MT::Array<Factor> bb;
  for(t=0;t<100;t++) lbp.step();
  lbp.getVarBeliefs(bb);
  for(i=0;i<bb.N;i++) b(i) = .5 * log(bb(i).P(1)/bb(i).P(0));
  cout <<"beliefs (old LBP)   = " <<b <<endl;
  /////////////////////////////////////////////////////////////////
  //elimination:
  Factor post;
  for(i=0;i<b.N;i++){
    eliminationAlgorithm(post,lbp.facs,TUP(i));
    b(i) = .5 * log(post.P(1)/post.P(0));
  }
  cout <<"beliefs (exact)     = " <<b <<endl;

  cout <<"beliefs (corrected) = " <<-b0 <<endl;
}
