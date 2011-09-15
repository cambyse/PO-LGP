void interpolateTrajectory(arr& x,uint t1, uint T, const arr& x0, const arr& x1, const arr& xT){
  uint t;
  double a;
  x.resize(T+1,x0.N);
  for(t=0; t<=t1; t++){
    a = (double)t/t1;
    x[t]() = ((double)1.-a)*x0 + a*x1;
  }
  for(t=t1; t<=T; t++){
    a = (double)(t-t1)/(T-t1);
    x[t]() = ((double)1.-a)*x1 + a*xT;
  }
}

