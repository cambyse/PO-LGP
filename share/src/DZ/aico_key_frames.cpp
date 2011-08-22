#include <MT/soc.h>
#include <MT/array.h>
#include <MT/array_t.cpp>
#include <MT/algos_LU.cpp>
#include <MT/opengl.h>
#include <aico_key_frames.h>

void OneStepKinematic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha)
{
  sys.dynamic = false;
  arr R,r,Hinv,Q,Winv,W;
  arr q0,q_old,tp;
  sys.getq0(q0);
  sys.getQ(Q,T);
  b=q0;

  arr q_ar = arr(14); // GetQ doesn't check 'dynamic'
  for (int i=0;i<14;i++) q_ar(i) = Q(i,i);

  sys.getHinv(Hinv,T);
  double old_r,dr=1e6;
  W = (diag(q_ar)+Hinv);
  inverse_SymPosDef(Winv,W);
  double D =2.8;

  for (int k=0;k<100;k++){
    q_old = b;
    sys.setq(b,T);
    sys.getCosts(R,r,b,T);
    transpose(tp,(b-q0));
    if(  (sys.taskCost(NULL,T,-1)  + sum(tp*Winv*(b-q0)) +D) >old_r) alpha=alpha*0.5;
    else alpha=pow(alpha,0.5);  
    inverse_SymPosDef(Winv,W*D);   // M-step here
    Binv = Winv+ R;
    lapack_Ainv_b_sym(b, Binv,  Winv*q0  + r);
    b = q_old + alpha*(b-q_old);
    transpose(tp,(b-q0));
    dr = old_r;
    old_r = sys.taskCost(NULL,T,-1)+ sum(tp*Winv*(b-q0))+D;
    dr -= old_r; 
    cout << old_r << endl;
    if (fabs(dr)<1e0) break;
  }
  cout << D << endl;
}


void OneStepDynamic(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,double alpha)
{
  arr R,r,Hinv,Q,Winv,W,A,B,tB,a,sumA,Ai,tAi,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;
  sys.getqv0(q0,v0);
  sys.getQ(Q,T);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!
  b=qv0;
  sys.getHinv(Hinv,T);
  sys.getProcess(A,a,B,T);
  transpose(tB,B);
  double old_r,dr=1e6;
  //sys.getW(W,T);
  W = Q + B*Hinv*tB ;

  inverse_SymPosDef(Winv,W);
  double D =T;

  sumA=W;
  suma=a;
  Ai=A;
  transpose(tAi,Ai);
  cout <<Hinv;
  //cout <<Ai*W*tAi;
  for (int i=1;i<T+1;i++) {
    sumA += Ai*W*tAi;
    suma += Ai*a;
    Ai*=A;
    transpose(tAi,Ai);
  }
  cout <<"---------------------\n";
  cout <<5.0*A;

  inverse_SymPosDef(sumAinv,sumA);
  suma+= A*Ai*qv0;

  for (int k=0;k<100;k++){
    q_old = b;
    bq=q0;bv=q0;
    for (int i=0; i< 14;i++){ bq(i)=b(i); bv(i)=b(i+14);} // can not set joint state q and v in one variable
    sys.setqv(bq,bv);
    sys.getCosts(R,r,b,T);
    transpose(tp,b-qv0);

    if(  sys.taskCost(NULL,T,-1)+ sum(tp*sumAinv*(b-qv0)) +D >old_r) alpha=alpha*0.5;
    else alpha=pow(alpha,0.5);  


    Binv = sumAinv+ R;
    lapack_Ainv_b_sym(b, Binv,  sumAinv*suma  + r);
    b = q_old + alpha*(b-q_old);

    transpose(tp,(b-qv0));
    dr = old_r;
    old_r = sys.taskCost(NULL,T,-1)+ sum(tp*sumAinv*(b-qv0)) +D;
    dr -= old_r; 
    cout << old_r << endl;
    if (fabs(dr)<1e0) break;
  }
  cout << D << endl;
}


void decomposeMatrix(arr& A1,arr& A2,arr A){
  int dim = 14;
  A.resize(2*dim,2*dim);
  A1.resize(dim,dim);
  A2.resize(dim,dim);
  for (int i=0;i<14;i++)
    for (int j=0;j<14;j++)
    {A1(i,j) = A(i,j); 
      A2(i,j) = A(14+i,14+j);
    }  
}

double SumOfRow(int p,int k){
  double sum=0;
  switch(k){
    case 0: sum=p;  break;
    case 1: sum=p*(p+1)/2.0;  break;
    case 2: sum=p*(p+1)*(2.0*p+1)/6.0;  break;
    default: NIY;  
  }
  if (sum>0)
    return sum;
  else return 1;
}

double dSumOfRow(int p,int k){
  double sum=0;
  switch(k){
    case 0: sum=1.0;  break;
    case 1: sum=p +0.5;  break;
    case 2: sum=p*p+p +1.0/6.0;  break;
    default: NIY;  
  }
  return sum;    
}

void OneStepDynamicGradient(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,arr& R,arr& r,double alpha)
{
  arr Hinv,Rinv,Q,Winv,W,A,B,tB,a,sumA,Ai,tAi,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;

  arr Minv,F,B1,B2,H1,H2,Q1,Q2;

  double tau=sys.getTau(false);//+1e-1;
  double T = sys.nTime();
  sys.getQ(Q,T);
  Q = Q/sqrt(tau);
  tau = tau*T;
  T=1.0;

  double tau2=tau*tau;
  int dim =14;
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!

  b=qv0;
  sys.getHinv(Hinv,1);

  H1=Hinv;
  decomposeMatrix(Q1,Q2,Q);

  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 

  arr Lx,La,LA; //parameters of Gaussian - Likelihood;

  AT.setBlockMatrix(I,SumOfRow(T,0)*tau*I,Z,I); 

  //Dynamic case
  // arr sumAa; sumAa.resize(2*dim);
  //     sumAa.setZero();
  //     sumAa.setVectorBlock(tau*tau*(I*Minv*F*0.5  + SumOfRow(T-1,1)*I*Minv*F),0);
  //     sumAa.setVectorBlock(tau*I*Minv*F,dim);
  //inverse_SymPosDef(Rinv,R);
  Lx= AT*qv0 ;
  arr dLx; 
  dLx = cat(1.0*T*I*v0,Zv);
  //La= Rinv*r;

  arr sigma1,sigma2,sigma3,sigma4;
  double S0 = SumOfRow(T-1,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);
  //double dS0 = dSumOfRow(T-1,0);double dS1 = dSumOfRow(T-1,1);double dS2 = dSumOfRow(T-1,2);
  //double S0=1.0;double S1=1.0;double S2=1.0;
  //S0=S0/T; S1=S1/T;S2=S2/T; 
  //double dS0=1.0;double dS1=1.0;double dS2=1.0;

  sigma1 = S0*(tau2*tau2*H1+Q1*sqrt(tau)) +2.0*S1*tau2*tau2*H1 + S2*tau2*(tau2*H1+sqrt(tau)*Q2);
  sigma2 = S0*tau2*tau*H1 + S1*tau*(tau2*H1 + sqrt(tau)*Q2);
  sigma3 = sigma2;
  sigma4 = S0*(tau2*H1 + sqrt(tau)*Q2);

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  double D = 1.0*tau*T;

  // Dsigma1 = 4.0*tau*tau2*H1*(S0 + 2.0*S1 +S2) + S0*Q1/sqrt(tau) +2.5*tau*sqrt(tau)*S2*Q2;
  // Dsigma4 = 2.0*tau*S0*H1 +S0*Q2/sqrt(tau);
  // Dsigma2 = 3.0*tau2*H1*(S0+   S1) +1.5*S1*Q2*sqrt(tau);
  // Dsigma3 = Dsigma2;

  Dsigma1 = 4.0*D*D*D*H1*(S0 + 2.0*S1 +S2)/pow(T,4) + 0.5*S0*Q1/sqrt(D) +2.5*D*sqrt(D)*S2*Q2/pow(T,2.5);
  Dsigma4 = 2.0*D*S0*H1/pow(T,2) +S0*Q2/sqrt(D*T);
  Dsigma2 = 3.0*D*D*H1*(S0+   S1)/pow(T,3) +1.5*S1*Q2*sqrt(D)/pow(T,1.5);
  Dsigma3 = Dsigma2;

  // Dsigma1 = dS0*(tau2*tau2*H1+Q1*sqrt(tau)) +2.0*dS1*tau2*tau2*H1 + dS2*tau2*(tau2*H1+sqrt(tau)*Q2);
  // Dsigma2 = dS0*tau2*tau*H1 + dS1*tau*(tau2*H1 + sqrt(tau)*Q2);
  // Dsigma3 = Dsigma2;
  // Dsigma4 = dS0*(tau2*H1 + sqrt(tau)*Q2);

  LA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 
  //LA = 2.0*LA;
  arr dLA,LAinv;

  //inverse_SymPosDef(LAinv,LA);
  inverse(LAinv,LA);

  arr h,th,sumInv;
  //inverse_SymPosDef(sumInv,(LAinv+R));
  inverse(sumInv,(LAinv+R));

  h = LAinv*sumInv*(R*Lx - r);
  transpose(th,h);


  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 
  //dLA = 2.0*dLA;
  cout << "HELL\n";
  arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  (LAinv*sumInv*R) *dLA);
  cout << "\n Gradient=" << gradient<< endl;


}
////--------------------
// Full dynamic versions

void OneStepDynamicFull_old(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double time,double alpha)
{
  arr H1,R,r,Hinv,Q,B,sumA,Q1,Q2,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;
  double tau=sys.getTau(false);// we need this tau only to get pure Q1 and Q2
  double T = sys.nTime();
  //initial state
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!
  b=qv0;

  double old_r,dr=1e6;

  int dim=14;
  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();  
  AT.setBlockMatrix(I,I,Z,I);  // A to the power of T 
  sys.getHinv(Hinv,1);
  H1=Hinv;
  sys.getQ(Q,T);
  Q = Q/sqrt(tau); // Pure Q.
  decomposeMatrix(Q1,Q2,Q);
  tau = time;//tau*T; // tau is basically = time


  double tau2=tau*tau;
  double rtau=sqrt(tau); // terms come from the definition of Q
  double rT = sqrt(T);
  double S0 = SumOfRow(T,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);  // sums of geometric series
  arr sigma1,sigma2,sigma3,sigma4; // Blocks of sigma matrix
  sigma1 = tau2*tau2*H1*(S0+2.0*S1 + S2)/pow(T,4) + S2*tau2*rtau*Q2/pow(T,2.5)+ S0*Q1*rtau/rT;
  sigma2 = tau2*tau*H1*(S0+S1)/pow(T,3) + S1*tau*rtau*Q2/pow(T,1.5);
  sigma3 = sigma2;
  sigma4 = S0*(tau2*H1/pow(T,2.0) + Q2*rtau/rT);

  double D =tau;

  sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 

  inverse_SymPosDef(sumAinv,sumA);
  suma= AT*qv0;

  for (int k=0;k<100;k++){
    q_old = b;
    bq=q0;bv=q0;
    for (int i=0; i< 14;i++){ bq(i)=b(i); bv(i)=b(i+14);} // can not set joint state q and v in one variable
    sys.setqv(bq,bv);
    sys.getCosts(R,r,b,T); // costs at the current position
    transpose(tp,b-qv0);

    if(  sys.taskCost(NULL,T,-1)+ sum(tp*sumAinv*(b-qv0)) +D >old_r) alpha=alpha*0.5;
    else  
      alpha=pow(alpha,0.5);  



    Binv = sumAinv+ R;
    lapack_Ainv_b_sym(b, Binv,  sumAinv*suma  + r);
    b = q_old + alpha*(b-q_old);

    transpose(tp,(b-qv0));
    dr = old_r;
    old_r = sys.taskCost(NULL,T,-1)+ sum(tp*sumAinv*(b-qv0)) ;//+D;
    dr -= old_r; 
    //if (k>0) alpha = 0.005*fabs(dr); // !!
    cout << old_r << endl;
    if (fabs(dr)<1e0) break;
    sys.gl->watch("dd");
  }
  cout << D << endl;
}

void OneStepDynamicFull(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,double time,double alpha)
{
  arr H1,R,r,Hinv,Q,B,sumA,Q1,Q2,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;
  double tau=sys.getTau(false);// we need this tau only to get pure Q1 and Q2
  double T = sys.nTime();
  //initial state
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!
  b=qv0;
  bq=q0;bv=q0; // defines size

  double old_r,dr=1e6;

  int dim=14;
  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();  
  AT.setBlockMatrix(I,I,Z,I);  // A to the power of T 
  sys.getHinv(Hinv,1);
  H1=Hinv;
  sys.getQ(Q,T);
  Q = Q/sqrt(tau); // Pure Q.
  decomposeMatrix(Q1,Q2,Q);
  tau = time;//tau*T; // tau is basically = time


  double tau2=tau*tau;
  double rtau=sqrt(tau); // terms come from the definition of Q
  double rT = sqrt(T);
  double S0 = SumOfRow(T,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);  // sums of geometric series
  arr sigma1,sigma2,sigma3,sigma4; // Blocks of sigma matrix
  sigma1 = tau2*tau2*H1*(S0+2.0*S1 + S2)/pow(T,4) + S2*tau2*rtau*Q2/pow(T,2.5)+ S0*Q1*rtau/rT;
  sigma2 = tau2*tau*H1*(S0+S1)/pow(T,3) + S1*tau*rtau*Q2/pow(T,1.5);
  sigma3 = sigma2;
  sigma4 = S0*(tau2*H1/pow(T,2.0) + Q2*rtau/rT);

  double D =tau;

  sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 

  inverse_SymPosDef(sumAinv,sumA);
  suma= AT*qv0;

  // one run with very small alpha
  old_r = sys.taskCost(NULL,T,-1);

  arr b_old=qv0;
  arr b_best=qv0;
  for (uint i=0; i< 14;i++){ bq(i)=b(i); bv(i)=b(i+14);} // can not set joint state q and v in one variable
  sys.setqv(bq,bv);
  sys.getCosts(R,r,b,T); // costs at the current position

  Binv = sumAinv+ R;
  lapack_Ainv_b_sym(b, Binv,  sumAinv*suma  + r);
  b = b_old + alpha*(b-b_old);
  bool restore;

  for (uint k=0;k<100;k++){

    for (uint i=0; i< 14;i++){ bq(i)=b(i); bv(i)=b(i+14);} // can not set joint state q and v in one variable
    sys.setqv(bq,bv);
    sys.getCosts(R,r,b,T); // costs at the current position

    if(  sys.taskCost(NULL,T,-1)>old_r) {alpha=alpha*alpha; b=b_best; restore=true;}
    else  
    {
      if (!restore) alpha=pow(alpha,0.5); 
      Binv = sumAinv+ R;
      b_best = b;
      b_old = b;
      lapack_Ainv_b_sym(b, Binv,  sumAinv*suma  + r);
      b = b_old + alpha*(b-b_old);


      dr = old_r;
      old_r = sys.taskCost(NULL,T,-1);
      dr -= old_r; 
      if (!restore) cout << old_r << endl;
      if ((!restore) && fabs(dr)<1e0) break;
      restore = false;

      //sys.gl->watch("dd");
    }
  }
}

void OneStepDynamicGradientFull(double& grad,soc::SocSystemAbstraction& sys,arr& R,arr& r,double time)
{
  arr Hinv,Rinv,Q,Winv,W,A,B,sumA,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;

  arr H1,Q1,Q2;

  double tau=sys.getTau(false);
  double T = sys.nTime();
  sys.getQ(Q,T);
  Q = Q/sqrt(tau);
  tau = time; // tau is basically = time

  double tau2=tau*tau;
  double rtau=sqrt(tau); // terms come from definition of Q
  double rT = sqrt(T);
  int dim =14;
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!

  sys.getHinv(Hinv,1);

  H1=Hinv;
  decomposeMatrix(Q1,Q2,Q);

  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 

  arr Lx,La,LA; //parameters of Gaussian - Likelihood;

  AT.setBlockMatrix(I,I,Z,I);  // A to the power of T 

  Lx= AT*qv0 ;
  arr dLx; 
  dLx = cat(1.0*T*I*v0,Zv);

  arr sigma1,sigma2,sigma3,sigma4;
  double S0 = SumOfRow(T-1,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);
  // decouple TIME and T
  sigma1 = tau2*tau2*H1*(S0+2.0*S1 + S2)/pow(T,4) + S2*tau2*rtau*Q2/pow(T,2.5)+ S0*Q1*rtau/rT;
  sigma2 = tau2*tau*H1*(S0+S1)/pow(T,3) + S1*tau*rtau*Q2/pow(T,1.5);
  sigma3 = sigma2;
  sigma4 = S0*(tau2*H1/pow(T,2.0) + Q2*rtau/rT);

  /*
     sigma1 = tau2*tau2*H1*( 2.0*S0+2.0*S1 + S2)/pow(T,4) + S2*tau2*rtau*Q2/pow(T,2.5)+ 2.0*S0*Q1*rtau/rT;
     sigma2 = tau2*tau*H1*(2.0*S0+S1)/pow(T,3) + S1*tau*rtau*Q2/pow(T,1.5);
     sigma3 = sigma2;
     sigma4 = 2.0*S0*(tau2*H1/pow(T,2.0) + Q2*rtau/rT);*/

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  Dsigma1 = 4.0*tau*tau2*H1*(S0 + 2.0*S1 +S2)/pow(T,4) + 2.5*tau*rtau*S2*Q2/pow(T,2.5)     + 0.5*S0*Q1/(rtau*rT) ;
  Dsigma2 = 3.0*tau2*H1*(S0+ S1)/pow(T,3) +S1*rtau*Q2/pow(T,1.5);
  Dsigma3 = Dsigma2;
  Dsigma4 = 2.0*tau*S0*H1/pow(T,2.0)         + 0.5*S0*Q2/(rtau*rT);

  // Dsigma1 = 4.0*tau*tau2*H1*(2.0*S0 + 2.0*S1 +S2)/pow(T,4) + 2.5*tau*rtau*S2*Q2/pow(T,2.5)     + Q1/(rtau*rT) ;
  // Dsigma2 = 3.0*tau2*H1*(2.0*S0+ S1)/pow(T,3) +S1*rtau*Q2/pow(T,1.5);
  // Dsigma3 = Dsigma2;
  // Dsigma4 = 4.0*tau*S0*H1/pow(T,2.0)         +Q2/(rtau*rT);

  LA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 

  arr dLA,LAinv;

  inverse(LAinv,LA);
  arr h,th,sumInv;
  inverse(sumInv,(LAinv+R));
  h = LAinv*sumInv*(R*Lx - r);
  transpose(th,h);

  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 
  arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  (LAinv*sumInv*R) *dLA);
  cout << "\n Gradient=" << gradient<< endl;
  grad = sum(gradient);
}

void GetOptimalDynamicTime(double& time,soc::SocSystemAbstraction& sys,double alpha,double step)
{
  arr  R,r,b,Binv,q0,v0;
  double old_time=sys.getTau(false);//+1e-1;
  double T = sys.nTime();
  double gr,new_time;
  old_time*=T;
  sys.getqv0(q0,v0);
  arr b0=cat(q0,v0); 

  for (uint k=0;k<20;k++){
    sys.setqv(b0);
    OneStepDynamicFull(b,Binv,sys,old_time,alpha); // final posture estimation
    sys.setqv(b);
    sys.getCosts(R,r,b,T);
    OneStepDynamicGradientFull(gr,sys,R,r,old_time); // gradient of likelihood for a given time
    old_time = old_time + gr*step;
    cout << old_time;
  }

  time = old_time;
}
/////////////////////////////

void TwoStepDynamicGradient(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,arr& R,arr& r,double alpha)
{
  arr Hinv,Rinv,Q,Winv,W,A,B,tB,a,sumA,Ai,tAi,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;

  arr Minv,F,B1,B2,H1,H2,Q1,Q2;

  double tau=sys.getTau(false);//+1e-1;
  double T = sys.nTime();
  sys.getQ(Q,T);
  Q = Q/sqrt(tau);
  tau = tau*T;
  T=2.0;

  double tau2=tau*tau;
  int dim =14;
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!

  b=qv0;
  sys.getHinv(Hinv,1);

  H1=Hinv;
  decomposeMatrix(Q1,Q2,Q);

  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 

  arr Lx,La,LA; //parameters of Gaussian - Likelihood;

  AT.setBlockMatrix(I,2.0*tau*I,Z,I); 

  Lx= AT*qv0 ;
  arr dLx; 
  dLx = cat(1.0*T*I*v0,Zv);
  //La= Rinv*r;

  arr sigma1,sigma2,sigma3,sigma4;

  double D = 1.0*tau;

  sigma1 = 5.0*tau2*tau2*H1/pow(T,4.0)+2.0*Q1 +tau2*Q2/pow(T,2.0);
  sigma2 = 3.0*tau2*tau*H1/pow(T,3.0) + tau*Q2/pow(T,1.0);
  sigma3 = sigma2;
  sigma4 = 2.0*tau2*H1/pow(T,2.0) + 2.0*Q2;

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  Dsigma1 = 20.0*D*D*D*H1/pow(T,4.0) + 2.0*D*Q2/pow(T,2.0);
  Dsigma2 = 9.0*D*D*H1/pow(T,3.0) +Q2/T;
  Dsigma3 = Dsigma2;
  Dsigma4 = 4.0*D*H1/pow(T,2.0);

  // Dsigma1 = dS0*(tau2*tau2*H1+Q1*sqrt(tau)) +2.0*dS1*tau2*tau2*H1 + dS2*tau2*(tau2*H1+sqrt(tau)*Q2);
  // Dsigma2 = dS0*tau2*tau*H1 + dS1*tau*(tau2*H1 + sqrt(tau)*Q2);
  // Dsigma3 = Dsigma2;
  // Dsigma4 = dS0*(tau2*H1 + sqrt(tau)*Q2);

  LA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 
  //LA = 2.0*LA;
  arr dLA,LAinv;

  //inverse_SymPosDef(LAinv,LA);
  inverse(LAinv,LA);

  arr h,th,sumInv;
  //inverse_SymPosDef(sumInv,(LAinv+R));
  inverse(sumInv,(LAinv+R));

  h = LAinv*sumInv*(R*Lx - r);
  transpose(th,h);


  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 
  //dLA = 2.0*dLA;
  cout << "HELL\n";
  arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  (LAinv*sumInv*R) *dLA);
  cout << "\n Gradient=" << gradient<< endl;


}


void SimpleGradient(soc::SocSystemAbstraction& sys,arr& R,arr& r)
{
  arr Hinv,Rinv,Q,Winv,W,A,tB,a,sumA,Ai,tAi,sumAinv,suma,AInv;
  arr q0,q_old,tp,qv0,v0,bq,bv;

  arr Minv,F,B1,B2,H1,H2,Q1,Q2,E,H;

  double tau=sys.getTau(false);//+1e-1;

  sys.getQ(Q,1);
  Q = Q/sqrt(tau); 

  double D=3.01;
  double step = 1.0;
  int dim =14;
  sys.getqv0(q0,v0);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!
  sys.getHinv(Hinv,1);
  H1=Hinv;

  decomposeMatrix(Q1,Q2,Q);


  arr I,Z,AT,Zv;;

  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 

  arr Lx,La,LA,dLx,dLa,dLA,LAinv; //parameters of Gaussian - Likelihood;

  AT.setBlockMatrix(I,D*I,Z,I); 
  E.setBlockMatrix(Z,I,Z,Z); 
  Lx= AT*qv0 ;
  dLx = E*qv0;

  arr sigma1,sigma2,sigma3,sigma4;

  sigma1 = pow(D,4)*H1/pow(step,4)+Q1*sqrt(D)/sqrt(step);
  sigma2 = pow(D,3)*H1/pow(step,3);
  sigma3 = sigma2;
  sigma4 = D*D*H1/pow(step,2) + sqrt(D)*Q2/sqrt(step);

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  Dsigma1 = 4.0*pow(D,3)*H1/pow(step,4)+0.5*Q1/sqrt(D)/sqrt(step);
  Dsigma2 = 3.0*pow(D,2)*H1/pow(step,3);
  Dsigma3 = sigma2;
  Dsigma4 = 2.0*D*H1/pow(step,2) + 0.5*Q2/sqrt(D)/sqrt(step);

  LA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 
  //LA *=1e3;
  //inverse_SymPosDef(LAinv,LA);
  inverse(LAinv,LA);
  //inverse_SymPosDef(Rinv,R);
  //inverse_SymPosDef(AInv,(LA+Rinv));

  arr h,th,sumInv;
  //inverse_SymPosDef(sumInv,(LAinv+R));
  inverse(sumInv,(LAinv+R));

  h = LAinv*sumInv*(R*Lx - r);

  transpose(th,h);

  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 
  cout<< H1*pow(D,4);
  arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  (LAinv*sumInv*R) *dLA);
  //arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  AInv *dLA);
  cout<< dLA ;
  cout << "\n Simple Gradient=" << gradient<< endl;


}

void OneStepDynamicGradientT(arr& b,arr& Binv, soc::SocSystemAbstraction& sys,uint T,arr& R,arr& r,double alpha)
{
  arr Hinv,Rinv,Q,Winv,W,A,B,tB,a,sumA,Ai,tAi,sumAinv,suma;
  arr q0,q_old,tp,qv0,v0,bq,bv;

  arr Minv,F,B1,B2,H1,H2,Q1,Q2;
  double tau=sys.getTau(false);
  int dim =14;

  sys.getqv0(q0,v0);
  sys.getQ(Q,T);
  qv0=cat(q0,v0);  //q0 with velocity!!!!!!
  b=qv0;
  sys.getHinv(Hinv,T);
  sys.getProcess(A,a,B,T);
  sys.getMinvF(Minv,F,T-3);
  decomposeMatrix(H1,H2,Hinv);
  decomposeMatrix(Q1,Q2,Q);

  arr I,Z,AT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 

  arr Lx,La,LA; //parameters of Gaussian - Likelihood;

  AT.setBlockMatrix(I,SumOfRow(T,0)*tau*I,Z,I); 
  Lx= AT*qv0+qv0 ;
  arr dLx; 
  dLx = cat(1.0*I*tau*v0,Zv);
  cout << Lx<<endl;
  //La= Rinv*r;

  arr sigma1,sigma2,sigma3,sigma4;
  double tau2=tau*tau;
  double S0 = SumOfRow(T-1,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);
  double dS0 = dSumOfRow(T-1,0);double dS1 = dSumOfRow(T-1,1);double dS2 = dSumOfRow(T-1,2);

  sigma1 = S0*(tau2*tau2*H1+Q1) +2.0*S1*tau2*tau2*H1 + S2*tau2*(tau2*H1+Q2);
  sigma2 = S0*tau2*tau*H1 + S1*tau*(tau2*H1 + Q2);
  sigma3 = sigma2;
  sigma4 = S0*(tau2*H1 + Q2) ;

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  Dsigma1 = dS0*(tau2*tau2*H1+Q1) +2.0*dS1*tau2*tau2*H1 + dS2*tau2*(tau2*H1+Q2) -  4.0*S0*tau2*tau2/T*H1 -8.0*S1/T*tau2*tau2*H1 -4.0*S2*tau2*tau2/T*H1;  ;
  Dsigma2 = dS0*tau2*tau*H1 + dS1*tau*(tau2*H1 + Q2) - 3.0*S0*tau*tau2/T*H1 -  3.0*S1*tau2*tau/T*H1;
  Dsigma3 = Dsigma2;
  Dsigma4 = dS0*(tau2*H1 + Q2) - 2.0*S0*(tau2/T*H1);

  LA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 
  arr dLA,LAinv;
  cout << sum(LA)<<"Works\n";

  //LAinv=LA*1e15;
  inverse_SymPosDef(LAinv,LA);
  arr h,th,sumInv;
  inverse_SymPosDef(sumInv,(LAinv+R));

  h = LAinv*sumInv*(R*Lx - r);
  transpose(th,h);

  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 

  arr gradient = -th*dLx + 0.5*th*dLA*h - 0.5*trace(  (LAinv*sumInv*R) *dLA);
  cout << gradient<< endl;


}

void OneStepKinematicT(arr& b,arr& Binv,double& duration, soc::SocSystemAbstraction& sys,uint T,double alpha)
{

  sys.dynamic = false;
  arr R,r,Hinv,Q,Winv,Winv0,W,Wsys,A,Ainv,A2inv,dif,dif2,ad2,A2,a2,Rinv,a2t,X,t0,t1,t2,Rs,h,ht,Minv,F;
  arr q0,q_old,tp,cost,qt,bt,rt,ga,ta,a,at;
  sys.getq0(q0);
  sys.getQ(Q,T);
  sys.getW(Wsys,T);
  b=q0;
  arr data=arr(60,2);
  double tau = sqrt(sys.getTau());
  sys.getHinv(Hinv,T);
  double old_r,dr=1e6;
  W = Q/tau+Hinv;
  //W = Wsys; // test only
  inverse_SymPosDef(Winv,W);
  inverse_SymPosDef(Winv0,W);
  double D = 0.27405;//0.15;//2.0; //0.25;
  double  tr,dL,step,dist;
  int k; 
  for (k=0;k<100;k++){
    q_old = b;
    sys.setq(b,T);
    sys.getCosts(R,r,b,T);
    transpose(tp,(b-q0));
    //	if(  (sys.taskCost(NULL,T,-1)  + sum(tp*Winv*(b-q0)) +D) >old_r) alpha=alpha*0.5; 
    //	else alpha=pow(alpha,0.5); 
    inverse_SymPosDef(Winv,W*0.27405);   
    Binv = Winv+ R;
    lapack_Ainv_b_sym(b, Binv,  Winv*q0  + r);
    b = q_old + alpha*(b-q_old);
    transpose(tp,(b-q0));
    dr = old_r;
    old_r = sys.taskCost(NULL,T,-1)+ sum(tp*Winv*(b-q0))+D;
    dr -= old_r; 
    if (fabs(dr)<1e0) break;
    cout << old_r << endl;
    //} whole cycle
    //sys.getMinvF(Minv,F,T);
    transpose(qt,q0);
    transpose(bt,b);
    Winv = Winv0; 
    D=0.12;//0.12;//0.007;//0.25;
    dL=1.0;
    cout << r << endl;
    sys.setq(b,T);
    sys.getCosts(R,r,b,T);  
    //D=0.067;
    double grad_step=1e-5;
    for (int c=0;c<300;c++){
      A = (Winv/D+R);
      a = (Winv/D)*q0+r;
      transpose(at,a);
      inverse_SymPosDef(Ainv,A);
      A2inv = (Winv/D)*Ainv*R;
      h = A2inv*q0 - (Winv/D)*Ainv*r;
      transpose(ht,h);
      dif = (ht*W*h  -trace(A2inv*W) );
      // cout << "ML\n"; 

      dL = sum(dif);
      grad_step =0.00001*fabs(dL);
      D = D +grad_step*dL;


      if ((fabs(dL)<1e-0)||(D<1e-6)) break;
    }
    cout <<  dL  << endl;
    cout << D <<endl; 
    dist = sum(tp*Wsys*(b-q0))/14.0;
    cout << dist <<endl; 
    if (D>1e-4) data[k]= cat(ARRAY(dist),ARRAY(D));
    //if ( (dist>270)&&(dist<290) ) {ofstream os("inter.point"); b.writeRaw(os); os.close(); }
    //data[k](2) = sum(tp*Wsys*(b-q0))/14.0;
}
duration = D;
data.removeAllValues(0);
data.resize(k,2);
//delRows(30,1);

cout << "ctrl\n";
cout << sum(tp*W*(b-q0))<<endl;
cout <<data;

gnuplot(data);

//for (int i=0;i<size(data,1);i++)

sys.gl->watch();
}
