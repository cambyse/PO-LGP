#include <MT/soc.h>
#include <Core/array.h>
#include <Core/array_t.h>
#include <Gui/opengl.h>
#include <aico_key_frames.h>

double OneStepKinematic(arr& q, arr& _Binv, uint& counter, soc::SocSystemAbstraction& sys, double stopTolerance, bool q_is_initialized){
  int steps = sys.get_T();
  arr R,r,H,Q,Winv,W;
  arr q0,q_old,tp,Binv;
  //sys.getH(H,0); //H_step
  //sys.getQ(Q,0); //Q_step
  
  sys.getq0(q0);
  if(!q_is_initialized) q=q0;
  counter = 0; // number of iterations
  
  double alpha = .001;
  double old_r = 0.;
  W = double(steps)*(Q+H);
  inverse_SymPosDef(Winv,W);
  
  for (int k=0;k<100;k++){
    q_old = q;
    sys.setq(q);
    sys.displayState(NULL, NULL, "posture", true);
    sys.gl->watch();
    sys.getTaskCosts(R, r, q, steps);
    counter++; // Basically counts number of getTaskCosts calls
    if(sys.taskCost(NULL,steps,-1) + sum(~(q-q0)*Winv*(q-q0)) > old_r) alpha=alpha*0.5;
    else alpha=pow(alpha,0.5); 
    Binv = Winv + R;
    lapack_Ainv_b_sym(q, Binv, Winv*q0 + r);
    q = q_old + alpha*(q-q_old);
    old_r = sys.taskCost(NULL,steps,-1) + sum(~(q-q0)*Winv*(q-q0));
    cout <<"cost=" <<old_r << endl;
    if (maxDiff(q, q_old)/alpha<stopTolerance) break;
  }
  
  if(&_Binv) _Binv=Binv;
  return old_r;
}

void decomposeMatrix(arr& A1,arr& A2,arr A){ // returns diagonal blocks of equal size
  int dim = sqrt(A.N)/2;
  A.resize(2*dim,2*dim);
  A1.resize(dim,dim);
  A2.resize(dim,dim);
  for (int i=0;i<dim;i++)
    for (int j=0;j<dim;j++)
    {A1(i,j) = A(i,j); 
      A2(i,j) = A(dim+i,dim+j);
    }  
}

double SumOfRow(int p,int k){ // sum of geometric series
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

double LogLikelihood(const arr& x,const arr& a,const arr& A)
{
  double llk=0.0;
  arr diff_tp,Ainv;
  static const double pi = 3.14159265358979; 
  double det = lapack_determinantSymPosDef(A); 
  arr diff = (x - a);
  inverse_SymPosDef(Ainv,A);
  llk = sum (log(  1.0/sqrt(2.0*pi*det)) - 0.5*~diff*Ainv*diff);
  return llk;
}

double OneStepDynamicFull(arr& b,arr& Binv, uint& counter,
                        soc::SocSystemAbstraction& sys,
                        double time,double alpha,double task_eps,double eps_alpha,
			uint verbose, bool b_is_initialized)
{
  //if(!sys.dynamic)  return OneStepKinematic(b, Binv, sys, time, alpha);
  //CHECK(sys.dynamic,"call this function only for dynamic systems");
  arr H1,R,Rinv,r,Q,B,sumA,Q1,Q2,sumAinv,suma;
  arr x0; 
  double T = sys.get_T();
  //initial state
  sys.getx0(x0);
  if(!b_is_initialized) b=x0;
  double old_r;
  counter = 0; // number of iterations
  //sys.getHrateInv(H1);
  //sys.getQrate(Q);

  decomposeMatrix(Q1,Q2,Q);
  double tau = time;// tau is basically = time
  double tau2=tau*tau;
  int dim=sqrt(Q.N)/2;
  
  arr I,Z,AT,Zv;
  I.setId(dim); Z.resize(dim,dim); Z.setZero();
  AT.setBlockMatrix(I,time*I,Z,I);  // A to the power of T
 
  double S0 = SumOfRow(T,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);  // sums of geometric series
  arr sigma1,sigma2,sigma3,sigma4; // Blocks of sigma matrix
  sigma1 = tau2*tau*H1*(S0+2.0*S1 + S2)/pow(T,3) + tau2*tau*Q2*S2/pow(T,3)+ tau*S0*Q1/T;
  sigma2 = tau2*H1*(S0+S1)/pow(T,2) + tau2*S1*Q2/pow(T,2);
  sigma3 = sigma2;
  sigma4 = tau*S0*(H1 + Q2)/T;
  
  sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4);
  inverse_SymPosDef(sumAinv,sumA);
  suma= AT*x0;

 arr b_old = b;  arr b_best = b;
  sys.setx(b_old);
  old_r = sys.taskCost(NULL,T,-1);
  bool restore;
     
  for (uint k=0;k<1000;k++){
    sys.setx(b);
   
    if ((sys.taskCost(NULL, T, -1) + sum(~(b-x0)*sumAinv*(b-x0)) >old_r)&&(!restore)){
      alpha=alpha*0.5; //failure
      b=b_best;
      restore=true;
    }else{
      if (!restore) alpha=pow(alpha,0.5); //success
      sys.getTaskCosts(R,r,b,T); // costs at the current position
      counter++; // Basically counts number of getTaskCosts calls
      double eps=1e-10; arr id; id.setId(dim*2);R= R+eps*id; //Trick against small negative eigenvalues of R
      Binv = sumAinv+ R;
      b_best = b; b_old = b;
      lapack_Ainv_b_sym(b, Binv,  sumAinv*suma  + r);
      b = b_old + alpha*(b-b_old);

      cout <<MT_HERE <<"cost=" <<old_r <<" step_size=" <<alpha <<endl;
      if ((!restore) && (k>1) && ((fabs(alpha)<eps_alpha) || ((old_r - sys.taskCost(NULL, T, -1) - sum(~(b-x0)*sumAinv*(b-x0)) )<task_eps))) break;
      
      old_r = sys.taskCost(NULL, T, -1, verbose) + sum(~(b-x0)*sumAinv*(b-x0)); // task+control costs
      restore = false;

      if(verbose>0) sys.gl->update();
      if(verbose>1) sys.displayState(NULL, NULL, "posture estimate", true);
      if(verbose>2) sys.gl->watch();
    }
  }
  b=b_best;
  return old_r;
}

void OneStepDynamicGradientFull(double& grad,double& likelihood,soc::SocSystemAbstraction& sys,arr& R,arr& r,double time)
{
  arr Rinv,Q,Winv,W,A,B,sumA,sumAinv,suma;
  arr x0,q0,q_old,qv0,v0,bq,bv;

  arr H1,Q1,Q2;
  double T = sys.get_T();
  double tau = time; // tau is basically = time
  double tau2=tau*tau;
  //sys.getHrateInv(H1);
  //sys.getQrate(Q);
  
  decomposeMatrix(Q1,Q2,Q);
  
  int dim =sqrt(Q.N)/2;;
  sys.getx0(x0);
  sys.getqv0(q0,v0);
 
  arr I,Z,AT,dAT,Zv;
  I.setId(dim);
  Z.resize(dim,dim); Z.setZero();    Zv.resize(dim); Zv.setZero(); 
  AT.setBlockMatrix(I,time*I,Z,I);  // A to the power of T 
  dAT.setBlockMatrix(Z,I,Z,Z);
  arr Lx,dLx,dLA; //parameters of Gaussian - Likelihood;

  Lx= AT*x0 ;
  dLx = dAT*x0; 

  arr sigma1,sigma2,sigma3,sigma4;
  double S0 = SumOfRow(T-1,0);double S1 = SumOfRow(T-1,1);double S2 = SumOfRow(T-1,2);
  sigma1 = tau2*tau*H1*(S0+2.0*S1 + S2)/pow(T,3) + tau2*tau*Q2*S2/pow(T,3)+ tau*S0*Q1/T;
  sigma2 = tau2*H1*(S0+S1)/pow(T,2) + tau2*S1*Q2/pow(T,2);
  sigma3 = sigma2;
  sigma4 = tau*S0*(H1 + Q2)/T;

  arr Dsigma1,Dsigma2,Dsigma3,Dsigma4;

  Dsigma1 = 3.0*tau2*H1*(S0 + 2.0*S1 +S2)/pow(T,3) + 3*tau2*S2*Q2/pow(T,3)     + S0*Q1/T ;
  Dsigma2 = 2.0*tau*H1*(S0+ S1)/pow(T,2) +2.0*S1*Q2/pow(T,2);
  Dsigma3 = Dsigma2;
  Dsigma4 = S0*(H1 + Q2)/T;
  
  sumA.setBlockMatrix(sigma1,sigma2,sigma3,sigma4); 

  double eps=1e-10; arr id; id.setId(dim*2);R= R+eps*id; //Trick against small negative eigenvalues of R
  inverse_SymPosDef(Rinv,R);
  arr h,sumInv;
  inverse_SymPosDef(sumInv,(sumA+Rinv));
  h = sumInv*(Lx - Rinv*r);

  dLA.setBlockMatrix(Dsigma1,Dsigma2,Dsigma3,Dsigma4); 
  grad = sum(-~h*dLx+ 0.5*~h*dLA*h - 0.5*trace(  sumInv*dLA));
  cout << "\nGradient=" << grad<< endl;
  likelihood= LogLikelihood(x0, Rinv*r, sumA+Rinv);
  cout<<"Log-Likelihood ="<<  likelihood<<endl;
}

void GetOptimalDynamicTime(double& time, int& counter,
			   arr& b,arr& Binv,soc::SocSystemAbstraction& sys,
			   double alpha,double task_eps,double eps_alpha,double step,
			   double min_step, uint verbose){
  arr  R,r,q0,x0;
  double old_time=sys.getTau(false);//+1e-1;
  double T = sys.get_T();
  old_time*=T;
  arr lk;
  double gr=1e10;
  sys.getx0(x0);
  arr b0=x0;  arr b_old=b0; 
  double old_r = 1e6;
  double old_llk = -1e6;
  double llk;
  uint cnt;
  counter = 0;
 while (step>min_step) {
    sys.setx(b0);
    OneStepDynamicFull(b,Binv,cnt,sys,old_time,alpha,task_eps,eps_alpha,verbose,false); // final posture estimation
    counter+=cnt;
    sys.setx(b);
    if (sys.taskCost(NULL,T,-1)<old_r) { // in case best costs do not coincide with the best time
      sys.getTaskCosts(R,r,b,T);
      b_old=b;
      old_r = sys.taskCost(NULL,T,-1);
    }
    else
      sys.getTaskCosts(R,r,b_old,T);
 
    OneStepDynamicGradientFull(gr,llk,sys,R,r,old_time); // gradient of likelihood for a given time and costs
    old_time = old_time + step*gr/fabs(gr);
    if (llk>old_llk){
      old_llk=llk; 
      step = sqrt(step);
    }
    else step = 0.5*step;
    
    cout << "old_time="<<old_time<<endl;
    lk.append(llk);
   }
  b=b_old;
  cout << "R of best b:"<<old_r<<endl;;
if (verbose){
  lk.reshape(lk.N,1);
  ofstream fil("like");
  lk.write(fil," ","\n","  ");
  fil.close();
  gnuplot("plot 'like' us 1");
  sys.gl->watch();
}
  time = old_time;
}
