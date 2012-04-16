#include "aicot.h"
#include "MT/plot.h"

void AICOT::init(soc::SocSystemAbstraction& sys_){
  AICO::init(sys_);
  tauCostRate = MT::Parameter<double>("Tcost",2e7);

  int K  = sys->nTime();
  int n  = 2*sys->qDim();//xDim

  p.resize(K+1,n);
  P.resize(K+1,n,n);
  PP.resize(K,n,n);
}

void AICOT::iterate_to_convergence(const arr* q_initialization){
        calcEStep(q_initialization);
        std::ofstream file("Q.dat");
        for(int n = 0; n < MaxEMIter; ++n){
            //sys->displayTrajectory(q,NULL,1,"AICO (planned trajectory)");
            //std::cout << "Beliefs:" <<std::endl;
            calcBeliefs();
            //std::cout << "MStep :" <<std::endl;
            //if(n == 0){
            //}
            //calcMStepNumerical();
            calcMStep();
            //std::cout << "EStep :" <<std::endl;
            //calcEStep();
//             if(delta_dt < MinDtEpsilon){
//                 break;
//             }
            //stats.tau.push_back(sys->getTau());
            //stats.cost.push_back(sys->analyzeTrajectory(b,false));
        }
        file.close();
}

void AICOT::calcEStep(const arr* q_init){
  AICO::iterate_to_convergence(q_init);
}

void AICOT::calcBeliefs(){
    int K = sys->nTime();
    //one slice
    for(int k = 0; k <= K; ++k){
        p[k] = b[k];
        //P[k] = Binv[k];
        inverse_SymPosDef(P[k](),Binv[k]);
        //std::cout << "P[" << k << "] = " << P[k] << std::endl;
    }
    //two slice
    arr A,tA,invA,invtA,a,B,tB,Q,H,invH,invJ,invW;
    for(int k = 0; k < K; ++k){
        //get local process
        if(sys->dynamic){
            sys->setx(b[k]); //!setx instead of DZ setqv?
        } else {
            sys->setq(b[k]);
        }
        sys->getQ(Q,k); //
        sys->getH(H,k);
        inverse_SymPosDef(invH,H);
        sys->getProcess(A,tA,invA,invtA,a,B,tB,k);
        //calc cross cov
        inverse_SymPosDef(invW,Q+B*invH*tB);
        //arr RR = S[k] + R[k];
        //arr SS = tA*invW*A;
        inverse_SymPosDef(invJ, Sinv[k] + R[k] + tA*invW*A);
        arr Tmp = invJ*tA*invW*P[k+1];
        PP[k] = Tmp;
        //std::cout << "PP[" << k << "] = " << PP[k] << std::endl;
    }
}

void  AICOT::getLocalProcess(arr& A, arr& a, arr& B, uint k){
  uint n=sys->qDim();
  if(!sys->dynamic){
      A.resize(n,n); A.setZero();
      B.setId(n);
      a.resize(n);
      a.setZero();
  }else{
    arr I,Z,Minv,F;
    I.setId(n);
    Z.resize(n,n); Z.setZero();


    sys->getMinvF(Minv,F,k);
    //std::cout << "Minv = " << Minv << std::endl;
    //std::cout << "F = " << F << std::endl;


    A.setBlockMatrix(Z,I,
                     Z,Z);
    //double alpha = .1; //with fricion
    //A.setBlockMatrix(I,tau*I-tau*alpha*Minv,Z,I-tau*alpha*Minv);

    B.resize(2*n,n);
    B.setZero();
    //B.setMatrixBlock(tau*tau*Minv,0,0);
    B.setMatrixBlock(Minv,n,0);

    a.resize(2*n);
    a.setZero();
    //a.setVectorBlock(tau*tau*Minv*F,0);
    a.setVectorBlock(Minv*F,n);
  }
}

double AICOT::calcQFun(double dt){
    int K = sys->nTime();

    arr M_0,M_1,M_01,cur_p, nxt_p, tmp, a, A, tA, B, tB, Q, H, invH, invW;
    int dim = 2*sys->qDim();//xDim
    arr I; I.setId(dim);
    double val = 0;

    nxt_p = p[0];
    for(int k = 0; k < K; ++k){
        cur_p = nxt_p;
        nxt_p = p[k+1];

        getLocalProcess(A,a,B,k);
        A = I + dt*A;
        a = dt*a;
        B = dt*B;
        transpose(tA, A);
        transpose(tB, B);
        sys->getQrate(Q);
        Q = dt*Q;
        sys->getHrateInv(invH);
        invH = invH/dt;
        //inverse_SymPosDef(invH,H);

        outerProduct(tmp,cur_p,cur_p);
        M_0  = P[k] + tmp;
        //std::cout << " M_0 = " << M_0 << std::endl;

        outerProduct(tmp,nxt_p,nxt_p);
        M_1  = P[k+1] + tmp;
        //std::cout << " M_1 = " << M_1 << std::endl;

       outerProduct(tmp,cur_p,nxt_p);
       M_01 = PP[k] + tmp;

       //invW = Q+B*invH*tB;
       inverse_SymPosDef(invW,Q+B*invH*tB);
       val += log(dt)*dim/2 + 0.5*trace(invW*M_1) - trace(A*invW*M_01) + 0.5*trace(A*invW*tA*M_0) + dt*tauCostRate;
    }
    val = -val;
    return val;
}

void AICOT::plotQFun(){
    double max = 1, step = 0.001, dt = step;
    arr f; f.resize((int)(max/step)+1,2);
    ofstream file("Q.dat");
    for(int k = 0; dt < max; dt+=step, ++k){
        f(k,0) = dt;
        f(k,1) = calcQFun(dt);
        file << f(k,0) << " " << f(k,1) << std::endl;
    }
    file.close();
}

void AICOT::calcMStepNumerical(){
    /*for debug purposes*/
    std::ofstream file("Q.dat");
    //plotQFun();
    double max = 10000, step = 10, dt = step;
    double opt = calcQFun(0.01);
    for(dt+=step; dt < max; dt += step){
        double v = calcQFun(dt);
        if(v > opt){
            opt = v;
        } else {
            double opt_dt = dt - step;
            if(opt_dt <= 0){opt_dt = step;}
            sys->setTau(opt_dt);
            std::cout << "dt* = " << opt_dt << std::endl;
            break;
        }
    }
    file.close();
    //std::cout << "FAIL" << std::endl;

}

void AICOT::calcMStep()
{
  double u = 0, v = 0, w = 0;
  double opt_dt;
  int K = sys->nTime();

  int DimX = 2*sys->qDim();//xDim

  arr M_0,M_1,M_01,cur_p, nxt_p, tmp, a, A, tA, B, tB, Q, H, invH, invW;

  nxt_p = p[0];
  for(int k = 0; k < K; ++k) {
      cur_p = nxt_p;
      nxt_p = p[k+1];
      //std::cout << cur_p << std::endl;
      if(sys->dynamic){
         sys->setx(b[k]); //!setx instead of DZ setqv?
      } else {
        sys->setq(p[k]);
      }
      getLocalProcess(A,a,B,k);
      transpose(tA, A);
      transpose(tB, B);
      sys->getQrate(Q);
      //std::cout << "Q = " << Q << std::endl;
      sys->getHrateInv(invH);
     // inverse_SymPosDef(invH,H);

      //std::cout << "A = " << A << "a = " << a << "B = " << B << std::endl;
      outerProduct(tmp,cur_p,cur_p);
      M_0  = P[k] + tmp;
      //std::cout << " M_0 = " << M_0 << std::endl;

      outerProduct(tmp,nxt_p,nxt_p);
      M_1  = P[k+1] + tmp;
      //std::cout << " M_1 = " << M_1 << std::endl;

      outerProduct(tmp,cur_p,nxt_p);
      M_01 = PP[k] + tmp;
      //std::cout << " M_01= " << M_01 << std::endl;

      inverse_SymPosDef(invW,Q+B*invH*tB);
      //std::cout << "T = " << M_0 - (M_01+M_01) + M_1 << std::endl;
      //std::cout << "U = " << invW*(M_0 - (M_01+M_01) + M_1) << std::endl;

      u += trace(invW*(M_0 - (M_01+M_01) + M_1));
      v += DimX*DimX;
      w += trace(A*invW*tA*M_0) + 2*tauCostRate;
      w -= scalarProduct(tmp, a, invW*a);
      w -= scalarProduct(tmp, a, invW*A*cur_p);

      /*
              double det = v*v + 4*u*w;
              if(0 < det){
                  opt_dt = 2*u/(v + sqrt(det));
                  std::cout << "dt*[" << k << "] = " << opt_dt << std::endl;
              } else {
              CHECK(false, "MStep Error!!");
                  std::cout << "MStep Error Determinant = " << det << "< 0";
              }
              dt(k) = opt_dt;
      */
    }

    //std::cout << "partition " << i << " " << K0 << " - " << K <<std::endl;
    //std::cout << u << " " << v << " " << w << std::endl;
    double det = v*v + 4*u*w;
    if(0 < det)
    {
      opt_dt = 2*u/(v + sqrt(det));
      std::cout << "dt* = " << opt_dt << std::endl;
    }
    else
    {
      //CHECK(false, "MStep Error!!");
      std::cout << "MStep Error Determinant = " << det << "< 0... skipping tau update" << std::endl;
      opt_dt = sys->getTau();
    }
    sys->setTau(opt_dt);
}
