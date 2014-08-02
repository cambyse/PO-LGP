#ifndef IOC_H
#define IOC_H

#include <Ors/ors.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Optim/optimization.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <vector>
#include <future>
#include <GL/glu.h>
#include <Gui/opengl.h>

struct IOC_DemoCost {
  arr x0; // Demonstrated joint space
  arr lambda0; // Demonstrated constrains
  arr lambda; // Current lambda estimate
  arr Dwdx; // Matrix mapping x to w
  uint numParam; // number of x parameter

  // task vars
  arr J_Jt,PHI,J,JP;
  // constrain vars
  arr Jg,g, JgP,Jg_Jgt,Jg_JgtP;
  arr J_Jgt;
  arr dWdx_dPHI_J_G_Jt_dPHI_dWdx;
  arr Jgt_JgJgtI_Jg;
  arr dPHI_J_Jt_dPHI;
  arr JgJgtI_Jg_J_dPHI;

  IOC_DemoCost(MotionProblem &_MP,arr &_x0,arr &_lambda0, arr &_Dwdx,uint _numParam):x0(_x0),lambda0(_lambda0),Dwdx(_Dwdx),numParam(_numParam) {
    // precompute some terms
    MotionProblemFunction MPF(_MP);
    ConstrainedProblem & v = Convert(MPF);
    v.fc(NoArr,NoArr,g,JgP,_x0);
    cout << g << endl;

    MT::timerStart();
    // reduce Jg to only active part (lambda !=0)
    MT::Array<uint> idx;
    lambda0.findValues(idx,0.);
    lambda0.removeAllValues(0.);
    Jg = unpack(JgP);
    arr Jgr;
    uint j = 0;
    for (uint i =0;i<Jg.d0;i++) {
      if(!idx.contains(i)) {
        Jgr.append(~Jg[i]);
      }
      if(idx.contains(i)) {
        JgP.delRows(j);
        ((RowShiftedPackedMatrix*)JgP.aux)->rowShift.remove(j);
        j--;
      }
      j++;
    }
    Jg = Jgr;

    Jg_JgtP = comp_A_At(JgP);
    Jg_Jgt = unpack(Jg_JgtP); Jg_Jgt.special = arr::noneST;
    Jgt_JgJgtI_Jg = ~Jg*inverse_SymPosDef(Jg_Jgt)*Jg;
    PHI = v.y;
    JP = v.Jy;

    arr J_JtP = comp_A_At(JP);
    J = unpack(JP); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_JtP);
    cout << MT::timerRead(true) << endl;
    dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - ( (PHI*~PHI)%(J*Jgt_JgJgtI_Jg*~J));
    cout << MT::timerRead(true) << endl;
    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;
    JgJgtI_Jg_J_dPHI = inverse_SymPosDef(Jg_Jgt)*Jg*~J*diag(PHI);
    J_Jgt = J*~Jg;
  }

  double eval(arr& df, arr& Hf,arr& gLambda, arr& JgLambda, const arr& x) {
    // compute w vector
    arr w = Dwdx*x;
    arr PHIw = PHI%w;
    arr JP_PHIw = comp_At_x(JP,PHIw);
    arr J_Jt_PHIw = comp_A_x(JP,JP_PHIw);
    arr JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
    arr J_G_Jt_PHIw = J_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);

    lambda = -JgJgtI_Jg_Jt_PHIw;

    arr f = 4.*(~PHIw)*J_G_Jt_PHIw;
    double y = f(0);

    if (&df) {
      arr h = 8.*(PHI%(J_G_Jt_PHIw));
      df = ~h*Dwdx ;
      df.flatten();
    }
    if (&Hf) {
      Hf = 8.*(dWdx_dPHI_J_G_Jt_dPHI_dWdx);
    }
    if (&gLambda) {
      gLambda = 2.*JgJgtI_Jg_Jt_PHIw;
    }
    if (&JgLambda) {
      JgLambda = 2.*JgJgtI_Jg_J_dPHI ;
    }
    return y;
  }
};


struct Demonstration {
  MotionProblem& MP; // MP containing the world state,
  arr x;             // joint trajectory
  arr lambda;        // constraint trajectory
  IOC_DemoCost* cost;// cost function for this demonstrations

  Demonstration (MotionProblem &_MP):MP(_MP) {
  }
};


struct IOC:ConstrainedProblem {
  MT::Array<Demonstration*> &demos;
  arr xOpt;
  uint numParam;
  uint numLambda;
  arr Dwdx;
  uint n;
  uint T;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() { return numParam+numLambda+1;}

  IOC(MT::Array<Demonstration*> &_demos,uint _numParam):demos(_demos),numParam(_numParam) {
    n = demos(0)->MP.world.getJointStateDimension();
    T = demos(0)->MP.T;

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      Dwdx.append(catCol(eye(n),zeros(n,numParam-n)));

      // add task cost elements
      for (uint c=0;c<demos(0)->MP.taskCosts.N;c++) {
        if ( demos(0)->MP.taskCosts(c)->prec.N >t && (demos(0)->MP.taskCosts(c)->prec(t) > 0) && demos(0)->MP.taskCosts(c)->active && !demos(0)->MP.taskCosts(c)->map.constraint) {

          uint m;
          if ((demos(0)->MP.taskCosts(c)->target.N-1) == T) {
            m = demos(0)->MP.taskCosts(c)->target.d1;
          } else {
            m = demos(0)->MP.taskCosts(c)->target.N;
          }

          arr tmp = zeros(m,n);
          tmp = catCol(tmp,zeros(m,c));
          tmp = catCol(tmp,ones(m,1));
          tmp = catCol(tmp,zeros(m,numParam-tmp.d1));
          Dwdx.append(tmp);
        }
      }
    }
    cout << "Dwdx: " << Dwdx << endl;

    numLambda = 0;
    // initialize cost functions for demonstrations
    for (uint i=0;i<demos.d0;i++) {
      demos(i)->cost = new IOC_DemoCost(demos(i)->MP,demos(i)->x,demos(i)->lambda,Dwdx,numParam);
      numLambda += demos(i)->cost->lambda0.d0;
    }
  }


  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    xOpt = x;

    double y = 0.;
    if (&df) {
      df.resize(numParam);df.setZero();
    }
    if (&Hf) {
      Hf.resize(numParam,numParam);Hf.setZero();
    }


    if (&g) {
      g.clear();
      g.append((sumOfSqr(x)-1)*(sumOfSqr(x)-1)); // ||w|| = 1
      g.append(-x); // w > 0
    }
    if (&Jg) {
      Jg.clear();
      Jg=4.*~x*(sumOfSqr(x)-1);
      Jg.append(-eye(numParam));
    }

    // iterate over demonstrations
    for (uint i=0;i<demos.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      y += demos(i)->cost->eval(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, x);

      if (&df) {
        df = df + dfi;
      }
      if (&Hf) {
        Hf += Hfi;
      }
      if (&g) {
        g.append(gi);
      }
      if (&Jg) {
        Jg.append(Jgi*Dwdx);
      }
    }
    return y;
  }

  void costReport() {
    cout << "\n \n The solution is " <<xOpt << endl;
    for (uint i=0;i<demos.d0;i++) {
      cout << "\nDemonstration " << i << endl;
      cout << "Costs: " << demos(i)->cost->eval(NoArr,NoArr,NoArr,NoArr,xOpt) << endl;
      cout << "Lambda: " << demos(i)->cost->lambda/sqrt(sumOfSqr(demos(i)->cost->lambda)) << endl;
      cout << "Lambda GT: " << demos(i)->cost->lambda0/sqrt(sumOfSqr(demos(i)->cost->lambda0)) << endl;
    }
  }
};


#endif // IOC_H
