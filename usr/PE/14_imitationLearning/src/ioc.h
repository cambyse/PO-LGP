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

//#include "scene.h"



struct IOC_DemoCost {
  arr x0; // Demonstrated joint space
  arr lambda0; // Demonstrated constrains
  arr lambda; // Current lambda estimate
  arr Dwdx; // Matrix mapping x to w
  uint numParam; // number of x parameter

  bool constrainsActive; // constrained or unconstrained variant
  bool learnTransitionCosts;

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
    v.fc(NoArr,NoArr,g,JgP,x0);
    cout << g << endl;

    for (uint i =0;i<g.d0;i++){
      lambda0(i) = g(i)>=0.;
    }
    cout << lambda0 << endl;

    MT::timerStart();

    PHI = v.y;
    JP = v.Jy;

    arr J_JtP = comp_A_At(JP);
    J = unpack(JP); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_JtP);

    if (lambda0.N==0 || max(lambda0) == 0.) {
      /// Unconstrained case
      constrainsActive = false;
      dWdx_dPHI_J_G_Jt_dPHI_dWdx = ~Dwdx*dPHI_J_Jt_dPHI*Dwdx;
      //    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;

    } else {
      /// Constrained case
      constrainsActive = true;
      // reduce Jg to only active part (lambda !=0)
      MT::Array<uint> idx;
      lambda0.findValues(idx,0.);
      lambda0.removeAllValues(0.);
      Jg = unpack(JgP);
      cout << lambda0 << endl;
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
      //      dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - ( (PHI*~PHI)%(J*Jgt_JgJgtI_Jg*~J));
      arr tmp = ~J*(diag(PHI)*Dwdx);
      dWdx_dPHI_J_G_Jt_dPHI_dWdx = ~Dwdx*dPHI_J_Jt_dPHI*Dwdx - (~tmp*Jgt_JgJgtI_Jg*tmp);
      JgJgtI_Jg_J_dPHI = inverse_SymPosDef(Jg_Jgt)*Jg*~J*diag(PHI);
      J_Jgt = J*~Jg;
    }

    //    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;
  }

  double eval(arr& df, arr& Hf,arr& gLambda, arr& JgLambda, const arr& x) {
    // compute w vector
    arr augx = x;
    if (!learnTransitionCosts) {
      augx(0) = 0.;
    }

    arr w = Dwdx*augx;
    arr PHIw = PHI%w;
    arr JP_PHIw = comp_At_x(JP,PHIw);
    arr J_Jt_PHIw = comp_A_x(JP,JP_PHIw);
    arr J_G_Jt_PHIw = J_Jt_PHIw;

    arr JgJgtI_Jg_Jt_PHIw;
    if (constrainsActive) {
      JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
      J_G_Jt_PHIw = J_G_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);
      lambda = -JgJgtI_Jg_Jt_PHIw;
    }

    arr f = 4.*(~PHIw)*J_G_Jt_PHIw;
    double y = f(0);
    //    cout << "f: " << f << endl;
    if (&df) {
      arr h = 8.*(PHI%J_G_Jt_PHIw);
      df = ~h*Dwdx ;
      df.flatten();
//      df.subRange(0,6) = 0.;
      //      cout << "df: " << df << endl;
    }
    if (&Hf) {
      Hf = 8.*(dWdx_dPHI_J_G_Jt_dPHI_dWdx);
//      if (learnTransitionCosts) {
//        Hf.subRange(0,6) = 0.;
//        Hf = ~Hf;
//        Hf.subRange(0,6) = 0.;
//        Hf = ~Hf;
//      }
      //      cout <<"Hfa: " << Hf << endl;
    }
    if (&gLambda && constrainsActive) {
      gLambda = 2.*JgJgtI_Jg_Jt_PHIw;
    }
    if (&JgLambda && constrainsActive) {
      JgLambda = 2.*JgJgtI_Jg_J_dPHI ;
    }
    return y;
  }
};



struct Scene {
  arr xRef; // reference solution
  arr lambdaRef; // constraint trajectory
  arr paramRef;
  arr xInit; // init solution for optimization
  uint contactTime;
  MotionProblem* MP;
  ors::KinematicWorld* world;
  IOC_DemoCost* cost;// cost function for this demonstrations

  Scene () { }
};


struct IOC:ConstrainedProblem {
  MT::Array<Scene> scenes;
  arr xOpt;
  uint numParam;
  uint numLambda;
  arr Dwdx;
  uint n;
  uint T;
  bool learnTransitionCosts;
  bool normalizeParam;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() {
    if (normalizeParam) return numParam+numLambda+1;
    return numParam+numLambda;
  }
  //    virtual uint dim_g() { return numParam+numLambda+1+7;}
  //  virtual uint dim_g() { return numParam+numLambda;}

  IOC(MT::Array<Scene> &_scenes,uint _numParam):scenes(_scenes),numParam(_numParam) {
    n = scenes(0).MP->world.getJointStateDimension();
    T = scenes(0).MP->T;

    learnTransitionCosts = MT::getParameter<bool>("learnTransitionCost");
    normalizeParam = MT::getParameter<bool>("normalizeParam");

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      if (learnTransitionCosts)
        Dwdx.append(catCol(ones(n,1),zeros(n,numParam-1)));
      else {
        Dwdx.append(catCol(ones(n,1)*0.,zeros(n,numParam-1)));
      }

      // add task cost elements
      for (uint c=0;c<scenes(0).MP->taskCosts.N;c++) {
        if ( scenes(0).MP->taskCosts(c)->prec.N >t && (scenes(0).MP->taskCosts(c)->prec(t) > 0) && scenes(0).MP->taskCosts(c)->active && !scenes(0).MP->taskCosts(c)->map.constraint) {

          uint m;
          if ((scenes(0).MP->taskCosts(c)->target.N-1) == T) {
            m = scenes(0).MP->taskCosts(c)->target.d1;
          } else {
            m = scenes(0).MP->taskCosts(c)->target.N;
          }

          arr tmp = zeros(m,1);
          tmp = catCol(tmp,zeros(m,c));
          tmp = catCol(tmp,ones(m,1));
          tmp = catCol(tmp,zeros(m,numParam-tmp.d1));
          Dwdx.append(tmp);
        }
      }
    }
//    cout << "Dwdx: " << Dwdx << endl;

    numLambda = 0;
    // initialize cost functions for demonstrations
    for (uint i=0;i<scenes.d0;i++) {
      scenes(i).cost = new IOC_DemoCost(*scenes(i).MP,scenes(i).xRef,scenes(i).lambdaRef,Dwdx,numParam);
      scenes(i).cost->learnTransitionCosts = learnTransitionCosts;
      if ( scenes(i).cost->constrainsActive )
        numLambda += scenes(i).cost->lambda0.N;
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
      if (normalizeParam) g.append((sumOfSqr(x)-1e0)*(sumOfSqr(x)-1e0)); // ||w|| = 1

      g.append(-x); // w > 0
      //      g.append(-x.subRange(0,6)+1e-2); // w_trans > 2
      //      g.append(-x.subRange(7,x.d0-1)+1e-2); // w_trans > 2
    }
    if (&Jg) {
      Jg.clear();
      if (normalizeParam) Jg=4.*~x*(sumOfSqr(x)-1e0);
      Jg.append(-eye(numParam));
      //      Jg.append(catCol(-eye(7),zeros(7,numParam-7)));
      //      Jg.append(catCol(zeros(/*7*/),eye(numParam-7),numParam-7)));
    }

    // iterate over demonstrations
    for (uint i=0;i<scenes.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      y += scenes(i).cost->eval(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, x);

      if (&df) {
        df += dfi;
      }
      if (&Hf) {
        Hf += Hfi;
      }
      if (&g) {
        g.append(gi);
      }
      if (&Jg && Jgi.N>0) {
        Jg.append(Jgi*Dwdx);
      }
    }
    return y;
  }

  void costReport(arr &w) {
    xOpt = w;
    cout << "\n*****************************************************" << endl;
    cout << "******************** COST REPORT ********************" << endl;
    cout << "*****************************************************" << endl;
    cout << "Found parameter: " <<xOpt << endl;
    cout << "Found parameter (scaled): " <<xOpt/sqrt(sumOfSqr(xOpt))*1e1 << endl;
    cout << "Reference parameter (scaled): " << scenes(0).paramRef/sqrt(sumOfSqr(scenes(0).paramRef))*1e1 << endl;
    for (uint i=0;i<scenes.d0;i++) {
      cout << "\nDemonstration: " << i << endl;
      cout << "IOC costs: " << scenes(i).cost->eval(NoArr,NoArr,NoArr,NoArr,xOpt) << endl;
      cout << "Lambda: " << scenes(i).cost->lambda/sqrt(sumOfSqr(scenes(i).cost->lambda)) << endl;
      cout << "Lambda Ground Truth: " << scenes(i).cost->lambda0/sqrt(sumOfSqr(scenes(i).cost->lambda0+1e-12)) << "\n\n"<< endl;
    }
  }
};


#endif // IOC_H
