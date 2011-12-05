/*
 * File:   aciot.h
 * Author: s0342671
 *
 * Created on 01 September 2011, 14:13
 */

#ifndef _ACIOT_H
#define	_ACIOT_H

#include "MT/aico.h"
#include <vector>



struct AICOT : public AICO {
    public:
        AICOT(){
            MaxEMIter   = MT::Parameter<int>("EMiter",10);
            tauCostRate = 1;
        }
        AICOT(soc::SocSystemAbstraction &sys){
            MaxEMIter   = MT::Parameter<int>("EMiter",10);
            init(sys);
        }

    public:
      virtual void init(soc::SocSystemAbstraction &sys);
      virtual void iterate_to_convergence(const arr* q_initialization=NULL);
      void  getLocalProcess(arr& A, arr& a, arr& B, uint k);
      double calcQFun(double dt);
      void   plotQFun();
      //      virtual double step(){
//        double b_diff   = AICO::step();
//        double tau_diff = updateTau();
//        return tau_diff;
//      };

//      void updateTau(){
//        calcBeliefs();
//        calcMStep();
//      };
    private:
        void calcBeliefs();
        void calcEStep(const arr* q_init = NULL);
        void calcMStep();
        void calcMStepNumerical();
    public:
        //arr partition;
        arr p,P,PP; //belief cross covariance
        int MaxEMIter;
        double tauCostRate;
};


#endif	/* _ACIOT_H */

