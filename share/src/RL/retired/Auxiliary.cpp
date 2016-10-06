
/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/
#include <Core/util.h>
#include <Core/array.h>

#include "Auxiliary.h"

#ifndef CHECK_EPS
#define CHECK_EPS 1e-8
#endif

using namespace mlr;

namespace mdp {


void gradient_FD(PolicySearch obj, arr x)
{
    arr theta = x, d_theta;
    arr gradient(x.d0);
    arr grad_final(x.d0); grad_final.setZero();
    double J_theta, d_J_theta;
    double Reward, sumR;
    double eps = CHECK_EPS;
    uint numEp = obj.getNumEps();

    for(uint seed=0; seed<100; seed++)
    {
        mlr::rnd.seed(seed);
        sumR = 0;

        for(uint ep=0; ep<numEp; ep++)
        {
            Reward = obj.rollout(theta);
            sumR += Reward;
        }
        J_theta = sumR / (double)numEp;

        gradient.setZero();
        for(uint index=0; index<theta.d0; index++)
        {
            d_theta = theta;
            d_theta(index) += eps;
            sumR = 0;
            for(uint ep=0; ep<numEp; ep++)
            {
                Reward = obj.rollout(d_theta);
                sumR += Reward;
            }
            d_J_theta = sumR / (double)numEp;
            gradient(index) = (d_J_theta - J_theta) / eps;
        }
        grad_final += gradient;
    }
    grad_final /= 100.;

    cout<<"The gradient computed via FD is "<< grad_final <<endl;
}


void myCheckGradient(PolicySearch obj, arr x)
{
    arr gradJ_Reinforce, gradJ_GPOMDP;
    gradJ_Reinforce.resize(obj.getPolicy()->getActionDim(), obj.getPolicy()->getPolicyDim());
    gradJ_GPOMDP.resize(obj.getPolicy()->getActionDim(), obj.getPolicy()->getPolicyDim());
    arr grad_final_Reinforce;
    grad_final_Reinforce.resize(obj.getPolicy()->getActionDim(), obj.getPolicy()->getPolicyDim());
    grad_final_Reinforce.setZero();
    double rew_R, rew_G;

    gradient_FD(obj, x);

    for(uint seed=0; seed<100; seed++)
    {
        gradJ_Reinforce.setZero();
        mlr::rnd.seed(seed);
        rew_R = obj.updateREINFORCE(gradJ_Reinforce, x);
        grad_final_Reinforce += gradJ_Reinforce;
    }
    grad_final_Reinforce /= 100.;
    cout<<"The gradient computed via Reinforce is "<< grad_final_Reinforce <<endl;

//    rew_G = obj.updateGPOMDP(gradJ_GPOMDP, x);
//    cout<<"The gradient computed via GPOMDP is "<< gradJ_GPOMDP <<endl;
}


} //end of namespace
