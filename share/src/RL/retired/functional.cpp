
#include<Core/util.h>
#include<Core/array.h>
#include "functional.h"
#include <assert.h>
#include <float.h>


/*  ---------------------------------------------------------------------

    Copyright 2015 NGO ANH VIEN

    This code is under the GNU General Public License <http://www.gnu.org/licenses/>

---------------------------------------------------------------------*/


using namespace mlr;

namespace mdp {

RKHSPol::RKHSPol(const ENVIRONMENT& env, int numCentre,uint horizon,uint numEps,int numIterations)
    : NumCentre(numCentre),
      Horizon(horizon),
      NumEps(numEps),
      Env(env),
      NumIterations(numIterations)
{
    mlr::useLapack = true;
    dim_S = Env.GetStateDim();
    dim_A = Env.GetActionDim();


    //This exploration parameters can be changed in file "MT.cfg"
    double var1 = mlr::getParameter<double>("var1",0.02);
    double var2 = mlr::getParameter<double>("var2",0.1);


    RBFVariance2 = ARR(var1,var2);            //Variance of the policy
    SigmaInv = diag(ARR(1./RBFVariance2(0),1./RBFVariance2(1)));


    KernelVars   = diag(ARR(1./1.2,1./0.1)); //Variances for the RBF kernel


    FuncPolicy.resize(1, dim_S + dim_A); // weight + centre dimensions
    FuncPolicy.setZero(); //initialized to 0: (that means: h_0 = 0)

    for(int i=0;i<dim_A; i++)
        FuncPolicy[0](i) = 0.;

}


arr RKHSPol::run()
 {    


    return runRKHS();   //


 }



void RKHSPol::EvalPolicy(const arr funcPolicy, double& Total_R)
{

    Total_R = 0;
    for(int s=0;s < 20; s++){
        arr states,  actions,  rewards;

        double Reward = rollout(funcPolicy, states, actions, rewards);
        Total_R += Reward;
    }

    Total_R = Total_R/20.;

}

 ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

arr RKHSPol::runRKHS()
 {
    arr IterationReward;
    IterationReward.clear();

    cout<<"============ FUNCTIONAL POLICY GRADIENT ==========================="<<endl<<endl;


    for(int iteration = 0; iteration<NumIterations; iteration ++){

        // Computing gradient (use Num episodes, horizon is 20)
        double total = 0.0;
        arr nxtFuncPolicy = FuncPolicy;

        for(int k=0;k<NumEps;k++)
        {
            arr states,  actions,  rewards;
            double Reward = rollout(FuncPolicy, states, actions, rewards);
            total += Reward;
            //cout<< Reward<<endl;
            update(nxtFuncPolicy,states,actions, (double)Reward/NumEps, FuncPolicy);
        }

        double average = (double) total/NumEps;
        cout<< " The reward of this iteration "<<iteration<<" is: " << average <<"  centres= "<<FuncPolicy.d0<<endl;

        double step_size =  lineSearch(nxtFuncPolicy);


        //now the functional gradient is: average*nxtFuncPolicy
        if(step_size !=0.){
            for(int k=FuncPolicy.d0; k<nxtFuncPolicy.d0; k++){
                for(int ind=0; ind < dim_A; ind++)
                    nxtFuncPolicy[k](ind) *= step_size;

                FuncPolicy.append(nxtFuncPolicy[k]());
            }
        }



        //////////////////////////////////////

        //Sparsification

        if(step_size !=0.){
          arr sparse_functional;
          Sparsification(sparse_functional, FuncPolicy);
          FuncPolicy = sparse_functional;
        }

        //delete the preamble data (all zeros, see this initialization in SetStart)
        if(iteration == 0)
             FuncPolicy.delRows(0);


        cout<<" step_size "<<step_size <<endl;
        IterationReward.append(average);

    }
    return IterationReward;

}



 double RKHSPol::lineSearch(const arr& gradient)
 {


     double best_alpha = 0.0;
     double best_value = -10000000.0;

     uint numEval = 40;
     uint numEpisodes = 20;

     for(int grid=0;grid<numEval;grid +=1){
         //each is evaluated over NumEps
         double alpha_new = 0.000;

         alpha_new =  0.01*pow(1.1,grid)- 0.01 + 0.0000001; //for plain PG

         arr TempGradient = gradient;
         double Total_R = 0.;

         for(int k=FuncPolicy.d0; k<TempGradient.d0; k++){
             for(int ind=0; ind < dim_A; ind++)
                TempGradient[k](ind) *= alpha_new;// change the weight to scaling
         }

         for(int k=0;k<numEpisodes;k++)
         {
             arr states,  actions,  rewards;
             double Reward = rollout(TempGradient, states, actions, rewards);
             Total_R += Reward;
         }
         //cout<<" ["<<grid<<"]:"<< Total_R/numEpisodes<<"  "<<endl;
         if((double)Total_R/numEpisodes > best_value ){
             best_value = (double) Total_R/numEpisodes;
             best_alpha = alpha_new;
          //   cout<< "alpha_new = " <<alpha_new <<" best_value=  "<<best_value <<endl;
         }

         //cout<< Total_R/numEval <<endl;
     }

     cout<<"[line-search] best_value = "<<best_value<<endl;

     return best_alpha;

 }


 void RKHSPol::update(arr &newPol, arr& states, arr& actions, double scale, const arr oldPol)
 {

     for(int i=0;i < states.d0; i++){
             if(states[i](0) < -1000) {
                 cout<< " FFFF ";
                 break; //terminal. end of a trajectory
             }
             arr temp(dim_A + states.d1); //weight, and centre

             //ram action into temp
             arr temp2;
             arr hs;
             evaluate(states[i], oldPol, hs);
             temp2 = scale * SigmaInv *(actions[i] - hs);
             for(int j=0;j<dim_A;j++)
                 temp(j) = temp2(j);

             //ram state into temp
             for(int j=0;j<states.d1;j++)
                 temp(dim_A+j) = states[i](j);

             //add temp into the library
             newPol.append(temp);
     }

 }

 //return a trajectory (one rollout)
 //note that: we use exponential rewards for learning, but still reporting environment rewards
 double RKHSPol::rollout(const arr ht, arr& states, arr& actions, arr& rewards)
 {

     arr startS;
     Env.getStartState(startS);
     states.clear();
     actions.clear();
     rewards.clear();


     double totalReturn = 0.0;
     double discount = Env.GetDiscount();


     bool terminal;
     int i=0;
     for(i=0;i < Horizon;i++){
        arr action ;
        sampleAction(ht, startS, action);
        double reward;
        arr nxtState;

        terminal = Env.forwardDynamics(action,startS,nxtState,reward);

        states.append(~startS);
        actions.append(~action);
         //rewards.append(exp(reward));
        rewards.append(reward);
        totalReturn += pow(discount,i)*reward;

        startS = nxtState;       
     }



     return totalReturn;

 }

 void RKHSPol::sampleAction(const arr ht, const arr state, arr& action)
 {
    action.resize(dim_A);
    arr mean;
    evaluate(state,ht,mean);

    for(int i=0; i<dim_A; i++)
        action(i) = mean(i) + sqrt(RBFVariance2(i))*mlr::rnd.gauss();

 }


 void RKHSPol::evaluate(const arr state, const arr& ht, arr &hs)
 {
     //eval: h(s) = sum_i \alpha_i K(s_i,s)
    arr sum(dim_A,1);
    sum.setZero();

    for(int i=0;i<ht.d0;i++){
        arr temp(1,dim_A);
        temp[0]() = ht[i].sub(0,dim_A-1);

        sum = sum + kernelFunc(ht[i].sub(dim_A,-1),state)*eye(dim_A) * (~temp);
    }

    hs = (~sum)[0];


    //bounding the function
    hs(0) = MIN(hs(0),MT_PI/2);
    hs(0) = MAX(hs(0),0.0);
    hs(1) = MIN(hs(1),10.);
    hs(1) = MAX(hs(1),0.0);


 }

 double RKHSPol::kernelFunc(const arr state1, const arr state2)
 {
     arr prod = ~(state1-state2)*KernelVars*(state1-state2);
     return exp(-prod(0)/2.);
 }

 void RKHSPol::Sparsification(arr& Dictionary, const arr base)
  {

      //see Algorithm 1: Kernel Matching Pursuit for Large Datasets
      //Vlad Popovici, Samy Bengio, Jean–Philippe Thiran
     //arr Dictionary;
     uint DataSetNum =  base.d0;
     arr Dataset;
     Dataset= base;
     int k,j;
     arr Kernel(DataSetNum,DataSetNum);
     arr Residue(DataSetNum,dim_A);
     arr OUTPUT(DataSetNum,dim_A);

     for(k=0; k<DataSetNum; k++){ //run over functions g
         for(j=0;j<DataSetNum;j++){ //run over samples x_i
             if(j<k)
                 Kernel[k](j) = Kernel[j](k);
             else
                 Kernel[k](j) = kernelFunc(Dataset[k].sub(dim_A,-1),Dataset[j].sub(dim_A,-1));
         }
         arr temp;
         evaluate(Dataset[k].sub(dim_A,-1),base,temp);
         Residue[k]() = temp;
     }
     OUTPUT = Residue;
     KernelMatchingPursuit(Dictionary,Dataset,Residue,Kernel);


  }

void RKHSPol::KernelMatchingPursuit(arr& Dictionary, const arr Dataset, arr Residue, const arr Kernel)
{

    //TODO: The code is not correct for multiple-dim action cases.
    //CHANGE: At the use of Kernel (multiply with an identity matrix)

    int i,k;
    arr OUTPUT = Residue;
    double Sparsification_ERR=0;

    arr ADDED(Dataset.d0);ADDED.setZero();
    arr NEW_W;
    double old_ERR = 0;
    arr PHI;//(Dataset.d0,Dataset.d0);
    uint REJECT = 0;
    uint dim_W = Residue.d1;



    for(i=0;i < Dataset.d0; i++){ //add maxmimally all centres from database
        double best_score = -100000.;
        arr weight;
        uint best_index=0;       

        for(k=0; k<Dataset.d0; k++)
            if(ADDED(k)==0){ //run over functions g
                arr normm1, normm2;
                innerProduct(normm1,~Residue,Kernel[k]);
                innerProduct(normm2,~Kernel[k],Kernel[k]);
                arr tempp = ~normm1 * normm1;
                double nominator = sqrt(tempp(0));
                double denominator = sqrt(normm2(0));

               // cout<<nominator<<"  "<<denominator <<"  "<<fabs(nominator)/denominator<<endl;

                if(best_score < fabs(nominator)/denominator){
                    best_score = fabs(nominator)/denominator;
                    best_index = k;
                    weight = normm1/normm2(0);                 
                }           

        }

        //compute the current residue
        for(int ind=0;ind<Residue.d0;ind++){
            Residue[ind]() = Residue[ind] - Kernel[best_index](ind)*weight;
        }

        ADDED(best_index) = 1;


        //////////////////////////////////////////////////////////////////////////
        /// \brief temp
        /// ////////////////////////////////////////////////////////
        //back-fitting the weights

        arr new_PHI(Dataset.d0);

        for(int j=0; j<Dataset.d0; j++){
            new_PHI(j)= kernelFunc(Dataset[best_index].sub(dim_W,-1), Dataset[j].sub(dim_W,-1));
        }
        PHI.append(~new_PHI);



        arr normm3;
        arr Res_t = ~Residue;
        //cout<< "Residue "<<~Residue<<endl;
        Sparsification_ERR = 0;
        for(int jj=0; jj < Res_t.d0; jj++){
            innerProduct(normm3,~Res_t[jj],Res_t[jj]);
            Sparsification_ERR  += sqrt(normm3(0));
        }


        if(1){//fabs(old_ERR - Sparsification_ERR) > 0.001){
            arr new_centre(Dataset[best_index].d0);
            new_centre = Dataset[best_index];
            Dictionary.append(~new_centre);
            if((i - REJECT) >=NumCentre-1) break;

            old_ERR = Sparsification_ERR ;
            //cout<<Sparsification_ERR<<endl;

        }
        else
            REJECT ++ ;

    }

    arr temp;
    arr I;
    I.setId(PHI.d0);
    inverse_SVD(temp, 1e-10*eye(PHI.d0) + PHI * (~PHI));

    //cout <<maxDiff(temp*(1e-9*eye(PHI.d0) + PHI * (~PHI)),I,&mi) << " ; " ;

    NEW_W = temp*PHI*OUTPUT;

    Residue = OUTPUT - ~(~NEW_W * PHI);

    arr normm3;
    arr Res_t = ~Residue;
    Sparsification_ERR = 0;
    for(int jj=0; jj < Res_t.d0; jj++){
        innerProduct(normm3,~Res_t[jj],Res_t[jj]);
        Sparsification_ERR  += sqrt(normm3(0));
    }

    for(int ii=0; ii<Dictionary.d0; ii++)
        for(int dim=0; dim < dim_W; dim++){
            Dictionary[ii](dim) = NEW_W[ii](dim);

        }

    //cout<< Dictionary<<endl;


    cout<<" Sparsification_ERR = "<<Sparsification_ERR << " Dictionary:"<<Dictionary.d0 <<" REJECT "<<REJECT<<endl;



}













}







