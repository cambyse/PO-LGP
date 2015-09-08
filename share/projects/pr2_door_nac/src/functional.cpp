

#include "functional.h"
#include <assert.h>
#include <float.h>


using namespace MT;

namespace mdp {

RKHSPol::RKHSPol(ors::KinematicWorld& world_, bool useRos, arr Xdemo,arr FLdemo,arr Mdemo, arr paramLim, uint numCentre, uint horizon,uint numEps,uint kernel_Type, int numIterations)
    : Horizon(horizon),
      NumEps(numEps), Xdemo(Xdemo), FLdemo(FLdemo), Mdemo(Mdemo), paramLim(paramLim), useRos(useRos),
      NumCentre(numCentre),
      Kernel_Type(kernel_Type),
      NumIterations(numIterations)

{
    world=new ors::KinematicWorld(world_);
    task = new DoorTask(world_);
    if(useRos)mi = new Motion_Interface(world_);
    /// load demonstration from file



    /// compute parts of motion where robot is in contact with handle
    task->computeConstraintTime(FLdemo,Xdemo);

    /// visualize demonstration
    //world.gl().resize(800,800);
    //task->updateVisualization(world,Xdemo);
    //displayTrajectory(Xdemo,-1,world,"demonstration");


    //alpha = .005; // for vanilla gradient alpha is [0.001;0.01] ;; Increase the number of trajectories also increase the performance and stability.
    MT::useLapack=true;
    Algorithm = 1; //default using natural policy gradient (functional policy)
    dim_A = 1; //this is default, and set again in main.cpp
    currIteration = 0;

    RBFVariance = 0.00001;//0.0001; for Toy// 1)RBFVariance = 0.0001;()for Toy

}
void RKHSPol::loadOldFuncPolicy()
{
    FuncPolicy << FILE(STRING("FuncPolicy.dat"));
    cout<< FuncPolicy <<endl;
    arr temp;
    temp<< FILE(STRING("currIteration.dat"));
    currIteration = temp(0);

}
void RKHSPol::setStart(const arr &start)
{
    dim_S = start.d0;
    StartingState = start;
    FuncPolicy.resize(1, dim_A + dim_S); // weight + centre dimensions
    currIteration = 0;
    FuncPolicy.setZero(); //initialized to 0: (that means: h_0 = 0)
    for(int i=0;i<dim_A; i++)
        FuncPolicy[0](i) = 0.;

    Sigma = 1./RBFVariance*eye(dim_A); //this is Sigma^-1
}

arr RKHSPol::run()
 {    

     //cout<<Sigma<<endl;

     if(Algorithm ==0)
        return runPG();
     else if(Algorithm ==1)
         return runNPG();
     else{
         cout<<"Algorithm should be 0 (PG) or 1 (NPG)" <<endl;
         return ARR(0);
     }

 }

// void RKHSPol::setStart(const arr& start)

 ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////

arr RKHSPol::runPG()
 {
    arr IterationReward;
    IterationReward.clear();
    arr states,  actions,  rewards;
    arr nxtFuncPolicy ;

    cout<<"============ FUNCTIONAL POLICY GRADIENT ==========================="<<endl<<endl;


    for(int iteration = currIteration; iteration<NumIterations; iteration ++){

        // Computing gradient (use Num episodes, horizon is 20)
        double total = 0.0;

        nxtFuncPolicy = FuncPolicy;
        for(int k=0;k<NumEps;k++)
        {

            double Reward = rollout(FuncPolicy, states, actions, rewards);
            total += Reward;

            //cout<< states<<endl;
           // cout<< ~actions<<endl;
            //cout<< rewards<<endl;
            //cout<< Reward <<" " ;

            update(nxtFuncPolicy,states,actions, (double)Reward/NumEps, FuncPolicy);
        }
        //cout<<endl;
        double average = (double) total/NumEps;



        //sparsify the gradient
       // Sparsification(temppp,nxtFuncPolicy);
       // nxtFuncPolicy = temppp;

        double step_size =  lineSearch(nxtFuncPolicy);
        //now the functional gradient is: average*nxtFuncPolicy
        if(step_size !=0.){
            for(int k=FuncPolicy.d0; k<nxtFuncPolicy.d0; k++){
                for(int ind=0; ind < dim_A; ind++)
                    nxtFuncPolicy[k](ind) *= step_size;
                FuncPolicy.append(nxtFuncPolicy[k]());
            }
        }

        //delete the preamble data (all zeros, see this initialization in SetStart)
        if(iteration == 0)
             FuncPolicy.delRows(0);

        //////////////////////////////////////

        //Sparsification

        //if(step_size !=0.){
       //    arr sparse_functional;
       //    Sparsification(sparse_functional, FuncPolicy);
       //     FuncPolicy = sparse_functional;
        //}

        /////////////////////////////////////////////////////////////////

        //if(RBFVariance> 0.001){
        //    RBFVariance *= 0.8;
        //    Sigma = 1./RBFVariance*eye(dim_A); //this is Sigma^-1
       // }
        /////////////////////////////////////////////////////////////////


        cout<< " Iteration "<<iteration<<" is: " << average <<endl;
        //cout<<" step_size "<<step_size <<endl;
        IterationReward.append(average);



        ///UPDATE MEAN
        arr newMean;
        evaluate(StartingState,FuncPolicy,newMean);
        FuncPolicy.clear();
        FuncPolicy.resize(2,dim_A + dim_S);
        FuncPolicy.setZero();
        FuncPolicy[0](0) = newMean(0) + 0.03; // see the way we compute the bias (just fix the diff)
        FuncPolicy[0](1) = newMean(1) + 0.06;
        FuncPolicy[0](2) = StartingState(0);
        cout<< FuncPolicy <<endl;
        //////////////////////////////////////////////////


        write(LIST<arr>(FuncPolicy),STRING("FuncPolicy.dat"));
        write(LIST<arr>(ARR(iteration+1)),STRING("currIteration.dat"));

    }
    return IterationReward;


}




///////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////////////////////////////////
//// Natural function PG
 arr RKHSPol::runNPG()
 {
    arr IterationReward;
    IterationReward.clear();
    arr nxtFuncPolicy;
    arr DATA_S,DATA_A,DATA_R;
    arr Horizons;
    arr Gram;

    cout<<"===========FUNCTIONAL NATURAL POLICY GRADIENT ============================"<<endl<<endl;


    for(int iteration=currIteration;iteration<NumIterations;iteration ++){

        // Computing gradient (use Num episodes, horizon is 20)
        double total = 0.0;
        nxtFuncPolicy = FuncPolicy;

        DATA_S.resize(NumEps,Horizon,StartingState.d0);
        DATA_S.setZero();
        DATA_A.resize(NumEps,Horizon, dim_A);
        DATA_A.setZero();
        DATA_R.resize(NumEps,1);
        DATA_R.setZero();
        Horizons.resize(Horizon);
        Horizons.setZero();

        for(int k=0;k<NumEps;k++)
        {
            arr states,  actions,  rewards;
            double Reward = rollout(FuncPolicy, states, actions, rewards);

            DATA_S[k]() = states;
            DATA_A[k]() = actions;
            DATA_R[k](0)   = Reward;
            total += Reward;            
        }


        GramMatrix(DATA_S,DATA_A,Gram);
        arr newR(NumEps,1);// = (Gram + 0.01*eye(NumEps));
        arr invGram;
        long svdr = inverse_SVD(invGram,(Gram + 0.000001*NumEps*eye(NumEps)));
        innerProduct(newR,invGram,DATA_R);

        for(int i=0;i<NumEps;i++)
            update(nxtFuncPolicy,DATA_S[i](),DATA_A[i](), newR[i](0), FuncPolicy);


        //sparsify the gradient
        //arr temppp;
        //Sparsification(temppp,nxtFuncPolicy);
        //nxtFuncPolicy = temppp;


        double average = (double) total/NumEps;


        double step_size = 0;


        step_size = lineSearch(nxtFuncPolicy);

        if(step_size !=0.){
            //now the functional gradient is: average*nxtFuncPolicy
            for(int k=FuncPolicy.d0; k<nxtFuncPolicy.d0; k++){
                for(int ind=0; ind<dim_A; ind++){
                    nxtFuncPolicy[k](ind) *= step_size;
                }
                FuncPolicy.append(nxtFuncPolicy[k]);
            }

        }

        if(iteration == 0)
                FuncPolicy.delRows(0);

        //if(step_size !=0.){
        //    //Sparsification
        //    arr sparse_functional;
        //    Sparsification(sparse_functional,FuncPolicy);
        //    FuncPolicy = sparse_functional;
        //}


        cout<< " Iteration "<<iteration<<" is: " << average <<"  Step size: "<<step_size <<endl;
        IterationReward.append(average);



        ///UPDATE MEAN
        arr newMean;
        evaluate(StartingState,FuncPolicy,newMean);
        FuncPolicy.clear();
        FuncPolicy.resize(2,dim_A + dim_S);
        FuncPolicy.setZero();
        FuncPolicy[0](0) = newMean(0) + 0.03; // see the way we compute the bias (just fix the diff)
        FuncPolicy[0](1) = newMean(1) + 0.06;
        FuncPolicy[0](2) = StartingState(0);
        cout<< FuncPolicy <<endl;
        //////////////////////////////////////////////////


        write(LIST<arr>(FuncPolicy),STRING("FuncPolicy.dat"));
        write(LIST<arr>(ARR(iteration+1)),STRING("currIteration.dat"));

    }
    return IterationReward;


}



///////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////////////////////////////////
 ///////////////////////////////////////////////////////////////////////////////



 double RKHSPol::lineSearch(const arr& gradient)
 {


     double best_alpha = 0.0;
     double best_value = -10000000.0;

     uint numEval = 12;
     uint numEpisodes = 3;
     arr TempGradient;

     arr states,  actions,  rewards;
     //uint numEpisodes = 20;

     for(int grid=0;grid<numEval;grid +=1){
         //each is evaluated over NumEps
         double alpha_new = 0.000;

         if(Algorithm==0)
              alpha_new = 0.00000001 + 0.02*grid; //for plain PG
         else if (Algorithm==1)
              alpha_new = 0.01*pow(1.5,grid) - 0.01;// + 0.00000001; //use 1.2 for polynomial kernels (Kernel_Type>0)
              //alpha_new = 0.00000001 + 0.07*grid; //0.01*pow(1.5,grid) - 0.01 + 0.00000001; //use 2. for NPG (without sparsification: because when the step-size is very large (2000.), sparsification gives poor approximation
            //alpha_new = 0.00000001 + 0.02*grid;
         else
              alpha_new = 0.000 + 0.005*grid; //for plain PG

         TempGradient = gradient;
         double Total_R = 0.;

         for(int k=FuncPolicy.d0; k<TempGradient.d0; k++){
             for(int ind=0; ind < dim_A; ind++)
                TempGradient[k](ind) *= alpha_new;// change the weight to scaling
         }

         for(int k=0;k<numEpisodes;k++)
         {

             double Reward = rollout(TempGradient, states, actions, rewards);
             Total_R += Reward;
         }
         cout<< Total_R/numEpisodes<<"  ";
         if((double)Total_R/numEpisodes > best_value){
             best_value = (double) Total_R/numEpisodes;
             best_alpha = alpha_new;
          //   cout<< "alpha_new = " <<alpha_new <<" best_value=  "<<best_value <<endl;
         }

         //cout<< Total_R/numEval <<endl;
     }

     cout<<"[line-search] best_value = "<<best_value<<endl;

     return best_alpha;

 }
  void RKHSPol::GramMatrix(const arr&  DATA_S,const arr&  DATA_A, arr& Gram)
  {
      uint dim0 = DATA_S.d0;
      Gram.resize(dim0,dim0);
      for(int i=0;i<dim0;i++)
          for(int j=0;j<dim0;j++){
             if(j<i)
                  Gram[i](j) = Gram[j](i);
             else
                  Gram[i](j) = TrajectoryKernel(DATA_S[i](),DATA_A[i](),DATA_S[j](),DATA_A[j]());
          }

    //cout << Gram<<endl;
  }


  //here is a kernel function of trajectories: states1,actions1 and states2,actions2
 double RKHSPol::TrajectoryKernel(const arr& states1, const arr& actions1,const arr& states2, const arr& actions2)
 {
    uint dim1 = states1.d0;
    uint dim2 = states2.d0;
    double sum = 0.0;
    arr hs1;
    arr sub1(1,dim_A);
    arr hs2;
    arr sub2(1,dim_A);
    arr sumMatrix;

    for(int i=0; i<dim1; i++){
        if(states1[i](0) < -100) break;

        evaluate(states1[i],FuncPolicy,hs1);
        sub1[0]() = actions1[i] - hs1;

        for(int j=0; j<dim2; j++){
            if(states1[j](0) < -100) break;

            evaluate(states2[j],FuncPolicy,hs2);
            sub2[0]() = actions2[j] - hs2;

            sumMatrix = ~(kernelFunc(states1[i],states2[j])*eye(dim_A)*Sigma*(~sub1)) * Sigma * (~sub2);
            sum += sumMatrix[0](0) ;
        }
    }


    return sum;
 }

 void RKHSPol::update(arr &newPol, arr& states, arr& actions, double scale, const arr oldPol)
 {     
     arr temp(dim_A + dim_S); //weight, and centre
     arr temp2;
     arr hs;

     for(int i=0;i < states.d0; i++){
             if(states[i](0) < -100) {
                 cout<< " FFFF ";
                 break; //terminal. end of a trajectory
             }

             //ram action into temp

             evaluate(states[i],oldPol,hs);
             temp2 = scale * Sigma *(actions[i] - hs);
             for(int j=0;j<dim_A;j++)
                 temp(j) = temp2(j);

             //ram state into temp
             for(int j=0;j<dim_S;j++)
                 temp(dim_A+j) = states[i](j);

             //add temp into the library
             newPol.append(temp);
     }

 }

 //return a trajectory (one rollout)
 double RKHSPol::rollout(const arr ht, arr& states, arr& actions, arr& rewards)
 {

     arr startS;
     startS = StartingState;
     states.clear();
     actions.clear();
     rewards.clear();


     double totalReturn = 0.0;
     double discount = 0.99;//Env.GetDiscount();

     states.resize(1,StartingState.d0);
     actions.resize(1,dim_A);

     bool terminal;
     int i=0;
     arr action ;
     arr nxtState;
     for(i=0;i < Horizon;i++){

        sampleAction(ht, startS, action);
        double reward;

        terminal= forwardDynamics(action,startS,nxtState,reward);
        states.append(startS);

        actions.append(action);
        rewards.append(reward);
        totalReturn += pow(discount,i)*reward;

        startS = nxtState;

        if(terminal)
            break;
     }
     states.delRows(0);
     actions.delRows(0);

     if(terminal){
         for(int k=i+1;k<Horizon;k++)
         {
             arr pseudo_S(dim_S);
             arr pseudo_A(dim_A);
             pseudo_S(0) = -1000;
             states.append(pseudo_S); //indicate the terminal
             actions.append(pseudo_A);
             rewards.append(0.0);
         }
     }

     return totalReturn;

 }

 bool RKHSPol::forwardDynamics(arr action, arr state, arr &nxtState, double &reward)
 {
     arr Xn;
     nxtState.resize(state.d0);
     nxtState.setZero();


     if(action(0) >= paramLim[0](1) || action(0) <= paramLim[0](0) || action(1) >= paramLim[1](1) || action(1) <= paramLim[1](0))
     {
          reward = 0;
          return true;
     }
     /*/
     arr abc = action;

     abc(0) = MAX(abc(0), paramLim[0](0));
     abc(0) = MIN(abc(0), paramLim[0](1));

     abc(1) = MAX(abc(1), paramLim[1](0));
     abc(1) = MIN(abc(1), paramLim[1](1));
     /*/



     bool result = task->transformTrajectory(Xn,action,Xdemo);

     task->updateVisualization(*world,Xn);
     world->watch();

     arr forces;
     bool success = 1;
     if(result && useRos){
         mi->gotoPosition(Xn[0]);
         mi->executeTrajectory(Xn,15.,true);
         forces = mi->FLact;
         arr Xreverse = Xn;
         Xreverse.reverseRows();
         mi->executeTrajectory(Xreverse,15.);
         success = task->success(mi->Mact,Mdemo);
     }

     /// compute reward function
     /// currently consist of one term that measures squared acceleration
     /// and another term that measures how close the current trajectory is to the demonstration


    // if (result && success) {
    if (result) {
        arr Xdd;
       getAcc(Xdd,Xn,1.);
       //double part1 = sumOfAbs(forces)/forces.N;
       double part2 = (sumOfAbs(Xn-Xdemo)/Xn.d0);
       //cout<< "force: "<<part1<<" ; task "<<part2<<endl;
       //reward = exp(-part1 - 10.*part2);//exp(-0.5*sumOfAbs(Xdd)) +
       reward = exp(-2*part2*part2);//exp(-0.5*sumOfAbs(Xdd)) +
     }else{
       reward = 0.;
     }
     //cout<< "reward " <<reward <<endl;


     return true;
 }


 void RKHSPol::sampleAction(const arr ht, const arr state, arr& action)
 {
    action.resize(dim_A);
    arr mean;
    arr DEVS(2);
    DEVS(0) = 0.0001;
    DEVS(1) = 0.001;


    evaluate(state,ht,mean);
    //cout<< mean<<endl;
    for(int i=0; i<dim_A; i++){
        action(i) = mean(i) + DEVS(i)*MT::rnd.gauss();
        //action(i) = MT::MIN(action(i),1.);
        //action(i) = MT::MAX(action(i),-1.0);
    }


 }

 void RKHSPol::evaluate(const arr state, const arr& ht, arr &hs)
 {
     //eval: h(s) = sum_i \alpha_i K(s_i,s)
     arr sum(dim_A,1);
     sum.setZero();
   // cout<< ht<<endl;
    //cout<< state<<endl;
    for(int i=0;i<ht.d0;i++){
        arr temp(1,dim_A);
        temp[0]() = ht[i].sub(0,dim_A-1);
      //  cout<< temp<<endl;

        sum = sum + kernelFunc(ht[i].sub(dim_A,-1),state)*eye(dim_A) * (~temp);
    }

    hs = (~sum)[0];

    hs(0) = hs(0) - 0.03;
    hs(1) = hs(1) - 0.06;
 }

 double RKHSPol::kernelFunc(const arr state1, const arr state2)
 {
     //Polynomial
     //(xy +c)^d; d=2; c=10. (bad, slow to converge); c=1.0 (faster, converges to a score of 10.

     //should do experiment with diff d. Look promising
     //return pow(state1*state2 + 10.,5);
     //RBF
     arr prod;


     if(Kernel_Type==0){
         //double dd1= state1(0)-state2(0);
         //double dd2= state1(1)-state2(1);
         //return exp(-dd1*dd1/(2.*0.02)-dd2*dd2/(2.*0.00015));
         prod = ~(state1-state2)*(state1-state2);
         return exp(-prod(0)/(2.*0.01));
     }
     else if(Kernel_Type==1){
          prod = ~state1*state2;
          return pow(prod(0) + 1.2,3);
     }
     else if(Kernel_Type==2){
         prod = ~state1*state2;
         return pow(prod(0) + 10.2,3);
    }
     else {
              prod = ~(state1-state2)*(state1-state2);
              return exp(-prod(0)/(2*RBFVariance));
    }
 }


}


