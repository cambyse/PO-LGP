
#include<Core/util.h>
#include<Core/array.h>
#include<RL/environment.h>
#include<RL/functional.h>
#include <stdlib.h>



using namespace std;
using namespace mdp;
using namespace mlr;


int main(int argc, char *argv[]){

    ENVIRONMENT * env;


    char *filename = "results.dat";
    cout<< "Writing results to .............. " <<filename<<endl;

    int numEpisode; //number of episodes to evaluate Gradient (at each iteration);
    int H; //horizon  
    int numRuns; // runs for averaging performance
    int numIterations;
    int numCentres;// numnber of centres for the policy (RKHS policy)


    env = new CANNON(2,2, 1.); //CANNON
    numEpisode = 200;
    H = 1;
    numCentres = 100;
    numRuns = 10;
    numIterations = 5000;


    //environment,#centre,horizon,runs
    RKHSPol rkhs1(*env,numCentres,H,numEpisode,numIterations);
    mlr::rnd.clockSeed();
    arr rewards;


    rewards = rkhs1.run();
    cout<< "rewards "<<rewards <<endl;

    arr DATA(1,rewards.d0);
    DATA.append(rewards);
    for(int run=1;run<numRuns;run++){
         cout<<"--------- RUNS: "<<run << " --------------------"<<endl;
         mlr::rnd.clockSeed();
         RKHSPol rkhs2(*env,numCentres,H,numEpisode,numIterations);
         DATA.append(rkhs2.run());
    }

    //rewards = rewards* 1./(double)numRuns;
    DATA.delRows(0);

    arr means(rewards.d0);
    arr vars(rewards.d0);

    arr DATA_Tranpose = ~DATA;
    for(int i=0;i<rewards.d0;i++){
        means(i) = sum(DATA_Tranpose[i])/DATA_Tranpose[i].d0;
        vars(i)  = var (DATA_Tranpose[i]);
    }

    arr all(4,means.d0);
    for(int index=0;index<means.d0;index++){
        all[0](index) = index;
        vars(index) = sqrt(vars(index))/sqrt(numRuns); //standard deviation of the mean estimator
    }

    all[1]()=means;
    all[2]()=means - 1.96*vars; //95% of confidence interval
    all[3]()=means + 1.96*vars;


    std::ofstream fil(filename);
    all.write(fil, "","\n", " ", false, false);
    fil.close();

    cout<<"mean performance: "<< means <<endl;

  //gnuplot(means);
  //gnuplot("splot 'z.pltX' matrix with pm3d, 'z.pltX' matrix with lines");

  //MT::wait(100.);


  return 0;
}

