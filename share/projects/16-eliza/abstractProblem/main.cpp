#include<Core/util.h>
#include<Core/array.h>
#include<RL/myEnvironment.h>
#include<RL/myPolicy.h>
#include <stdlib.h>


using namespace std;
using namespace mdp; //myEnvironment + myENV
using namespace mlr;


int main(int argc, char *argv[]){

    myENVIRONMENT * env; //pointer to class; defined in myEnvironment.h

    char *filename = "results.dat";
    cout<< "Writing results to .............. " <<filename<<endl;

    uint H; //horizon of an episode
    uint numEpisodes; //number of episodes to evaluate the gradient(for each iteration);
    uint numIterations; //number of updates
    uint numRuns; //runs for averaging performance - ?

    env = new myENV(4, 1, 1.); //myENV - inherits ENVIRONMENT - sDim, aDim, discount
    H = 5000; //just one step? - in rollout() mentioned 20
    numEpisodes = 20;
    numIterations = 500;
    numRuns = 10;

    myPol specificProbl1(*env, H, numEpisodes, numIterations); //defined in myPol.h
    mlr::rnd.clockSeed();
    arr rewards;

    uint alg = 0;
    if(argc>1)
        if(strcmp(argv[1],"gpomdp") == 0)
            alg = 1;

    rewards = specificProbl1.run(alg);
    cout<<"rewards "<< rewards <<endl;

    arr DATA(1,rewards.d0);
    //~rewards or not?
    DATA.append(~rewards);
    for(uint run=1; run<numRuns; run++)
    {
         cout<<"--------- RUNS: "<< run <<" ---------"<<endl;
         mlr::rnd.clockSeed();
         myPol specificProbl2(*env, H, numEpisodes, numIterations);
         DATA.append( ~(specificProbl2.run(alg)) );
    }

    DATA.delRows(0);

    arr means(rewards.d0);
    arr vars(rewards.d0);

    arr DATA_Transpose = ~DATA;
    for(uint i=0; i<rewards.d0; i++){
        means(i) = sum(DATA_Transpose[i]) / DATA_Transpose[i].d0;
        vars(i)  = var(DATA_Transpose[i]);
    }

    arr all(4,means.d0);
    for(uint index=0; index<means.d0; index++){
        all[0](index) = index;
        vars(index) = sqrt(vars(index)) / sqrt(numRuns); //standard deviation of the mean estimator
    }

    all[1]() = means;
    all[2]() = means - 1.96*vars; //95% of confidence interval
    all[3]() = means + 1.96*vars;


    std::ofstream fil(filename);
    all.write(fil, "","\n", " ", false, false);
    fil.close();

    cout<<"mean performance: " << means <<endl;

  //gnuplot(means);
  //gnuplot("splot 'z.pltX' matrix with pm3d, 'z.pltX' matrix with lines");

  //MT::wait(100.);

  return 0;
}

