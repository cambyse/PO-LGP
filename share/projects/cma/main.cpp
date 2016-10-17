
#include<Core/util.h>
#include<Core/array.h>
#include<RL/environment.h>
#include<RL/functional.h>
#include <stdlib.h>



using namespace std;
using namespace mdp;
using namespace mlr;


int main(int argc, char *argv[]){


  arr means;


  gnuplot(means);


  mlr::wait(1.);


  return 0;
}

