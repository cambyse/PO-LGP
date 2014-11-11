#include <Ors/ors.h>
#include <Optim/search.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Motion/feedbackControl.h>
#include <vector>
#include <future>

#include "OuterCostFunction.h"
#include "InnerCostFunction.h"
#include "Simulator.h"
#include "TrajectoryFactory.h"


struct ImitationLearningAlgorithm{
  SearchCMA cma;
  arr samples,values;
  MT::Array<Demonstration> demos;
  InnerCostFunction* icf;
  OuterCostFunction* ocf;
  double costs;

  ImitationLearningAlgorithm(MT::Array<Demonstration> _demos, InnerCostFunction* _icf, OuterCostFunction* _ocf,
                             int _cmaD, int _cmaMu, int _cmaLambda) {
    demos = _demos;
    icf = _icf;
    ocf = _ocf;
    cma.init(_cmaD,_cmaMu,_cmaLambda);
  }

  void start(){
    while(true) {
      cma.step(samples, values);

      // iterate over all samples
      for(uint i=0;i<samples.d0;i++) {
        costs = 0.;

        // iterate over all demonstrations (in parallel)
        MT::Array<std::future<Demonstration>> runs;
        for (uint j=0;j<demos.d0 ;j++) {
          icf->setParam(samples[i],demos(j).world,demos(j).qTraj.d0);
          runs(j) = std::async(std::launch::async,execRun,demos(j),icf);
        }

        for (uint j=0;j<demos.d0 ;j++) {
//          costs = costs + ocf->eval(demos(j),runs(j).get());
        }
        values(i) = costs;
      }
    }
  }
};


int main(int argc,char **argv) {
  MT::initCmdLine(argc,argv);

  /// load/create demonstration data
  MT::Array<Demonstration> demos;
  createToyDemonstrations1(demos);

  /// define an inner cost function (for TrajOpt)
  InnerCostFunction* icf = new SimpleICF(ors::KinematicWorld("scene"));

  /// define an outer cost function (for CMA)
  OuterCostFunction* ocf = new SquaredDistanceOCF();

  cout <<"Test Function: " << ocf->eval(demos(1),demos(0)) << endl;

  /// put everything together in one algorithm and start
  int numParam = 3;
  int cmaMu = -1;
  int cmaLambda = -1;
  ImitationLearningAlgorithm il(demos,icf,ocf,numParam,cmaMu,cmaLambda);
  il.start();
  ///*/*/
  return 0;
}
