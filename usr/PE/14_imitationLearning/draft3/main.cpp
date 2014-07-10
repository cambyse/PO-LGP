#include <Ors/ors.h>
#include <Optim/search.h>
#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/feedbackControl.h>
#include <vector>
#include <future>

#include "InnerCostFunction.h"
#include "OuterCostFunction.h"
#include "Demonstration.h"
#include "TrajectoryFactory.h"
#include "Simulator.h"

struct ImitationLearningAlgorithm{
  SearchCMA cma;
  arr samples,values;
  MT::Array<Demonstration> demos;
  InnerCostFunction* icf;
  OuterCostFunction* ocf;
  double costs;

  ImitationLearningAlgorithm(MT::Array<Demonstration> _demos, InnerCostFunction* _icf, OuterCostFunction* _ocf,
                             int _cmaD, int _cmaMu, int _cmaLambda, double _cmaLo, double _cmaHi) {
    demos = _demos;
    icf = _icf;
    ocf = _ocf;
    cma.init(_cmaD,_cmaMu,_cmaLambda,_cmaLo,_cmaHi);
  }

  void start(){
    while(true) {
      cma.step(samples, values);
      cout <<"Samples: " << samples << endl;
      cout <<"values: " << values << endl;

      // iterate over all samples
      for(uint i=0;i<samples.d0;i++) {
        costs = 0.;

        // iterate over all demonstrations (in parallel)
        std::vector<std::future<Demonstration*>> runs;
        for (uint j=0;j<demos.d0 ;j++) {
          icf->setParam(samples[i],demos(j).world,demos(j).qTraj.d0);

          runs.push_back(std::async(std::launch::async,execRun,&demos(j),icf));
        }

        for (uint j=0;j<demos.d0 ;j++) {
          costs = costs + ocf->eval(demos(j),*runs[j].get(),samples[i]);

        }

        values(i) = costs/demos.d0;

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
  icf->setParam(ARR(1.,1.,1.),demos(0).world,demos(0).qTraj.d0);

  /// define an outer cost function (for CMA)
  OuterCostFunction* ocf = new SquaredDistanceOCF(PenaltyType::quadratic,RegularizationType::noRegularization);
  ocf->paramBounds = ARR(0.,10.);
  ocf->penaltyFactor = 1e2;

  /// put everything together in one algorithm and start
  int numParam = icf->numParam;
  int cmaMu = -1;
  int cmaLambda = -1;
  double cmaLo = 2.;
  double cmaHi = 3.;
  ImitationLearningAlgorithm il(demos,icf,ocf,numParam,cmaMu,cmaLambda,cmaLo,cmaHi);
  il.start();

  return 0;
}
