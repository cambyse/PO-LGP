#define LOG_STRING (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line <<  " | "  << name << " (" << level_str[0] << ") | " << current_time << " | " << msg << " ]" <<  std::endl; 
#include <MT/opengl.h>

#include <JK/active_learning/al_logistic_regression.h>
#include <JK/active_learning/al_gaussian_process.h>
#include <JK/active_learning/al_grounded_symbol.h>
#include <JK/active_learning/al_tester.h>
#include <JK/active_learning/al_gui.h>
#include <JK/active_learning/al_process.h>
#include <JK/active_learning/al_tester.h>

#include <JK/utils/oracle.h>
#include <JK/utils/featureGenerator.h>
#include <JK/utils/sampler.h>
#include <biros/logging.h>

//#include <relational/robotManipulationSimulator.h>
//#include <JK/al/al_logistic_regression.h>
//#include "naiveBayesClassificator.h"
//#include "gaussProcClassificator.h"
//#include "dataReader.h"
//#include "activeLearningProcess.h"
//#include "oracle.h"
//#include "tester.h"
//#include "sampler.h"
//#include "gui.h"
SET_LOG(main, DEBUG);

#include <MT/array_t.cxx>
#include <MT/ors.h>

#include <csignal>

void generateTrueSamples(arr& n, arr& p, arr& fn, arr& fp, const ActiveLearner *al) {
  OnOracle o;
  n.clear();
  p.clear();
  fn.clear();
  fp.clear();
  int j=0, k=0, l=0, m=0;
  MT::Array<arr> sample;
  std::ifstream is("bw-samples");
  //for (double x=-1.; x<1.; x+=0.05) {
  //for (double y=-1.; y<1.; y+=0.05) {
  //for (double z=-1.; z<1.; z+=0.05) {
  for(int i=0; i<2000; ++i) {
    is >> sample;

      
     //MT::Array<arr> sample;
     //sample.append(ARR(0.,0.,0.));
     //sample.append(ARR(0));
     //sample.append(ARR(x,y,z));
     //sample.append(ARR(0));
     sample.reshape(1,4);
     //if(sample(0,1)(0) == 7) {
     if(al->classify(sample) == o.classify(sample) && o.classify(sample)) {
       j++;
       //inlier.append(ARR(-x, -y, -z));
       p.append(sample(0,2) - sample(0,0));
     }
     else if(al->classify(sample) == o.classify(sample) && !o.classify(sample)) {
       m++;
       //inlier.append(ARR(-x, -y, -z));
       n.append(sample(0,2) - sample(0,0));
     }
     else if(o.classify(sample)){
       k++;
       fn.append(sample(0,2) -sample(0,0));
      }
     else {
        l++;
        fp.append(sample(0,2) - sample(0,0));
      }
     //}
  //}}}
}

  p.reshape(j,3);
  n.reshape(m,3);
  fn.reshape(k, 3);
  fp.reshape(l, 3);
}

void shutdown(int) {
  std::cout << "Signal cought. Stop process." << std::endl;
  exit(0);
}
int main(int argc, char** argv) {
  srand(time(NULL));
  ClassifyData d;
  MT::initCmdLine(argc,argv);
  signal(SIGINT,shutdown);

  ActiveLearningProblem problem;
  problem.sampler = new BlocksWorldSampler;
  problem.oracle = new OnOracle;
  problem.generator = new DistanceFeatureGenerator;

  TrainingsDataV train;
  do {
    problem.sampler->sample(train.data);
    train.data.reshape(1,4);
  } while( !problem.oracle->classify(train.data) );
  intA classes;
  classes.append(1);
  train.classes = classes;


   ActiveLearner* al = new GaussianProcessAL(problem);
   //ActiveLearner* al = new LogisticRegression(&sampler);
   al->setTrainingsData(train.data, classes);

   std::ifstream ist("gp.data");

   int c;

   arr n, p, fn, fp;

   MT::Array<arr> sample;

   const char* rd = "";
   if(MT::getParameter<bool>("random_al", false)) rd = "rand-";

   for (int i=0; i<MT::getParameter<int>("steps", 20); ++i) {
     std::stringstream pname, nname, fpname, fnname;
     pname << "true-" << rd << i << ".data";
     nname << "false-" << rd << i << ".data";
     fpname << "fp-" << rd << i << ".data";
     fnname << "fn-" << rd << i << ".data";
     std::ofstream osp(pname.str().c_str());
     std::ofstream osn(nname.str().c_str());
     std::ofstream osfp(fpname.str().c_str());
     std::ofstream osfn(fnname.str().c_str());


     //ist >> sample >> c;
     al->nextSample(sample);
     c = problem.oracle->classify(sample);

     DEBUG_VAR(main, c);

     al->addData(sample, c);

     generateTrueSamples(n, p, fn, fp, al);
     osp << p;
     osn << n;
     osfn << fn;
     osfp << fp;
    }

 
}



