#include "al_tester.h"
#include "al.h"
#include "al_problem.h"

#include <JK/utils/sampler.h>
#include <JK/utils/oracle.h>

double test(const uint numTests, const ActiveLearner* al, const ActiveLearningProblem* problem, const MT::String filename) {
  uint correct;
  MT::Array<arr> sample;
#pragma omp parallel for
  for(uint i=0;i<numTests;++i) {
    problem->sampler->sample(sample);
    if (al->classify(sample) == problem->oracle->classify(sample)) correct++;
  }
  double result = correct/(double) numTests;
  if (filename != "") {
    std::ofstream out(filename.p, std::ios::app);
    out << result << std::endl;
  }
  return result;
}
