#if 0

#ifndef TL_relGround_h
#define TL_relGround_h

#include "plan.h"
#include "robotManipulationSimulator.h"


struct plan_params {
  uint no_runs;
  double discount;
  uint T;
  PRADA::Reward* reward;
  PRADA::RuleSet rules;
  RobotManipulationSimulator* sim;
};


// Object information
struct ObjectInfo {
  uint id;
  uint type;
  uint size;
  uint color;
  
  void write(std::ostream& = cout);
};




struct RelevanceDistribution {
  std::map<uint, uint> obj2id;
  uintA objects;
  arr weights;
  
  void init(uintA& objects);
  void write(std::ostream& = cout);
};




namespace RelevanceGrounding {
  
 
enum RelevanceType{TAKE_ALL, RANDOM, OPTIMAL, LEARNED};

void sampleObjects(uintA& sampledObjects, const PRADA::SymbolicState& s, PRADA::Reward* reward, RelevanceType relevanceType, uint sampleSize);

void plan_in_single_subnet(AtomL& plan, double& value, const uintA& objects, const PRADA::SymbolicState& s, PRADA::Reward* reward, RelevanceType relevanceType);




// Important (global) methods:

void plan(AtomL& plan, const PRADA::SymbolicState& s, PRADA::Reward* reward, uint subnets_num, uint verification_num, RelevanceType relevanceType, uint subnets_size, uint verification_size);

void initPlanParams(PRADA::Reward* reward, PRADA::RuleSet& rules, uint no_runs, double discount, uint T, RobotManipulationSimulator* sim);





void setObjectInfos(const MT::Array< ObjectInfo* >& oinfos);


};




#endif

#endif