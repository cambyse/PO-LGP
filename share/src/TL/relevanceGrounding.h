#ifndef TL_relGround_h
#define TL_relGround_h

#include "plan.h"
#include "ors_actionInterface.h"


struct plan_params {
  uint no_runs;
  double discount;
  uint T;
  TL::LogicEngine* le;
  TL::Goal* goal;
  TL::RuleSet rules;
  ActionInterface* ai;
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

void sampleObjects(uintA& sampledObjects, const TL::State& s, TL::Goal* goal, RelevanceType relevanceType, uint sampleSize);

void plan_in_single_subnet(PredIA& plan, double& value, const uintA& objects, const TL::State& s, TL::Goal* goal, RelevanceType relevanceType);




// Important (global) methods:

void plan(PredIA& plan, const TL::State& s, TL::Goal* goal, uint subnets_num, uint verification_num, RelevanceType relevanceType, uint subnets_size, uint verification_size);

void initPlanParams(TL::LogicEngine* le, TL::Goal* goal, TL::RuleSet& rules, uint no_runs, double discount, uint T, ActionInterface* ai);





void setObjectInfos(const MT::Array< ObjectInfo* >& oinfos);


};




#endif
