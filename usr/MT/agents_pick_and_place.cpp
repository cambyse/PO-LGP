#include "agents_pick_and_place.h"

struct sAgentsPickAndPlace{
  ors::KinematicWorld world;

  sAgentsPickAndPlace(const char* modelFile):world(modelFile){
  }
};

AgentsPickAndPlace::AgentsPickAndPlace(const char* modelFile):s(NULL){
  s = new sAgentsPickAndPlace(modelFile);
}

double AgentsPickAndPlace::move(uint agent, const arr& q){
  world.setJointState(q, NoArr, agent);
}

double AgentsPickAndPlace::link(uint shape1, uint shape2){
}

double AgentsPickAndPlace::delink(uint agent, uint shape){
}
