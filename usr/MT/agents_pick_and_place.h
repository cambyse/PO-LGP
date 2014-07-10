#include <Core/array.h>

struct AgentsPickAndPlace{
  struct sAgentsPickAndPlace *s;
  AgentsPickAndPlace(const char* modelFile);

  //-- actions: all functions return costs of the action
  double move(uint agent, const arr& q);
  double link(uint shape1, uint shape2);
  double delink(uint agent, uint shape);
};

