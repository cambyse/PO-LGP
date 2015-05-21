#include <Core/module.h>
#include "activity.h"
#include "relationalMachine.h"

struct RelationalMachineModule : Module{
  ACCESS(MT::String, effects)
  ACCESS(ActivityL, A)
  ACCESS(MT::String, state)
  ACCESS(RelationalMachine, RM)

private:

  ofstream fil;

public:
  RelationalMachineModule();
  ~RelationalMachineModule();

  /// @name module implementations
  void open();
  void step();
  void close();
};
