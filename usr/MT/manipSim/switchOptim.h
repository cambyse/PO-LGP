#include <Core/graph.h>
#include <Kin/kin.h>

double optimSwitchConfigurations(
    mlr::KinematicWorld& world_initial,
    mlr::KinematicWorld& world_final,
    Graph& symbolicState,
    uint microSteps);

