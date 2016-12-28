#include <Core/graph.h>
#include <Ors/ors.h>

double optimSwitchConfigurations(
    mlr::KinematicWorld& world_initial,
    mlr::KinematicWorld& world_final,
    Graph& symbolicState,
    uint microSteps);

