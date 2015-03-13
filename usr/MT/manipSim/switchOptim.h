#include <Core/graph.h>
#include <Ors/ors.h>

double optimSwitchConfigurations(
    ors::KinematicWorld& world_initial,
    ors::KinematicWorld& world_final,
    Graph& symbolicState,
    uint microSteps);

