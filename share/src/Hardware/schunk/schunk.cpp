#include "schunk.h"

#ifdef MLR_SCHUNK

struct shutdownFct { void *classP; void (*call)(void*); };
mlr::Array<shutdownFct> shutdownFunctions; ///< list of shutdown functions
void addShutdown(void *classP, void (*call)(void*)) { shutdownFunctions.memMove=true; shutdownFct f; f.classP=classP; f.call=call; shutdownFunctions.append(f); }
bool schunkShutdown=false;
void schunkEmergencyShutdown(int) {
  MLR_MSG("initiating smooth shutdown...");
  schunkShutdown=true;
  for (uint i=shutdownFunctions.N; i--;) { shutdownFunctions(i).call(shutdownFunctions(i).classP); }
  MLR_MSG("...done");
}

void shutdownLWA(void* p) { MLR_MSG("...");  SchunkArm *lwa=(SchunkArm*)p;  lwa->close(); }
void shutdownSDH(void* p) { MLR_MSG("...");  SchunkHand *sdh=(SchunkHand*)p;  sdh->stop();  sdh->close(); }
void shutdownDSA(void* p) { MLR_MSG("...");  SchunkSkin *dsa=(SchunkSkin*)p;  dsa->close(); }

#else

bool schunkShutdown=false;
void schunkEmergencyShutdown(int) {}

#endif
