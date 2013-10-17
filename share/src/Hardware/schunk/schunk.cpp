#include "schunk.h"

#ifdef MT_SCHUNK

struct shutdownFct { void *classP; void (*call)(void*); };
MT::Array<shutdownFct> shutdownFunctions; ///< list of shutdown functions
void addShutdown(void *classP, void (*call)(void*)) { shutdownFunctions.memMove=true; shutdownFct f; f.classP=classP; f.call=call; shutdownFunctions.append(f); }
bool schunkShutdown=false;
void schunkEmergencyShutdown(int) {
  MT_MSG("initiating smooth shutdown...");
  schunkShutdown=true;
  for (uint i=shutdownFunctions.N; i--;) { shutdownFunctions(i).call(shutdownFunctions(i).classP); }
  MT_MSG("...done");
}

void shutdownLWA(void* p) { MT_MSG("...");  SchunkArm *lwa=(SchunkArm*)p;  lwa->close(); }
void shutdownSDH(void* p) { MT_MSG("...");  SchunkHand *sdh=(SchunkHand*)p;  sdh->stop();  sdh->close(); }
void shutdownDSA(void* p) { MT_MSG("...");  SchunkSkin *dsa=(SchunkSkin*)p;  dsa->close(); }

#else

bool schunkShutdown=false;
void schunkEmergencyShutdown(int) {}

#endif
