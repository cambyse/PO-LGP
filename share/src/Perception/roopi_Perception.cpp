#include "roopi_Perception.h"

#include <Perception/filter.h>
#include <Perception/syncFiltered.h>
#include <Perception/percViewer.h>

ThreadL newPerceptionFilter(bool view){
  ThreadL threads;
  threads.append(new Filter());
  if(view) threads.append(new PercViewer("percepts_filtered"));
  return threads;
}
