#include "activity.h"

Singleton<Graph> ActivityRegistry;

Graph& activityRegistry(){ return ActivityRegistry(); }
