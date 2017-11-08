#include "geom.h"

#include <Core/thread.h>

template<class T> using list = mlr::Array<std::shared_ptr<T>>;

/** A store for geometries (maybe only a singleton, or distinguish dynamic/perceptual vs static/ad hoc known?) */
struct GeomStore : VariableBase{
  list<Geom> geoms;

  void checkConsistency(); //asserts geom IDs are one-to-one with index in this list
};
