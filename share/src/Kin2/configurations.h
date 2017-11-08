#include "frame.h"
#include "geomStore.h"

struct GeomConfiguration{
  const GeomStore& store;
  intA IDs;
  arr poses;
};

struct FrameConfiguration{
  list<Frame> frames;
};

struct KinematicConfiguration : FrameConfiguration{

  //proxies

  //loop constraints

  void checkConsistency(); //checks for consistency of the data structure (parent-children, topological sorting)
};

struct DynamicConfiguration {
  KinematicConfiguration K;

  enum LinkType { BT_none=-1, BT_dynamic=0, BT_kinematic, BT_static };

  std::vector<LinkType> linkTypes;

  void checkConsistency(); //asserts masses.size()==K.links.size()
};
