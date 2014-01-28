#include <relational/literals.h>
#include <Core/geo.h>
#include <Ore/ors.h>

struct Object{
  String name;
  geo::Mesh mesh;
  uint ID;
  LitL constantLiterals; ///< constant properties of the object

  geo::Transformation pose; ///< essentially a position and quaternion
  double posePrecision;
  
  Object():ID(0), posePrecision(0.){ pose.setZero(); }
};

struct SceneRepresentation{
  geo::Mesh pointCloud; ///< stores the direct kinect output
  
  ObjectL objects; ///< stores the output of perception
  
  SymL symbolDefinitions;
  SymbolicState symbolicState;

  void loadSymbolDefinitions(const char* filename);
  void loadObjectDescriptions(const char* filename);
  void computeSymbolicState(); ///< applies the symbol definitions on the geometric state to compute the symbolic state
};


