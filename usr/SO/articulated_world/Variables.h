#ifndef VARIABLES_H__
#define VARIABLES_H__

#include <biros/biros.h>
#include <MT/ors.h>

// ============================================================================
class PerceptsVar : public Variable {
public:
  mlr::Array<ors::Vector*> objects;

  PerceptsVar() : Variable("percpets variable") {
    reg_objects();
  }
};

// ============================================================================
struct RobotPosVar : public Variable {
  ors::Vector pos;

  RobotPosVar() : Variable("robot pos variable") {
    reg_pos();
  }
};

// ============================================================================
class MovementRequestVar : public Variable {
public:
  ors::Vector control_u;

  MovementRequestVar() : Variable("Movement Request") {
    reg_control_u();
  }
};


// ============================================================================
typedef arr Pose6D_t;


// ============================================================================
class WorldStateVar : public Variable {
public:
  /// list of DOFs (degree of freedom)
  arr  dofs;
  bool changed;

  WorldStateVar() : Variable("WorldState") {
    reg_dofs();
  }
};

#endif /* end of include guard: VARIABLES_H__ */
