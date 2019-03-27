#pragma once

#include <Optim/optimization.h>
#include <Kin/taskMap.h>

struct Branch
{
    std::vector< int > local_to_global;
    std::vector< int > global_to_local;
    double p; // probability to reach the leaf
    uint leaf_id;

    static Branch computeMicroStepBranch(const Branch& a, int stepsPerPhase);
    static Branch linearTrajectory(int T);
};

bool operator==(const Branch& a, const Branch& b);
bool operator<(const Branch& a, const Branch& b);


struct Task {
  TaskMap *map;
  const mlr::Enum<ObjectiveType> type;  ///< element of {sumOfSqr, inequality, equality}
  mlr::String name;
  arr target, prec;        ///< optional linear, time-dependent, rescaling (with semantics of target & precision) // in global time
  Branch branch;           ///< way to traverse x allow traj tree opt : link from branch index to x index

  Task(TaskMap *m, const ObjectiveType& type) : map(m), type(type){}
  ~Task(){ if(map) delete map; map=NULL; }

  void setCostSpecs(int fromTime, int toTime, int T,
                    const arr& _target=ARR(0.),
                    double _prec=1.);
  void setCostSpecs(double fromTime, double toTime,
                    int stepsPerPhase, uint T,
                    const arr& _target,
                    double _prec, const Branch& branch_time_spec=Branch());

  int to_local_t(int global_t) const { CHECK(global_t < branch.global_to_local.size(), "wrong dimensions!");return branch.global_to_local[global_t]; }
  int to_global_t(int local_t) const { CHECK(local_t < branch.local_to_global.size(), "wrong dimensions!"); return branch.local_to_global[local_t];/*local_t < branch.local_to_global.size() ? branch.local_to_global[local_t] : branch.local_to_global.back();*/ }

  bool isActive(uint t){ return (prec.N>t && prec(t)); }
  void write(std::ostream& os) const{
    os <<"TASK '" <<name <<"'"
      <<"  type=" <<type
      <<"  order=" <<map->order
      <<"  target=[" <<target <<']'
      <<"  prec=";
    writeConsecutiveConstant(os,prec);
  }

  static Task* newTask(const Node* specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T); ///< create a new Task from specs
};
stdOutPipe(Task)
