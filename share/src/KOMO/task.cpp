#include "task.h"
#include <Core/graph.h>

bool operator==(const Branch& a, const Branch& b)
{
    return (a.p == b.p) && (a.local_to_global == b.local_to_global) && (a.global_to_local == b.global_to_local) && (a.leaf_id == b.leaf_id);
}

bool operator<(const Branch& a, const Branch& b)
{
    return a.leaf_id < b.leaf_id;
}

Branch computeMicroStepBranch(const Branch& a, int stepsPerPhase)
{
    Branch b;

    b.p = a.p;
    b.leaf_id = a.leaf_id;

    // local
    int n_phases = a.local_to_global.size() - 1;

    b.local_to_global = std::vector< int >(n_phases * stepsPerPhase); // 2 == prefix

    for(int phaseEndIndex = 1; phaseEndIndex <= n_phases; ++phaseEndIndex)
    {
        const uint phaseEnd = a.local_to_global[phaseEndIndex];

        for(int s = 0; s < stepsPerPhase; ++s)
        {
            b.local_to_global[phaseEndIndex * stepsPerPhase - s - 1] = phaseEnd * stepsPerPhase - s -1;
        }
    }

    // global
    int n_global_phases = a.global_to_local.size() - 1;
    b.global_to_local = std::vector< int >(n_global_phases * stepsPerPhase, -1);

    for(int local = 0; local < n_phases * stepsPerPhase; ++local)
    {
        const auto global =b.local_to_global[local];
        b.global_to_local[global] = local;
    }

    return b;
}

//===========================================================================


//#define STEP(p) (floor(branch.pathFromRoot[p]*double(stepsPerPhase) + .500001))-1

double phaseToStep(double p, double stepsPerPhase)
{
    return floor(p*double(stepsPerPhase));//(floor(p*double(stepsPerPhase) + .500001))-1;
}

void Task::setCostSpecs(int fromTime,
                        int toTime, int T,
                        const arr& _target,
                        double _prec){
    if(&_target) target = _target; else target = {0.};
    if(fromTime<0) fromTime=0;
    CHECK(toTime>=fromTime,"");
    prec.resize(T).setZero();
    for(int local = 0; local < int(branch.local_to_global.size())-2; ++local)
    {
        if(local >= fromTime && local < toTime)
        {
            auto global = branch.local_to_global[local];
            prec(global) = _prec;
        }
    }
}

void Task::setCostSpecs(double fromTime, double toTime, int stepsPerPhase, uint T, const arr& _target, double _prec, const Branch& branch_time_spec){
    if(stepsPerPhase<0) stepsPerPhase=T;
    uint maxStepOnBranch = *std::max_element(branch_time_spec.local_to_global.begin(), branch_time_spec.local_to_global.end()) * stepsPerPhase;
    if(phaseToStep(toTime, stepsPerPhase)>maxStepOnBranch){
        LOG(-1) <<"beyond the time!: endTime=" <<toTime <<" phases=" <<double(T)/stepsPerPhase;
    }

    CHECK(&branch_time_spec, "case without branch not handled yet!");
    branch = computeMicroStepBranch(branch_time_spec, stepsPerPhase);

    int tFrom = (fromTime<0.?0              :phaseToStep(fromTime, stepsPerPhase));
    int tTo =   (toTime  <0.?maxStepOnBranch:phaseToStep(toTime, stepsPerPhase));
    if(tTo<0) tTo=0;
    if(tFrom>tTo && tFrom-tTo<=(int)map->order) tFrom=tTo;

    setCostSpecs(tFrom, tTo, T, _target, _prec);
}

//===========================================================================

Task* Task::newTask(const Node* specs, const mlr::KinematicWorld& world, int stepsPerPhase, uint T){
    if(specs->parents.N<2) return NULL; //these are not task specs

    //-- check the term type first
    ObjectiveType termType;
    mlr::String& tt=specs->parents(0)->keys.last();
    if(tt=="MinSumOfSqr") termType=OT_sumOfSqr;
    else if(tt=="LowerEqualZero") termType=OT_ineq;
    else if(tt=="EqualZero") termType=OT_eq;
    else return NULL;

    //-- try to crate a map
    TaskMap *map = TaskMap::newTaskMap(specs, world);
    if(!map) return NULL;

    //-- create a task
    Task *task = new Task(map, termType);

    if(specs->keys.N) task->name=specs->keys.last();
    else{
        task->name = map->shortTag(world);
        //    for(Node *p:specs->parents) task->name <<'_' <<p->keys.last();
        task ->name<<"_o" <<task->map->order;
    }

    //-- check for additional continuous parameters
    if(specs->isGraph()){
        const Graph& params = specs->graph();
        arr time = params.get<arr>("time",{0.,1.});
        task->setCostSpecs(time(0), time(1), stepsPerPhase, T, params.get<arr>("target", {}), params.get<double>("scale", {1.}));
    }else{
        task->setCostSpecs(0, T-1, T, {}, 1.);
    }
    return task;
}

//===========================================================================
