#include "TemporallyExtendedModel.h"

#include "ConjunctiveAdjacency.h"

typedef TemporallyExtendedModel TEM;

TEM::TemporallyExtendedModel(): N_plus(new ConjunctiveAdjacency()) {}

TEM::probability_t TEM::get_prediction(const_instance_ptr_t,
                                       const action_ptr_t&,
                                       const observation_ptr_t&,
                                       const reward_ptr_t&) const {
    return 0;
}
