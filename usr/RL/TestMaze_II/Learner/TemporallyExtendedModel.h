#ifndef TEMPORALLYEXTENDEDMODEL_H_
#define TEMPORALLYEXTENDEDMODEL_H_

#include "../HistoryObserver.h"
#include "../Predictor.h"

#include <memory>

class AdjacencyOperator;

class TemporallyExtendedModel: public HistoryObserver, public Predictor {
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(HistoryObserver);
    TemporallyExtendedModel();
    virtual ~TemporallyExtendedModel() = default;
    virtual probability_t get_prediction(const_instance_ptr_t,
                                         const action_ptr_t&,
                                         const observation_ptr_t&,
                                         const reward_ptr_t&) const;
private:
    std::shared_ptr<AdjacencyOperator> N_plus;
};

#endif /* TEMPORALLYEXTENDEDMODEL_H_ */
