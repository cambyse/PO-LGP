#ifndef TEMPORALLYEXTENDEDMODEL_H_
#define TEMPORALLYEXTENDEDMODEL_H_

#include "../HistoryObserver.h"
#include "../Predictor.h"
#include "../Representation/Feature.h"

#include <memory>
#include <vector>

//#define ARMA_NO_DEBUG
#include <armadillo>

class ConjunctiveAdjacency;

class TemporallyExtendedModel: public HistoryObserver, public Predictor {
    //---- typedefs ----//
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(HistoryObserver);
    //arma::SpMat<char> f_mat_t;
    //---- members ----//
private:
    feature_set_t feature_set;
    std::shared_ptr<ConjunctiveAdjacency> N_plus;
    //std::vector<f_mat_t>
    //---- methods ----//
public:
    TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency>);
    virtual ~TemporallyExtendedModel() = default;
    virtual probability_t get_prediction(const_instance_ptr_t,
                                         const action_ptr_t&,
                                         const observation_ptr_t&,
                                         const reward_ptr_t&) const;
    virtual void extend_features();
    virtual void print_features() const;
};

#endif /* TEMPORALLYEXTENDEDMODEL_H_ */
