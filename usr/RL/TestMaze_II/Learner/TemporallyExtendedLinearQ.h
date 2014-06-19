#ifndef TEMPORALLYEXTENDEDLINEARQ_H_
#define TEMPORALLYEXTENDEDLINEARQ_H_

#include "../Planning/Policy.h"
#include "TemporallyExtendedFeatureLearner.h"

class TemporallyExtendedLinearQ: public Policy, public TemporallyExtendedFeatureLearner {

    //----typedefs/classes----//

public:

    DISAMBIGUATE_CONFIG_TYPEDEFS(TemporallyExtendedFeatureLearner);

    //----members----//

private:

    double discount;

    //----methods----//

public:

    TemporallyExtendedLinearQ(std::shared_ptr<ConjunctiveAdjacency>,double);

    virtual ~TemporallyExtendedLinearQ() = default;

    virtual action_ptr_t get_action(const_instance_ptr_t) override;

    virtual void set_discount(double d) { discount = d; }

    virtual double get_discount() const { return discount; }

protected:



};

#endif /* TEMPORALLYEXTENDEDLINEARQ_H_ */
