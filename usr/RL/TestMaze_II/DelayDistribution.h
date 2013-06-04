#include "HistoryObserver.h"
#include "Data.h"
#include "Representation/Representation.h"


class DelayDistribution: public HistoryObserver {

public:

    USE_DATA_TYPEDEFS;
    USE_REPRESENTATION_TYPEDEFS;

    DelayDistribution() {}

    ~DelayDistribution() {}

    /** \brief Return delay probability.
     *
     * Return the probability p(s2|s1,delay) of reaching state s2 given one was
     * in state s1 delay time steps ago. */
    probability_t get_delay_probability(
        const state_t& s1,
        const state_t& s2,
        const idx_t& delay
        );

};
