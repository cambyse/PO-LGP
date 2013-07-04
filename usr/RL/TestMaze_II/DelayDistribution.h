#include "HistoryObserver.h"

#include "Config.h"

#include <vector>
#include <map>
#include <tuple>

class DelayDistribution: public HistoryObserver {

public:

    USE_CONFIG_TYPEDEFS;

    typedef std::map<std::tuple<state_t,state_t,unsigned int>,probability_t> pair_dist_map_t;

    DelayDistribution() {}

    ~DelayDistribution() {}

    /** \brief Return delay probability.
     *
     * Return the probability p(s2|s1,delay) of reaching state s2 given one was
     * in state s1 delay time steps ago. */
    probability_t get_fixed_delay_probability(
        const state_t& s1,
        const state_t& s2,
        const idx_t& delay
        );

    /** \brief Return delay probability.
     *
     * Return the probability distribution p(s2|s1,delay) of reaching state s2 given one was
     * in state s1 delay time steps ago. */
    std::vector<probability_t> get_fixed_delay_probability_distribution(
        const state_t& s1,
        const idx_t& delay
        );

    /** \brief Return delay distribution.
     *
     * Return the distribution p(t|s1,s2) of time delay t between visiting state
     * s1 and state s1. */
    void get_delay_distribution(
        const state_t& s1,
        const state_t& s2,
        std::vector<probability_t> & forward,
        std::vector<probability_t> & backward,
        const idx_t& max_delay = -1
        );

    /** \brief Return mediator distribution.
     *
     * Return the probability p(s2|s1,s3) of having visited state s2 on the way
     * from state s1 to state s3. */
    probability_t get_mediator_probability(
        const state_t& s1,
        const state_t& s2,
        const state_t& s3,
        const int& max_window = -1
        );

    /** \brief Return pairwise delay distribution.
     *
     * Return the probability distribution p(dt|s1,s2) of visiting states s1 and s2 with a time delay of dt. */
    void get_pairwise_delay_distribution(
        pair_dist_map_t & dist_map,
        const int& max_dt = -1
        );
};
