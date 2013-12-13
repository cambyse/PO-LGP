#ifndef LISTEDREWARD_H_
#define LISTEDREWARD_H_

#include "../AbstractReward.h"

#include <vector>

class ListedReward: public AbstractReward {
public:
    /** \brief Initialize a reward with specified list and index.
     *
     * @param list initializes ListedReward::reward_list
     * @param idx initializes ListedReward::reward_index */
    ListedReward(
        const std::vector<value_t> & list = std::vector<value_t>(),
        const uint & idx = 0
        );

    virtual Iterator begin() const override;

    virtual ptr_t next() const override;

    virtual bool operator!=(const AbstractReward &other) const override;

    virtual bool operator<(const AbstractReward &other) const override;

    virtual const char * print() const override;

    inline virtual const std::string space_descriptor() const override { return "ListedReward"; }

    /** \brief Get the reward value.
     *
     * Returns the element from ListedReward::reward_list specified by
     * ListedReward::reward_index. */
    virtual value_t get_value() const override { return reward_list[reward_index]; }

    /** \brief Set the reward value.
     *
     * Checks for a matching value in ListedReward::reward_list. */
    virtual void set_value(const value_t& v);

protected:
    virtual void set_type(REWARD_TYPE t) override;
    std::vector<value_t> reward_list; ///< List of all possible reward values.
    uint reward_index;                ///< Index specifying the reward value.
};

#endif /* LISTEDREWARD_H_ */
