#ifndef JOINTBUTTONWORLD_H_
#define JOINTBUTTONWORLD_H_

#include "ButtonWorld.h"

class JointButtonWorld: public ButtonWorld {

    //----typedefs/classes----//

    //----members----//
protected:
    action_ptr_t last_action;
    QGraphicsEllipseItem* reward_item;
    reward_ptr_t last_reward;

    //----methods----//
public:
    JointButtonWorld(int s = 1, std::vector<probability_t> p = std::vector<probability_t>());
    JointButtonWorld(int s, double alpha);
    virtual ~JointButtonWorld() override = default;
    virtual void render_initialize(QGraphicsView * v) override;
    virtual void render_update() override;
    virtual void render_tear_down() override;
    virtual void perform_transition(const action_ptr_t& action) override;
    probability_t get_prediction(const_instance_ptr_t,
                                 const action_ptr_t&,
                                 const observation_ptr_t&,
                                 const reward_ptr_t&) const override;
private:
    virtual probability_t prob_from_arrays(const std::vector<bool> & last_pushed,
                                           const std::vector<bool> & this_pushed) const final;
};

#endif /* JOINTBUTTONWORLD_H_ */
