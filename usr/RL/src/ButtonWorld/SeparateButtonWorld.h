#ifndef SEPARATEBUTTONWORLD_H_
#define SEPARATEBUTTONWORLD_H_

#include "ButtonWorld.h"

class SeparateButtonWorld: public ButtonWorld {

    //----typedefs/classes----//

    //----members----//
protected:
    action_t last_action;
    std::vector<QGraphicsEllipseItem*> reward_array;
    std::vector<int> last_reward;

    //----methods----//
public:
    SeparateButtonWorld(int s = 1, std::vector<probability_t> p = std::vector<probability_t>());
    SeparateButtonWorld(int s, double alpha);
    virtual ~SeparateButtonWorld() override = default;
    virtual void render_initialize(QGraphicsView * v) override;
    virtual void render_update() override;
    virtual void render_tear_down() override;
    virtual void perform_transition(const action_ptr_t& action) override;
    probability_t get_prediction(const_instance_ptr_t,
                                 const action_ptr_t&,
                                 const observation_ptr_t&,
                                 const reward_ptr_t&) const override;
};

#endif /* SEPARATEBUTTONWORLD_H_ */
