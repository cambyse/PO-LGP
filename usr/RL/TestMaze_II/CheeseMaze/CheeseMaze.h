#ifndef CHEESEMAZE_H_
#define CHEESEMAZE_H_

#include "../Environment.h"
#include "../Visualizer.h"

#include <QGraphicsSvgItem>

#include "../Config.h"
#include "../Representation/Feature.h"

#include "CheeseMazeObservation.h"
#include "CheeseMazeAction.h"
#include "../Representation/ListedReward.h"

class CheeseMaze: public Environment, public Visualizer {

public:

    USE_CONFIG_TYPEDEFS;
    typedef CheeseMazeAction action_t;
    typedef CheeseMazeObservation observation_t;
    typedef ListedReward reward_t;

    CheeseMaze();
    virtual ~CheeseMaze() override;

    virtual void render_initialize(QGraphicsView * v) override;
    virtual void render_update() override;
    virtual void render_tear_down() override;
    virtual void perform_transition(const action_ptr_t& action) override;
    virtual void perform_transition(const action_ptr_t & a, observation_ptr_t & o, reward_ptr_t & r ) override;
    virtual void get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const override;

private:

    /* Relation between state index and observation.
     *   ┏━━━┳━━━┳━━━┳━━━┳━━━┓
     *   ┃ 0 ┃ 3 ┃ 4 ┃ 7 ┃ 8 ┃
     *   ┣━━━╋━━━╋━━━╋━━━╋━━━┫
     *   ┃ 1 ┃   ┃ 5 ┃   ┃ 9 ┃
     *   ┣━━━┫   ┣━━━┫   ┣━━━┫
     *   ┃ 2 ┃   ┃ 6 ┃   ┃10 ┃
     *   ┗━━━┛   ┗━━━┛   ┗━━━┛
     *   ┏━━━┳━━━┳━━━┳━━━┳━━━┓
     *   ┃ NW┃ NS┃  N┃ NS┃ NE┃
     *   ┣━━━╋━━━╋━━━╋━━━╋━━━┫
     *   ┃ EW┃   ┃ EW┃   ┃ EW┃
     *   ┣━━━┫   ┣━━━┫   ┣━━━┫
     *   ┃ESW┃   ┃ESW┃   ┃ESW┃
     *   ┗━━━┛   ┗━━━┛   ┗━━━┛
     */

    QGraphicsSvgItem *mouse = nullptr;                         ///< Svg image for rendering the mouse.
    QGraphicsSvgItem *cheese = nullptr;                        ///< Svg image for rendering the cheese.
    QGraphicsLineItem *action_line = nullptr;                  ///< Line showing the last action.
    QGraphicsEllipseItem *action_point = nullptr;              ///< Circle showing the last position for showing the last action.

    QGraphicsSvgItem * CheeseMazeObservation_N = nullptr;
    QGraphicsSvgItem * CheeseMazeObservation_NE = nullptr;
    QGraphicsSvgItem * CheeseMazeObservation_NS = nullptr;
    QGraphicsSvgItem * CheeseMazeObservation_NW = nullptr;
    QGraphicsSvgItem * CheeseMazeObservation_EW = nullptr;
    QGraphicsSvgItem * CheeseMazeObservation_ESW = nullptr;

    QGraphicsSvgItem * cheese_reward = nullptr;
    QGraphicsSvgItem * wall_punish = nullptr;
    QGraphicsSvgItem * normal_punish = nullptr;

    int current_state_idx = 0;
    int last_state_idx    = 0;
    action_t last_action;
    observation_t last_observation;
    reward_t last_reward;

    static int get_x_pos(int state_idx);
    static int get_y_pos(int state_idx);
    int current_x_pos() const;
    int current_y_pos() const;
    int last_x_pos() const;
    int last_y_pos() const;
    observation_t get_observation(int state_idx) const;

};

#endif /* CHEESEMAZE_H_ */
