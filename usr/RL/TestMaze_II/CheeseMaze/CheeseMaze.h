#ifndef CHEESEMAZE_H_
#define CHEESEMAZE_H_

#include "../Environment.h"
#include "../Visualizer.h"

#include <QGraphicsSvgItem>

#include "../Config.h"
#include "../Feature.h"

#include "CheeseMazeObservation.h"
#include "CheeseMazeAction.h"
#include "../ListedReward.h"

class CheeseMaze: public Environment, public Visualizer {

public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::const_feature_ptr_t f_ptr_t;
    typedef CheeseMazeAction action_t;
    typedef CheeseMazeObservation observation_t;
    typedef ListedReward reward_t;

    CheeseMaze();
    virtual ~CheeseMaze();

    virtual void render_initialize(QGraphicsView * v) override;
    virtual void render_update() override;
    virtual void render_tear_down() override;
    virtual void perform_transition(const action_ptr_t& action) override;
    virtual void get_features(std::vector<f_ptr_t> & basis_features, FeatureLearner::LEARNER_TYPE type) const override;

private:

    QGraphicsSvgItem *mouse;                         ///< Svg image for rendering the mouse.
    QGraphicsLineItem *action_line;                  ///< Line showing the last action.
    QGraphicsEllipseItem *action_point;              ///< Circle showing the last position for showing the last action.
};

#endif /* CHEESEMAZE_H_ */
