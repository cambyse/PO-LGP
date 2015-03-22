/*
 * VisualizeMaze.h
 *
 *  Created on: Jun 28, 2012
 *      Author: robert
 */

#ifndef VISUALIZEMAZE_H_
#define VISUALIZEMAZE_H_

#include "../WorldModel/MazeModel.h"

#include <QtGui/QGraphicsView>

namespace Visualization
{

template<class Maze = WorldModel::MazeModel<> >
class VisualizeMaze
{
public:
    typedef typename Maze::StateType State;
    typedef typename Maze::ActionType Action;
    typedef typename Maze::TransitionType Transition;

    VisualizeMaze(Maze const * m, QGraphicsView * gv): maze(m), graphicsView(gv) {
        QGraphicsScene * scene = graphicsView->scene();
        if(scene==NULL) {
            scene = new QGraphicsScene();
            graphicsView->setScene(scene);
        }
        rescale();
    }
    virtual ~VisualizeMaze() {}

private:

    void rescale() {
        QGraphicsScene * scene = graphicsView->scene();
        scene->setSceneRect(scene->itemsBoundingRect());
        graphicsView->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
        graphicsView->scale(0.95,0.95);
    }

    Maze const * maze;
    QGraphicsView * graphicsView;
};

} /* namespace Visualization */
#endif /* VISUALIZEMAZE_H_ */
