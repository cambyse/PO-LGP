#include "VisualEnvironment.h"

void VisualEnvironment::render_initialize(QGraphicsView * v) {

    // Set view
    view = v;

    // Get scene or initialize.
    QGraphicsScene * scene = view->scene();
    if(scene==nullptr) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    // Rescale
    rescale_scene(view);
}

void VisualEnvironment::rescale_scene(QGraphicsView * view) {
    QGraphicsScene * scene = view->scene();
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}
