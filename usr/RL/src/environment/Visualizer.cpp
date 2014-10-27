#include "Visualizer.h"

Visualizer::~Visualizer() {
    delete scene;
}

void Visualizer::render_initialize(QGraphicsView * v) {

    // Set view
    view = v;

    // Get scene or initialize.
    scene = view->scene();
    if(scene==nullptr) {
        scene = new QGraphicsScene();
        view->setScene(scene);
    }

    // Rescale
    rescale_scene();
}

void Visualizer::rescale_scene() {
    scene->setSceneRect(scene->itemsBoundingRect());
    view->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
    view->scale(0.95,0.95);
}

void Visualizer::render_tear_down() {
    scene->clear();
}
