#ifndef VISUALWORLD_H_
#define VISUALWORLD_H_

#include <QGraphicsView>

#include <vector>
#include <tuple>

class VisualWorld {

public:

    typedef std::tuple<double,double,double> color_t;
    typedef std::vector<color_t>             color_vector_t;
    enum COLOR_IDX { COLOR_R, COLOR_G, COLOR_B };

    VisualWorld() = default;
    virtual ~VisualWorld() = default;

    /** \brief Initializes the graphical representation. */
    virtual void render_initialize(QGraphicsView * v);

    /** \brief Updates the graphical representation. */
    virtual void render_update();

protected:

    /** \brief QGraphicsView to render the World. */
    QGraphicsView * view = nullptr;

    /*! \brief Rescale the scene to fit into view. */
    virtual void rescale_scene(QGraphicsView * view);

};

#endif /* VISUALWORLD_H_ */
