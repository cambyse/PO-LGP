#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <QGraphicsView>

#include <vector>
#include <tuple>

class Visualizer {

public:

    typedef std::tuple<double,double,double> color_t;
    typedef std::vector<color_t>             color_vector_t;
    enum COLOR_IDX { COLOR_R, COLOR_G, COLOR_B };

    Visualizer() = default;
    virtual ~Visualizer() = default;

    /** \brief Initializes get graphics view (or initialize).
     *
     * After calling this method Visualizer::view points to the the view used
     * by the given QGraphcisView. */
    virtual void render_initialize(QGraphicsView * v);

    /** \brief Updates the graphical representation. */
    virtual void render_update() = 0;

    /** \brief Clears the scene used by Visualizer::view. */
    virtual void render_tear_down();

protected:

    /** \brief QGraphicsView to render the environment. */
    QGraphicsView * view = nullptr;

    /*! \brief Rescale the scene to fit into view. */
    virtual void rescale_scene(QGraphicsView * view);

};

#endif /* VISUALIZER_H_ */
