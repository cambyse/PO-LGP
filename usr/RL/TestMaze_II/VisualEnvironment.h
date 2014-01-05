#ifndef VISUALENVIRONMENT_H_
#define VISUALENVIRONMENT_H_

#include <QGraphicsView>

#include <vector>
#include <tuple>

class VisualEnvironment {

public:

    typedef std::tuple<double,double,double> color_t;
    typedef std::vector<color_t>             color_vector_t;
    enum COLOR_IDX { COLOR_R, COLOR_G, COLOR_B };

    VisualEnvironment() = default;
    virtual ~VisualEnvironment() = default;

    /** \brief Initializes get graphics view (or initialize).
     *
     * After calling this method VisualEnvironment::view points to the the view used
     * by the given QGraphcisView. */
    virtual void render_initialize(QGraphicsView * v);

    /** \brief Updates the graphical representation. */
    virtual void render_update() = 0;

    /** \brief Clears the scene used by VisualEnvironment::view. */
    virtual void render_tear_down();

protected:

    /** \brief QGraphicsView to render the environment. */
    QGraphicsView * view = nullptr;

    /*! \brief Rescale the scene to fit into view. */
    virtual void rescale_scene(QGraphicsView * view);

};

#endif /* VISUALENVIRONMENT_H_ */
