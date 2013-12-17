#ifndef MAZEOBSERVATION_H_
#define MAZEOBSERVATION_H_

#include "../AbstractObservation.h"

class MazeObservation: public AbstractObservation {
public:
    MazeObservation(
        uint x_dim = 0,
        uint y_dim = 0,
        uint x_pos = 0,
        uint y_pos = 0
        );
    /** \brief Defined explicitly because the maze dimensions need to be given. */
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
    inline virtual uint get_x_pos() const { return x_position; }
    inline virtual uint get_y_pos() const { return y_position; }
    inline virtual uint get_x_dim() const { return x_dimensions; }
    inline virtual uint get_y_dim() const { return y_dimensions; }
protected:
    virtual void set_type(OBSERVATION_TYPE t) override;
    uint x_dimensions, y_dimensions, x_position, y_position;
};

#endif /* MAZEOBSERVATION_H_ */
