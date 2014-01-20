#ifndef MAZEOBSERVATION_H_
#define MAZEOBSERVATION_H_

#include "../AbstractObservation.h"

class MazeObservation: public AbstractObservation {
public:
    //MazeObservation();
    MazeObservation(const MazeObservation& other);
    MazeObservation(
        int x_dim,
        int y_dim,
        int x_pos,
        int y_pos
        );
    MazeObservation(
        int x_dim,
        int y_dim,
        int index
        );
    virtual ~MazeObservation() override {}
    /** \brief Defined explicitly because the maze dimensions need to be given. */
    virtual Iterator begin() const override;
    virtual ptr_t next() const override;
    virtual bool operator!=(const AbstractObservation &other) const override;
    virtual bool operator<(const AbstractObservation &other) const override;
    virtual const std::string print() const override;
    inline virtual int get_x_pos() const { return x_position; }
    inline virtual int get_y_pos() const { return y_position; }
    inline virtual int get_x_dim() const { return x_dimensions; }
    inline virtual int get_y_dim() const { return y_dimensions; }
    inline virtual int get_index() const { return y_position*x_dimensions + x_position; }
    MazeObservation new_observation(int x_pos, int y_pos) const;
    MazeObservation new_observation(int index) const;
protected:
    int x_dimensions, y_dimensions, x_position, y_position;
    virtual void set_type(OBSERVATION_TYPE t) override;
};

#endif /* MAZEOBSERVATION_H_ */
