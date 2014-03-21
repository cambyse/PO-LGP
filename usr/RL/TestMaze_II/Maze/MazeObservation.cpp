#include "MazeObservation.h"

#include "../util/Macro.h"

#include <sstream>
#include <tuple> // for std::tie

#define DEBUG_LEVEL 2
#include "../util/debug.h"

using std::string;

//MazeObservation::MazeObservation(): MazeObservation(0,0,0,0) {};

MazeObservation::MazeObservation(const MazeObservation& other):
    MazeObservation(other.x_dimensions,
                    other.y_dimensions,
                    other.x_position,
                    other.y_position)
{}

MazeObservation::MazeObservation(
    int x_dim,
    int y_dim,
    int x_pos,
    int y_pos
    ):
    x_dimensions(x_dim),
    y_dimensions(y_dim),
    x_position(x_pos),
    y_position(y_pos)
{
    if(DEBUG_LEVEL>0) {
        if(x_dim<=0 ||
           y_dim<=0) {
            DEBUG_ERROR("(" << x_dim << "," << y_dim << ")-maze has invalid size");
        }
        if(x_pos>=x_dim ||
           x_pos<0 ||
           y_pos>=y_dim ||
           y_pos<0) {
            DEBUG_ERROR("Position out of bounds: (" << x_pos << "," << y_pos << ") in (" << x_dim << "," << y_dim << ")-maze");
        }
    }
    set_type(OBSERVATION_TYPE::MAZE_OBSERVATION);
}

MazeObservation::MazeObservation(int x_dim, int y_dim, int index):
    MazeObservation(x_dim, y_dim, index%x_dim, index/x_dim)
{}

MazeObservation::Iterator MazeObservation::begin() const {
    return Iterator(ptr_t(new MazeObservation(x_dimensions,y_dimensions,0,0)));
}

MazeObservation::ptr_t MazeObservation::next() const {
    int x_pos = x_position + 1;
    int y_pos = y_position + 1;
    if(x_pos>=x_dimensions) {
        if(y_pos>=y_dimensions) {
            return ptr_t(new AbstractObservation());
        } else {
            x_pos = 0;
        }
    } else {
        y_pos = y_position;
    }
    return ptr_t(new MazeObservation(x_dimensions,y_dimensions,x_pos,y_pos));
}

bool MazeObservation::operator!=(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(!=,get_type,const MazeObservation *);
    return (
        this->x_position   != ptr->x_position   ||
        this->y_position   != ptr->y_position   ||
        this->x_dimensions != ptr->x_dimensions ||
        this->y_dimensions != ptr->y_dimensions
        );
}

bool MazeObservation::operator<(const AbstractObservation &other) const {
    COMPARE_ABSTRACT_TYPE_AND_CAST(<,get_type,const MazeObservation *);
    auto this_tie = std::tie(this->x_position,this->y_position,this->x_dimensions,this->y_dimensions);
    auto other_tie = std::tie(ptr->x_position,ptr->y_position,ptr->x_dimensions,ptr->y_dimensions);
    return this_tie<other_tie;
}

const string MazeObservation::print() const {
    std::stringstream ret;
    if(DEBUG_LEVEL>1) {
        ret << "MazeObservation(" << "(" << x_dimensions << "," << y_dimensions << "), " <<  x_position << "," << y_position << ")";
    } else {
        ret << "MazeObservation(" << x_position << "," << y_position << ")";
    }
    return ret.str();
}

MazeObservation MazeObservation::new_observation(int x_pos, int y_pos) const {
    return MazeObservation(x_dimensions, y_dimensions, x_pos, y_pos);
}

MazeObservation MazeObservation::new_observation(int index) const {
    return MazeObservation(x_dimensions, y_dimensions, index);
}

void MazeObservation::set_type(OBSERVATION_TYPE t) {
    AbstractObservation::set_type(t);
}
