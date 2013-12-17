#include "MazeObservation.h"

#include <sstream>

#include "../debug.h"

using std::string;

MazeObservation::MazeObservation(
    uint x_dim,
    uint y_dim,
    uint x_pos,
    uint y_pos
    ):
    x_dimensions(x_dim),
    y_dimensions(y_dim),
    x_position(x_pos),
    y_position(y_pos)
{
    set_type(OBSERVATION_TYPE::MAZE_OBSERVATION);
}

MazeObservation::Iterator MazeObservation::begin() const {
    return Iterator(ptr_t(new MazeObservation(x_dimensions,y_dimensions,0,0)));
}

MazeObservation::ptr_t MazeObservation::next() const {
    uint x_pos = x_position + 1;
    uint y_pos = y_position + 1;
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
    if(this->get_type()!=other.get_type()) {
        return true;
    } else {
        auto maze_observation = dynamic_cast<const MazeObservation *>(&other);
        if(maze_observation==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return (
                this->x_position   != maze_observation->x_position   ||
                this->y_position   != maze_observation->y_position   ||
                this->x_dimensions != maze_observation->x_dimensions ||
                this->y_dimensions != maze_observation->y_dimensions
                );
        }
    }
}

bool MazeObservation::operator<(const AbstractObservation &other) const {
    if(this->get_type()<other.get_type()) {
        return true;
    } else {
        auto maze_observation = dynamic_cast<const MazeObservation *>(&other);
        if(maze_observation==nullptr) {
            DEBUG_ERROR("Dynamic cast failed");
            return true;
        } else {
            return (this->x_position < maze_observation->x_position ||
                    (this->x_position == maze_observation->x_position &&
                     this->y_position < maze_observation->y_position) ||
                    (this->y_position == maze_observation->y_position &&
                     this->x_dimensions < maze_observation->x_dimensions) ||
                    (this->x_dimensions == maze_observation->x_dimensions &&
                     this->y_dimensions < maze_observation->y_dimensions)
                );
        }
    }
}

const string MazeObservation::print() const {
    std::stringstream ret;
    ret << "MazeObservation(" << x_position << "," << y_position << ")";
    return ret.str();
}

void MazeObservation::set_type(OBSERVATION_TYPE t) {
    AbstractObservation::set_type(t);
}
