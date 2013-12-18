#include "MazeObservation.h"

#include <sstream>

#define DEBUG_LEVEL 0
#include "../debug.h"

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
    if(DEBUG_LEVEL>0) {
        ret << "MazeObservation(" << x_position << "," << y_position << ",(" << x_dimensions << "," << y_dimensions << "))";
    } else {
        ret << "MazeObservation(" << x_position << "," << y_position << ")";
    }
    return ret.str();
}

MazeObservation MazeObservation::new_observation(int x_pos, int y_pos) const {
    return MazeObservation(x_dimensions, y_dimensions, x_pos, y_pos);
}

MazeObservation MazeObservation::new_observation(int index) const {
    return MazeObservation(x_dimensions,
                           y_dimensions,
                           index%x_dimensions,
                           index/x_dimensions
        );
}

void MazeObservation::set_type(OBSERVATION_TYPE t) {
    AbstractObservation::set_type(t);
}
