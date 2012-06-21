/*
 * MazeState.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef MAZESTATE_H_
#define MAZESTATE_H_

#include "../Utils/Identifier.h"

namespace WorldModel
{

template <class Identifyer = Utils::Identifier2D<int> >
class MazeState
{

public:

    typedef Identifyer Id;
    enum WALL { LEFT, RIGHT, UP, DOWN };

    MazeState(const Id& i,
            const bool& left = false,
            const bool& right = false,
            const bool& up = false,
            const bool& down = false
            ): id(i) {
        walls[LEFT] = left;
        walls[RIGHT] = right;
        walls[UP] = up;
        walls[DOWN] = down;
    };
    MazeState(const Id& i,
            bool const * w
    ): id(i) {
        walls[LEFT]  = w[LEFT];
        walls[RIGHT] = w[RIGHT];
        walls[UP]    = w[UP];
        walls[DOWN]  = w[DOWN];
    };
    virtual ~MazeState() {};

    // Comparing states //
    bool operator==(const MazeState<Id>& other) const {
        return (id == other.id &&
                walls[LEFT]  == other.walls[LEFT]  &&
                walls[RIGHT] == other.walls[RIGHT] &&
                walls[UP]    == other.walls[UP]    &&
                walls[DOWN]  == other.walls[DOWN] );
    }
    bool operator< (const MazeState<Id>& other) const { return id<other.id; }
    bool operator!=(const MazeState<Id>& other) const { return !(*this==other); }
    bool operator<=(const MazeState<Id>& other) const { return (*this<other || *this==other); }
    bool operator> (const MazeState<Id>& other) const { return !(*this<=other); }
    bool operator>=(const MazeState<Id>& other) const { return !(*this>other || *this==other); }
    Id get_id() const { return id; }

    // Getting and setting walls //
    bool const * get_walls() { return walls; }
    bool get_left_wall()  const { return walls[LEFT];  }
    bool get_right_wall() const { return walls[RIGHT]; }
    bool get_up_wall()    const { return walls[UP];    }
    bool get_down_wall()  const { return walls[DOWN];  }
    void set_left_wall(const bool& left)   { walls[LEFT]  = left;  }
    void set_right_wall(const bool& right) { walls[RIGHT] = right; }
    void set_up_wall(const bool& up)       { walls[UP]    = up;    }
    void set_down_wall(const bool& down)   { walls[DOWN]  = down;  }

protected:
    bool walls[4];
    Id id;

};

} /* namespace WorldModel */
#endif /* MAZESTATE_H_ */
