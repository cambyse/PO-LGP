/*
 * MazeState.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef MAZESTATE_H_
#define MAZESTATE_H_

namespace WorldModel
{

class MazeState
{

public:

    enum WALL { LEFT, RIGHT, UP, DOWN };

    MazeState(const int& x, const int& y,
            const bool& left = false,
            const bool& right = false,
            const bool& up = false,
            const bool& down = false
            ): x_pos(x), y_pos(y) {
        walls[LEFT] = left;
        walls[RIGHT] = right;
        walls[UP] = up;
        walls[DOWN] = down;
    };
    MazeState(const int& x, const int& y,
            bool const * w
    ): x_pos(x), y_pos(y) {
        walls[LEFT]  = w[LEFT];
        walls[RIGHT] = w[RIGHT];
        walls[UP]    = w[UP];
        walls[DOWN]  = w[DOWN];
    };
    virtual ~MazeState() {};

    // Comparing and ordering states (including walls)
    bool operator==(const MazeState& other) const {
        return (x_pos == other.x_pos &&
                y_pos == other.y_pos &&
                walls[LEFT]  == other.walls[LEFT]  &&
                walls[RIGHT] == other.walls[RIGHT] &&
                walls[UP]    == other.walls[UP]    &&
                walls[DOWN]  == other.walls[DOWN] );
    }
    bool operator< (const MazeState& other) const {
        return ( x_pos<other.x_pos || (x_pos==other.x_pos && y_pos<other.y_pos) );
    }
    bool operator!=(const MazeState& other) const { return !(*this==other); }
    bool operator<=(const MazeState& other) const { return (*this<other || *this==other); }
    bool operator> (const MazeState& other) const { return !(*this<=other); }
    bool operator>=(const MazeState& other) const { return !(*this>other || *this==other); }

    // Comparing state positions.
    bool equal_x_y(const MazeState& other) const {
        return (x_pos == other.x_pos && y_pos == other.y_pos) ;
    }
    bool is_left_of( const MazeState& other) { return (x_pos==other.x_pos-1 && y_pos==other.y_pos  ); }
    bool is_right_of(const MazeState& other) { return (x_pos==other.x_pos+1 && y_pos==other.y_pos  ); }
    bool is_above(   const MazeState& other) { return (x_pos==other.x_pos   && y_pos==other.y_pos-1); }
    bool is_below(   const MazeState& other) { return (x_pos==other.x_pos   && y_pos==other.y_pos+1); }

    // Getting state positions.
    int get_x_pos() const { return x_pos; }
    int get_y_pos() const { return y_pos; }
    void get_x_y_pos(int& x, int& y) const { x=x_pos; y=y_pos; }

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
    int x_pos, y_pos;

};

} /* namespace WorldModel */
#endif /* MAZESTATE_H_ */
