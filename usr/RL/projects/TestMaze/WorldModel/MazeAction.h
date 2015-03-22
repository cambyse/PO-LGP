/*
 * MazeAction.h
 *
 *  Created on: Jun 21, 2012
 *      Author: robert
 */

#ifndef MAZEACTION_H_
#define MAZEACTION_H_

namespace WorldModel
{

class MazeAction
{
public:

    enum ACTION { STAY, LEFT, RIGHT, UP, DOWN };

    MazeAction(const ACTION& a = STAY): action(a) {}
    MazeAction(const char& a) {
        switch(a)
        {
        case 'L':
        case 'l':
            action = LEFT;
            break;
        case 'R':
        case 'r':
            action = RIGHT;
            break;
        case 'U':
        case 'u':
            action = UP;
            break;
        case 'D':
        case 'd':
            action = DOWN;
            break;
        case 'S':
        case 's':
        default:
            action = STAY;
            break;
        }
    }
    virtual ~MazeAction() {}

    bool operator==(const MazeAction& other) const { return action==other.action; }
    bool operator< (const MazeAction& other) const { return action<other.action; }
    bool operator!=(const MazeAction& other) const { return !(*this==other); }
    bool operator<=(const MazeAction& other) const { return (*this<other || *this==other); }
    bool operator> (const MazeAction& other) const { return !(*this<=other); }
    bool operator>=(const MazeAction& other) const { return !(*this>other || *this==other); }

    ACTION get_action() { return action; }

private:
    ACTION action;
};

} /* namespace WorldModel */
#endif /* MAZEACTION_H_ */
