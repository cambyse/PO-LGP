/*
 * StateInfo.cpp
 *
 *  Created on: May 1, 2012
 *      Author: robert
 */

#include "GridworldStateModel.h"

#define DEBUG_STRING "GridworldStateModel: "
#define DEBUG_LEVEL 1
#include "debug.h"

bool GridworldStateModel::get_wall(const char& w) const {
	switch(w) {
	case 'l':
	case 'L':
		return left_wall;
	case 'r':
	case 'R':
		return right_wall;
	case 't':
	case 'T':
		return top_wall;
	case 'b':
	case 'B':
		return bottom_wall;
	default:
		DEBUG_OUT(0,"ERROR: Unknown wall type");
		return false;
	}
}

void GridworldStateModel::set_wall(const char& w) {
	switch(w) {
	case 'l':
	case 'L':
		left_wall = true;
		break;
	case 'r':
	case 'R':
		right_wall = true;
		break;
	case 't':
	case 'T':
		top_wall = true;
		break;
	case 'b':
	case 'B':
		bottom_wall = true;
		break;
	default:
		DEBUG_OUT(0,"ERROR: Unknown wall type");
		break;
	}
}

void GridworldStateModel::unset_wall(const char& w) {
	switch(w) {
	case 'l':
	case 'L':
		left_wall = false;
		break;
	case 'r':
	case 'R':
		right_wall = false;
		break;
	case 't':
	case 'T':
		top_wall = false;
		break;
	case 'b':
	case 'B':
		bottom_wall = false;
		break;
	default:
		DEBUG_OUT(0,"ERROR: Unknown wall type");
		break;
	}
}
