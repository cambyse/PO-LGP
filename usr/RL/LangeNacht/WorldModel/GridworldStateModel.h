/*
 * StateInfo.h
 *
 *  Created on: May 1, 2012
 *      Author: robert
 */

#ifndef GRIDWORLDSTATEMODEL_H_
#define GRIDWORLDSTATEMODEL_H_

#define DEBUG_STRING "GridworldStateModel: "
#define DEBUG_LEVEL 1
#include "debug.h"

#include <QtGui/QGraphicsScene>

class GridworldStateModel {

public:

	GridworldStateModel(const double& xx = 0, const double& yy = 0, const double& s = 1):
		x(xx), y(yy), size(s), left_wall(false), right_wall(false), top_wall(false), bottom_wall(false) {}

	virtual ~GridworldStateModel() {}

	double get_x() const { return x; }
	double get_y() const { return y; }
	double get_size() const { return size; }
	bool get_wall(const char& w) const;
	void set_x(const double& xx) { x = xx; }
	void set_y(const double& yy) { y = yy; }
	void set_size(const double& s) { size = s; }
	void set_wall(const char& w);
	void unset_wall(const char& w);

	void display( QGraphicsScene* scene, QColor col = QColor(245,245,245) ) {
		if(scene==0) {
			DEBUG_OUT(0,"QGraphicsScene is NULL");
			return;
		}
		QPen wall_pen(QBrush(QColor(0,0,0)),0.1,Qt::SolidLine,Qt::SquareCap,Qt::MiterJoin);
		scene->addRect( x-size/2, y-size/2, size, size, QPen(QColor(0,0,0)), QBrush(col) );
		if(left_wall)   scene->addLine( x-size/2, y-size/2, x-size/2, y+size/2, wall_pen );
		if(right_wall)  scene->addLine( x+size/2, y-size/2, x+size/2, y+size/2, wall_pen );
		if(top_wall)    scene->addLine( x-size/2, y-size/2, x+size/2, y-size/2, wall_pen );
		if(bottom_wall) scene->addLine( x-size/2, y+size/2, x+size/2, y+size/2, wall_pen );
	}

private:

	double x,y, size;
	bool left_wall, right_wall, top_wall, bottom_wall;

};

#include "debug_exclude.h"

#endif /* STATEINFO_H_ */
