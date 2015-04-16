/*
 * StateInfo.h
 *
 *  Created on: May 1, 2012
 *      Author: robert
 */

#ifndef GRIDWORLDSTATEMODEL_H_
#define GRIDWORLDSTATEMODEL_H_

#include <vector>
#include <float.h>
#include <QtGui/QGraphicsScene>

#include "GridworldActionModel.h"

#define DEBUG_STRING "GridworldStateModel: "
#define DEBUG_LEVEL 1
#include "debug.h"

class GridworldStateModel {

public:

	typedef GridworldActionModel Action;

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

	void display( QGraphicsScene* scene, QColor col, std::vector<double> action_values, std::vector<const Action*> available_actions, const bool& show_actions) {
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
		if(!show_actions) return;
		else {
			double min =  DBL_MAX;
			double max = -DBL_MAX;
			for(uint idx=0; idx<action_values.size(); ++idx) {
				min = action_values[idx]<min ? action_values[idx] : min;
				max = action_values[idx]>max ? action_values[idx] : max;
			}
			double diff = max-min;
			if(diff==1e-5) diff=1;
			QPen action_pen(QBrush(QColor(0,0,0,100)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
			QPen max_action_pen(QBrush(QColor(0,0,0)),0.06,Qt::SolidLine,Qt::RoundCap,Qt::MiterJoin);
			double factor = 0.8*size;
			for(uint idx=0; idx<action_values.size(); ++idx) {
				if((*available_actions[idx])==Action('s')) {
					double ellSize = factor*(action_values[idx]-min)/diff;
					scene->addEllipse(x-ellSize/2, y-ellSize/2, ellSize, ellSize, action_values[idx]==max?max_action_pen:action_pen );
				}
				else if((*available_actions[idx])==Action('l')) scene->addLine(x, y, x-(factor/2)*(action_values[idx]-min)/diff, y, action_values[idx]==max?max_action_pen:action_pen );
				else if((*available_actions[idx])==Action('r')) scene->addLine(x, y, x+(factor/2)*(action_values[idx]-min)/diff, y, action_values[idx]==max?max_action_pen:action_pen );
				else if((*available_actions[idx])==Action('u')) scene->addLine(x, y, x, y-(factor/2)*(action_values[idx]-min)/diff, action_values[idx]==max?max_action_pen:action_pen );
				else if((*available_actions[idx])==Action('d')) scene->addLine(x, y, x, y+(factor/2)*(action_values[idx]-min)/diff, action_values[idx]==max?max_action_pen:action_pen );
				else DEBUG_OUT(0,"No matching action found for state (" << x << "," << y << ")" );
			}

		}
	}

private:

	double x,y, size;
	bool left_wall, right_wall, top_wall, bottom_wall;

};

#include "debug_exclude.h"

#endif /* STATEINFO_H_ */
