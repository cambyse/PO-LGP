/*
 * MyDisplay.h
 *
 *  Created on: May 6, 2012
 *      Author: robert
 */

#ifndef MYDISPLAY_H_
#define MYDISPLAY_H_

#define DEBUG_STRING "MyDisplay: "
#define DEBUG_LEVEL 0
#include "debug.h"

#include <QtGui/QGraphicsView>
#include <QtGui/QMouseEvent>

class MyDisplay: public QGraphicsView {

public:

	MyDisplay(QWidget *parent = 0): QGraphicsView(parent) {}
	MyDisplay(QGraphicsScene *scene, QWidget *parent = 0): QGraphicsView(scene, parent) {}
	virtual ~MyDisplay() {}

    void keyPressEvent(QKeyEvent * event) { event->ignore(); }
    void mousePressEvent(QMouseEvent * event) { remember_and_ignore(event); }
    void mouseReleaseEvent(QMouseEvent * event) { remember_and_ignore(event); }
    void mouseDoubleClickEvent(QMouseEvent * event) { remember_and_ignore(event); }
    void mouseMoveEvent(QMouseEvent * event) { remember_and_ignore(event); }

    double get_my_x() { return my_x_coordinate; }
    double get_my_y() { return my_y_coordinate; }

private:

    void remember_and_ignore(QMouseEvent * event) {
    	my_x_coordinate = this->mapToScene(event->pos()).x();
    	my_y_coordinate = this->mapToScene(event->pos()).y();
    	event->ignore();
    }
    double my_x_coordinate, my_y_coordinate;

};

#include "debug_exclude.h"

#endif /* MYDISPLAY_H_ */
