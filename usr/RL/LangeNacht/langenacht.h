#ifndef LANGENACHT_H
#define LANGENACHT_H

#include <QtGui/QWidget>
#include "ui_langenacht.h"
#include "WorldModel/GridworldModel.h"

#define DEBUG_STRING "LangeNacht: "
#define DEBUG_LEVEL 0
#include "WorldModel/debug.h"

class LangeNacht : public QWidget
{
    Q_OBJECT

public:
    LangeNacht(QWidget *parent = 0);
    ~LangeNacht();

protected:
    void keyPressEvent(QKeyEvent * event);
//    void focusInEvent(QFocusEvent * event) { DEBUG_OUT(0,"Focus In"); }
    void mousePressEvent(QMouseEvent * event);
//    void mouseReleaseEvent(QMouseEvent * event) { DEBUG_OUT(0,"Mouse Release"); }
//    void mouseDoubleClickEvent(QMouseEvent * event) { DEBUG_OUT(0,"Mouse Double Click"); }
    void mouseMoveEvent(QMouseEvent * event);

public slots:
	void redraw();
	void transition_random();
	void transition_left();
	void transition_right();
	void transition_up();
	void transition_down();
	void transition_stay();
	void show_rewards_changed(bool){redraw();}
	void show_actions_changed(bool){redraw();}
	void delete_all_rewards();
	void delete_all_walls();
	void random_changed(int);
	void discount_changed(int);
	void speed_changed(int);
	void loop();
	void reset_grid_world();

private:

	void set_click_type();
	void handle_mouse_event();

	enum CLICK_TYPE { STATE, WALL, NONE };
	enum BUTTON_MODE { LEFT, RIGHT, MIDDLE };
	CLICK_TYPE click_type;
	BUTTON_MODE button_mode;
	int old_state_x_idx, old_state_y_idx;
    Ui::LangeNachtClass ui;
    GridworldModel * world_model;
    QTimer *timer;

};

#include "WorldModel/debug_exclude.h"

#endif // LANGENACHT_H
