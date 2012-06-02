
#include "langenacht.h"

#include <QtGui/QKeyEvent>
#include <QTimer>

#include <cmath>
#include <time.h>
#include <unistd.h>

#define DEBUG_STRING "LangeNacht: "
#define DEBUG_LEVEL 0
#include "WorldModel/debug.h"

LangeNacht::LangeNacht(QWidget *parent)
    : QWidget(parent), click_type(NONE), rew_x(-1), rew_y(-1), world_model(NULL)
{
	ui.setupUi(this);

	this->setFocusPolicy(Qt::StrongFocus);
//	ui._wDisplay->setFocusPolicy(Qt::NoFocus);

	srand(time(NULL));

	reset_grid_world();

	redraw();

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(loop()));
	timer->start(1000/30);

	DEBUG_OUT(1,"Initialization done.");
}

LangeNacht::~LangeNacht() {
	delete world_model;
	delete timer;
}

void LangeNacht::redraw() {
	QGraphicsView* display = ui._wDisplay;
	QGraphicsScene* scene = display->scene();
	if(scene==NULL) {
		scene = new QGraphicsScene();
		display->setScene(scene);
	}
	scene->clear();

	char c = ui._wShowRewards->isChecked() ? 'r' : 'v';
	bool show_actions = ui._wShowActions->isChecked();
	world_model->display_all_states(scene, c, show_actions);
	world_model->display_agent(scene);

	scene->setSceneRect(scene->itemsBoundingRect());
	display->fitInView(scene->itemsBoundingRect(),Qt::KeepAspectRatio);
	display->scale(0.95,0.95);

//	double scale_fac = std::min(display->width()/bound.width(),display->height()/bound.height());
//	scale_fac *= 0.9;
//	display->resetMatrix();
//	display->scale(scale_fac,scale_fac);
};

void LangeNacht::keyPressEvent(QKeyEvent * event) {
	switch(event->key())
	{
	case Qt::Key_Left: // left
		transition_left();
		break;
	case Qt::Key_Right: // right
		transition_right();
		break;
	case Qt::Key_Up: // up
		transition_up();
		break;
	case Qt::Key_Down: // down
		transition_down();
		break;
	case Qt::Key_Space: // space
			transition_stay();
			break;
	case Qt::Key_R: // r
		transition_random();
			break;
	case Qt::Key_O: // o
	    world_model->perform_optimal_transition();
	    redraw();
	    break;
	case Qt::Key_Return:
	case Qt::Key_Enter:
		world_model->iterate_value_function();
		redraw();
		break;
	default:
		break;
	}
}

void LangeNacht::mousePressEvent(QMouseEvent * event) {
	if(event->button()==Qt::LeftButton) {
	    button_mode = LEFT;
	    set_click_type();
	}
	else if(event->button()==Qt::RightButton) {
	    button_mode = RIGHT;
	    set_click_type();
	}
	else if(event->button()==Qt::MiddleButton) {
	    button_mode = MIDDLE;
	    set_click_type();
	}
	else return;

	handle_mouse_event();
}

void LangeNacht::mouseMoveEvent(QMouseEvent * event) {
	handle_mouse_event();
}

void LangeNacht::transition_random() {
	DEBUG_OUT(1,"Random transition");
	GridworldModel::Action action;
	int a = rand()%5;
	switch(a) {
	case 0:
        world_model->perform_transition('u');
		break;
	case 1:
        world_model->perform_transition('d');
		break;
	case 2:
        world_model->perform_transition('l');
		break;
	case 3:
        world_model->perform_transition('r');
		break;
	case 4:
	default:
        world_model->perform_transition('s');
		break;
	}
	redraw();
}

void LangeNacht::transition_left() {
	DEBUG_OUT(1,"Transition left");
	world_model->perform_transition('l');
	redraw();
}

void LangeNacht::transition_right() {
	DEBUG_OUT(1,"Transition right");
	world_model->perform_transition('r');
	redraw();
}

void LangeNacht::transition_up() {
	DEBUG_OUT(1,"Transition up");
	world_model->perform_transition('u');
	redraw();
}

void LangeNacht::transition_down() {
	DEBUG_OUT(1,"Transition down");
	world_model->perform_transition('d');
	redraw();
}

void LangeNacht::transition_stay() {
	DEBUG_OUT(1,"Transition stay");
	world_model->perform_transition('s');
	redraw();
}

void LangeNacht::delete_all_rewards() {
    world_model->delete_all_rewards();
    redraw();
}

void LangeNacht::delete_all_walls() {
	world_model->remove_all_walls();
}

void LangeNacht::random_changed(int) {
	int val = ui._wRandom->value();
	QString valStr = QString().setNum(val).append('%');
	ui._wRandomValue->setText(valStr);
	double dVal = val/100.;
	world_model->set_random(dVal);
}
void LangeNacht::discount_changed(int) {
	int val = ui._wDiscount->value();
	QString valStr = QString().setNum(val).append('%');
	ui._wDiscountValue->setText(valStr);
	double dVal = val/100.;
	world_model->set_discount(1-dVal);
}

void LangeNacht::speed_changed(int) {
	int val = ui._wSpeed->value();
	QString valStr = QString().setNum(val);
	ui._wSpeedValue->setText(valStr);
	timer->setInterval(1000/val);
}

void LangeNacht::loop() {
	if(!ui._wLoopValueIteration->isChecked() && !ui._wLoopActions->isChecked()) return;
	if(ui._wLoopValueIteration->isChecked()) world_model->iterate_value_function();
	if(ui._wLoopActions->isChecked()) world_model->perform_optimal_transition();
	if(ui._wAutoRewards->isChecked()) {
		int x, y;
		world_model->get_current_state(x,y);
		if( rew_x==-1 || rew_y==-1 || (x==rew_x && y==rew_y) ) {
			world_model->delete_all_rewards();
			x = rand()%world_model->get_x_size();
			y = rand()%world_model->get_y_size();
			world_model->set_reward(x,y,1);
			rew_x=x;
			rew_y=y;
		}
	}
	redraw();
}

void LangeNacht::reset_grid_world() {
	if(world_model) delete world_model;
	world_model = new GridworldModel(ui._wXSize->value(), ui._wYSize->value(), (double)ui._wRandom->value()/100, 1-(double)ui._wDiscount->value()/100);
	world_model->set_current_state(0,0);
}

void LangeNacht::set_click_type() {

    old_state_x_idx = old_state_y_idx = -1;
    DEBUG_OUT(2,"Set old state index to (" << old_state_x_idx << "," << old_state_y_idx << ")");

    double x = ui._wDisplay->get_my_x();
    double y = ui._wDisplay->get_my_y();
    double in_state_tolerance = 0.2;
    double xMod = x-round(x);
    double yMod = y-round(y);

    // click inside state
    if( fabs(xMod)<=in_state_tolerance && fabs(yMod)<=in_state_tolerance ) {
        click_type = STATE;
        return;
    }

    // ambiguous click at intersection of four states
    if( fabs(xMod)>in_state_tolerance && fabs(yMod)>in_state_tolerance ) {
        click_type = NONE;
        return;
    }

    click_type = WALL;
}

void LangeNacht::handle_mouse_event() {
	double x = ui._wDisplay->get_my_x();
	double y = ui._wDisplay->get_my_y();
	double in_state_tolerance = 0.2;
	double xMod = x-round(x);
	double yMod = y-round(y);
	int xIdx = (int)round(x);
	int yIdx = (int)round(y);

	// click inside state
	if( fabs(xMod)<=in_state_tolerance && fabs(yMod)<=in_state_tolerance ) {
	    DEBUG_OUT(2,"Old index (" << old_state_x_idx << "," << old_state_y_idx << "), New index (" << xIdx << "," << yIdx << ")");
	    if(click_type!=STATE || ( xIdx==old_state_x_idx && yIdx==old_state_y_idx ) ) return;
	    else {
	        old_state_x_idx = xIdx;
	        old_state_y_idx = yIdx;
	        DEBUG_OUT(2,"Set old state index to (" << old_state_x_idx << "," << old_state_y_idx << ")");
	        switch(button_mode) {
	        case LEFT:
	            world_model->set_reward(xIdx,yIdx, world_model->get_reward(xIdx,yIdx) + 1 );
	            DEBUG_OUT(1, "Add reward in state (" << xIdx << "," << yIdx << "). Now is: " << world_model->get_reward(xIdx,yIdx) );
	            break;
	        case RIGHT:
	            world_model->set_reward(xIdx,yIdx, world_model->get_reward(xIdx,yIdx) - 1 );
	            DEBUG_OUT(1, "Subtract reward in state (" << xIdx << "," << yIdx << "). Now is: " << world_model->get_reward(xIdx,yIdx) );
	            break;
	        case MIDDLE:
	            world_model->set_reward(xIdx,yIdx, 0 );
	            DEBUG_OUT(1, "Delete reward in state (" << xIdx << "," << yIdx << "). Now is: " << world_model->get_reward(xIdx,yIdx) );
	            break;
	        default:
	            DEBUG_OUT(0,"Unhandled button mode");
	            break;
	        }
	        redraw();
	        return;
	    }
	}

	// ambiguous click at intersection of four states
	else if( fabs(xMod)>in_state_tolerance && fabs(yMod)>in_state_tolerance ) { return; }

	// unambiguous click between two states --> add/remove wall
	else {
	    if(click_type!=WALL) return;
	    else {
	        if( xMod >  yMod && xMod >= -yMod) { // right
	            if(button_mode==LEFT) world_model->add_wall(xIdx, yIdx, xIdx+1, yIdx  );
	            else if(button_mode==RIGHT) world_model->remove_wall(xIdx, yIdx, xIdx+1, yIdx  );
	            else return;
	        }
	        else if( xMod < -yMod && xMod >=  yMod) { // top
	            if(button_mode==LEFT) world_model->add_wall(xIdx, yIdx, xIdx  , yIdx-1);
	            else if(button_mode==RIGHT) world_model->remove_wall(xIdx, yIdx, xIdx  , yIdx-1);
	            else return;
	        }
	        else if( xMod <  yMod && xMod <= -yMod) { // left
	            if(button_mode==LEFT) world_model->add_wall(xIdx, yIdx, xIdx-1, yIdx  );
	            else if(button_mode==RIGHT) world_model->remove_wall(xIdx, yIdx, xIdx-1, yIdx  );
	            else return;
	        }
	        else if( xMod > -yMod && xMod <=  yMod) { // bottom
	            if(button_mode==LEFT) world_model->add_wall(xIdx, yIdx, xIdx  , yIdx+1);
	            else if(button_mode==RIGHT) world_model->remove_wall(xIdx, yIdx, xIdx  , yIdx+1);
	            else return;
	        }
	        else DEBUG_OUT(0,"Error: Unhandled mouse event at (" << x << "," << y << ")" );
	    }
	    redraw();
	    return;
	}
}
