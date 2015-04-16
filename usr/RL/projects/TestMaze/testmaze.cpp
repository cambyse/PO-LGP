#include "testmaze.h"
#include "WorldModel/MazeTransition.h"
#include "Visualization/VisualizeMaze.h"

#include <QTimer>

TestMaze::TestMaze(QWidget *parent)
    : QWidget(parent)
{
	ui.setupUi(this);

	// init() must be called after all
	// widgets have been initialized.
	QTimer::singleShot(1, this, SLOT(init()));
}

TestMaze::~TestMaze()
{

}

void TestMaze::init() {

    QGraphicsScene * scene = new QGraphicsScene();
    scene->addEllipse(-1,-1,2,2); // just to test (todo)
    ui._wDisplay->setScene(scene);

    WorldModel::MazeModel<> mm(10,10);
    Visualization::VisualizeMaze<> vm(&mm,ui._wDisplay);
}
