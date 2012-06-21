#include "testmaze.h"
#include "WorldModel/MazeTransition.h"

TestMaze::TestMaze(QWidget *parent)
    : QWidget(parent)
{
	ui.setupUi(this);
	WorldModel::MazeTransition<> mt;
}

TestMaze::~TestMaze()
{

}
