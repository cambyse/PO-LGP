#include "langenacht.h"

#include <QtGui>
#include <QApplication>

int main(int argc, char *argv[])
{

	QApplication a(argc, argv);
	LangeNacht w;
	w.show();
    return a.exec();
}
