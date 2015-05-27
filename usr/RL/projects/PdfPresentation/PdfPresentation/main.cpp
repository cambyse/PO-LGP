#include "PdfPresentation.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    srand(time(nullptr));
    QApplication a(argc, argv);
    PdfPresentation w;
    w.show();

    return a.exec();
}
