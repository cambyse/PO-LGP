#ifndef EDITOR_H
#define EDITOR_H

#include <QMainWindow>

#include "Parser.h"

namespace Ui {
class Editor;
}

class Editor : public QMainWindow
{
    Q_OBJECT

public:
    explicit Editor(QWidget *parent = 0);
    ~Editor();

private:
    Ui::Editor *ui;
    Parser parser;
private slots:
    void parse_file(QString file_name);
    void parse_file();
    void open_file();
    void quit();
};

#endif // EDITOR_H
