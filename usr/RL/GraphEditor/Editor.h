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
    int editor_blocked_state;
    QString editor_content;
    QString read_file(QString file_name);
    void parse_content(QString input);
    void block_editor_signals();
    void unblock_editor_signals();
private slots:
    void raw_display(bool raw);
    void update_content();
    void open_file();
    void save_file();
    void quit();
};

#endif // EDITOR_H
