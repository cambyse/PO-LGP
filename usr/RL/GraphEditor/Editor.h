#ifndef EDITOR_H
#define EDITOR_H

#include <QMainWindow>
#include <QTreeWidgetItem>

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
    Parser::KeyValueGraph key_value_graph;
    QString read_file(QString file_name);
    void parse_content(QString input);
    void block_editor_signals();
    void unblock_editor_signals();
    QTreeWidgetItem * item_to_tree_item(const Parser::KeyValueGraph::Item & item);
    QString random_alpha_num(int n) const;
private slots:
    void kvg_to_tree();
    void kvg_to_visual();
    void raw_display(bool raw);
    void update_content();
    void open_file();
    void save_file();
    void quit();
    void tree_item_clicked(QTreeWidgetItem * item, int column);
};

#endif // EDITOR_H
