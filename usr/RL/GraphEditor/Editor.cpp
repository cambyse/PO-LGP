#include "Editor.h"
#include "ui_Editor.h"

#include "util.h"

#include <QtCore>
#include <QFileDialog>
#include <QGraphicsSvgItem>
#include <QWheelEvent>

#include <stdio.h>
#include <list>

using namespace std;

Editor::Editor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Editor),
    zoom_view(new ZoomView(this))
{
    ui->setupUi(this);
    // install event filter
    ui->visualization->installEventFilter(zoom_view.get());
    // allow dragging
    ui->visualization->setDragMode(QGraphicsView::ScrollHandDrag);
    // parse default file
    parse_content(read_file(ui->file_name->text()));
}

Editor::~Editor()
{
    delete ui;
}

QString Editor::read_file(QString file_name)
{
    // open and read input file
    QFile input_file(file_name);
    QString input;
    if(!input_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        block_editor_signals();
        ui->graph_editor->setPlainText(QString("Error: Could not open input file '%1'").arg(file_name));
        unblock_editor_signals();
    } else {
        QTextStream input_file_stream(&input_file);
        while(!input_file_stream.atEnd()) {
            input += input_file_stream.readLine()+"\n";
        }
        input.chop(1); // remove last new-line
    }
    return input;
}

void Editor::parse_content(QString input)
{
    // highlight
    QString new_content = "";
    key_value_graph = Parser::parse_graph(input, new_content);

    if(new_content!=editor_content) {

        editor_content = new_content;

        // display result (raw/rich text)
        int old_pos = ui->graph_editor->textCursor().position();
        block_editor_signals();
        ui->graph_editor->setHtml(new_content);
        unblock_editor_signals();
        QTextCursor new_cursor = ui->graph_editor->textCursor();
        new_cursor.setPosition(old_pos);
        ui->graph_editor->setTextCursor(new_cursor);

        // display as tree
        if(ui->tree_view_dock->isVisible()) {
            kvg_to_tree();
        }

        // visualize
        if(ui->visualization_dock->isVisible()) {
            kvg_to_visual();
        }
    }
}

void Editor::block_editor_signals()
{
    editor_blocked_state = ui->graph_editor->blockSignals(true);
}

void Editor::unblock_editor_signals()
{
    ui->graph_editor->blockSignals(editor_blocked_state);
}

void Editor::kvg_to_tree()
{
    auto tree = ui->tree_view;
    tree->clear();
    for(const auto& item : key_value_graph.items) {
        tree->addTopLevelItem(item_to_tree_item(item));
    }
}

void Editor::kvg_to_visual()
{
    // random file names
    QString dot_file_name = random_alpha_num(50);
    QString svg_file_name = random_alpha_num(50);

    // write to file
    {
        QFile outfile(dot_file_name);
        if(!outfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            ERROR("Could not open file");
        }
        QTextStream dot_file_stream(&outfile);
        dot_file_stream << key_value_graph.dot() << endl;
    }

    // generate svg
    system(QString("dot -Tsvg -o %1 %2").arg(svg_file_name).arg(dot_file_name).toLatin1());

    // display
    QGraphicsScene * scene = ui->visualization->scene();
    if(scene==nullptr) {
        scene = new QGraphicsScene();
        ui->visualization->setScene(scene);
    }
    // clear scene
    scene->clear();
    // construct and add svg item
    QGraphicsSvgItem * svg_item = new QGraphicsSvgItem(svg_file_name);
    svg_item->setFlags(QGraphicsItem::ItemClipsToShape);
    svg_item->setCacheMode(QGraphicsItem::NoCache);
    svg_item->setZValue(0);
    scene->addItem(svg_item);
    // rescale
//    ui->visualization->resetTransform();
    scene->setSceneRect(svg_item->boundingRect().adjusted(-10, -10, 10, 10));

    // remove files
    remove(dot_file_name.toLatin1());
    remove(svg_file_name.toLatin1());
}

QTreeWidgetItem *Editor::item_to_tree_item(const Parser::KeyValueGraph::Item &item)
{
    // construct key string
    QString key_string;
    {
        bool first = true;
        for(auto key : item.keys) {
            if(first) {
                key_string += key;
                first = false;
            } else {
                key_string += ", " + key;
            }
        }
    }
//    key_string += QString("(%1, %2)").arg(item.keys_start).arg(item.keys_end);
    // construct parent string
    QString parent_string;
    {
        bool first = true;
        for(auto parent : item.parents) {
            if(first) {
                parent_string += parent;
                first = false;
            } else {
                parent_string += ", " + parent;
            }
        }
    }
//    parent_string += QString("(%1, %2)").arg(item.parents_start).arg(item.parents_end);
    // construct value string
    QString value_string = item.value;
//    value_string += QString("(%1, %2)").arg(item.value_start).arg(item.value_end);
    // construct value type string
    QString value_type_string;
    switch (item.value_type) {
    case Parser::KeyValueGraph::Item::BOOL:
        value_type_string = "BOOL";
        break;
    case Parser::KeyValueGraph::Item::STRING:
        value_type_string = "STRING";
        break;
    case Parser::KeyValueGraph::Item::FILE:
        value_type_string = "FILE";
        break;
    case Parser::KeyValueGraph::Item::DOUBLE:
        value_type_string = "DOUBLE";
        break;
    case Parser::KeyValueGraph::Item::ARRAY:
        value_type_string = "ARRAY";
        break;
    case Parser::KeyValueGraph::Item::LIST:
        value_type_string = "LIST";
        break;
    case Parser::KeyValueGraph::Item::SPECIAL:
        value_type_string = "SPECIAL";
        break;
    case Parser::KeyValueGraph::Item::KVG:
        value_type_string = "KVG";
        break;
    case Parser::KeyValueGraph::Item::NONE:
        value_type_string = "ERROR: NONE";
        break;
    }
    // construct item
    auto new_item = new QTreeWidgetItem(QStringList({key_string, parent_string, value_string, value_type_string}));
    // add sub-items if value is KVG
    if(item.value_type==Parser::KeyValueGraph::Item::KVG) {
        for(auto& sub_item : item.sub_graph->items) {
            new_item->addChild(item_to_tree_item(sub_item));
        }
    }
    // return
    return new_item;
}

QString Editor::random_alpha_num(int n) const
{
    QString alpha_num = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    QString random;
    for(int i=0; i<n; ++i) {
        random += alpha_num.at(rand()%alpha_num.length());
    }
    return random;
}

void Editor::raw_display(bool raw)
{
    block_editor_signals();
    if(raw) {
        ui->graph_editor->setReadOnly(true);
        ui->graph_editor->setPlainText(ui->graph_editor->toHtml());
    } else {
        ui->graph_editor->setReadOnly(false);
        ui->graph_editor->setHtml(editor_content);
    }
    unblock_editor_signals();
}

void Editor::update_content()
{
    DEBUG_OUT("UPDATE");
    parse_content(ui->graph_editor->toPlainText());
}

void Editor::open_file()
{
    QString old_file_name = ui->file_name->text();
    QString base_path = QFileInfo(old_file_name).absolutePath();
    QString file_name = QFileDialog::getOpenFileName(this, tr("Parse file:"), base_path, tr("Text (*.txt);; Other (*)"));
    if(file_name!="") {
        ui->file_name->setText(file_name);
        parse_content(read_file(file_name));
    }
}

void Editor::save_file()
{
    QString file_name = ui->file_name->text();
    QFile outfile;
    outfile.setFileName(file_name);
    outfile.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&outfile);
    out << ui->graph_editor->toPlainText() << endl;
}

void Editor::quit()
{
    QApplication::quit();
}

void Editor::tree_item_clicked(QTreeWidgetItem * item, int column)
{
    // get chain of items (prepend new items)
    int max_nesting = 10;
    std::list<QTreeWidgetItem*> item_list({item});
    while(ui->tree_view->indexOfTopLevelItem(item_list.front())==-1 && --max_nesting>0) {
        auto parent = item_list.front()->parent();
        item_list.push_front(parent);
    }
    // get corresponding indices
    std::list<int> idx_list({ui->tree_view->indexOfTopLevelItem(item_list.front())});
    auto par_it=item_list.begin();
    auto child_it=par_it; ++child_it;
    while(child_it!=item_list.end()) {
        idx_list.push_back((*par_it)->indexOfChild(*child_it));
        ++par_it;
        ++child_it;
    }
    // traverse sub-graphs
    std::shared_ptr<Parser::KeyValueGraph> sub_graph(new Parser::KeyValueGraph(key_value_graph));
    auto item_it = sub_graph->items.begin();
    for(auto idx_it=idx_list.begin(); idx_it!=idx_list.end();) {
        item_it = sub_graph->items.begin();
        for(int item_idx=0; item_idx<*idx_it; ++item_idx) {
            ++item_it;
        }
        ++idx_it;
        if(item_it->value_type!=Parser::KeyValueGraph::Item::KVG && idx_it!=idx_list.end()) {
            ERROR("Not KVG");
        }
        sub_graph = item_it->sub_graph;
    }
    // get position
    int pos;
    switch (column) {
    case 0:
        pos = item_it->keys_start;
        break;
    case 1:
        pos = item_it->parents_start;
        break;
    case 2:
        pos = item_it->value_start;
        break;
    case 3:
        break;
    default:
        ERROR("Column idx should not exceed 3");
        break;
    }
    // set cursor
    QTextCursor cursor = ui->graph_editor->textCursor();
    cursor.setPosition(pos);
    ui->graph_editor->setTextCursor(cursor);
    ui->graph_editor->setFocus();
}

bool ZoomView::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type()==QEvent::Wheel) {
        // process wheel event
        QWheelEvent * wheel_event = static_cast<QWheelEvent *>(event);
        if(wheel_event!=nullptr && wheel_event->modifiers()==Qt::ControlModifier) {
            double s = wheel_event->delta();
            s /= 8*360;
            s = 1+s;
            editor->ui->visualization->scale(s, s);
            return true;
        }
    }

    // standard event processing
    return QObject::eventFilter(obj, event);
}
