#include "SandBox.h"

#include <gtest/gtest.h>

#include <vector>
#include <map>
#include <iostream>

#include <QString>
#include <QFile>

#include "../util/util.h"
#include "../util/QtUtil.h"

#include "../util/debug.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;

void read_environment(QString file_name) {

    // the different factors constituting the state (factor_name --> factor_size)
    map<QString,int> factors;

    // to keep track of the line currently parsed
    int line_number;

    // returns 'file_name:line_number' as descriptor for messages
    auto file_line = [&](){ return QString("%1:%2").arg(file_name).arg(line_number); };

    // defines a new factor by adding an element to the factors map
    auto define_factor = [&](QStringList elem){
        QString name = elem[1];
        bool ok;
        int size = elem[2].toInt(&ok);
        if(!ok) {
            DEBUG_ERROR(file_line() << " could not interprete '" << elem[2] << "' as int");
        } else {
            factors[name] = size;
        }
    };

    // defines a new factor corresponding to a maze
    auto define_maze = [&](QStringList elem){
        bool ok_x, ok_y;
        int nx = elem[1].toInt(&ok_x);
        int ny = elem[2].toInt(&ok_y);
        if(!ok_x) {
            DEBUG_ERROR(file_line() << " could not interprete '" << elem[1] << "' as int");
        } else if(!ok_y) {
            DEBUG_ERROR(file_line() << " could not interprete '" << elem[2] << "' as int");
        } else {
            factors["maze"] = nx*ny;
        }
    };

    // open and read input file
    {
        QFile input_file(file_name);
        if(input_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QTextStream input_file_stream(&input_file);
            bool maze_defined = false;

            for(line_number = 1; !input_file_stream.atEnd(); ++line_number) {
                QStringList elem = input_file_stream.readLine().simplified().split(QRegExp("\\s"));
                if(elem[0].compare("factor", Qt::CaseInsensitive)==0) {
                    // define a factor
                    if(elem.size()<3) {
                        DEBUG_ERROR(file_line() << " expecting: factor <name> <size>");
                    } else {
                        define_factor(elem);
                    }
                } else if(elem[0].compare("maze", Qt::CaseInsensitive)==0) {
                    // define a maze
                    if(maze_defined) {
                        DEBUG_ERROR("Maze already defined");
                    } else if(elem.size()<3) {
                        DEBUG_ERROR(file_line() << " expecting: maze <nx> <ny>");
                    } else {
                        maze_defined = true;
                        define_maze(elem);
                    }
                } else if(elem[0]=="") {
                    // skip empty lines
                } else {
                    DEBUG_ERROR("Unknown item '" << elem[0] << "'");
                }
            }
        } else {
            DEBUG_ERROR("Could not read file '" << file_name << "'");
        }
    }

    for(auto f : factors) {
        DEBUG_OUT(0, "Factor '" << f.first << "' of size " << f.second);
    }

}

TEST(SandBox, SomeTest) {
    read_environment("tmp.env");
}
