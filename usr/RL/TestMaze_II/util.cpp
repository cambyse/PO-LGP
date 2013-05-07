/*
 * util.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "util.h"

#include <iostream>
#include <float.h>

using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::sort;

#include <QtCore>

#define DEBUG_STRING "util: "
#define DEBUG_LEVEL 0
#include "debug.h"

namespace util {

    bool arg_int(const QString& string_in, const int& n, int& i_arg_out) {
        QString arg = string_in.section(QRegExp("\\s+"),n,n);
        bool ok;
        i_arg_out = arg.toInt(&ok);
        return ok;
    }

    bool arg_double(const QString& string_in, const int& n, double& d_arg_out) {
        QString arg = string_in.section(QRegExp("\\s+"),n,n);
        bool ok;
        d_arg_out = arg.toDouble(&ok);
        return ok;
    }

    bool arg_string(const QString& string_in, const int& n, QString& s_arg_out) {
        s_arg_out = string_in.section(QRegExp("\\s+"),n,n);
        return true;
    }

    int print_progress(const int& prog, const int& max_prog, const int& width, const char* label, const int& last_prog) {
        int this_progress = width*prog/max_prog;
        if(width*prog/max_prog>last_prog) {
            cout << '\xd' << label << ": |";
            bool before = true;
            for(int idx=0; idx<width; ++idx) {
                if((double)idx/width<(double)prog/max_prog) {
                    cout << "-";
                } else {
                    if(before) {
                        cout << "o";
                        before = false;
                    }
                    cout << " ";
                }
            }
            if(before) {
                cout << "o";
            }
            cout << "|" << flush;
            return this_progress;
        } else {
            return last_prog;
        }
    }

    double approx_equal_tolerance() { return 1e-10; }

    const InvalidBase INVALID = InvalidBase(true);

} // end namespace util
