/*
 * util.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "util.h"

#include <iostream>
#include <algorithm>
#include <float.h>

using std::cout;
using std::endl;
using std::flush;
using std::vector;
using std::sort;

#include <QtCore>

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

    double kolmogorov_smirnov_test(const std::vector<double>& s1, const std::vector<double>& s2, bool sorted) {
        // copy data only of not sorted
        const vector<double> *s1Ptr, *s2Ptr ;
        if(sorted) {
            s1Ptr = &s1;
            s2Ptr = &s2;
        } else {
            vector<double> *tmp_s1 = new vector<double>(s1);
            vector<double> *tmp_s2 = new vector<double>(s2);
            sort(tmp_s1->begin(), tmp_s1->end());
            sort(tmp_s2->begin(), tmp_s2->end());
            s1Ptr = tmp_s1;
            s2Ptr = tmp_s2;
        }

        // use reference to data
        const vector<double> &samples_1 = *s1Ptr;
        const vector<double> &samples_2 = *s2Ptr;

        auto samples_1_it = samples_1.begin();
        auto samples_2_it = samples_2.begin();
        double max_diff = -DBL_MAX, current_point;
        unsigned long int s1_cumulative = 0, s2_cumulative = 0;
        while(samples_1_it!=samples_1.end() || samples_2_it!=samples_2.end()) {
            // todo
        }

        // if data was copied to sort delete copies
        if(!sorted) {
            delete s1Ptr;
            delete s2Ptr;
        }
    }

    const InvalidBase INVALID = InvalidBase(true);

} // end namespace util
