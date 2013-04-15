/*
 * util.cpp
 *
 *  Created on: Dec 12, 2012
 *      Author: robert
 */

#include "util.h"

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

    double approx_equal_tolerance() { return 1e-10; }

    const InvalidBase INVALID = InvalidBase(true);

} // end namespace util
