#include "util.h"

#include <QString>

std::ostream& operator<<(std::ostream &out, const QString& s) {
    out << (const char*)s.toLatin1();
    return out;
}
