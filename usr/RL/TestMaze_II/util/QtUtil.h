#ifndef QT_UTIL
#define QT_UTIL

#include <QtCore>

#if QT_VERSION < QT_VERSION_CHECK(4, 9, 9)
#define MY_QT_STR QString::fromUtf8
#else
#define MY_QT_STR QStringLiteral
#endif

#include <ostream>
class QString;
std::ostream& operator<<(std::ostream &out, const QString& s);

#endif /* QT_UTIL */
