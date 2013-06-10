#ifndef QT_VERSION_ADAPT
#define QT_VERSION_ADAPT

#if QT_VERSION < QT_VERSION_CHECK(4, 9, 9)
#define MY_QT_STR QString::fromUtf8
#else
#define MY_QT_STR QStringLiteral
#endif

#endif
