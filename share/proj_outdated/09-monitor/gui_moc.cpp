/****************************************************************************
** Meta object code from reading C++ file 'gui.h'
**
** Created: Tue May 4 11:22:12 2010
**      by: The Qt Meta Object Compiler version 61 (Qt 4.5.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "gui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 61
#error "This file was generated using the moc from 4.5.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Gui[] = {

 // content:
       2,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   12, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors

 // slots: signature, parameters, type, tag, flags
       5,    4,    4,    4, 0x0a,
      18,   14,    4,    4, 0x0a,
      37,   14,    4,    4, 0x0a,
      56,   14,    4,    4, 0x0a,
      75,   14,    4,    4, 0x0a,
      94,   14,    4,    4, 0x0a,
     113,   14,    4,    4, 0x0a,
     132,   14,    4,    4, 0x0a,
     157,  151,    4,    4, 0x0a,
     180,    4,    4,    4, 0x0a,
     193,    4,    4,    4, 0x0a,
     218,    4,    4,    4, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Gui[] = {
    "Gui\0\0update()\0val\0changeTarget3(int)\0"
    "changeTarget4(int)\0changeTarget5(int)\0"
    "changeTarget6(int)\0changeTarget7(int)\0"
    "changeTarget8(int)\0changeTarget9(int)\0"
    "m,val\0changeTarget(uint,int)\0updateDisp()\0"
    "setThisPosAsHomeOffset()\0report()\0"
};

const QMetaObject Gui::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_Gui,
      qt_meta_data_Gui, 0 }
};

const QMetaObject *Gui::metaObject() const
{
    return &staticMetaObject;
}

void *Gui::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Gui))
        return static_cast<void*>(const_cast< Gui*>(this));
    return QDialog::qt_metacast(_clname);
}

int Gui::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: update(); break;
        case 1: changeTarget3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: changeTarget4((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: changeTarget5((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: changeTarget6((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: changeTarget7((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: changeTarget8((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: changeTarget9((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: changeTarget((*reinterpret_cast< uint(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 9: updateDisp(); break;
        case 10: setThisPosAsHomeOffset(); break;
        case 11: report(); break;
        default: ;
        }
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
