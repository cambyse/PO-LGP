/****************************************************************************
** Meta object code from reading C++ file 'langenacht.h'
**
** Created: Wed May 30 00:15:20 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../langenacht.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'langenacht.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LangeNacht[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   12,   11,   11, 0x0a,
      27,   11,   11,   11, 0x2a,
      36,   11,   11,   11, 0x0a,
      56,   11,   11,   11, 0x0a,
      74,   11,   11,   11, 0x0a,
      93,   11,   11,   11, 0x0a,
     109,   11,   11,   11, 0x0a,
     127,   11,   11,   11, 0x0a,
     145,   11,   11,   11, 0x0a,
     172,   11,   11,   11, 0x0a,
     199,   11,   11,   11, 0x0a,
     220,   11,   11,   11, 0x0a,
     239,   11,   11,   11, 0x0a,
     259,   11,   11,   11, 0x0a,
     281,   11,   11,   11, 0x0a,
     300,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LangeNacht[] = {
    "LangeNacht\0\0c\0redraw(char)\0redraw()\0"
    "transition_random()\0transition_left()\0"
    "transition_right()\0transition_up()\0"
    "transition_down()\0transition_stay()\0"
    "show_rewards_changed(bool)\0"
    "show_actions_changed(bool)\0"
    "delete_all_rewards()\0delete_all_walls()\0"
    "random_changed(int)\0discount_changed(int)\0"
    "speed_changed(int)\0loop()\0"
};

void LangeNacht::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LangeNacht *_t = static_cast<LangeNacht *>(_o);
        switch (_id) {
        case 0: _t->redraw((*reinterpret_cast< const char(*)>(_a[1]))); break;
        case 1: _t->redraw(); break;
        case 2: _t->transition_random(); break;
        case 3: _t->transition_left(); break;
        case 4: _t->transition_right(); break;
        case 5: _t->transition_up(); break;
        case 6: _t->transition_down(); break;
        case 7: _t->transition_stay(); break;
        case 8: _t->show_rewards_changed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->show_actions_changed((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->delete_all_rewards(); break;
        case 11: _t->delete_all_walls(); break;
        case 12: _t->random_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->discount_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->speed_changed((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->loop(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LangeNacht::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LangeNacht::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_LangeNacht,
      qt_meta_data_LangeNacht, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LangeNacht::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LangeNacht::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LangeNacht::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LangeNacht))
        return static_cast<void*>(const_cast< LangeNacht*>(this));
    return QWidget::qt_metacast(_clname);
}

int LangeNacht::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
