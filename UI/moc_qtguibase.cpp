/****************************************************************************
** Meta object code from reading C++ file 'qtguibase.h'
**
** Created: Sun Feb 23 22:57:06 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qtguibase.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qtguibase.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QtGUIBase[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      11,   10,   10,   10, 0x05,
      23,   10,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      40,   38,   10,   10, 0x0a,
      68,   38,   10,   10, 0x0a,
      97,   38,   10,   10, 0x0a,
     126,   38,   10,   10, 0x0a,
     157,   38,   10,   10, 0x0a,
     181,   38,   10,   10, 0x0a,
     203,   10,   10,   10, 0x0a,
     226,  214,   10,   10, 0x0a,
     260,  253,   10,   10, 0x0a,
     276,  253,   10,   10, 0x0a,
     318,  292,   10,   10, 0x0a,
     362,  348,   10,   10, 0x0a,
     399,  348,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_QtGUIBase[] = {
    "QtGUIBase\0\0EndDelete()\0EndConstrain()\0"
    "e\0SendMouseMove(QMouseEvent*)\0"
    "SendMouseWheel(QWheelEvent*)\0"
    "SendMousePress(QMouseEvent*)\0"
    "SendMouseRelease(QMouseEvent*)\0"
    "SendKeyDown(QKeyEvent*)\0SendKeyUp(QKeyEvent*)\0"
    "SendIdle()\0index,value\0"
    "SendDriverValue(int,float)\0sensor\0"
    "ShowSensor(int)\0HideSensor(int)\0"
    "sensor,measurement,status\0"
    "SendMeasurement(int,int,bool)\0"
    "setting,value\0SendControllerSetting(string,string)\0"
    "SendControllerCommand(string,string)\0"
};

void QtGUIBase::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QtGUIBase *_t = static_cast<QtGUIBase *>(_o);
        switch (_id) {
        case 0: _t->EndDelete(); break;
        case 1: _t->EndConstrain(); break;
        case 2: _t->SendMouseMove((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 3: _t->SendMouseWheel((*reinterpret_cast< QWheelEvent*(*)>(_a[1]))); break;
        case 4: _t->SendMousePress((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 5: _t->SendMouseRelease((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 6: _t->SendKeyDown((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 7: _t->SendKeyUp((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 8: _t->SendIdle(); break;
        case 9: _t->SendDriverValue((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 10: _t->ShowSensor((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->HideSensor((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->SendMeasurement((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 13: _t->SendControllerSetting((*reinterpret_cast< string(*)>(_a[1])),(*reinterpret_cast< string(*)>(_a[2]))); break;
        case 14: _t->SendControllerCommand((*reinterpret_cast< string(*)>(_a[1])),(*reinterpret_cast< string(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QtGUIBase::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QtGUIBase::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_QtGUIBase,
      qt_meta_data_QtGUIBase, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QtGUIBase::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QtGUIBase::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QtGUIBase::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QtGUIBase))
        return static_cast<void*>(const_cast< QtGUIBase*>(this));
    if (!strcmp(_clname, "GenericGUIBase"))
        return static_cast< GenericGUIBase*>(const_cast< QtGUIBase*>(this));
    return QObject::qt_metacast(_clname);
}

int QtGUIBase::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void QtGUIBase::EndDelete()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void QtGUIBase::EndConstrain()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
