/****************************************************************************
** Meta object code from reading C++ file 'logoptions.h'
**
** Created: Wed Feb 19 20:16:43 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "logoptions.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'logoptions.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LogOptions[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      20,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   12,   11,   11, 0x05,
      73,   47,   11,   11, 0x05,
     113,  106,   11,   11, 0x05,
     131,  106,   11,   11, 0x05,
     152,   11,   11,   11, 0x05,
     168,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
     189,  184,   11,   11, 0x0a,
     216,  208,   11,   11, 0x0a,
     243,  184,   11,   11, 0x0a,
     267,   12,   11,   11, 0x0a,
     285,   11,   11,   11, 0x0a,
     295,   11,   11,   11, 0x0a,
     311,  305,   11,   11, 0x0a,
     325,   11,   11,   11, 0x0a,
     336,  305,   11,   11, 0x0a,
     350,   11,   11,   11, 0x0a,
     361,  305,   11,   11, 0x0a,
     386,  106,   11,   11, 0x0a,
     403,  106,   11,   11, 0x0a,
     429,  423,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LogOptions[] = {
    "LogOptions\0\0sensor\0SyncSensorMeasurements(int)\0"
    "sensor,measurement,status\0"
    "toggle_measurement(int,int,bool)\0"
    "status\0toggle_plot(bool)\0toggle_logging(bool)\0"
    "ShowSensor(int)\0HideSensor(int)\0name\0"
    "AddSensor(QString)\0sensors\0"
    "addSensors(vector<string>)\0"
    "AddMeasurement(QString)\0ChangeSensor(int)\0"
    "Isolate()\0ShowAll()\0index\0ShowItem(int)\0"
    "ShowItem()\0HideItem(int)\0HideItem()\0"
    "ChangeSelectedIndex(int)\0TogglePlot(bool)\0"
    "ToggleLogging(bool)\0names\0"
    "AddMeasurements(vector<string>)\0"
};

void LogOptions::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        LogOptions *_t = static_cast<LogOptions *>(_o);
        switch (_id) {
        case 0: _t->SyncSensorMeasurements((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->toggle_measurement((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 2: _t->toggle_plot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->toggle_logging((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->ShowSensor((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->HideSensor((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->AddSensor((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->addSensors((*reinterpret_cast< vector<string>(*)>(_a[1]))); break;
        case 8: _t->AddMeasurement((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->ChangeSensor((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->Isolate(); break;
        case 11: _t->ShowAll(); break;
        case 12: _t->ShowItem((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->ShowItem(); break;
        case 14: _t->HideItem((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->HideItem(); break;
        case 16: _t->ChangeSelectedIndex((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 17: _t->TogglePlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->ToggleLogging((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 19: _t->AddMeasurements((*reinterpret_cast< vector<string>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData LogOptions::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject LogOptions::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_LogOptions,
      qt_meta_data_LogOptions, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LogOptions::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LogOptions::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LogOptions::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LogOptions))
        return static_cast<void*>(const_cast< LogOptions*>(this));
    return QDialog::qt_metacast(_clname);
}

int LogOptions::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 20)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 20;
    }
    return _id;
}

// SIGNAL 0
void LogOptions::SyncSensorMeasurements(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LogOptions::toggle_measurement(int _t1, int _t2, bool _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LogOptions::toggle_plot(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void LogOptions::toggle_logging(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void LogOptions::ShowSensor(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void LogOptions::HideSensor(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
