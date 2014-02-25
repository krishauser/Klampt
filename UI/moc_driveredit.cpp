/****************************************************************************
** Meta object code from reading C++ file 'driveredit.h'
**
** Created: Wed Feb 19 20:16:44 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "driveredit.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'driveredit.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_DriverEdit[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   12,   11,   11, 0x05,
      45,   35,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      80,   71,   11,   11, 0x0a,
     103,   98,   11,   11, 0x0a,
     136,  122,   11,   11, 0x0a,
     175,   11,   11,   11, 0x0a,
     201,   11,   11,   11, 0x0a,
     229,  222,   11,   11, 0x0a,
     255,  251,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_DriverEdit[] = {
    "DriverEdit\0\0i\0GetDriverValues(int)\0"
    "index,val\0SetDriverValue(int,float)\0"
    "_current\0NewSelection(int)\0name\0"
    "AddDriver(QString)\0min,max,value\0"
    "GetDriverParameters(float,float,float)\0"
    "RequestDriverParameters()\0"
    "RequestDriverValue()\0_value\0"
    "HandleSpinBox(double)\0pos\0HandleSlider(int)\0"
};

void DriverEdit::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        DriverEdit *_t = static_cast<DriverEdit *>(_o);
        switch (_id) {
        case 0: _t->GetDriverValues((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->SetDriverValue((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->NewSelection((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->AddDriver((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->GetDriverParameters((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 5: _t->RequestDriverParameters(); break;
        case 6: _t->RequestDriverValue(); break;
        case 7: _t->HandleSpinBox((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 8: _t->HandleSlider((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData DriverEdit::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject DriverEdit::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_DriverEdit,
      qt_meta_data_DriverEdit, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &DriverEdit::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *DriverEdit::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *DriverEdit::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_DriverEdit))
        return static_cast<void*>(const_cast< DriverEdit*>(this));
    return QDialog::qt_metacast(_clname);
}

int DriverEdit::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void DriverEdit::GetDriverValues(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void DriverEdit::SetDriverValue(int _t1, float _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
