/****************************************************************************
** Meta object code from reading C++ file 'qsimtestbackend.h'
**
** Created: Wed Feb 19 20:16:39 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "qsimtestbackend.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'qsimtestbackend.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QSimTestBackend[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   17,   16,   16, 0x05,
      46,   17,   16,   16, 0x05,
      70,   17,   16,   16, 0x05,
      95,   17,   16,   16, 0x05,
     116,   17,   16,   16, 0x05,
     139,   17,   16,   16, 0x05,
     164,   17,   16,   16, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_QSimTestBackend[] = {
    "QSimTestBackend\0\0e\0ResizeFrame(QResizeEvent*)\0"
    "MouseMove(QMouseEvent*)\0"
    "MouseWheel(QWheelEvent*)\0KeyPress(QKeyEvent*)\0"
    "KeyRelease(QKeyEvent*)\0MousePress(QMouseEvent*)\0"
    "MouseRelease(QMouseEvent*)\0"
};

void QSimTestBackend::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        QSimTestBackend *_t = static_cast<QSimTestBackend *>(_o);
        switch (_id) {
        case 0: _t->ResizeFrame((*reinterpret_cast< QResizeEvent*(*)>(_a[1]))); break;
        case 1: _t->MouseMove((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 2: _t->MouseWheel((*reinterpret_cast< QWheelEvent*(*)>(_a[1]))); break;
        case 3: _t->KeyPress((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 4: _t->KeyRelease((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 5: _t->MousePress((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        case 6: _t->MouseRelease((*reinterpret_cast< QMouseEvent*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData QSimTestBackend::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject QSimTestBackend::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_QSimTestBackend,
      qt_meta_data_QSimTestBackend, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QSimTestBackend::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QSimTestBackend::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QSimTestBackend::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QSimTestBackend))
        return static_cast<void*>(const_cast< QSimTestBackend*>(this));
    if (!strcmp(_clname, "SimTestBackend"))
        return static_cast< SimTestBackend*>(const_cast< QSimTestBackend*>(this));
    if (!strcmp(_clname, "GLScreenshotPlugin"))
        return static_cast< GLScreenshotPlugin*>(const_cast< QSimTestBackend*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int QSimTestBackend::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void QSimTestBackend::ResizeFrame(QResizeEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QSimTestBackend::MouseMove(QMouseEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QSimTestBackend::MouseWheel(QWheelEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QSimTestBackend::KeyPress(QKeyEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void QSimTestBackend::KeyRelease(QKeyEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void QSimTestBackend::MousePress(QMouseEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void QSimTestBackend::MouseRelease(QMouseEvent * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}
QT_END_MOC_NAMESPACE
