/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Sun Feb 23 22:57:04 2014
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      43,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      19,   12,   11,   11, 0x0a,
      37,   12,   11,   11, 0x0a,
      54,   12,   11,   11, 0x0a,
      73,   12,   11,   11, 0x0a,
      89,   12,   11,   11, 0x0a,
     107,   12,   11,   11, 0x0a,
     122,   12,   11,   11, 0x0a,
     140,   12,   11,   11, 0x0a,
     156,   12,   11,   11, 0x0a,
     176,   12,   11,   11, 0x0a,
     194,   12,   11,   11, 0x0a,
     212,   12,   11,   11, 0x0a,
     228,   11,   11,   11, 0x0a,
     244,   11,   11,   11, 0x0a,
     257,   11,   11,   11, 0x0a,
     270,   11,   11,   11, 0x0a,
     281,   11,   11,   11, 0x0a,
     295,   11,   11,   11, 0x0a,
     310,   11,   11,   11, 0x0a,
     325,   11,   11,   11, 0x0a,
     344,   11,   11,   11, 0x0a,
     362,   11,   11,   11, 0x0a,
     377,   11,   11,   11, 0x0a,
     392,   11,   11,   11, 0x0a,
     410,   11,   11,   11, 0x0a,
     427,   11,   11,   11, 0x0a,
     441,   11,   11,   11, 0x0a,
     455,   11,   11,   11, 0x0a,
     466,   11,   11,   11, 0x0a,
     478,   11,   11,   11, 0x0a,
     492,   11,   11,   11, 0x0a,
     511,   11,   11,   11, 0x0a,
     522,   11,   11,   11, 0x0a,
     530,   11,   11,   11, 0x0a,
     553,  549,   11,   11, 0x0a,
     579,   11,   11,   11, 0x0a,
     599,   11,   11,   11, 0x0a,
     619,   11,   11,   11, 0x0a,
     640,   11,   11,   11, 0x0a,
     662,   11,   11,   11, 0x0a,
     683,   11,   11,   11, 0x0a,
     692,   12,   11,   11, 0x0a,
     714,   11,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0status\0SetWrenches(bool)\0"
    "SetDesired(bool)\0SetEstimated(bool)\0"
    "SetBBoxes(bool)\0SetContacts(bool)\0"
    "SetPoser(bool)\0SetExpanded(bool)\0"
    "SetPoseIK(bool)\0SetSensorPlot(bool)\0"
    "SetLogCheck(bool)\0SetSimulate(bool)\0"
    "SetRecord(bool)\0SendMilestone()\0"
    "SetMode(int)\0DoFreeMode()\0DoIKMode()\0"
    "DoForceMode()\0LoadResource()\0"
    "SaveScenario()\0SaveLastScenario()\0"
    "DefaultViewport()\0LoadViewport()\0"
    "SaveViewport()\0ShowPlotOptions()\0"
    "ShowDriverEdit()\0ShowOptions()\0"
    "ShowCommand()\0ShowHelp()\0ShowAbout()\0"
    "IKConstrain()\0IKConstrainPoint()\0"
    "IKDelete()\0Reset()\0ChangeRecordFile()\0"
    "w,h\0ChangeResolution(int,int)\0"
    "Resolution640x480()\0Resolution800x600()\0"
    "Resolution1280x768()\0Resolution1920x1080()\0"
    "FlexibleResolution()\0Shrink()\0"
    "DoMultipleParts(bool)\0Encode()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->SetWrenches((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->SetDesired((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->SetEstimated((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->SetBBoxes((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->SetContacts((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->SetPoser((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->SetExpanded((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->SetPoseIK((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->SetSensorPlot((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->SetLogCheck((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 10: _t->SetSimulate((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: _t->SetRecord((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 12: _t->SendMilestone(); break;
        case 13: _t->SetMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->DoFreeMode(); break;
        case 15: _t->DoIKMode(); break;
        case 16: _t->DoForceMode(); break;
        case 17: _t->LoadResource(); break;
        case 18: _t->SaveScenario(); break;
        case 19: _t->SaveLastScenario(); break;
        case 20: _t->DefaultViewport(); break;
        case 21: _t->LoadViewport(); break;
        case 22: _t->SaveViewport(); break;
        case 23: _t->ShowPlotOptions(); break;
        case 24: _t->ShowDriverEdit(); break;
        case 25: _t->ShowOptions(); break;
        case 26: _t->ShowCommand(); break;
        case 27: _t->ShowHelp(); break;
        case 28: _t->ShowAbout(); break;
        case 29: _t->IKConstrain(); break;
        case 30: _t->IKConstrainPoint(); break;
        case 31: _t->IKDelete(); break;
        case 32: _t->Reset(); break;
        case 33: _t->ChangeRecordFile(); break;
        case 34: _t->ChangeResolution((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 35: _t->Resolution640x480(); break;
        case 36: _t->Resolution800x600(); break;
        case 37: _t->Resolution1280x768(); break;
        case 38: _t->Resolution1920x1080(); break;
        case 39: _t->FlexibleResolution(); break;
        case 40: _t->Shrink(); break;
        case 41: _t->DoMultipleParts((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 42: _t->Encode(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 43)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 43;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
