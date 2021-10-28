/****************************************************************************
** Meta object code from reading C++ file 'OutliningParWin.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/OutliningParWin.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'OutliningParWin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OutliningParametersWindow_t {
    QByteArrayData data[13];
    char stringdata0[320];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OutliningParametersWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OutliningParametersWindow_t qt_meta_stringdata_OutliningParametersWindow = {
    {
QT_MOC_LITERAL(0, 0, 25), // "OutliningParametersWindow"
QT_MOC_LITERAL(1, 26, 39), // "SetMaximumDistancePointInters..."
QT_MOC_LITERAL(2, 66, 0), // ""
QT_MOC_LITERAL(3, 67, 9), // "new_value"
QT_MOC_LITERAL(4, 77, 34), // "SetMinimumAnglePreferredDirec..."
QT_MOC_LITERAL(5, 112, 23), // "SetHoughBinSizeDistance"
QT_MOC_LITERAL(6, 136, 24), // "SetHoughBinSizeDirection"
QT_MOC_LITERAL(7, 161, 30), // "SetMaximumDistancePointOutline"
QT_MOC_LITERAL(8, 192, 34), // "SetMaximumPointGapInOutlineSe..."
QT_MOC_LITERAL(9, 227, 10), // "new_number"
QT_MOC_LITERAL(10, 238, 33), // "SetMaximumGapSizeInOutlineSeg..."
QT_MOC_LITERAL(11, 272, 40), // "SetMinimumNumberOfPointsInOut..."
QT_MOC_LITERAL(12, 313, 6) // "Update"

    },
    "OutliningParametersWindow\0"
    "SetMaximumDistancePointIntersectionLine\0"
    "\0new_value\0SetMinimumAnglePreferredDirections\0"
    "SetHoughBinSizeDistance\0"
    "SetHoughBinSizeDirection\0"
    "SetMaximumDistancePointOutline\0"
    "SetMaximumPointGapInOutlineSegment\0"
    "new_number\0SetMaximumGapSizeInOutlineSegment\0"
    "SetMinimumNumberOfPointsInOutlineSegment\0"
    "Update"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OutliningParametersWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   59,    2, 0x0a /* Public */,
       4,    1,   62,    2, 0x0a /* Public */,
       5,    1,   65,    2, 0x0a /* Public */,
       6,    1,   68,    2, 0x0a /* Public */,
       7,    1,   71,    2, 0x0a /* Public */,
       8,    1,   74,    2, 0x0a /* Public */,
      10,    1,   77,    2, 0x0a /* Public */,
      11,    1,   80,    2, 0x0a /* Public */,
      12,    0,   83,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,

       0        // eod
};

void OutliningParametersWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OutliningParametersWindow *_t = static_cast<OutliningParametersWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SetMaximumDistancePointIntersectionLine((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->SetMinimumAnglePreferredDirections((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->SetHoughBinSizeDistance((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->SetHoughBinSizeDirection((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->SetMaximumDistancePointOutline((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->SetMaximumPointGapInOutlineSegment((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->SetMaximumGapSizeInOutlineSegment((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->SetMinimumNumberOfPointsInOutlineSegment((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->Update(); break;
        default: ;
        }
    }
}

const QMetaObject OutliningParametersWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_OutliningParametersWindow.data,
      qt_meta_data_OutliningParametersWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *OutliningParametersWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OutliningParametersWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OutliningParametersWindow.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "OutliningParameters"))
        return static_cast< OutliningParameters*>(this);
    return QWidget::qt_metacast(_clname);
}

int OutliningParametersWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 9;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
