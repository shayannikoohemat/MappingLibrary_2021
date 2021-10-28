/****************************************************************************
** Meta object code from reading C++ file 'PCMCanvas.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/PCMCanvas.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PCMCanvas.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PCMCanvas_t {
    QByteArrayData data[34];
    char stringdata0[429];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PCMCanvas_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PCMCanvas_t qt_meta_stringdata_PCMCanvas = {
    {
QT_MOC_LITERAL(0, 0, 9), // "PCMCanvas"
QT_MOC_LITERAL(1, 10, 19), // "SelectedObjectPoint"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 11), // "PointNumber"
QT_MOC_LITERAL(4, 43, 6), // "number"
QT_MOC_LITERAL(5, 50, 10), // "Position3D"
QT_MOC_LITERAL(6, 61, 7), // "map_pos"
QT_MOC_LITERAL(7, 69, 8), // "DataType"
QT_MOC_LITERAL(8, 78, 14), // "selection_type"
QT_MOC_LITERAL(9, 93, 9), // "base_type"
QT_MOC_LITERAL(10, 103, 19), // "SelectedLaserPoints"
QT_MOC_LITERAL(11, 123, 12), // "LaserPoints*"
QT_MOC_LITERAL(12, 136, 6), // "points"
QT_MOC_LITERAL(13, 143, 3), // "add"
QT_MOC_LITERAL(14, 147, 20), // "SelectedLaserSegment"
QT_MOC_LITERAL(15, 168, 10), // "LaserPoint"
QT_MOC_LITERAL(16, 179, 5), // "point"
QT_MOC_LITERAL(17, 185, 4), // "type"
QT_MOC_LITERAL(18, 190, 26), // "RequestForPointInformation"
QT_MOC_LITERAL(19, 217, 11), // "ObjectPoint"
QT_MOC_LITERAL(20, 229, 22), // "ToggleSelectionRequest"
QT_MOC_LITERAL(21, 252, 16), // "CanvasKeyPressed"
QT_MOC_LITERAL(22, 269, 10), // "QKeyEvent*"
QT_MOC_LITERAL(23, 280, 5), // "event"
QT_MOC_LITERAL(24, 286, 12), // "SplitRequest"
QT_MOC_LITERAL(25, 299, 9), // "MouseMode"
QT_MOC_LITERAL(26, 309, 4), // "mode"
QT_MOC_LITERAL(27, 314, 13), // "LineSegment2D"
QT_MOC_LITERAL(28, 328, 4), // "line"
QT_MOC_LITERAL(29, 333, 21), // "RequestLastEdgeDelete"
QT_MOC_LITERAL(30, 355, 17), // "LineHasBeenClosed"
QT_MOC_LITERAL(31, 373, 15), // "NewCanvasBounds"
QT_MOC_LITERAL(32, 389, 12), // "DataBounds2D"
QT_MOC_LITERAL(33, 402, 26) // "LastModelDataHasBeenEdited"

    },
    "PCMCanvas\0SelectedObjectPoint\0\0"
    "PointNumber\0number\0Position3D\0map_pos\0"
    "DataType\0selection_type\0base_type\0"
    "SelectedLaserPoints\0LaserPoints*\0"
    "points\0add\0SelectedLaserSegment\0"
    "LaserPoint\0point\0type\0RequestForPointInformation\0"
    "ObjectPoint\0ToggleSelectionRequest\0"
    "CanvasKeyPressed\0QKeyEvent*\0event\0"
    "SplitRequest\0MouseMode\0mode\0LineSegment2D\0"
    "line\0RequestLastEdgeDelete\0LineHasBeenClosed\0"
    "NewCanvasBounds\0DataBounds2D\0"
    "LastModelDataHasBeenEdited"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PCMCanvas[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      12,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   74,    2, 0x06 /* Public */,
      10,    2,   83,    2, 0x06 /* Public */,
      14,    2,   88,    2, 0x06 /* Public */,
      18,    1,   93,    2, 0x06 /* Public */,
      18,    2,   96,    2, 0x06 /* Public */,
      20,    1,  101,    2, 0x06 /* Public */,
      21,    1,  104,    2, 0x06 /* Public */,
      24,    2,  107,    2, 0x06 /* Public */,
      29,    0,  112,    2, 0x06 /* Public */,
      30,    1,  113,    2, 0x06 /* Public */,
      31,    1,  116,    2, 0x06 /* Public */,
      33,    0,  119,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, 0x80000000 | 7, 0x80000000 | 7,    4,    6,    8,    9,
    QMetaType::Void, 0x80000000 | 11, QMetaType::Bool,   12,   13,
    QMetaType::Void, 0x80000000 | 15, 0x80000000 | 7,   16,   17,
    QMetaType::Void, 0x80000000 | 15,   16,
    QMetaType::Void, 0x80000000 | 19, 0x80000000 | 7,   16,   17,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, 0x80000000 | 22,   23,
    QMetaType::Void, 0x80000000 | 25, 0x80000000 | 27,   26,   28,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 25,    2,
    QMetaType::Void, 0x80000000 | 32,    2,
    QMetaType::Void,

       0        // eod
};

void PCMCanvas::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PCMCanvas *_t = static_cast<PCMCanvas *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->SelectedObjectPoint((*reinterpret_cast< const PointNumber(*)>(_a[1])),(*reinterpret_cast< const Position3D(*)>(_a[2])),(*reinterpret_cast< DataType(*)>(_a[3])),(*reinterpret_cast< DataType(*)>(_a[4]))); break;
        case 1: _t->SelectedLaserPoints((*reinterpret_cast< LaserPoints*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->SelectedLaserSegment((*reinterpret_cast< const LaserPoint(*)>(_a[1])),(*reinterpret_cast< DataType(*)>(_a[2]))); break;
        case 3: _t->RequestForPointInformation((*reinterpret_cast< const LaserPoint(*)>(_a[1]))); break;
        case 4: _t->RequestForPointInformation((*reinterpret_cast< const ObjectPoint(*)>(_a[1])),(*reinterpret_cast< DataType(*)>(_a[2]))); break;
        case 5: _t->ToggleSelectionRequest((*reinterpret_cast< DataType(*)>(_a[1]))); break;
        case 6: _t->CanvasKeyPressed((*reinterpret_cast< QKeyEvent*(*)>(_a[1]))); break;
        case 7: _t->SplitRequest((*reinterpret_cast< MouseMode(*)>(_a[1])),(*reinterpret_cast< const LineSegment2D(*)>(_a[2]))); break;
        case 8: _t->RequestLastEdgeDelete(); break;
        case 9: _t->LineHasBeenClosed((*reinterpret_cast< MouseMode(*)>(_a[1]))); break;
        case 10: _t->NewCanvasBounds((*reinterpret_cast< const DataBounds2D(*)>(_a[1]))); break;
        case 11: _t->LastModelDataHasBeenEdited(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (PCMCanvas::*_t)(const PointNumber & , const Position3D & , DataType , DataType );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::SelectedObjectPoint)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(LaserPoints * , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::SelectedLaserPoints)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(const LaserPoint & , DataType );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::SelectedLaserSegment)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(const LaserPoint & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::RequestForPointInformation)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(const ObjectPoint & , DataType );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::RequestForPointInformation)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(DataType );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::ToggleSelectionRequest)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(QKeyEvent * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::CanvasKeyPressed)) {
                *result = 6;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(MouseMode , const LineSegment2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::SplitRequest)) {
                *result = 7;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::RequestLastEdgeDelete)) {
                *result = 8;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(MouseMode );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::LineHasBeenClosed)) {
                *result = 9;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)(const DataBounds2D & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::NewCanvasBounds)) {
                *result = 10;
                return;
            }
        }
        {
            typedef void (PCMCanvas::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PCMCanvas::LastModelDataHasBeenEdited)) {
                *result = 11;
                return;
            }
        }
    }
}

const QMetaObject PCMCanvas::staticMetaObject = {
    { &QGLCanvas::staticMetaObject, qt_meta_stringdata_PCMCanvas.data,
      qt_meta_data_PCMCanvas,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *PCMCanvas::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PCMCanvas::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PCMCanvas.stringdata0))
        return static_cast<void*>(this);
    return QGLCanvas::qt_metacast(_clname);
}

int PCMCanvas::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLCanvas::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void PCMCanvas::SelectedObjectPoint(const PointNumber & _t1, const Position3D & _t2, DataType _t3, DataType _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PCMCanvas::SelectedLaserPoints(LaserPoints * _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void PCMCanvas::SelectedLaserSegment(const LaserPoint & _t1, DataType _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void PCMCanvas::RequestForPointInformation(const LaserPoint & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void PCMCanvas::RequestForPointInformation(const ObjectPoint & _t1, DataType _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void PCMCanvas::ToggleSelectionRequest(DataType _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void PCMCanvas::CanvasKeyPressed(QKeyEvent * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void PCMCanvas::SplitRequest(MouseMode _t1, const LineSegment2D & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void PCMCanvas::RequestLastEdgeDelete()
{
    QMetaObject::activate(this, &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void PCMCanvas::LineHasBeenClosed(MouseMode _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void PCMCanvas::NewCanvasBounds(const DataBounds2D & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void PCMCanvas::LastModelDataHasBeenEdited()
{
    QMetaObject::activate(this, &staticMetaObject, 11, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
