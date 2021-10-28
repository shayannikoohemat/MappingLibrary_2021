/****************************************************************************
** Meta object code from reading C++ file 'QGLCanvas.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/QGLCanvas.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'QGLCanvas.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_QGLCanvas_t {
    QByteArrayData data[17];
    char stringdata0[190];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_QGLCanvas_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_QGLCanvas_t qt_meta_stringdata_QGLCanvas = {
    {
QT_MOC_LITERAL(0, 0, 9), // "QGLCanvas"
QT_MOC_LITERAL(1, 10, 21), // "CentreOfCanvasChanged"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 1), // "X"
QT_MOC_LITERAL(4, 35, 1), // "Y"
QT_MOC_LITERAL(5, 37, 16), // "CanvasTranslated"
QT_MOC_LITERAL(6, 54, 1), // "x"
QT_MOC_LITERAL(7, 56, 1), // "y"
QT_MOC_LITERAL(8, 58, 12), // "CanvasScaled"
QT_MOC_LITERAL(9, 71, 5), // "scale"
QT_MOC_LITERAL(10, 77, 13), // "CanvasPainted"
QT_MOC_LITERAL(11, 91, 21), // "SetDataOffsetInCanvas"
QT_MOC_LITERAL(12, 113, 15), // "TranslateCanvas"
QT_MOC_LITERAL(13, 129, 11), // "ScaleCanvas"
QT_MOC_LITERAL(14, 141, 23), // "ResizeFocalLengthCanvas"
QT_MOC_LITERAL(15, 165, 12), // "scale_factor"
QT_MOC_LITERAL(16, 178, 11) // "PaintCanvas"

    },
    "QGLCanvas\0CentreOfCanvasChanged\0\0X\0Y\0"
    "CanvasTranslated\0x\0y\0CanvasScaled\0"
    "scale\0CanvasPainted\0SetDataOffsetInCanvas\0"
    "TranslateCanvas\0ScaleCanvas\0"
    "ResizeFocalLengthCanvas\0scale_factor\0"
    "PaintCanvas"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_QGLCanvas[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    2,   59,    2, 0x06 /* Public */,
       5,    2,   64,    2, 0x06 /* Public */,
       8,    1,   69,    2, 0x06 /* Public */,
      10,    0,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      11,    2,   73,    2, 0x0a /* Public */,
      12,    2,   78,    2, 0x0a /* Public */,
      13,    1,   83,    2, 0x0a /* Public */,
      14,    1,   86,    2, 0x0a /* Public */,
      16,    0,   89,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Float, QMetaType::Float,    6,    7,
    QMetaType::Void, QMetaType::Float,    9,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    3,    4,
    QMetaType::Void, QMetaType::Float, QMetaType::Float,    6,    7,
    QMetaType::Void, QMetaType::Float,    9,
    QMetaType::Void, QMetaType::Float,   15,
    QMetaType::Void,

       0        // eod
};

void QGLCanvas::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QGLCanvas *_t = static_cast<QGLCanvas *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->CentreOfCanvasChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 1: _t->CanvasTranslated((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 2: _t->CanvasScaled((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: _t->CanvasPainted(); break;
        case 4: _t->SetDataOffsetInCanvas((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 5: _t->TranslateCanvas((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 6: _t->ScaleCanvas((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: _t->ResizeFocalLengthCanvas((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: _t->PaintCanvas(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (QGLCanvas::*_t)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLCanvas::CentreOfCanvasChanged)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (QGLCanvas::*_t)(float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLCanvas::CanvasTranslated)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (QGLCanvas::*_t)(float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLCanvas::CanvasScaled)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (QGLCanvas::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&QGLCanvas::CanvasPainted)) {
                *result = 3;
                return;
            }
        }
    }
}

const QMetaObject QGLCanvas::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_QGLCanvas.data,
      qt_meta_data_QGLCanvas,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *QGLCanvas::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *QGLCanvas::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_QGLCanvas.stringdata0))
        return static_cast<void*>(this);
    return QGLWidget::qt_metacast(_clname);
}

int QGLCanvas::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void QGLCanvas::CentreOfCanvasChanged(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void QGLCanvas::CanvasTranslated(float _t1, float _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void QGLCanvas::CanvasScaled(float _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void QGLCanvas::CanvasPainted()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
