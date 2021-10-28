/****************************************************************************
** Meta object code from reading C++ file 'FilteringParWin.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../include/FilteringParWin.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FilteringParWin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_FilteringParametersWindow_t {
    QByteArrayData data[8];
    char stringdata0[146];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FilteringParametersWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FilteringParametersWindow_t qt_meta_stringdata_FilteringParametersWindow = {
    {
QT_MOC_LITERAL(0, 0, 25), // "FilteringParametersWindow"
QT_MOC_LITERAL(1, 26, 6), // "Update"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 26), // "SetHeightDifAtSameLocation"
QT_MOC_LITERAL(4, 61, 15), // "SetFilterRadius"
QT_MOC_LITERAL(5, 77, 26), // "SetHeightDifAtFilterRadius"
QT_MOC_LITERAL(6, 104, 24), // "SetRemoveNonGroundPoints"
QT_MOC_LITERAL(7, 129, 16) // "new_switch_value"

    },
    "FilteringParametersWindow\0Update\0\0"
    "SetHeightDifAtSameLocation\0SetFilterRadius\0"
    "SetHeightDifAtFilterRadius\0"
    "SetRemoveNonGroundPoints\0new_switch_value"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FilteringParametersWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    1,   40,    2, 0x0a /* Public */,
       4,    1,   43,    2, 0x0a /* Public */,
       5,    1,   46,    2, 0x0a /* Public */,
       6,    1,   49,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::Bool,    7,

       0        // eod
};

void FilteringParametersWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FilteringParametersWindow *_t = static_cast<FilteringParametersWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->Update(); break;
        case 1: _t->SetHeightDifAtSameLocation((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->SetFilterRadius((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->SetHeightDifAtFilterRadius((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->SetRemoveNonGroundPoints((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject FilteringParametersWindow::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_FilteringParametersWindow.data,
      qt_meta_data_FilteringParametersWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *FilteringParametersWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FilteringParametersWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_FilteringParametersWindow.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "FilteringParameters"))
        return static_cast< FilteringParameters*>(this);
    return QWidget::qt_metacast(_clname);
}

int FilteringParametersWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
