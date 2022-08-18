/****************************************************************************
** Meta object code from reading C++ file 'opencvimageprovider.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "opencvimageprovider.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'opencvimageprovider.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OpencvImageProvider_t {
    QByteArrayData data[7];
    char stringdata0[69];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OpencvImageProvider_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OpencvImageProvider_t qt_meta_stringdata_OpencvImageProvider = {
    {
QT_MOC_LITERAL(0, 0, 19), // "OpencvImageProvider"
QT_MOC_LITERAL(1, 20, 11), // "QML.Element"
QT_MOC_LITERAL(2, 32, 4), // "auto"
QT_MOC_LITERAL(3, 37, 12), // "imageChanged"
QT_MOC_LITERAL(4, 50, 0), // ""
QT_MOC_LITERAL(5, 51, 11), // "updateImage"
QT_MOC_LITERAL(6, 63, 5) // "image"

    },
    "OpencvImageProvider\0QML.Element\0auto\0"
    "imageChanged\0\0updateImage\0image"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OpencvImageProvider[] = {

 // content:
       8,       // revision
       0,       // classname
       1,   14, // classinfo
       2,   16, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // classinfo: key, value
       1,    2,

 // signals: name, argc, parameters, tag, flags
       3,    0,   26,    4, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   27,    4, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::QImage,    6,

       0        // eod
};

void OpencvImageProvider::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<OpencvImageProvider *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->imageChanged(); break;
        case 1: _t->updateImage((*reinterpret_cast< const QImage(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (OpencvImageProvider::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OpencvImageProvider::imageChanged)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject OpencvImageProvider::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_OpencvImageProvider.data,
    qt_meta_data_OpencvImageProvider,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *OpencvImageProvider::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OpencvImageProvider::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OpencvImageProvider.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QQuickImageProvider"))
        return static_cast< QQuickImageProvider*>(this);
    return QObject::qt_metacast(_clname);
}

int OpencvImageProvider::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void OpencvImageProvider::imageChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
