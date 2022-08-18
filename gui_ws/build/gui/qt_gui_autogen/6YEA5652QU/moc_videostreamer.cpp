/****************************************************************************
** Meta object code from reading C++ file 'videostreamer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "videostreamer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'videostreamer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_VideoStreamer_t {
    QByteArrayData data[8];
    char stringdata0[70];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VideoStreamer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VideoStreamer_t qt_meta_stringdata_VideoStreamer = {
    {
QT_MOC_LITERAL(0, 0, 13), // "VideoStreamer"
QT_MOC_LITERAL(1, 14, 11), // "QML.Element"
QT_MOC_LITERAL(2, 26, 4), // "auto"
QT_MOC_LITERAL(3, 31, 8), // "newImage"
QT_MOC_LITERAL(4, 40, 0), // ""
QT_MOC_LITERAL(5, 41, 7), // "QImage&"
QT_MOC_LITERAL(6, 49, 15), // "openVideoCamera"
QT_MOC_LITERAL(7, 65, 4) // "path"

    },
    "VideoStreamer\0QML.Element\0auto\0newImage\0"
    "\0QImage&\0openVideoCamera\0path"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VideoStreamer[] = {

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
       3,    1,   26,    4, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   29,    4, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 5,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    7,

       0        // eod
};

void VideoStreamer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VideoStreamer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->newImage((*reinterpret_cast< QImage(*)>(_a[1]))); break;
        case 1: _t->openVideoCamera((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VideoStreamer::*)(QImage & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VideoStreamer::newImage)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject VideoStreamer::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_VideoStreamer.data,
    qt_meta_data_VideoStreamer,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *VideoStreamer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VideoStreamer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_VideoStreamer.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int VideoStreamer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void VideoStreamer::newImage(QImage & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
