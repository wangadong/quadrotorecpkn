/****************************************************************************
** Meta object code from reading C++ file 'MachineThread.h'
**
** Created: Wed Mar 9 00:48:39 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../Mycom/MachineThread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MachineThread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_machineThread[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      11,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      28,   14,   14,   14, 0x05,
      51,   44,   14,   14, 0x05,
      84,   78,   14,   14, 0x05,
     107,  101,   14,   14, 0x05,
     145,  143,   14,   14, 0x05,
     167,  165,   14,   14, 0x05,
     189,  187,   14,   14, 0x05,
     216,  209,   14,   14, 0x05,
     245,  238,   14,   14, 0x05,
     273,  267,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
     295,   14,  290,   14, 0x0a,
     316,  306,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_machineThread[] = {
    "machineThread\0\0readsignal()\0showdataSpace()\0"
    "string\0setopenbutton(const char*)\0"
    "error\0threadError(int)\0dataD\0"
    "setinfoDynamique(UN_DATA_DYNAMIQUE)\0"
    "C\0setCRotation(float)\0P\0setPRotation(float)\0"
    "R\0setRRotation(float)\0recevu\0"
    "byteRecevuChange(int)\0envoie\0"
    "byteEnvoieChange(int)\0state\0"
    "portIsOpen(bool)\0char\0readport()\0"
    "port_name\0openport(QString)\0"
};

const QMetaObject machineThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_machineThread,
      qt_meta_data_machineThread, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &machineThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *machineThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *machineThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_machineThread))
        return static_cast<void*>(const_cast< machineThread*>(this));
    return QThread::qt_metacast(_clname);
}

int machineThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: readsignal(); break;
        case 1: showdataSpace(); break;
        case 2: setopenbutton((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 3: threadError((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: setinfoDynamique((*reinterpret_cast< UN_DATA_DYNAMIQUE(*)>(_a[1]))); break;
        case 5: setCRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 6: setPRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: setRRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 8: byteRecevuChange((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: byteEnvoieChange((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: portIsOpen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 11: { char _r = readport();
            if (_a[0]) *reinterpret_cast< char*>(_a[0]) = _r; }  break;
        case 12: openport((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void machineThread::readsignal()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void machineThread::showdataSpace()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void machineThread::setopenbutton(const char * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void machineThread::threadError(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void machineThread::setinfoDynamique(UN_DATA_DYNAMIQUE _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void machineThread::setCRotation(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void machineThread::setPRotation(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void machineThread::setRRotation(float _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void machineThread::byteRecevuChange(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void machineThread::byteEnvoieChange(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void machineThread::portIsOpen(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}
QT_END_MOC_NAMESPACE
