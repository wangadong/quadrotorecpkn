/****************************************************************************
** Meta object code from reading C++ file 'Module3D.h'
**
** Created: Sat Mar 5 23:30:37 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../3D/Module3D.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Module3D.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Module3D[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   10,    9,    9, 0x0a,
      36,   10,    9,    9, 0x0a,
      56,   10,    9,    9, 0x0a,
      76,    9,    9,    9, 0x0a,
      95,   93,    9,    9, 0x0a,
     114,   93,    9,    9, 0x0a,
     133,   93,    9,    9, 0x0a,
     152,    9,    9,    9, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Module3D[] = {
    "Module3D\0\0angle\0setCRotation(float)\0"
    "setPRotation(float)\0setRRotation(float)\0"
    "upDateAttitude()\0v\0setCVitesse(float)\0"
    "setPVitesse(float)\0setRVitesse(float)\0"
    "upselondataSlot()\0"
};

const QMetaObject Module3D::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_Module3D,
      qt_meta_data_Module3D, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Module3D::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Module3D::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Module3D::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Module3D))
        return static_cast<void*>(const_cast< Module3D*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int Module3D::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: setCRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: setPRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 2: setRRotation((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 3: upDateAttitude(); break;
        case 4: setCVitesse((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: setPVitesse((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 6: setRVitesse((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: upselondataSlot(); break;
        default: ;
        }
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
