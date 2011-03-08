/****************************************************************************
** Meta object code from reading C++ file 'Mycom.h'
**
** Created: Wed Mar 9 00:48:34 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../Mycom/Mycom.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Mycom.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Mycom[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      17,    7,    6,    6, 0x05,

 // slots: signature, parameters, type, tag, flags
      34,    6,    6,    6, 0x0a,
      55,   49,    6,    6, 0x0a,
      72,    6,    6,    6, 0x0a,
      99,    6,    6,    6, 0x0a,
     115,    6,    6,    6, 0x0a,
     128,    6,    6,    6, 0x0a,
     175,    6,    6,    6, 0x0a,
     223,    6,    6,    6, 0x0a,
     269,    6,    6,    6, 0x0a,
     315,    6,    6,    6, 0x0a,
     371,  365,    6,    6, 0x0a,
     416,  407,    6,    6, 0x0a,
     438,    6,    6,    6, 0x0a,
     461,  455,    6,    6, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_Mycom[] = {
    "Mycom\0\0port_name\0comopen(QString)\0"
    "opencom_port()\0state\0portisopen(bool)\0"
    "setopenbutton(const char*)\0cleartextArea()\0"
    "recountNum()\0"
    "comboBox_Baudrate_currentIndexChanged(QString)\0"
    "comboBox_Paritybit_currentIndexChanged(QString)\0"
    "comboBox_Databit_currentIndexChanged(QString)\0"
    "comboBox_Stopbit_currentIndexChanged(QString)\0"
    "comboBox_Controlflow_currentIndexChanged(QString)\0"
    "dataD\0setinfoDynamique(UN_DATA_DYNAMIQUE)\0"
    "datatemp\0writedataSpace(char*)\0"
    "verifier_check()\0error\0threaderror(int)\0"
};

const QMetaObject Mycom::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_Mycom,
      qt_meta_data_Mycom, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Mycom::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Mycom::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Mycom::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Mycom))
        return static_cast<void*>(const_cast< Mycom*>(this));
    return QWidget::qt_metacast(_clname);
}

int Mycom::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: comopen((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: opencom_port(); break;
        case 2: portisopen((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: setopenbutton((*reinterpret_cast< const char*(*)>(_a[1]))); break;
        case 4: cleartextArea(); break;
        case 5: recountNum(); break;
        case 6: comboBox_Baudrate_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: comboBox_Paritybit_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 8: comboBox_Databit_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 9: comboBox_Stopbit_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 10: comboBox_Controlflow_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 11: setinfoDynamique((*reinterpret_cast< UN_DATA_DYNAMIQUE(*)>(_a[1]))); break;
        case 12: writedataSpace((*reinterpret_cast< char*(*)>(_a[1]))); break;
        case 13: verifier_check(); break;
        case 14: threaderror((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void Mycom::comopen(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
