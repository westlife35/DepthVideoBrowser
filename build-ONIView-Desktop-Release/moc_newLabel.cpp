/****************************************************************************
** Meta object code from reading C++ file 'newLabel.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ONIView/newLabel.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'newLabel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_NewLabel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      10,    9,    9,    9, 0x05,
      28,    9,    9,    9, 0x05,

 // slots: signature, parameters, type, tag, flags
      52,    9,    9,    9, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_NewLabel[] = {
    "NewLabel\0\0NewLabelClicked()\0"
    "NewLabelDoubleClicked()\0SlotTimerOut()\0"
};

void NewLabel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        NewLabel *_t = static_cast<NewLabel *>(_o);
        switch (_id) {
        case 0: _t->NewLabelClicked(); break;
        case 1: _t->NewLabelDoubleClicked(); break;
        case 2: _t->SlotTimerOut(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData NewLabel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject NewLabel::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_NewLabel,
      qt_meta_data_NewLabel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &NewLabel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *NewLabel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *NewLabel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_NewLabel))
        return static_cast<void*>(const_cast< NewLabel*>(this));
    return QLabel::qt_metacast(_clname);
}

int NewLabel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void NewLabel::NewLabelClicked()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void NewLabel::NewLabelDoubleClicked()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
QT_END_MOC_NAMESPACE
