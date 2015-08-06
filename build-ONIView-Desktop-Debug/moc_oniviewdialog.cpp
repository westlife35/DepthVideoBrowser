/****************************************************************************
** Meta object code from reading C++ file 'oniviewdialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ONIView/oniviewdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'oniviewdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ONIViewDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x0a,
      35,   14,   14,   14, 0x0a,
      56,   14,   14,   14, 0x0a,
      82,   14,   14,   14, 0x0a,
     114,   14,   14,   14, 0x0a,
     156,  146,   14,   14, 0x0a,
     200,  195,   14,   14, 0x0a,
     250,   14,   14,   14, 0x0a,
     280,   14,   14,   14, 0x0a,
     328,   14,   14,   14, 0x0a,
     377,   14,   14,   14, 0x0a,
     417,   14,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_ONIViewDialog[] = {
    "ONIViewDialog\0\0updateImageForCam()\0"
    "updateImageForFile()\0updateForFileSliderShow()\0"
    "on_pushButton_openCam_clicked()\0"
    "on_pushButton_openOni_clicked()\0"
    "qstrSpeed\0on_comboBox_playSpeed_changed(QString)\0"
    "nPos\0on_horizontalSlider_playControl_valueChanged(int)\0"
    "on_pushButton_pause_clicked()\0"
    "on_horizontalSlider_playControl_sliderPressed()\0"
    "on_horizontalSlider_playControl_sliderReleased()\0"
    "on_pushButton_fullscreenDepth_clicked()\0"
    "on_pushButton_fullscreenColor_clicked()\0"
};

void ONIViewDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ONIViewDialog *_t = static_cast<ONIViewDialog *>(_o);
        switch (_id) {
        case 0: _t->updateImageForCam(); break;
        case 1: _t->updateImageForFile(); break;
        case 2: _t->updateForFileSliderShow(); break;
        case 3: _t->on_pushButton_openCam_clicked(); break;
        case 4: _t->on_pushButton_openOni_clicked(); break;
        case 5: _t->on_comboBox_playSpeed_changed((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->on_horizontalSlider_playControl_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->on_pushButton_pause_clicked(); break;
        case 8: _t->on_horizontalSlider_playControl_sliderPressed(); break;
        case 9: _t->on_horizontalSlider_playControl_sliderReleased(); break;
        case 10: _t->on_pushButton_fullscreenDepth_clicked(); break;
        case 11: _t->on_pushButton_fullscreenColor_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ONIViewDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ONIViewDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ONIViewDialog,
      qt_meta_data_ONIViewDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ONIViewDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ONIViewDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ONIViewDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ONIViewDialog))
        return static_cast<void*>(const_cast< ONIViewDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ONIViewDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
