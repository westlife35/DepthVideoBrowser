#ifndef ONIVIEWDIALOG_H
#define ONIVIEWDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QMouseEvent>
#include "LogicLevel/prime_sensor_frame_get.h"
#include <QApplication>
#include <QDesktopWidget>
#include<QInputDialog>
using namespace dg_controls;
class Qtimer;
class QPaintEvent;
class QMessageBox;
class QImage;

namespace Ui {
  class ONIViewDialog;
}

class ONIViewDialog : public QDialog
{
  Q_OBJECT

public:
  explicit ONIViewDialog(QWidget *parent = 0);
  ~ONIViewDialog();

public:
  QTimer theTimerForCam;
  QTimer theTimerForFile;
  QTimer theTimerForFileSliderShow;
  class ONICapture oniCapture;
  bool bIsOpenCam;
  bool bIsOpenFile;
  bool bIsCamPicInit;//because when the first time to show pic, there is no pic.
  bool bIsFilePicInit;
  int nNumOfFrameForFile;
  int nNoOfFrameForFile;
  bool bIsPausedForFile;
  bool bIsNeedSeekToExcludePauseShake;
  bool bIsSilderPressed;
  bool bIsDepthFullscreen;
  bool bIsColorFullscreen;
  QRect mainDialogGeometry;
  int horizontalSliderPlayControlLastValue;
  //by hand, fix the bug that the on_horizontalSlider_playControl_valueChanged() will be evaluated twice.
  int flag;
  //by hand, fix the bug that when paused and do some thing, reset the focus to the slider, on_horizontalSlider_playControl_valueChanged() the  will be evaluated about the back play.
  //int flagSliderPaused;
  //bool bIsDeleteIDMode;
public:
  void paintEvent(QPaintEvent *e);
  //bool eventFilter(QObject *obj, QEvent *event);
  void keyPressEvent(QKeyEvent * e);
  //void mousePressEvent(QMouseEvent * e);

public slots:
  void updateImageForCam();
  void updateImageForFile();
  void updateForFileSliderShow();
  void on_pushButton_openCam_clicked();
  void on_pushButton_openOni_clicked();
  void on_comboBox_playSpeed_changed(QString qstrSpeed);
  void on_horizontalSlider_playControl_valueChanged(int nPos);
  void on_pushButton_pause_clicked();
  void on_horizontalSlider_playControl_sliderPressed();
  void on_horizontalSlider_playControl_sliderReleased();
  void on_pushButton_fullscreenDepth_clicked();
  void on_pushButton_fullscreenColor_clicked();
  //void on_pushButton_setFocusOnSlider_clicked();

//  void on_checkBox_isDelete_clicked();


private:
  Ui::ONIViewDialog *ui;
};

#endif // ONIVIEWDIALOG_H
