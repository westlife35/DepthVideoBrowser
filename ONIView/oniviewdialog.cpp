//20150615 go to a new branch:ONIBrowser-Tracking
#include "oniviewdialog.h"
#include "ui_oniviewdialog.h"

ONIViewDialog::ONIViewDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::ONIViewDialog)
{
  ui->setupUi(this);
  setWindowFlags(windowFlags() & Qt::WindowMinimizeButtonHint & ~Qt::WindowMinimizeButtonHint );
  //setFixedSize(this->width(), this->height());

  bIsOpenCam=false;
  bIsOpenFile=false;
  bIsCamPicInit=false;
  bIsFilePicInit=false;
  nNumOfFrameForFile=0;
  nNoOfFrameForFile=0;
  bIsPausedForFile=false;
  bIsNeedSeekToExcludePauseShake=false;
  bIsSilderPressed=false;
  ui->label_ONIShowColor->setScaledContents(true);

  ui->comboBox_playSpeed->setEditable(true);
  ui->comboBox_playSpeed->insertItem(0,"Normal");
  ui->comboBox_playSpeed->insertItem(1,"0.3");
  ui->comboBox_playSpeed->insertItem(2,"0.5");
  ui->comboBox_playSpeed->insertItem(3,"3");
  ui->comboBox_playSpeed->insertItem(4,"5");
  ui->comboBox_playSpeed->insertItem(5,"10");
  connect(ui->comboBox_playSpeed,SIGNAL(currentIndexChanged(QString)),this,SLOT(on_comboBox_playSpeed_changed(QString)) );
  ui->horizontalSlider_playControl->setTracking(false);

  bIsDepthFullscreen=false;
  bIsColorFullscreen=false;

//  debug
  //this->setWindowFlags(Qt::WindowSystemMenuHint);
  //this->setWindowFlags(Qt::FramelessWindowHint);
  //this->setWindowState( Qt::WindowFullScreen );

//  debug
//  ui->horizontalSlider_playControl->setRange(0,3000);
//  ui->horizontalSlider_playControl->setSliderPosition(1500);
//  int a=0;

  //test the function of tracking mouse
  ui->label_ONIShowColor->setMouseTracking(true);

  flag=0;
  bIsDeleteIDMode=false;
  connect(ui->checkBox_isDelete, SIGNAL(clicked()), this, SLOT(on_checkBox_isDelete_clicked()));
}

ONIViewDialog::~ONIViewDialog()
{
//  delete depthImage;
//  delete colorImage;
  if(bIsOpenCam)
  {
      bIsOpenCam=false;
      oniCapture.Close();
      theTimerForCam.stop();
  }
  if(bIsOpenFile)
  {
      bIsOpenFile=false;
      oniCapture.Close();
      theTimerForFile.stop();
  }
  delete ui;
}

void ONIViewDialog::paintEvent(QPaintEvent *e)
{
  //show the frame get from logic level in updateImage
  if( (bIsOpenCam&&bIsCamPicInit) || (bIsOpenFile&&bIsFilePicInit)  )
  {
      //color show
      unsigned char *colorData=(unsigned char *)oniCapture.GetColorData();
      QImage colorImage=QImage(colorData,oniCapture.GetColorWidth(),oniCapture.GetColorHeight(),QImage::Format_RGB888);
      QPixmap pixmapColor;
      pixmapColor.convertFromImage(colorImage);
      ui->label_ONIShowColor->setScaledContents(true);
      ui->label_ONIShowColor->setPixmap(pixmapColor);
      //depth show
      unsigned char *depthData=(unsigned char *)oniCapture.GetColorFromDepthData();
      QImage depthImage=QImage(depthData,oniCapture.GetDepthWidth(),oniCapture.GetDepthHeight(),QImage::Format_RGB888);
      QPixmap pixmapDepth;
      pixmapDepth.convertFromImage(depthImage);
      ui->label_ONIShowDepth->setScaledContents(true);
      ui->label_ONIShowDepth->setPixmap(pixmapDepth);
   }
}

void ONIViewDialog::updateImageForCam()
{
  oniCapture.Update();
  if(!bIsCamPicInit)
  {
      bIsCamPicInit=true;
      this->repaint();
  }
}

void ONIViewDialog::updateImageForFile()
{

//  oniCapture.Update();
//  if(!bIsFilePicInit)
//  {
//      bIsFilePicInit=true;
//      this->repaint();
//  }


  if(!bIsPausedForFile)
  {
    //exclude the shake after pause to play but make the play speed seem to faster than before
    if(bIsNeedSeekToExcludePauseShake)
    {
        oniCapture.SeekColorStream(nNoOfFrameForFile);
        //add SeekDepthStream
        //oniCapture.SeekDepthStream(nNoOfFrameForFile);
        bIsNeedSeekToExcludePauseShake=false;
    }
    oniCapture.bIsBackPlayOrForwardPlay=false;
    oniCapture.Update();
    //this sencentance takes some time and make the playing is stucking
//    ui->horizontalSlider_playControl->setSliderPosition(nNoOfFrameForFile);
//    ui->horizontalSlider_playControl->setValue(nNoOfFrameForFile);
    //debug for the bug of slider
//    nNoOfFrameForFile++;
    nNoOfFrameForFile=oniCapture.GetFrameIndex();


    if(nNoOfFrameForFile>=nNumOfFrameForFile)
    {
        nNoOfFrameForFile=0;
    }
    if(!bIsFilePicInit)
    {
        bIsFilePicInit=true;
        this->repaint();
    }
  }
}

void ONIViewDialog::updateForFileSliderShow()
{
  ui->horizontalSlider_playControl->setSliderPosition(nNoOfFrameForFile);
  ui->label_frameNoShow->setText(QString::number(nNoOfFrameForFile+1));
  //ui->horizontalSlider_playControl->setSliderPosition(oniCapture.GetFrameIndex());
  //ui->label_frameNoShow->setText(QString::number(oniCapture.GetFrameIndex()));
}

void ONIViewDialog::on_pushButton_openCam_clicked()
{
  if(bIsOpenFile)
  {
      QMessageBox::information(NULL, QString("Info"), QString("There is a file opened, please close it and then open the cam."));
      return;
  }
  if(!bIsOpenCam)
  {
      if(!oniCapture.Open() )
      {
            QMessageBox::information(NULL, QString("Info"), QString("Fail to open cam"));
            return;
      }
      bIsOpenCam=true;
      connect(&theTimerForCam, SIGNAL(timeout()), this, SLOT(updateImageForCam()) );
      theTimerForCam.start(33);
      ui->pushButton_openCam->setText("&CloseCam");
      //cout<<oniCapture.GetColorHeight()<<"+"<<oniCapture.GetColorWidth()<<"+"<<oniCapture.GetDepthHeight()<<"+"<<oniCapture.GetDepthWidth()<<endl;
  }
  else
  {
      bIsOpenCam=false;
      bIsCamPicInit=false;
      oniCapture.Close();
      theTimerForCam.stop();
      ui->pushButton_openCam->setText("&OpenCam");
  }
}

void ONIViewDialog::on_pushButton_openOni_clicked()
{
  if(bIsOpenCam)
  {
      QMessageBox::information(NULL, QString("Info"), QString("There is a cam opened, please close it and then open the file."));
      return;
  }

  //QMessageBox::information(NULL, QString("Info"), qstrFilePath);
  if(!bIsOpenFile)
  {
      QString qstrFilePath = QFileDialog::getOpenFileName(this,"Choose *.oni","/home/qc/","ONI (*.oni)" );
      if(qstrFilePath=="")
      {
        return;
      }
      QString qstrFileName=qstrFilePath.right(qstrFilePath.length()-qstrFilePath.lastIndexOf("/")-1);
      qstrFileName=qstrFileName.left(qstrFileName.length()-4);
      if(!oniCapture.Open(qstrFileName.toStdString(),qstrFilePath.toStdString()))
      {
            QMessageBox::information(NULL, QString("Info"), "Fail to open file: "+qstrFileName);
            return;
      }
      bIsOpenFile=true;
      //set slider
      ui->horizontalSlider_playControl->setMinimum(0);
      ui->horizontalSlider_playControl->setMaximum(oniCapture.GetNumberOfColorFrames());//the number of color frame equals to the number of depth frame
//      ui->horizontalSlider_playControl->setRange(0,oniCapture.GetNumberOfColorFrames());
      nNumOfFrameForFile=oniCapture.GetNumberOfColorFrames();
      ui->label_totalFrameNumShow->setText(" / "+QString::number(nNumOfFrameForFile));
      nNoOfFrameForFile=0;

      connect(ui->horizontalSlider_playControl, SIGNAL(valueChanged(int)), this, SLOT(on_horizontalSlider_playControl_valueChanged(int)));
      connect(ui->horizontalSlider_playControl, SIGNAL(sliderPressed()), this, SLOT(on_horizontalSlider_playControl_sliderPressed()));
      connect(ui->horizontalSlider_playControl, SIGNAL(sliderReleased()), this, SLOT(on_horizontalSlider_playControl_sliderReleased()));

      //set timer to play oni
      connect(&theTimerForFile, SIGNAL(timeout()), this, SLOT(updateImageForFile()) );
      theTimerForFile.start(33);
      connect(&theTimerForFileSliderShow ,SIGNAL(timeout()), this, SLOT(updateForFileSliderShow()) );
      theTimerForFileSliderShow.start(5);
      ui->pushButton_openOni->setText("&CloseFile");
      on_comboBox_playSpeed_changed(ui->comboBox_playSpeed->currentText());
      //cout<<oniCapture.GetColorHeight()<<"+"<<oniCapture.GetColorWidth()<<"+"<<oniCapture.GetDepthHeight()<<"+"<<oniCapture.GetDepthWidth()<<endl;
  }
  else
  {
      bIsOpenFile=false;
      bIsFilePicInit=false;
      theTimerForFile.stop();
      oniCapture.SetNewOniFlag();
      oniCapture.Close();      
      ui->pushButton_openOni->setText("&OpenFile");
  }
}

void ONIViewDialog::on_comboBox_playSpeed_changed(QString qstrSpeed)
{
  ui->comboBox_playSpeed->setFocus();
  //QMessageBox::information(NULL, QString("Info"), qstrSpeed);
  if(qstrSpeed=="Normal")
  {
        oniCapture.SetSpeed(1);
        return;
  }
  QByteArray ba = qstrSpeed.toLatin1();//QString 转换为 char*
  const char *s = ba.data();
  while( (*s && *s>='0' && *s<='9' ) || (*s && *s=='.') )
    s++;
  if (*s)
  {
    QMessageBox::information(NULL, QString("Info"), "The speed should be number.");
    return;
  }
  oniCapture.SetSpeed(qstrSpeed.toFloat());
}

void ONIViewDialog::on_horizontalSlider_playControl_valueChanged(int nPos)
{
  if(bIsOpenFile)
  {
      //  if(bIsPausedForFile)
      //  {
      //      flagSliderPaused=0;
      //      return;
      //  }
      //  if(flagSliderPaused!=0)
      //    return;

        if(horizontalSliderPlayControlLastValue>nPos)//if back playing
        {
            //all the tracked people`s info are stored in a vector. Use the info in that to reset the tracking state. If back playing, reset. If forward palying, merge the result.
            cout<<"back play"<<endl;
            horizontalSliderPlayControlLastValue=nPos;
            oniCapture.bIsBackPlayOrForwardPlay=true;
            oniCapture.SetTrackingStateToCurrentFrame();
            //by hand, fix the bug that the on_horizontalSlider_playControl_valueChanged() will be evaluated twice.
            flag=2;//2 means back play just now
        }
        else if(horizontalSliderPlayControlLastValue<nPos)
        {
            //when back, there will be an event of forward play.

            if(flag==0)
            {
                cout<<"forward play"<<endl;
                horizontalSliderPlayControlLastValue=nPos;
                oniCapture.bIsBackPlayOrForwardPlay=false;
            }
            flag=0;
        }
        //one setting to depth or color is enough to make the depth and color to both be setted
        oniCapture.SeekColorStream(nPos);
        //add SeekDepthStream
        //oniCapture.SeekDepthStream(nPos);
        nNoOfFrameForFile=nPos;

        //all the tracked people`s info are stored in a vector. Use the info in that to reset the tracking state. If back playing, reset. If forward palying, merge the result.
        oniCapture.Update();
        //paintEvent(0);
  }
}

void ONIViewDialog::on_pushButton_pause_clicked()
{
  if(bIsOpenFile)
  {
      if(!bIsPausedForFile)
      {//make play pause
          bIsPausedForFile=true;
          ui->pushButton_pause->setText("&Play");
          bIsNeedSeekToExcludePauseShake=true;
          setCursor(Qt::CrossCursor);
      }
      else
      {//make play reset
          bIsPausedForFile=false;
          ui->pushButton_pause->setText("&Pause");
          setCursor(Qt::ArrowCursor);
      }
  }

}

void ONIViewDialog::on_horizontalSlider_playControl_sliderPressed()
{
//  bIsSilderPressed=true;
//  bIsPausedForFile=true;
  if(bIsOpenFile)
  {
    ui->horizontalSlider_playControl->setTracking(true);
  }
}

void ONIViewDialog::on_horizontalSlider_playControl_sliderReleased()
{
//  bIsSilderPressed=false;
//  bIsPausedForFile=false;
  if(bIsOpenFile)
  {
    ui->horizontalSlider_playControl->setTracking(false);
  }
}

void ONIViewDialog::keyPressEvent(QKeyEvent * e)
{
   if(e->key() == Qt::Key_Enter)
  {
      on_comboBox_playSpeed_changed(ui->comboBox_playSpeed->currentText());
  }
   if(e->key() == Qt::Key_Escape)
   {
    if(bIsColorFullscreen)
    {
      bIsColorFullscreen=false;
      ui->label_ONIShowColor->setGeometry(656,10,640,480);
      this->resize(1310,472);
      this->setWindowFlags(Qt::SubWindow);
      this->showNormal();
      this->move(200,150);
      ui->label_ONIShowDepth->show();
      //this->repaint();
    }
    if(bIsDepthFullscreen)
    {
        bIsDepthFullscreen=false;
        ui->label_ONIShowDepth->setGeometry(10,10,640,480);
        this->resize(1310,472);
        this->setWindowFlags(Qt::SubWindow);
        this->showNormal();
        this->move(200,150);
        //this->repaint();
    }
   }
}

void ONIViewDialog::mousePressEvent(QMouseEvent * e)
{
  if(bIsPausedForFile)
  {
      if(e->button()==Qt::LeftButton)
      {
        QPoint colorLabelPos=ui->label_ONIShowColor->pos();
        int nColorLabelWidth=ui->label_ONIShowColor->width();
        int nColorLabelHeight=ui->label_ONIShowColor->height();
        QPoint clickedPoint=e->pos();
        int x=clickedPoint.x();
        int y=clickedPoint.y();
        if( (colorLabelPos.x()<x && x<colorLabelPos.x()+nColorLabelWidth ) && (colorLabelPos.y()<y && y<colorLabelPos.y()+nColorLabelHeight) )
        {
          QMessageBox::information(NULL, QString("Info"), QString::number(x-colorLabelPos.x(), 10)+","+QString::number(y-colorLabelPos.y(), 10));
          int nIndex=oniCapture.IsFindNearstTrackGoal(x-colorLabelPos.x(),y-colorLabelPos.y());
          if(nIndex>=0)
          {
              if(bIsDeleteIDMode)
              {
                oniCapture.DeletePeopleByIndex(nIndex);
              }
              else
              {
                  bool isOK;
                  QString text = QInputDialog::getText(NULL, "Input Dialog",
                                                                     "Please input the ID you want to change to",
                                                                     QLineEdit::Normal,
                                                                     "ID you want to change to",
                                                                     &isOK);
                  if(isOK) {
                    oniCapture.SetTrackIDByHand(text.toInt(),nIndex);
                  }
              }

          }
          else
          {
              oniCapture.AddTrackPosByHand(x-colorLabelPos.x(),y-colorLabelPos.y());
          }
//          //fix the user`s feelings
//          bIsPausedForFile=false;
//          int nTempPos=oniCapture.GetFrameIndex();
//          //oniCapture.SeekColorStream(nNoOfFrameForFile);
//          oniCapture.SeekColorStream(nTempPos+1);
//          //add SeekDepthStream
//          //oniCapture.SeekDepthStream(nNoOfFrameForFile);
//          oniCapture.SeekDepthStream(nTempPos+1);
//          updateImageForFile();
//          bIsPausedForFile=true;
        }
      }
  }
}

void ONIViewDialog::on_pushButton_fullscreenDepth_clicked()
{
  //QMessageBox::information(NULL, QString("Info"), "adad");
//  ui->label_ONIShowDepth->showFullScreen();
  if(bIsOpenFile)
  {
      if(!bIsDepthFullscreen)
      {
          bIsDepthFullscreen=true;
          this->setWindowFlags(Qt::FramelessWindowHint);
          this->setWindowState( Qt::WindowFullScreen );
          this->show();
          ui->label_ONIShowDepth->showFullScreen();
          ui->label_ONIShowDepth->setGeometry(0,0,QApplication::desktop()->width(),QApplication::desktop()->height());
      }
  }
}

void ONIViewDialog::on_pushButton_fullscreenColor_clicked()
{
  if(bIsOpenFile)
  {
      if(!bIsColorFullscreen)
      {
          bIsColorFullscreen=true;
          this->setWindowFlags(Qt::FramelessWindowHint);
          this->setWindowState( Qt::WindowFullScreen );
          this->show();
          ui->label_ONIShowColor->showFullScreen();
          ui->label_ONIShowColor->setGeometry(0,0,QApplication::desktop()->width(),QApplication::desktop()->height());
          ui->label_ONIShowDepth->hide();
      }
  }
}

void ONIViewDialog::on_checkBox_isDelete_clicked()
{
  bIsDeleteIDMode=ui->checkBox_isDelete->isChecked();
//    bool b=ui->checkBox_isDelete->isChecked();
//    if(b)
//    {
//        QMessageBox::information(NULL, QString("Info"), QString::number(1, 10));
//    }
//    else
//    {
//        QMessageBox::information(NULL, QString("Info"), QString::number(0, 10));
//    }

}

//void ONIViewDialog::on_pushButton_setFocusOnSlider_clicked()
//{
//  ui->horizontalSlider_playControl->setFocus();
//  ui->horizontalSlider_playControl->setSliderPosition(1000);
//}


//bool ONIViewDialog::eventFilter(QObject *obj, QEvent *event)
//{
//  switch(event->type())
//  {
//  case QEvent::MouseButtonPress:
//              {
//                  if(((QMouseEvent *)event)->button() == Qt::LeftButton)
//                  {
//                      int max = ui->horizontalSlider_playControl->maximum();
//                      int min = ui->horizontalSlider_playControl->minimum();
//                      int pointPos = ((double)((QMouseEvent *)event)->x()) / ((QSlider*)obj)->width()*(max - min)+min;
//                      if(abs(pointPos - (ui->horizontalSlider_playControl->value()) )> (ui->horizontalSlider_playControl->pageStep()) )
//                      {
//                          ((QSlider*)obj)->setValue(pointPos);
//                          return true;
//                      }
//                      else
//                      {
//                          return false;
//                      }
//                  }
//                  else
//                      return false;
//             }
//  }
//}



