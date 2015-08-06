/********************************************************************************
** Form generated from reading UI file 'oniviewdialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ONIVIEWDIALOG_H
#define UI_ONIVIEWDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ONIViewDialog
{
public:
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_3;
    QSlider *horizontalSlider_playControl;
    QHBoxLayout *horizontalLayout_5;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QPushButton *pushButton_pause;
    QPushButton *pushButton_fullscreenDepth;
    QGroupBox *groupBox_speed;
    QGridLayout *gridLayout;
    QComboBox *comboBox_playSpeed;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_4;
    QSpacerItem *horizontalSpacer;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_frameNoShow;
    QLabel *label_totalFrameNumShow;
    QSpacerItem *horizontalSpacer_2;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *pushButton_fullscreenColor;
    QHBoxLayout *horizontalLayout_6;
    QPushButton *pushButton_openOni;
    QPushButton *pushButton_openCam;
    QPushButton *pushButton_quit;
    QLabel *label_ONIShowColor;
    QLabel *label_ONIShowDepth;

    void setupUi(QDialog *ONIViewDialog)
    {
        if (ONIViewDialog->objectName().isEmpty())
            ONIViewDialog->setObjectName(QString::fromUtf8("ONIViewDialog"));
        ONIViewDialog->resize(1310, 710);
        ONIViewDialog->setMinimumSize(QSize(1310, 672));
        ONIViewDialog->setCursor(QCursor(Qt::ArrowCursor));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        ONIViewDialog->setWindowIcon(icon);
        ONIViewDialog->setModal(false);
        layoutWidget = new QWidget(ONIViewDialog);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 500, 1281, 166));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget);
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalSlider_playControl = new QSlider(layoutWidget);
        horizontalSlider_playControl->setObjectName(QString::fromUtf8("horizontalSlider_playControl"));
        horizontalSlider_playControl->setOrientation(Qt::Horizontal);

        verticalLayout_3->addWidget(horizontalSlider_playControl);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        pushButton_pause = new QPushButton(layoutWidget);
        pushButton_pause->setObjectName(QString::fromUtf8("pushButton_pause"));

        verticalLayout->addWidget(pushButton_pause);

        pushButton_fullscreenDepth = new QPushButton(layoutWidget);
        pushButton_fullscreenDepth->setObjectName(QString::fromUtf8("pushButton_fullscreenDepth"));

        verticalLayout->addWidget(pushButton_fullscreenDepth);


        horizontalLayout->addLayout(verticalLayout);

        groupBox_speed = new QGroupBox(layoutWidget);
        groupBox_speed->setObjectName(QString::fromUtf8("groupBox_speed"));
        groupBox_speed->setMinimumSize(QSize(135, 62));
        groupBox_speed->setMaximumSize(QSize(135, 62));
        groupBox_speed->setStyleSheet(QString::fromUtf8("QGroupBox{\n"
"	border-width:2px;\n"
"	border-style:solid;\n"
"	border-color:white;\n"
"	margin-top:0.5ex;\n"
"}\n"
"QGroupBox:title{\n"
"	subcontrol-origin:margin;\n"
"	subcontrol-position:top left;\n"
"	left:10px;\n"
"	margin-left:0px;\n"
"	padding:0 1px;\n"
"}"));
        gridLayout = new QGridLayout(groupBox_speed);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        comboBox_playSpeed = new QComboBox(groupBox_speed);
        comboBox_playSpeed->setObjectName(QString::fromUtf8("comboBox_playSpeed"));
        comboBox_playSpeed->setFocusPolicy(Qt::StrongFocus);
        comboBox_playSpeed->setEditable(true);

        gridLayout->addWidget(comboBox_playSpeed, 0, 0, 1, 1);


        horizontalLayout->addWidget(groupBox_speed);


        horizontalLayout_5->addLayout(horizontalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_frameNoShow = new QLabel(layoutWidget);
        label_frameNoShow->setObjectName(QString::fromUtf8("label_frameNoShow"));
        label_frameNoShow->setMinimumSize(QSize(67, 15));
        label_frameNoShow->setMaximumSize(QSize(67, 15));
        label_frameNoShow->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 127);"));
        label_frameNoShow->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        horizontalLayout_2->addWidget(label_frameNoShow);

        label_totalFrameNumShow = new QLabel(layoutWidget);
        label_totalFrameNumShow->setObjectName(QString::fromUtf8("label_totalFrameNumShow"));
        label_totalFrameNumShow->setMinimumSize(QSize(67, 15));
        label_totalFrameNumShow->setMaximumSize(QSize(67, 15));
        label_totalFrameNumShow->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 127);"));

        horizontalLayout_2->addWidget(label_totalFrameNumShow);


        horizontalLayout_4->addLayout(horizontalLayout_2);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        verticalLayout_2->addItem(horizontalSpacer_2);


        horizontalLayout_5->addLayout(verticalLayout_2);


        verticalLayout_3->addLayout(horizontalLayout_5);


        horizontalLayout_8->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);

        pushButton_fullscreenColor = new QPushButton(layoutWidget);
        pushButton_fullscreenColor->setObjectName(QString::fromUtf8("pushButton_fullscreenColor"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(pushButton_fullscreenColor->sizePolicy().hasHeightForWidth());
        pushButton_fullscreenColor->setSizePolicy(sizePolicy);

        horizontalLayout_7->addWidget(pushButton_fullscreenColor);


        verticalLayout_4->addLayout(horizontalLayout_7);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        pushButton_openOni = new QPushButton(layoutWidget);
        pushButton_openOni->setObjectName(QString::fromUtf8("pushButton_openOni"));

        horizontalLayout_6->addWidget(pushButton_openOni);

        pushButton_openCam = new QPushButton(layoutWidget);
        pushButton_openCam->setObjectName(QString::fromUtf8("pushButton_openCam"));

        horizontalLayout_6->addWidget(pushButton_openCam);

        pushButton_quit = new QPushButton(layoutWidget);
        pushButton_quit->setObjectName(QString::fromUtf8("pushButton_quit"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton_quit->sizePolicy().hasHeightForWidth());
        pushButton_quit->setSizePolicy(sizePolicy1);

        horizontalLayout_6->addWidget(pushButton_quit);


        verticalLayout_4->addLayout(horizontalLayout_6);


        horizontalLayout_8->addLayout(verticalLayout_4);

        label_ONIShowColor = new QLabel(ONIViewDialog);
        label_ONIShowColor->setObjectName(QString::fromUtf8("label_ONIShowColor"));
        label_ONIShowColor->setGeometry(QRect(656, 10, 640, 480));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(label_ONIShowColor->sizePolicy().hasHeightForWidth());
        label_ONIShowColor->setSizePolicy(sizePolicy2);
        label_ONIShowColor->setMinimumSize(QSize(640, 480));
        label_ONIShowColor->setFrameShape(QFrame::WinPanel);
        label_ONIShowColor->setTextFormat(Qt::AutoText);
        label_ONIShowColor->setAlignment(Qt::AlignCenter);
        label_ONIShowDepth = new QLabel(ONIViewDialog);
        label_ONIShowDepth->setObjectName(QString::fromUtf8("label_ONIShowDepth"));
        label_ONIShowDepth->setGeometry(QRect(10, 10, 640, 480));
        sizePolicy2.setHeightForWidth(label_ONIShowDepth->sizePolicy().hasHeightForWidth());
        label_ONIShowDepth->setSizePolicy(sizePolicy2);
        label_ONIShowDepth->setMinimumSize(QSize(640, 480));
        label_ONIShowDepth->setSizeIncrement(QSize(0, 0));
        label_ONIShowDepth->setFrameShape(QFrame::WinPanel);
        label_ONIShowDepth->setTextFormat(Qt::AutoText);
        label_ONIShowDepth->setAlignment(Qt::AlignCenter);

        retranslateUi(ONIViewDialog);
        QObject::connect(pushButton_quit, SIGNAL(clicked()), ONIViewDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(ONIViewDialog);
    } // setupUi

    void retranslateUi(QDialog *ONIViewDialog)
    {
        ONIViewDialog->setWindowTitle(QApplication::translate("ONIViewDialog", "ONIView", 0, QApplication::UnicodeUTF8));
        pushButton_pause->setText(QApplication::translate("ONIViewDialog", "&Pause", 0, QApplication::UnicodeUTF8));
        pushButton_fullscreenDepth->setText(QApplication::translate("ONIViewDialog", "FullscreenDepth", 0, QApplication::UnicodeUTF8));
        groupBox_speed->setTitle(QApplication::translate("ONIViewDialog", "Speed", 0, QApplication::UnicodeUTF8));
        label_frameNoShow->setText(QString());
        label_totalFrameNumShow->setText(QString());
        pushButton_fullscreenColor->setText(QApplication::translate("ONIViewDialog", "FullscreenColor", 0, QApplication::UnicodeUTF8));
        pushButton_openOni->setText(QApplication::translate("ONIViewDialog", "Open&File", 0, QApplication::UnicodeUTF8));
        pushButton_openCam->setText(QApplication::translate("ONIViewDialog", "Open&Cam", 0, QApplication::UnicodeUTF8));
        pushButton_quit->setText(QApplication::translate("ONIViewDialog", "&Qiut", 0, QApplication::UnicodeUTF8));
        label_ONIShowColor->setText(QApplication::translate("ONIViewDialog", "Preview area", 0, QApplication::UnicodeUTF8));
        label_ONIShowDepth->setText(QApplication::translate("ONIViewDialog", "Preview area(depth)", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ONIViewDialog: public Ui_ONIViewDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ONIVIEWDIALOG_H
