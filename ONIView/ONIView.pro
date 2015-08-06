#-------------------------------------------------
#
# Project created by QtCreator 2015-05-29T17:38:24
#
#-------------------------------------------------

#INCLUDEPATH　+=-I./openni/include/openni -I./ThirdParty/Eigen3.1.3
INCLUDEPATH  += ./ThirdParty/Eigen3.1.3 /usr/include/opencv
LIBS +=-L./openni/lib/openni/Linux-x86_64 -lOpenNI2 -lglog -lglut -lGL -lopencv_highgui -lopencv_core -lopencv_imgproc   #-L./../CamLevel/lib
QMAKE_LFLAGS += -Xlinker -rpath ./ #-rpath ./openni/lib




QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ONIView
TEMPLATE = app


SOURCES += main.cpp\
        oniviewdialog.cpp \
    LogicLevel/prime_sensor_frame_get.cpp \
    CamLevel/primesense_sensor.cpp \
    CamLevel/sensor.cpp \
    LogicLevel/rgbdframe.cpp \
    LogicLevel/log.cpp \
    LogicLevel/color_space.cpp \
    EventLevel/eventInfo.cpp

HEADERS  += \
    ../LogicLevel/prime_sensor_frame_get.h \
    LogicLevel/prime_sensor_frame_get.h \
    CamLevel/depth_sensor.h \
    CamLevel/primesense_sensor.h \
    CamLevel/sensor.h \
    oniviewdialog.h \
    LogicLevel/colorspace.h \
    LogicLevel/rgbdframe.h \
    LogicLevel/log.h \
    LogicLevel/color_space.h \
    EventLevel/eventInfo.h

FORMS    += oniviewdialog.ui

RESOURCES += \
    res/res.qrc
