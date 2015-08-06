#-------------------------------------------------
#
# Project created by QtCreator 2015-05-29T17:38:24
#
#-------------------------------------------------

#INCLUDEPATH　+=-I./openni/include/openni -I./ThirdParty/Eigen3.1.3
INCLUDEPATH  += ./ThirdParty/Eigen3.1.3
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
    TrackingLevel/basic_geometry.cpp \
    TrackingLevel/colormap.cpp \
    TrackingLevel/depth_MoG_background.cpp \
    TrackingLevel/floor_calib.cpp \
    TrackingLevel/location_tracker.cpp \
    TrackingLevel/planview.cpp \
    TrackingLevel/Tracker.cpp \
    TrackingLevel/back_projection.cpp \
    LogicLevel/rgbdframe.cpp \
    LogicLevel/log.cpp \
    TrackingLevel/util/color_space.cpp \
    TrackingLevel/util/tools.cc

HEADERS  += \
    ../LogicLevel/prime_sensor_frame_get.h \
    LogicLevel/prime_sensor_frame_get.h \
    CamLevel/depth_sensor.h \
    CamLevel/primesense_sensor.h \
    CamLevel/sensor.h \
    oniviewdialog.h \
    LogicLevel/colorspace.h \
    TrackingLevel/basic_geometry.h \
    TrackingLevel/colormap.hpp \
    TrackingLevel/depth_MoG_background.h \
    TrackingLevel/floor_calib.h \
    TrackingLevel/location_tracker.h \
    TrackingLevel/planview.h \
    TrackingLevel/Tracker.h \
    TrackingLevel/back_projection.h \
    TrackingLevel/util/common_things.h \
    LogicLevel/rgbdframe.h \
    LogicLevel/log.h \
    TrackingLevel/util/color_space.h \
    TrackingLevel/util/tools.h

FORMS    += oniviewdialog.ui

RESOURCES += \
    res/res.qrc
