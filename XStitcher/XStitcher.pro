#-------------------------------------------------
#
# Project created by QtCreator 2022-07-04T08:48:43
#
#-------------------------------------------------

QT       -= gui

TARGET = XStitcher
TEMPLATE = lib

DEFINES += XSTITCHER_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS += /usr/local/lib/libopencv*.so
LIBS += -L./lib/ -lThreadPool
LIBS += -L./lib/ -lRtspServer
LIBS += -L./lib/ -lAVH264Encoder
#LIBS += /usr/local/ffmpeg/lib/lib*.so

LIBS += -L./release_mpp_rk3588_aarch64-rockchip1031-linux-gnu/lib/ -lrockchip_mpp

INCLUDEPATH += ./release_mpp_rk3588_aarch64-rockchip1031-linux-gnu/include/rockchip
DEPENDPATH += ./release_mpp_rk3588_aarch64-rockchip1031-linux-gnu/include/rockchip

LIBS += -L./release_mpp_rk3588_aarch64-rockchip1031-linux-gnu/lib/ -lrockchip_vpu

INCLUDEPATH += /usr/local/include       \
            /usr/local/include/opencv4  \
            ./include/

SOURCES += \
        xstitcher.cpp

HEADERS += \
        xstitcher.h \
        xstitcher_global.h 

unix {
    target.path = /usr/lib
    INSTALLS += target
}
