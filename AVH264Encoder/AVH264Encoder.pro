#-------------------------------------------------
#
# Project created by QtCreator 2022-04-14T17:15:33
#
#-------------------------------------------------

QT       -= gui

TARGET = AVH264Encoder
TEMPLATE = lib

DEFINES += AVH264ENCODER_LIBRARY

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

FFMPEG_PATH = /usr/local/ffmpeg

INCLUDEPATH += $$FFMPEG_PATH/include  \
            += /usr/local/include

LIBS += /usr/local/lib/libopencv_*.so
LIBS += $$FFMPEG_PATH/lib/lib*.so


SOURCES += \
        avh264encoder.cpp

HEADERS += \
        avh264encoder.h \
        avh264encoder_global.h 

unix {
    target.path = /usr/lib
    INSTALLS += target
}
