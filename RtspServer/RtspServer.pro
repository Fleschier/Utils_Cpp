#-------------------------------------------------
#
# Project created by QtCreator 2022-04-08T15:27:07
#
#-------------------------------------------------

QT       -= gui

TARGET = RtspServer
TEMPLATE = lib

DEFINES += RTSPSERVER_LIBRARY

QMAKE_CXXFLAGS += -std=c++14

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


INCLUDEPATH += /usr/local/include       \
            /usr/local/ffmpeg/include \
            ./include/

LIBS += /usr/local/lib/libopencv_*.so
LIBS += -L./lib/ -lAVH264Encoder


SOURCES += \
    net/Acceptor.cpp \
    net/BufferReader.cpp \
    net/BufferWriter.cpp \
    net/EpollTaskScheduler.cpp \
    net/EventLoop.cpp \
    net/Logger.cpp \
    net/MemoryManager.cpp \
    net/NetInterface.cpp \
    net/Pipe.cpp \
    net/SelectTaskScheduler.cpp \
    net/SocketUtil.cpp \
    net/TaskScheduler.cpp \
    net/TcpConnection.cpp \
    net/TcpServer.cpp \
    net/TcpSocket.cpp \
    net/Timer.cpp \
    net/Timestamp.cpp \
    xop/AACSource.cpp \
    xop/DigestAuthentication.cpp \
    xop/G711ASource.cpp \
    xop/H264Parser.cpp \
    xop/H264Source.cpp \
    xop/H265Source.cpp \
    xop/MediaSession.cpp \
    xop/RtpConnection.cpp \
    xop/RtspConnection.cpp \
    xop/RtspMessage.cpp \
    xop/RtspPusher.cpp \
    xop/RtspServer.cpp \
    xop/VP8Source.cpp \
    mrtspserver.cpp

HEADERS += \
        rtspserver_global.h \ 
    md5/md5.hpp \
    net/Acceptor.h \
    net/BufferReader.h \
    net/BufferWriter.h \
    net/Channel.h \
    net/EpollTaskScheduler.h \
    net/EventLoop.h \
    net/log.h \
    net/Logger.h \
    net/MemoryManager.h \
    net/NetInterface.h \
    net/Pipe.h \
    net/RingBuffer.h \
    net/SelectTaskScheduler.h \
    net/Socket.h \
    net/SocketUtil.h \
    net/TaskScheduler.h \
    net/TcpConnection.h \
    net/TcpServer.h \
    net/TcpSocket.h \
    net/Timer.h \
    net/Timestamp.h \
    xop/AACSource.h \
    xop/DigestAuthentication.h \
    xop/G711ASource.h \
    xop/H264Parser.h \
    xop/H264Source.h \
    xop/H265Source.h \
    xop/media.h \
    xop/MediaSession.h \
    xop/MediaSource.h \
    xop/rtp.h \
    xop/RtpConnection.h \
    xop/rtsp.h \
    xop/RtspConnection.h \
    xop/RtspMessage.h \
    xop/RtspPusher.h \
    xop/RtspServer.h \
    xop/VP8Source.h \
    rtspserver_global.h \
    mrtspserver.h \

unix {
    target.path = /usr/lib
    INSTALLS += target
}
