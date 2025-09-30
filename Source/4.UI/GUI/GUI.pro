QT       += core gui sql

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
# OpenCV include path
INCLUDEPATH += /usr/include/opencv4

# OpenCV 라이브러리 링크
LIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    tab1_control.cpp \
    tab2_cropstatus.cpp \
    tab3_temhumiilludb.cpp \
    tab4_camera.cpp \
    tab_widget.cpp

HEADERS += \
    tab1_control.h \
    tab2_cropstatus.h \
    tab3_temhumiilludb.h \
    tab4_camera.h \
    tab_widget.h

FORMS += \
    tab1_control.ui \
    tab2_cropstatus.ui \
    tab3_temhumiilludb.ui \
    tab4_camera.ui \
    tab_widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
