QT       += core gui sql widgets network
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
    changesetting.cpp \
    main.cpp \
    mainwidget.cpp \
    socketclient.cpp \
    tab1_button.cpp \
    tab2_set.cpp \
    tab3_cctv.cpp \
    tab4_tomato.cpp

HEADERS += \
    changesetting.h \
    mainwidget.h \
    socketclient.h \
    tab1_button.h \
    tab2_set.h \
    tab3_cctv.h \
    tab4_tomato.h

FORMS += \
    changesetting.ui \
    mainwidget.ui \
    tab1_button.ui \
    tab2_set.ui \
    tab3_cctv.ui \
    tab4_tomato.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    fonts.qrc \
    icons.qrc
