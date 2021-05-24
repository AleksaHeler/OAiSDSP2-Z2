QT += core gui widgets

CONFIG += qt c++11


SOURCES += \
    ../../../src/ImageProcessing.cpp \
    ../../../src/main.cpp \
	../../../src/ColorSpaces.cpp \
	../../../src/FeatureDetection.cpp \
	../../../src/PredictSVM.cpp \
	../../../src/svm/svm.cpp 


HEADERS += \
    ../../../inc/ColorSpaces.h \
    ../../../inc/FeatureDetection.h \
    ../../../inc/ImageProcessing.h \
	../../../inc/PredictSVM.h \
	../../../inc/svm/svm.h 


INCLUDEPATH += ../../../lib_inc/ ../../../inc/ ../../../inc/svm


unix:!macx|win32: LIBS += -L$$PWD/../../../lib/windows-mingw/ -lImageDSPLib

win32:!win32-g++: PRE_TARGETDEPS += $$PWD/../../../lib/windows-mingw/ImageDSPLib.lib


