QT += core

CONFIG += qt c++11


SOURCES += \
    ../../../src/SVMConsoleMain.cpp \
	../../../src/ColorSpaces.cpp \
	../../../src/FeatureDetection.cpp \
	../../../src/DataBase.cpp \
    ../../../src/TrainSVM.cpp \
	../../../src/PredictSVM.cpp \
	../../../src/svm/svm.cpp


HEADERS += \
    ../../../inc/ColorSpaces.h \
    ../../../inc/FeatureDetection.h \
	../../../inc/PredictSVM.h \
	../../../inc/DataBase.h \
	../../../inc/TrainSVM.h \
	../../../inc/svm/svm.h 


INCLUDEPATH += ../../../lib_inc/ ../../../inc/ ../../../inc/svm




