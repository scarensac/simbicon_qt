#-------------------------------------------------
#
# Project created by QtCreator 2015-09-23T10:08:46
#
#-------------------------------------------------

QT       -= core gui


CONFIG += object_parallel_to_source
CONFIG += no_batch
CONFIG += c++11
CONFIG += MD

TARGET = ode
TEMPLATE = lib
CONFIG += staticlib

DEFINES += dDOUBLE CCD_DOUBLE WIN32 _CRT_SECURE_NO_DEPRECATE _USE_MATH_DEFINES _OU_NAMESPACE=odeou ODE_LIB


QMAKE_CFLAGS +=  /wd4100
QMAKE_CXXFLAGS +=  /wd4100


SOURCES += ode\src\*.cpp \
        ode\src\joints\*.cpp \
        OPCODE\*.cpp \
        OPCODE\Ice\*.cpp \
        ou\src\ou\*.cpp \
        ou\test\*.cpp \
        ode\src\*.c \
        ode\src\joints\*.c \
        OPCODE\*.c \
        OPCODE\Ice\*.c \
        ou\src\ou\*.c \
        ou\test\*.c



HEADERS += include\ode\*.h \
        include\*.h \
        ode\src\*.h \
        ode\src\joints\*.h \
        OPCODE\*.h \
        GIMPACT\include\*.h \
        libccd\src\*.h \
        ou\include\*.h

INCLUDEPATH += include \
        ode\src \
        ode\src\joints \
        OPCODE \
        GIMPACT\include \
        libccd\src \
        ou\include

# Define output directories
Release:DESTDIR = lib\release
Release:OBJECTS_DIR = lib\release\obj

Debug:DESTDIR = lib\debug
Debug:OBJECTS_DIR = lib\debug\obj



