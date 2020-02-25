#-------------------------------------------------
#
# Project created by QtCreator 2015-09-17T16:48:02
#
#-------------------------------------------------

CONFIG += object_parallel_to_source
CONFIG += no_batch
#CONFIG += FLUID_COMPIL
CONFIG += SHARK_UTILISATION
CONFIG += BOOST_UTILISATION
#CONFIG += MT
CONFIG += MD
CONFIG += CUDA8
#CONFIG += CUDA10

SHARK_UTILISATION{
    CONFIG += BOOST_UTILISATION
}

QT       += core gui

CONFIG += c++17


greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = simbicon_qt
#TEMPLATE=vcapp
TEMPLATE = app

SOURCES += main.cpp \
    gui/qtmessenger.cpp \
    Physics/softbodypointmass.cpp \
    core/ground_contact_control/estimationworld.cpp

#gui module
SOURCES += gui\mainwindow.cpp \
    gui/simulationwindow.cpp \
    gui/consolewindow.cpp
HEADERS  += gui\mainwindow.h \
    gui/simulationwindow.h \
    gui/consolewindow.h \
    gui/qtmessenger.h \
    Physics/softbodypointmass.h \
    core/ground_contact_control/estimationworld.h

FORMS    += gui\mainwindow.ui \
    gui/simulationwindow.ui\
    gui/consolewindow.ui

#globals module
SOURCES += globals.cpp
HEADERS += globals.h

#old gui
SOURCES += gui\*.cpp
HEADERS += gui\*.h

#gsl module
SOURCES += gsl\*.c \
        gsl\blas\blas.c \
        gsl\blas\cblas.c \
        gsl\block\block.c \
        gsl\block\file.c \
        gsl\block\init.c \
        gsl\matrix\copy.c \
        gsl\matrix\file.c \
        gsl\matrix\getset.c \
        gsl\matrix\init.c \
        gsl\matrix\matrix.c \
        gsl\matrix\minmax.c \
        gsl\matrix\oper.c \
        gsl\matrix\prop.c \
        gsl\matrix\swap.c \
        gsl\vector\copy.c \
        gsl\vector\file.c \
        gsl\vector\init.c \
        gsl\vector\minmax.c \
        gsl\vector\oper.c \
        gsl\vector\prop.c \
        gsl\vector\swap.c \
        gsl\vector\vector.c
HEADERS  += gsl\*.h \
        gsl\blas\*.h \
        gsl\block\*.h \
        gsl\matrix\*.h \
        gsl\vector\*.h

#utils module
SOURCES += Utils\*.cpp
HEADERS  += Utils\*.h \
    Utils\std_redirector.h

#Mathlib module
SOURCES += MathLib\*.cpp
HEADERS  += MathLib\*.h


#GLutils module
SOURCES += GLutils\*.cpp
HEADERS  += GLutils\*.h

#Physics module
SOURCES += Physics\*.cpp
HEADERS += Physics\*.h
SOURCES += Physics\rb\*.cpp
HEADERS += Physics\rb\*.h
SOURCES += Physics\cdp\*.cpp
HEADERS += Physics\cdp\*.h
SOURCES += Physics\joints\*.cpp
HEADERS += Physics\joints\*.h

#core module
SOURCES += core\*.cpp
HEADERS += core\*.h
SOURCES += core\velocity_control\*.cpp
HEADERS += core\velocity_control\*.h
SOURCES += core\pose_control\*.cpp
HEADERS += core\pose_control\*.h
SOURCES += core\ground_contact_control\*.cpp
HEADERS += core\ground_contact_control\*.h
SOURCES += core\medium_control\*.cpp
HEADERS += core\medium_control\*.h

#particlefluid
FLUID_COMPIL {
    DEFINES += FLUID_COMPIL

    HEADERS += SPlisHSPlasH\*.h
    SOURCES += SPlisHSPlasH\*.cpp

    HEADERS +=SPlisHSPlasH\DFSPH\*.h
    SOURCES +=SPlisHSPlasH\DFSPH\*.cpp


    HEADERS +=SPlisHSPlasH\Utilities\*.h
    SOURCES +=SPlisHSPlasH\Utilities\*.cpp
}

#opengl libs
INCLUDEPATH += .
INCLUDEPATH += include
INCLUDEPATH += SPlisHSPlasH/

#glew
Debug:LIBS += -L.\external_libs\GL\glew\msvc2015_MD\lib\Debug\x64
Release:LIBS += -L.\external_libs\GL\glew\msvc2015_MD\lib\Release\x64
Debug:LIBS += -lglew32d
Debug:LIBS += -lglew32sd
Release:LIBS += -lglew32
Release:LIBS += -lglew32s

Debug:LIBS += -L.\external_libs\GL\freeglut\msvc2015_MD\lib\Debug
Release:LIBS += -L.\external_libs\GL\freeglut\msvc2015_MD\lib\Release
Debug:LIBS += -lfreeglut_staticd
Debug:LIBS += -lfreeglutd
Release:LIBS += -lfreeglut_static
Release:LIBS += -lfreeglut

# Define output directories
Release:DESTDIR = release
Release:OBJECTS_DIR = release/obj
Release:MOC_DIR= release/moc
Release:UI_DIR= release/ui

Debug:DESTDIR = debug
Debug:OBJECTS_DIR = debug/obj
Debug:MOC_DIR= debug/moc
Debug:UI_DIR= debug/ui


# configure ODE
DEFINES += dDOUBLE CCD_DOUBLE
Release:LIBS += ode_0_15_1\lib\release\ode.lib
Debug:LIBS += ode_0_15_1\lib\debug\ode.lib
INCLUDEPATH += ode_0_15_1\include

#now including boost
BOOST_UTILISATION {
    DEFINES += BOOST_PARAMETER_MAX_ARITY=15 BOOST_FILESYSTEM_VERSION=3
    Release:DEFINES += BOOST_UBLAS
    Debug:DEFINES += BOOST_UBLAS_NDEBUG
    BOOST_DIR = .\external_libs\boost_1_64_0
    INCLUDEPATH += $$BOOST_DIR
    #BOOST_DIR


    #C:\boost\boost_1_64_0
MD{
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_serialization-vc140-mt-1_64.lib
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_filesystem-vc140-mt-1_64.lib
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_system-vc140-mt-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_serialization-vc140-mt-gd-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_filesystem-vc140-mt-gd-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_system-vc140-mt-gd-1_64.lib
}
MT{
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_serialization-vc140-mt-s-1_64.lib
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_filesystem-vc140-mt-s-1_64.lib
    Release:LIBS += $$BOOST_DIR\lib64-msvc-14.0\libboost_system-vc140-mt-s-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_serialization-vc140-mt-sgd-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_filesystem-vc140-mt-sgd-1_64.lib
    Debug:LIBS +=  $$BOOST_DIR\lib64-msvc-14.0\libboost_system-vc140-mt-sgd-1_64.lib
}
}

SHARK_UTILISATION{
    #and including shark
    DEFINES += SHARK_VERSION_MAJOR=3 SHARK_VERSION_MINOR=0 SHARK_VERSION_PATCH=0
    SHARK_DIR = .\external_libs\shark
    INCLUDEPATH += $$SHARK_DIR\include
    Release:LIBS += $$SHARK_DIR\lib\Release\shark.lib
    Debug:LIBS += $$SHARK_DIR\lib\Debug\shark_debug.lib
}

FLUID_COMPIL {
    #######################################################################
    #########                CUDA COMPILATION                     #########
    #######################################################################

    #specify the cuda file to compile
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/DFSPH_cuda_basic.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/DFSPH_memory_management_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/DFSPH_rendering_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/DFSPH_static_variables_structure_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/DFSPH_core_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/basic_kernels_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/SPH_other_systems_cuda.cu
    CUDA_SOURCES += SPlisHSPlasH/DFSPH/SPH_dynamic_window_buffer.cu

    #so that the cuda fila are visible in editor
    OTHER_FILES +=  $$CUDA_SOURCES

    # Define output directories
    Release:DESTDIR = release
    Release:OBJECTS_DIR = release/obj
    Release:CUDA_OBJECTS_DIR = release/cuda
    Release:MOC_DIR= release/moc

    Debug:DESTDIR = debug
    Debug:OBJECTS_DIR = debug/obj
    Debug:CUDA_OBJECTS_DIR = debug/cuda
    Debug:MOC_DIR= debug/moc

    # CUDA settings <-- may change depending on your system
    SYSTEM_NAME = x64         # Depending on your system either 'Win32', 'x64', or 'Win64'
    SYSTEM_TYPE = 64            # '32' or '64', depending on your system

CUDA8{
    CUDA_DIR = "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v8.0"            # Path to cuda toolkit install
    CUDA_ARCH = compute_30          # Type of CUDA architecture, for example 'compute_10', 'compute_11', 'sm_10'
}
CUDA10{
    CUDA_DIR = "C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v10.1"            # Path to cuda toolkit install
    CUDA_ARCH = compute_61          # Type of CUDA architecture, for example 'compute_10', 'compute_11', 'sm_10'
}

    NVCC_OPTIONS =
    INCLUDEPATH += $$CUDA_DIR/include\


    # library directories
    QMAKE_LIBDIR += $$CUDA_DIR/lib/$$SYSTEM_NAME

    # Add the necessary libraries
    LIBS += -lcuda -lcudart

    INCLUDEPATH+= external_libs\cub-1.7.4\cub
    INCLUDEPATH+= SPlisHSPlasH/

    # The following makes sure all path names (which often include spaces) are put between quotation marks
    CUDA_INC =
    CUDA_INC += $$join(INCLUDEPATH,'" -I"','-I"','"')


    # Configuration of the Cuda compiler
    CONFIG(debug, debug|release) {
        # Debug mode

        #read as: --compiler-options options,... + ISO-standard C++ exception handling
        # + speed over size, + create debug symbols, + code generation multi-threaded debug
        NVCC_OPTIONS += -Xcompiler /EHsc,/O2,/MDd,/Zi

        cuda_d.input = CUDA_SOURCES
        cuda_d.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}_cuda.o
        cuda_d.commands = $$CUDA_DIR/bin/nvcc.exe -D_DEBUG -G $$NVCC_OPTIONS $$CUDA_INC --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}
        cuda_d.dependency_type = TYPE_C
        QMAKE_EXTRA_COMPILERS += cuda_d
    }
    else {
        # Release mode

        #read as: --compiler-options options,... + ISO-standard C++ exception handling
        # + speed over size, + code generation multi-threaded
        NVCC_OPTIONS += -Xcompiler /EHsc,/O2,/MD

        cuda.input = CUDA_SOURCES
        cuda.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}_cuda.o
        cuda.commands = $$CUDA_DIR/bin/nvcc.exe $$NVCC_OPTIONS $$CUDA_INC --machine $$SYSTEM_TYPE -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME}

        cuda.dependency_type = TYPE_C
        QMAKE_EXTRA_COMPILERS += cuda
    }
}
