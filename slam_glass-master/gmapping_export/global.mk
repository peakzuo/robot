### You should not need to change anything below.
LINUX=1
MACOSX=0

# Compilers
CC=gcc
CXX=g++

# Paths
MAPPING_ROOT=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export
LIBDIR=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/lib
BINDIR=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/bin

# Build tools
PRETTY=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/pretty_compiler
MESSAGE=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/message
TESTLIB=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/testlib

# QT support
MOC=moc-qt3
QT_LIB=-lqt-mt
QT_INCLUDE=-I/usr/include/qt3

# ARIA support
ARIA_LIB=
ARIA_INCLUDE=


# # KDE support
# KDE_LIB=
# KDE_INCLUDE=
# UIC=

# Generic makefiles
MAKEFILE_GENERIC=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/Makefile.generic-shared-object
MAKEFILE_APP=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/Makefile.app
MAKEFILE_SUBDIRS=/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/build_tools/Makefile.subdirs


# Flags
CPPFLAGS+=-DLINUX -I/home/leishen/catkin_zz/src/slam_glass-master/gmapping_export 
CXXFLAGS+=
LDFLAGS+= -Xlinker -rpath /home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/lib
CARMENSUPPORT=0
ARIASUPPORT=0



include /home/leishen/catkin_zz/src/slam_glass-master/gmapping_export/manual.mk

