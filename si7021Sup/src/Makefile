TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += si7021

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install si7021.dbd into <top>/dbd
DBD += si7021.dbd

# specify all source files to be compiled and added to the library
si7021_SRCS += drvAsynSi7021.cpp

si7021_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

