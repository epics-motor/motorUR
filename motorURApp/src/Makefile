TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += motorUR

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install motorUR.dbd into <top>/dbd
DBD += motorUR.dbd

# specify all source files to be compiled and added to the library
motorUR_SRCS += urRtdeDriver.cpp

motorUR_LIBS += rtde
motorUR_LIBS += $(EPICS_BASE_IOC_LIBS)

motorUR_SYS_LIBS += boost_system
motorUR_SYS_LIBS += boost_thread

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

