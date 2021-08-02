TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = ur
# ur.dbd will be created and installed
DBD += ur.dbd

# ur.dbd will be made up from these files:
ur_DBD += base.dbd

# Include dbd files from all support applications:
ur_DBD += asyn.dbd 
ur_DBD += drvAsynIPPort.dbd
ur_DBD += motorSupport.dbd
ur_DBD += motorUR.dbd

# Add all the support libraries needed by this IOC
#ur_LIBS_SYS += boost_system
#ur_LIBS_SYS += boost_thread
ur_LIBS += rtde
ur_LIBS += motorUR
ur_LIBS += motor
ur_LIBS += asyn

# ur_registerRecordDeviceDriver.cpp derives from ur.dbd
ur_SRCS += ur_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
ur_SRCS_DEFAULT += urMain.cpp
ur_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#ur_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
ur_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

