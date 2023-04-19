TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

USR_INCLUDES += -I$(SEABREEZE)/vendor/SeaBreeze/include

#========================================
# Build an IOC support library
LIBRARY_IOC += spectrometer

# Install dbd files
DBD += spectrometer.dbd

spectrometer_DBD += base.dbd
spectrometer_DBD += asyn.dbd
spectrometer_DBD += spectrometerSupport.dbd

# The following are compiled and added to the Support library
spectrometer_SRCS += drvSeaBreezeWrapper.cpp
spectrometer_SRCS += drvSeaBreezeAPI.cpp

spectrometer_LIBS += seabreeze
spectrometer_LIBS += asyn
spectrometer_LIBS += $(EPICS_BASE_IOC_LIBS)

#========================================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
# End of file

