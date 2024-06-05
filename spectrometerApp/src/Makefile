TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#========================================

# Set a flag if we're using an EVR, so that the driver gets EVR timestamps
ifdef EVRCLIENT
USR_CXXFLAGS += -DEVR_SUPPORT
endif

# =====================================================
# Path to "NON EPICS" External PACKAGES: USER INCLUDES
# ======================================================
USR_INCLUDES += -I$(SEABREEZE_INCLUDE)
LIB_INSTALLS += $(SEABREEZE_LIB)/libseabreeze.a

#========================================
# Build an IOC support library
PROD_NAME = spectrometer
LIBRARY_IOC += $(PROD_NAME)
PROD_IOC += $(PROD_NAME)

# Install dbd files
DBD += $(PROD_NAME).dbd

$(PROD_NAME)_DBD += base.dbd
$(PROD_NAME)_DBD += asyn.dbd
$(PROD_NAME)_DBD += $(PROD_NAME)Support.dbd

# The following are compiled and added to the Support library
$(PROD_NAME)_SRCS += drvSeaBreezeWrapper.cpp
$(PROD_NAME)_SRCS += drvSeaBreezeAPI.cpp
$(PROD_NAME)_SRCS += spectrometerMain.cpp

$(PROD_NAME)_LIBS += asyn
$(PROD_NAME)_LIBS += seabreeze
$(PROD_NAME)_SYS_LIBS_Linux += usb
$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

#========================================
include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
#
# End of file

