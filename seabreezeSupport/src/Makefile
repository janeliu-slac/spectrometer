TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

SEABREEZE_TOP = $(TOP)/seabreezeSupport/vendor/SeaBreeze

USR_LDFLAGS += -lusb

#==================================================
# Build an IOC support library

LIBRARY_IOC_Linux += seabreeze


#========================================
include $(TOP)/configure/RULES

#----------------------------------------
#  ADD RULES AFTER THIS LINE

# This will only build a library for the host architecture
# So no cross-compiling at the moment

install:
	$(MAKE) -C $(SEABREEZE_TOP)
	-$(MKDIR) -p $(TOP)/lib/$(EPICS_HOST_ARCH)
	$(CP) $(SEABREEZE_TOP)/lib/libseabreeze.a $(TOP)/lib/$(EPICS_HOST_ARCH)

clean:
	$(MAKE) -C $(SEABREEZE_TOP) clean

# End of file

