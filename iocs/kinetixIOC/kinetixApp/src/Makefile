TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
ifeq ($(STATIC_BUILD), YES)
  USR_LDFLAGS_Linux += -Wl,-rpath,$(abspath $(ADPVCAM)/lib/linux-x86)
endif
PROD_NAME    = kinetixApp

PROD_IOC += $(PROD_NAME)


# <name>.dbd will be created from <name>Include.dbd
DBD += $(PROD_NAME).dbd
$(PROD_NAME)_DBD += kinetixSupport.dbd


# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp $(PROD_NAME)Main.cpp


# Add locally compiled object code
$(PROD_NAME)_LIBS += ADKinetix

# 64 bit windows support
ifeq (windows-x64, $(findstring windows-x64, $(T_A)))
$(PROD_NAME)_LIBS += pvcam

else ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
$(PROD_NAME)_LIBS += pvcam32 pv_icl32

else ifeq (cygwin-x86, $(findstring cygwin-x86, $(T_A)))
$(PROD_NAME)_LIBS += pvcam32 pv_icl32


# For linux we only have a shared library, so use SYS_LIBS
else ifeq (linux-x86, $(T_A))
$(PROD_NAME)_SYS_LIBS += pvcam

else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
$(PROD_NAME)_SYS_LIBS += pvcam

endif

include $(ADCORE)/ADApp/commonDriverMakefile

#=============================


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

