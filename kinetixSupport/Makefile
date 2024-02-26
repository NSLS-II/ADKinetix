TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# The files in this directory are installed first because they are
# used by PVCam driver

INC += pvcam.h master.h #pv_icl.h
#INC_linux-x86 += master.h
#INC_linux-x86_64 += master.h
#INC_WIN32 += master.h
#INC_cygwin32 += master.h

# Note, the following files were manually copied from the PVCam distribution
# to this directory after building



ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))

LIB_INSTALLS_Linux += ../os/linux-x86_64/libpvcam.so

else ifeq (linux-x86,  $(T_A))

LIB_INSTALLS_Linux    += ../os/linux-x86/libpvcam.so

else ifeq (WIN32, $(findstring WIN32, $(T_A)))

BIN_INSTALLS_WIN32    += ../os/WIN32/pv_icl32.dll ../os/WIN32/pvcam32.dll
LIB_INSTALLS_WIN32    += ../os/WIN32/pv_icl32.lib ../os/WIN32/pvcam32.lib


else ifeq (windows-x64, $(findstring windows-x64, $(T_A)))

BIN_INSTALLS_WIN32    += ../os/WIN32/pv_icl32.dll ../os/WIN32/pvcam32.dll
LIB_INSTALLS_WIN32    += ../os/WIN32/pv_icl32.lib ../os/WIN32/pvcam32.lib


else ifeq (cygwin32, $(findstring cygwin32, $(T_A)))

BIN_INSTALLS_cygwin32 += ../os/cygwin32/pv_icl32.dll ../os/cygwin32/pvcam32.dll
LIB_INSTALLS_cygwin32 += ../os/cygwin32/pv_icl32.lib ../os/cygwin32/pvcam32.lib

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

