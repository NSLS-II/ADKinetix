TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
LIBRARY_NAME = ADKinetix 

# Only build for 32bit versions of Windows and Linux.
# Cygwin no longer recognizes the .lib files.

ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
USR_CFLAGS +=   -DWIN32
USR_CXXFLAGS += -DWIN32
LIBRARY_IOC = $(LIBRARY_NAME)

#else ifeq (cygwin-x86, $(findstring cygwin-x86, $(T_A)))
#USR_CFLAGS +=   -DWIN32
#USR_CXXFLAGS += -DWIN32
#LIBRARY_IOC = $(LIBRARY_NAME)
else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
USR_CXXFLAGS += -pthread
LIBRARY_IOC = $(LIBRARY_NAME)

else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIBRARY_IOC = $(LIBRARY_NAME)

endif

#Include driver code
LIB_SRCS += ADKinetix.cpp

#Add required libraries
ifeq (win32-x86, $(findstring win32-x86, $(T_A)))
LIB_LIBS += pvcam32 pv_icl32

else ifeq (cygwin-x86, $(findstring cygwin-x86, $(T_A)))
LIB_LIBS += pvcam32 pv_icl32


else ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_LIBS += pvcam


else ifeq (linux-x86, $(findstring linux-x86, $(T_A)))
LIB_LIBS += pvcam
endif

DBD += kinetixSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

