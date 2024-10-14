TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================
LIBRARY_NAME = ADKinetix 

USR_CFLAGS_WIN32 +=   -DWIN32
USR_CXXFLAGS_WIN32 += -DWIN32

LIBRARY_IOC_WIN32 = $(LIBRARY_NAME)
LIBRARY_IOC_Linux = $(LIBRARY_NAME)

#Include driver code
LIB_SRCS += ADKinetix.cpp

#Add required libraries

LIB_LIBS_WIN32 += pvcam64
LIB_LIBS_LINUX += pvcam

DBD += kinetixSupport.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================


include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

