TOP = ../..
include $(TOP)/configure/CONFIG
#ARCH = win32-x86
#ARCH = windows-x64
#ARCH = windows-x64-static
#ARCH = linux-x86
ARCH = linux-x86_64
ifeq (windows, $(findstring windows, $(T_A)))
TARGETS = envPaths dllPath.bat
else
TARGETS = envPaths
endif

include $(TOP)/configure/RULES.ioc
