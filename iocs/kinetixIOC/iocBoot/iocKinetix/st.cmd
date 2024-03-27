#!../../bin/linux-x86_64/kinetixApp

< /epics/common/xf31id1-lab3-ioc1-netsetup.cmd

errlogInit(20000)

< envPaths

dbLoadDatabase("$(TOP)/dbd/kinetixApp.dbd")
kinetixApp_registerRecordDeviceDriver(pdbbase)

# Prefix for all records
epicsEnvSet("PREFIX", "XF:31ID1-ES{Kinetix-Det:1}")
# The port name for the detector
epicsEnvSet("PORT",   "KTX")
# The queue size for all plugins
epicsEnvSet("QSIZE",  "20")
# The maximim image width; used for row profiles in the NDPluginStats plugin
epicsEnvSet("XSIZE",  "3200")
# The maximim image height; used for column profiles in the NDPluginStats plugin
epicsEnvSet("YSIZE",  "3200")
# The maximum number of time series points in the NDPluginStats plugin
epicsEnvSet("NCHANS", "2048")
# The maximum number of frames buffered in the NDPluginCircularBuff plugin
epicsEnvSet("CBUFFS", "500")
# The search path for database files
epicsEnvSet("EPICS_DB_INCLUDE_PATH", "$(ADCORE)/db")

ADKinetixConfig(0, "$(PORT)", $(XSIZE), $(YSIZE), 3, 0, 0)
dbLoadRecords("$(ADKINETIX)/db/ADKinetix.template", "P=$(PREFIX),R=cam1:,PORT=$(PORT),ADDR=0,TIMEOUT=1")

# Create a standard arrays plugin, set it to get data from pvCamera driver.
NDStdArraysConfigure("Image1", 1, 0, "$(PORT)", 0, 10000000)
dbLoadRecords("$(ADCORE)/db/NDStdArrays.template", "P=$(PREFIX),R=image:,PORT=Image1,ADDR=0,TIMEOUT=1,NDARRAY_PORT=$(PORT),TYPE=Int16,FTVL=SHORT,NELEMENTS=4194304")

# Load all other plugins using commonPlugins.cmd
< $(ADCORE)/iocBoot/commonPlugins.cmd
set_requestfile_path("$(ADKINETIX)/db")

#asynSetTraceIOMask("$(PORT)",0,2)
#asynSetTraceMask("$(PORT)",0,255)

iocInit()

# save things every thirty seconds
create_monitor_set("auto_settings.req", 30,"P=$(PREFIX)")
