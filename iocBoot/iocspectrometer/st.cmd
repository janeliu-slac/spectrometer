#!../../bin/rhel6-x86_64/spectrometer

< envPaths
cd "${TOP}"

epicsEnvSet("P",          "SPEC:LI10:SP01")
epicsEnvSet("PORT",       "SP01")
# Spectrometer number of pixels:
epicsEnvSet("NELM",       "2048")
# Spectrometer serial number:
epicsEnvSet("SERIAL_NUM", "HR+D0587")

dbLoadDatabase("dbd/spectrometer.dbd")
spectrometer_registerRecordDeviceDriver(pdbbase)

# drvSeaBreezeAPIConfigure(const char* port, const char* serialNum)
drvSeaBreezeAPIConfigure("$(PORT)", "$(SERIAL_NUM)")

#asynSetTraceMask("$(PORT)", -1, 0x11)
asynSetTraceIOMask("$(PORT)", -1, 0x2)

dbLoadRecords(db/seabreezeAPI.template, "P=$(P),PORT=$(PORT),NELM=$(NELM)")
dbLoadRecords("db/asynRecord.db", "P=$(P):,R=Asyn,PORT=$(PORT),ADDR=0,IMAX=0,OMAX=0")

iocInit()

# End of file

