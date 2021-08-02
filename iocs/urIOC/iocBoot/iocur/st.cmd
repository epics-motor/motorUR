#!../../bin/linux-x86_64/ur

< envPaths

cd "${TOP}/iocBoot/${IOC}"

## Register all support components
dbLoadDatabase "$(TOP)/dbd/ur.dbd"
ur_registerRecordDeviceDriver pdbbase

## motorUtil (allstop & alldone)
dbLoadRecords("$(MOTOR)/db/motorUtil.db", "P=ur:")

#
< ur.cmd

iocInit

## motorUtil (allstop & alldone)
motorUtilInit("ur:")

# Boot complete
