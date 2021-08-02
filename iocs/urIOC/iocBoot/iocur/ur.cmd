### UR controller support

epicsEnvSet("PREFIX", "ur:")

dbLoadTemplate("ur.substitutions", "P=$(PREFIX)")

# URController(
#    portName          The name of the asyn port that will be created for this driver
#    URPortName        The ip address of the UR controller
#    numAxes           The number of axes that this controller supports 
#    movingPollPeriod  The time between polls when any axis is moving 
#    idlePollPeriod    The time between polls when no axis is moving 

URCreateController("UR1", "164.54.104.107", 6, 250, 1000)

dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(PREFIX),R=asyn1,PORT=UR1,ADDR=0,OMAX=0,IMAX=0")
