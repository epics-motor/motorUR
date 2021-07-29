/*
FILENAME... urRtdeDriverDriver.cpp
USAGE...    Motor driver support for the virtual motor controller

Kevin Peterson
January 6, 2015

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "urRtdeDriver.h"

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

using namespace ur_rtde;

/************************************************
 * These are the URController methods *
 ************************************************/


/** Creates a new URController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] URPortName     The name of the drvAsynSerialPort that was created previously to connect to the UR controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
URController::URController(const char *portName, const char *URPortName, int numAxes, 
                                 double movingPollPeriod,double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;
  int axis;
  URAxis *pAxis;
  static const char *functionName = "URController::URController";

  /* Connect to UR controller */
  // TODO: call rtde connect method

  RTDEControlInterface rtde_control(URPortName);
  RTDEReceiveInterface rtde_receive(URPortName);
  rtde_control_ = &rtde_control;
  rtde_receive_ = &rtde_receive;

  std::vector<double> init_q = rtde_receive_->getActualQ();

  /*
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to virtual motor controller\n",
      functionName);
  }
  */

  /*
   * Controller, NOT axis-specific, initialization can go here
   */

  // If additional information is required for creating axes (stepsPerUnit), comment out 
  // the following loop and make the user call URCreateAxis from the cmd file
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new URAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new URController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] URPortName       The name of the drvAsynIPPPort that was created previously to connect to the UR controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int URCreateController(const char *portName, const char *URPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  URController *pURController
    = new URController(portName, URPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pURController = NULL;
  return(asynSuccess);
}


/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void URController::report(FILE *fp, int level)
{
  fprintf(fp, "Virtual Motor Controller driver %s\n", this->portName);
  fprintf(fp, "    numAxes=%d\n", numAxes_);
  fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
  fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

  /*
   * It is a good idea to print private variables that were added to the URController class in URDriver.h, here
   * This allows you to see what is going on by running the "dbior" command from iocsh.
   */

  // Call the base class method
  asynMotorController::report(fp, level);
}


/** Returns a pointer to an URAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
URAxis* URController::getAxis(asynUser *pasynUser)
{
  return static_cast<URAxis*>(asynMotorController::getAxis(pasynUser));
}


/** Returns a pointer to an URAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
URAxis* URController::getAxis(int axisNo)
{
  return static_cast<URAxis*>(asynMotorController::getAxis(axisNo));
}

asynStatus URController::poll(bool *moving)
{ 
  asynStatus status = asynSuccess;

  jointActPos_ = rtde_receive_->getActualQ();

  /*
  // Read the moving status of this motor
  sprintf(pC_->outString_, "%d ST?", axisIndex_);
  comStatus = pC_->writeReadController();
  if (comStatus) 
    goto skip;
  // The response string is of the form "1"
  status = atoi((const char *) &pC_->inString_);

  // Read the direction
  direction = (status & 0x1) ? 1 : 0;
  setIntegerParam(pC_->motorStatusDirection_, direction);

  // Read the moving status
  done = (status & 0x2) ? 1 : 0;
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_, !done);
  *moving = done ? false:true;

  // Read the limit status
  limit = (status & 0x8) ? 1 : 0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (status & 0x10) ? 1 : 0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);

  // Read the home status
  // TODO: implementing homing
  */
}

/******************************************
 * These are the URAxis methods *
 ******************************************/


/** Creates a new URAxis object.
  * \param[in] pC Pointer to the URController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
// Note: the following constructor needs to be modified to accept the stepSize argument if URCreateAxis
// will be called from iocsh, which is necessary for controllers that work in EGU instead of steps.
URAxis::URAxis(URController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  //asynStatus status;
  
  // If the axes of the controller are numbered from 1, this is useful, if not, remove it and use axisNo instead.
  axisIndex_ = axisNo + 1;

  /*
   * Axis-specific initialization can go here
   */
  
  // Allow CNEN to turn motor power on/off
  //setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

  // Make the changed parameters take effect
  callParamCallbacks();
}

/*
 * If the controller reports positions in EGU, rather than integer steps, and the number of stepsPerUnit
 * can vary between axes, the user is required to configure each axis.  The following function, as well as
 * the corresponding registration code at the end of this file should be uncommented.  The declarations in
 * URDrive.h also need to be uncommented.  The URAxis construtor will need to be modifed
 * to accept the (double) stepSize argument.
 * The Newport XPS support is an example of how this is done for a real controller.
 */
/*
extern "C" int URCreateAxis(const char *URName, int axisNo, const char *setpsPerUnit)
{
  URController *pC;
  double stepSize;
   static const char *functionName = "URCreateAxis";
 
  pC = (URController*) findAsynPortDriver(URName);
  if (!pC) 
  {
    printf("Error port %s not found\n", URName);
    return asynError;
  }

  stepSize = strtod(stepsPerUnit, NULL);

  pC->lock();
  new URAxis(pC, axisNo, 1./stepSize);
  pC->unlock();
  return asynSuccess;
}
*/

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void URAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "    Axis #%d\n", axisNo_);
    fprintf(fp, "        axisIndex_=%d\n", axisIndex_);
 }

  /*
   * It is a good idea to print private variables that were added to the URAxis class in URDriver.h, here
   * This allows you to see what is going on by running the "dbior" command from iocsh.
   */

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/*
 * sendAccelAndVelocity() is called by URAxis methods that result in the motor moving: move(), moveVelocity(), home()
 *
 * Arguments in terms of motor record fields:
 *     baseVelocity (steps/s) = VBAS / abs(MRES)
 *     velocity (step/s) = VELO / abs(MRES)
 *     acceleration (step/s/s) = (velocity - baseVelocity) / ACCL
 */
asynStatus URAxis::sendAccelAndVelocity(double acceleration, double velocity, double baseVelocity) 
{
  asynStatus status = asynSuccess;
  // static const char *functionName = "UR::sendAccelAndVelocity";

  /*
  // Send the base velocity
  sprintf(pC_->outString_, "%d BAS %f", axisIndex_, baseVelocity);
  status = pC_->writeReadController();

  // Send the velocity
  sprintf(pC_->outString_, "%d VEL %f", axisIndex_, velocity);
  status = pC_->writeReadController();

  // Send the acceleration
  sprintf(pC_->outString_, "%d ACC %f", axisIndex_, acceleration);
  status = pC_->writeReadController();
  */

  return status;
}


/*
 * move() is called by asynMotor device support when an absolute or a relative move is requested.
 * It can be called multiple times if BDST > 0 or RTRY > 0.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = RVAL = DVAL / MRES
 *     baseVelocity (steps/s) = VBAS / abs(MRES)
 *     velocity (step/s) = VELO / abs(MRES)
 *     acceleration (step/s/s) = (velocity - baseVelocity) / ACCL
 */
asynStatus URAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  // static const char *functionName = "URAxis::move";

  // TODO: call rtde_control.moveJ
  /*
  status = sendAccelAndVelocity(acceleration, maxVelocity, minVelocity);
  
  // Set the target position
  if (relative) {
    sprintf(pC_->outString_, "%d MR %d", axisIndex_, NINT(position));
  } else {
    sprintf(pC_->outString_, "%d MV %d", axisIndex_, NINT(position));
  }
  status = pC_->writeReadController();
  */

  // If controller has a "go" command, send it here
  
  return status;
}


/*
 * home() is called by asynMotor device support when a home is requested.
 * Note: forwards is set by device support, NOT by the motor record.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = HVEL / abs(MRES)
 *     acceleration (step/s/s) = (maxVelocity - minVelocity) / ACCL
 *     forwards = 1 if HOMF was pressed, 0 if HOMR was pressed
 */
/*
asynStatus URAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  // static const char *functionName = "URAxis::home";

  // Homing isn't currently implemented

  return asynSuccess;
}
*/


/*
 * moveVelocity() is called by asynMotor device support when a jog is requested.
 * If a controller doesn't have a jog command (or jog commands), this a jog can be simulated here.
 *
 * Arguments in terms of motor record fields:
 *     minVelocity (steps/s) = VBAS / abs(MRES)
 *     maxVelocity (step/s) = (jog_direction == forward) ? JVEL * DIR / MRES : -1 * JVEL * DIR / MRES
 *     acceleration (step/s/s) = JAR / abs(EGU)
 */
asynStatus URAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "URAxis::moveVelocity";

  // This jogging

  /*
  // Call this to set the max current and acceleration
  status = sendAccelAndVelocity(acceleration, maxVelocity, minVelocity);

  sprintf(pC_->outString_, "%d JOG %f", axisIndex_, maxVelocity);
  status = pC_->writeReadController();
  */
  return status;
}


/*
 * stop() is called by asynMotor device support whenever a user presses the stop button.
 * It is also called when the jog button is released.
 *
 * Arguments in terms of motor record fields:
 *     acceleration = ??? 
 */
asynStatus URAxis::stop(double acceleration)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "URAxis::stop";

  /*
  sprintf(pC_->outString_, "%d AB", axisIndex_);
  status = pC_->writeReadController();
  */
  return status;
}

/*
 * setPosition() is called by asynMotor device support when a position is redefined.
 * It is also required for autosave to restore a position to the controller at iocInit.
 *
 * Arguments in terms of motor record fields:
 *     position (steps) = DVAL / MRES = RVAL 
 */
asynStatus URAxis::setPosition(double position)
{
  asynStatus status = asynSuccess;
  //static const char *functionName = "URAxis::setPosition";

  /*
  sprintf(pC_->outString_, "%d POS %d", axisIndex_, NINT(position));
  status = pC_->writeReadController();
  */
  return status;
}


/*
 * setClosedLoop() is called by asynMotor device support when a user enables or disables torque, 
 * usually from the motorx_all.adl, but only for drivers that set the following params to 1:
 *   pC->motorStatusGainSupport_
 *   pC->motorStatusHasEncoder_
 * What is actually implemented here varies greatly based on the specfics of the controller.
 * 
 * Arguments in terms of motor record fields:
 *     closedLoop = CNEN 
 */
/*
asynStatus URAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  //static const char *functionName = "URAxis::setClosedLoop";
  
  if (closedLoop)
  {
    // Build "Enable" command
    sprintf(pC_->outString_, "%d EN", axisIndex_);
  }
  else
  {
    // Build "Disable" command
    sprintf(pC_->outString_, "%d DI", axisIndex_);
  }

  // Send the command
  status = pC_->writeController();
  return status;
}
*/


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus URAxis::poll(bool *moving)
{ 
  int position;
  int status;
  int done;
  int direction;
  int limit;
  asynStatus comStatus;

  URController* controller = (URController*) pC_;

  //setDoubleParam(controller->motorPosition_, (controller->jointActPos_[axisNo_] / resolution_));
  // Ignore the resolution for now
  setDoubleParam(controller->motorPosition_, (controller->jointActPos_[axisNo_]));
  
  // Read the drive power on status
  //driveOn = (status & 0x100) ? 0 : 1;
  //setIntegerParam(pC_->motorStatusPowerOn_, driveOn);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}


/** Code for iocsh registration */
static const iocshArg URCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg URCreateControllerArg1 = {"UR RTDE port name", iocshArgString};
static const iocshArg URCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg URCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg URCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const URCreateControllerArgs[] = {&URCreateControllerArg0,
                                                             &URCreateControllerArg1,
                                                             &URCreateControllerArg2,
                                                             &URCreateControllerArg3,
                                                             &URCreateControllerArg4};
static const iocshFuncDef URCreateControllerDef = {"URCreateController", 5, URCreateControllerArgs};
static void URCreateContollerCallFunc(const iocshArgBuf *args)
{
  URCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* URCreateAxis */
/*
static const iocshArg URCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg URCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg URCreateAxisArg2 = {"stepsPerUnit", iocshArgString};
static const iocshArg * const URCreateAxisArgs[] = {&URCreateAxisArg0,
                                                     &URCreateAxisArg1,
                                                     &URCreateAxisArg2};
static const iocshFuncDef URCreateAxisDef = {"URCreateAxis", 3, URCreateAxisArgs};
static void URCreateAxisCallFunc(const iocshArgBuf *args)
{
  URCreateAxis(args[0].sval, args[1].ival, args[2].sval);
}
*/


static void URRegister(void)
{
  iocshRegister(&URCreateControllerDef, URCreateContollerCallFunc);
  //iocshRegister(&URCreateAxisDef,       URCreateAxisCallFunc);
}


extern "C" {
epicsExportRegistrar(URRegister);
}
