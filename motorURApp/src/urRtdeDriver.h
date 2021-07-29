/*
FILENAME...   urRtdeDriverDriver.h
USAGE...      Motor driver support for the virtual motor controller.

Kevin Peterson
January 6, 2015

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"

using namespace ur_rtde;

#define MAX_VIRTUAL_MOTOR_AXES 32     /* motor.h sets the maximum number of axes */
//#define BUFF_SIZE 20		/* Maximum length of string to/from UR */

// No controller-specific parameters yet
#define NUM_VIRTUAL_MOTOR_PARAMS 0  

class epicsShareClass URAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  URAxis(class URController *pC, int axisNo);
  //URAxis(class URController *pC, int axisNo, double stepSize);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  //asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  //asynStatus setClosedLoop(bool closedLoop);

private:
  URController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  int axisIndex_;
  //double stepsSize_;
  asynStatus sendAccelAndVelocity(double accel, double velocity, double baseVelocity);
  
friend class URController;
};

class epicsShareClass URController : public asynMotorController {
public:
  URController(const char *portName, const char *URPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  asynStatus poll(bool *moving);

  void report(FILE *fp, int level);
  URAxis* getAxis(asynUser *pasynUser);
  URAxis* getAxis(int axisNo);

private:
//  char buff_[BUFF_SIZE];
  std::vector<double> jointActPos_;
  RTDEControlInterface control_interface_;
  RTDEReceiveInterface receive_interface_;
  
friend class URAxis;
};
