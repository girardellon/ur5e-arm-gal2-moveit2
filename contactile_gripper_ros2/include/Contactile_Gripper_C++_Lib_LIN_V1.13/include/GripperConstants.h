#ifndef GRIPPERCONSTANTS_H
#define GRIPPERCONSTANTS_H

//STATES
constexpr const int ERROR_STATE =           -1;
constexpr const int IDLE_READY_STATE =      0;
constexpr const int IDLE_STOPPING_STATE =   1;
constexpr const int PC_MOVE_STATE =         2;
constexpr const int FF_APPROACH_STATE =     3;
constexpr const int FF_HOLD_STATE =         4;
constexpr const int DF_APPROACH_STATE =     5;
constexpr const int DF_HOLD_STATE =         6;
constexpr const int BRAKING_STATE =         7;
constexpr const int RELEASING_STATE =       8;
constexpr const int BUTTON_PRESS_STATE =    9;
 
//ERROR CODES: +ve numbers reserved for error states of the gripper (See GripperConstants.py); -ve numbers reserved for errors from the contactile_gripper_library (see GripperClass.py)
constexpr const int ERR_NONE =                          0;
constexpr const int ERR_OUT_OF_BOUNDS =                 1;
constexpr const int ERR_PC_RINGING =                    2;
constexpr const int ERR_MOTION_OBSTRUCTION =            3;
constexpr const int ERR_UNABLE_TO_REACH_TARGET_FORCE =  4;
constexpr const int ERR_NO_MOTOR_VOLTAGE =              100;
constexpr const int ERR_OVER_TEMP =                     101;
  
//SENSOR CONSTANTS
constexpr const int IMU_DOF =               6;
constexpr const int IMU_NDIM =              3;
constexpr const int IMU_ACC_OFFSET =        0;
constexpr const int IMU_GYR_OFFSET =        1;
constexpr const int IMU_N =                 2*IMU_NDIM;
constexpr const int TACTILE_GLOBAL_DOF =    6;
constexpr const int TACTILE_NDIM =          3;
constexpr const int TACTILE_NSENSOR =       2;
constexpr const int TACTILE_NPILLAR =       9;
constexpr const int TACTILE_FORCE_OFFSET =  0;
constexpr const int TACTILE_TORQUE_OFFSET = TACTILE_NDIM;
constexpr const int TACTILE_N =             TACTILE_NSENSOR*(TACTILE_GLOBAL_DOF + TACTILE_NPILLAR*TACTILE_NDIM);
constexpr const int X_IND =                 0;
constexpr const int Y_IND =                 1;
constexpr const int Z_IND =                 2;

#endif