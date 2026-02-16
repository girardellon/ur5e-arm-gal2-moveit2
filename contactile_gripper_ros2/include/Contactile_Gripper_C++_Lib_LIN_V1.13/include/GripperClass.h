#ifndef GRIPPERCLASS_H
#define GRIPPERCLASS_H

#include <queue>
#include <string>
#include <vector>
#include <mutex>
#include <memory>
#include <thread>

#include "Structures.h"

//Forward Declarations to avoid requiring #include "SerialManager.h"
class SerialManager;


/**
 * @brief GripperClass is the main class of the library. Create an instance of GripperClass to use the library, once created the instance will wait for a serial port to be supplied.
 * 
 */
class GripperClass {
public:
    /**
     * @brief Constructs a new Gripper Class object.
     * 
     * @param DEBUG True or false for debug terminal printout. If false the class will output nothing to the terminal.
     */
    GripperClass(bool DEBUG = false);

    /**
     * @brief Destroys the Gripper Class object and closes the open threads.
     */
    ~GripperClass();

    /**
     * @brief This function is called by the destructor, used needed to cleanup threads. This function can be called when wanting to manually manage resources.  
     * 
     */
    void terminate();

    /**
     * @brief Sets the serial port. If this function has not been called the library will wait untill it is as it is required to communicate with the gripper
     * 
     * @param comPort eg. "COM12", "ACM0"
     */
    bool setComPort(std::string comPort);

    /**
     * @brief Returns True if the gripper is ready to send and receive messages, use this function to wait after setting the serial port. After receiving a true response you may send commands. If a user does not wait untill the library is ready to send commands then the recognition of responses from the gripper will be disrupted. On slow systems an small extra wait period (100-200ms) may be required after send and receive ready returns true to ensure that the first command sent will be recognised.
     * 
     * @return Returns true if the library is ready to read and write to gripper.
     * @return Returns false if not ready.
     */
    bool sendReceiveReady();

    /**
     * @brief Returns the Command Queue object, normally the command queue will be empty as when commands are executed they are removed from the command queue. If you send many commands at once then this will fill up. 
     * 
     * @return std::vector<std::string> This function returns a vector which has been converted from an internal queue for compatability with the python pybind implementation 
     */
    std::vector<std::string> getCommandQueue();

    /**
     * @brief Returns the Response Queue object, normally the Response queue will be empty as when Response is acted upon it is removed from the queue.
     * 
     * @return std::vector<responseStruct> Returns a vector of responses. The internal queue is coverted to a vector for return for compatability with the python pybind implementation 
     */
    std::vector<responseStruct> getResponseQueue();

    /**
     * @brief Returns data that has been not yet been requested through this function. Data is removed from the data queue when this function is called. Internal queue is converted to a vector for return for compatibility with the pybind python implementation.
     * 
     * @return std::vector<std::vector<double>> the most recent data will be at the end of the vector.
     */
    std::vector<std::vector<double>> getDataStream();

    /**
     * @brief Starts logging data in the Log Manager of the Serial Manager.
     * 
     */
    void startLogging();

    /**
     * @brief Stops logging data in the Log Manager of the Serial Manager.
     * 
     */
    void stopLogging();

    // COMMANDS: DATA STREAMING
    /**
     * @brief Sends DATA_STREAM, checks acknowledgement.
     * 
     * @return CMD_STATUS struct representing the result of the command.
     */
    CMD_STATUS data_stream();

    /**
     * @brief Sends DATA_STOP and checks acknowledgement. After 1.5s library will stop looking for acknowledgement from the gripper
     * @return CMD_STATUS returns result.
     */
    CMD_STATUS data_stop();

    /**
     * @brief Operates like data_stream except with less data streamed. This function starts the data stream.
     * 
     * @return CMD_STATUS returns result.
     */
    CMD_STATUS data_stream_global();

    /**
     * @brief Stops the global data stream. Sends DATA_STOP_GLOBAL, discards acknowledgement, flushes input buffer. If acknowledgement can't be found in 1.5 it stops checking for acknowlegement.
     * @return CMD_STATUS returns result.
     */
    CMD_STATUS data_stop_global();


    // COMMANDS: GET/CLEAR STATE AND ERROR

    /**
     * @brief Sends CLEAR_LAST_ERROR, checks acknowledgement. 
     * For Backwards compatibility only.
     * 
     * @return CMD_STATUS returns result.
     */
    CMD_STATUS clear_last_error();

    /**
     * @brief Sends CLEAR_ERROR_STATE, checks acknowledgement.
     * 
     * @return CMD_STATUS returns result.
     */
    CMD_STATUS clear_error_state();

    /**
     * @brief Sends GET_STATE, checks acknowledgement, returns result.
     * 
     * @return int state -> see GripperConstants.h for states for states documentation .
     */
    int get_state();

    /**
     * @brief Sends GET_LAST_ERROR, checks acknowledgement, returns result.
     * 
     * @return int last error -> see GripperConstants.h ERROR CODES.
     */
    int get_last_error();

    // COMMANDS: GET SENSOR DATA

    /**
     * @brief Sends GET_WIDTH, checks acknowledgement, returns result.
     * 
     * @return double - distance between the tips of the sensor arrays.
     */
    double get_width();

    /**
     * @brief Sends GET_VEL, checks acknowledgement, returns result.
     * 
     * @return double velocity.
     */
    double get_vel();

    /**
     * @brief Sends GET_IMU, checks acknowledgement, returns result.
     * 
     * @return std::vector<double> set of IMU data
     */
    std::vector<double> get_imu();

    /**
     * @brief Sends GET_TACTILE, checks acknowledgement, returns result.
     * 
     * @return std::vector<double> set of tactile data
     */
    std::vector<double> get_tactile();

    /**
     * @brief Sends get_tactile_global, checks acknowledgement, returns result.
     * 
     * @return std::vector<double> set of global force and torque values 6 values for SEN0 and 6 values for SEN1
     */
    std::vector<double> get_tactile_global();

    // COMMANDS: GET PARAMETERS

    /**
     * @brief Sends PC_GET_VEL, checks acknowledgement, returns result.
     * 
     * @return double velocity.
     */
    double pc_get_vel();

    /**
     * @brief Sends FF_GET_VEL, checks acknowledgement, returns result.
     * 
     * @return double velocity.
     */
    double ff_get_vel();

    /**
     * @brief Sends FF_GET_FORCE, checks acknowledgement, returns result.
     * 
     * @return double force parameter.
     */
    double ff_get_force();

    /**
     * @brief Sends DF_GET_VEL, checks acknowledgement, returns result.
     * 
     * @return double df_velocity.
     */
    double df_get_vel();

    /**
     * @brief Sends DF_GET_EXF, checks acknowledgement, returns result.
     * 
     * @return double exploratory grip force.
     */
    double df_get_exf();

    /**
     * @brief Sends DF_GET_DYN_EXF, checks acknowledgement, returns result.
     * 
     * @return int flag for dynamic exploration force for DF_GRIP mode, 0 is disabled, 1 is enabled.
     */
    int df_get_dyn_exf();

    /**
     * @brief Sends DF_GET_SHEARG, checks acknowledgement, returns result.
     * 
     * @return double the shear gain for dynamic force control mode.
     */
    double df_get_shearg();

    /**
     * @brief Sends DF_GET_TORQG, checks acknowledgement, returns result.
     * 
     * @return double the torque gain for dynamic force control.
     */
    double df_get_torqg();

    // COMMANDS: SET PARAMETERS 

    /**
     * @brief Sends RESET_PARAMS, checks acknowledgement, Resets all parameters back to the default values.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS reset_params();

    /**
     * @brief Sends PC_SET_VEL, checks acknowledgement - Sets the target velocity for position control mode.
     * 
     * @param vel double mm/s (max: 333, min: 1).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS pc_set_vel(double vel);
    
    /**
     * @brief Sends FF_SET_VEL, checks acknowledgement - Sets the target approach velocity for fixed force control mode.
     * 
     * @param vel mm/s (max: 166, min: 1).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS ff_set_vel(double vel);
    
    /**
     * @brief Sends FF_SET_FORCE, checks acknowledgement - Sets the target grip force for fixed force control mode.
     * 
     * @param force N (max: 30, min: 1.5).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS ff_set_force(double force);
    
    /**
     * @brief Sends DF_SET_VEL, checks acknowledgement - Sets the target approach velocity for dynamic force control mode.
     * 
     * @param vel mm/s (max: 166, min: 1).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_set_vel(double vel);

    /**
     * @brief Sends DF_SET_EXF, checks acknowledgement - Sets the target exploratory grip force for dynamic force control mode.
     * 
     * @param force N (max: 15, min: 1.5).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_set_exf(double force);
    
    /**
     * @brief Sends DF_DYN_EXF_EN, checks acknowledgement - Enables dynamic exploration force for DF_GRIP mode, based on the measured compliance of the object.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_dyn_exf_en(); 

    /**
     * @brief Sends DF_DYN_EXF_DIS, checks acknowledgement - disables dynamic exploration force for DF_GRIP mode.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_dyn_exf_dis();

    /**
     * @brief Sends DF_SET_SHEARG, checks acknowledgement - Sets the shear gain for dynamic force control mode.
     * 
     * @param gain (max: 3, min: 1.1).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_set_shearg(double gain);
    
    /**
     * @brief Sends DF_SET_TORQG, checks acknowledgement - sets the torque gain for dynamic force control mode.
     * 
     * @param gain (max: 2000, min: 0).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_set_torqg(double gain);

    // COMMANDS: ACTION 

    /**
     * @brief Sends BIAS, checks acknowledgement - zeros the tactile sensor output.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS bias();
    
    /**
     * @brief Sends PC_MOVE_TO_WIDTH, checks acknowledgement - moves the fingers to target width - not intented to be used to grip an object.
     * 
     * @param width (max: 170, min: 0).
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS pc_move_to_width(double width);
    
    /**
     * @brief Sends FF_GRIP, checks acknowledgement - Starts fixed force gripping.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS ff_grip();
    
    /**
     * @brief Sends DF_GRIP, checks acknowledgement - Starts dynamic force control gripping.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS df_grip();
    
    /**
     * @brief Sends STOP, checks acknowledgement.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS stop();
    
    /**
     * @brief Sends BRAKE, checks acknowledgement.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS brake();
    
    /**
     * @brief Sends RELEASE, checks acknowledgement.
     * 
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS release();

    // Compound functions
    /**
     * @brief blocking until the state of the gripper is 0 i.e. the gripper is in idle state, unless the timeout period is exceeded.
     * 
     * @param timeout_s double max wait time before leaving the function.
     * @return int error code.
     */
    int waitUntil_idle_ready(double timeout_s);

    /**
     * @brief Blocking call untill the state of the gripper matches holdState. User can set the state they are waiting for, usually FF_HOLD_STATE, or DF_HOLD_STATE.
     * 
     * @param holdState int state must match gripper state for function to finish.
     * @param timeout_s double max wait time before returning.
     * @return int error code.
     */
    int waitUntil_hold(int holdState, double timeout_s);

    //Functions for access to Serial Manager Class.
    
    /**
     * @brief Flushes the serial input buffer - used when the communication with gripper goes out of sync.
     * public access if needed - use if command recognition is routinely failing, in normal operation users should not need to call this function.
     * @return CMD_STATUS - int result of command.
     */
    CMD_STATUS flushSerialInputBuffer();

    // Prevent copy, allow move if needed
    GripperClass(const GripperClass&) = delete;
    GripperClass& operator=(const GripperClass&) = delete;

private:
    /**
     * @brief constructs given strings into an error code to be printed.
     * 
     * @param cmdStr std::string command given to get the error code.
     * @param errorCode std::string error code.
     */
    void __printErrorCode__(std::string cmdStr, int errorCode);

    /**
     * @brief checks if a given response matches a command given returns error codes if not.
     * 
     * @param response responseStruct detailing reponse information.
     * @param commandName command that generated that response.
     * @param expectedNRet expected number of retVals.
     * @return CMD_STATUS - int error code.
     */
    CMD_STATUS __checkCommandAcknowledge__(responseStruct response, std::string commandName, int expectedNRet=0);

    /**
     * @brief Queues a gripper command into the command queue for the serial thread to implement.
     * 
     * @param commandName command name to be sent.
     * @param argStrs std::vector<std::string> arguments to be send along with the command, leave empty for no arguments.
     */
    void __queueGripperCommand__(std::string commandName, std::vector<std::string> argStrs);

    /**
     * @brief gets the response from the gripper stored in the response queue.
     * 
     * @return responseStruct
     */
    responseStruct __getQueuedGripperResponse__();

    /**
     * @brief Function used for calling sending, recieving and acknowledging functions for getting a single variable from the gripper 
     * 
     * @param commandStr command string for getting a variable
     * @param expectedNRet used for confirming correct response
     * @return std::vector<double> returns single value in a vector
     */
    std::vector<double> __getVariable__(std::string commandStr, int expectedNRet=0);

    /**
     * @brief calls getVariable and formats to a double
     * 
     * @param commandStr command string for getting a double
     * @param defaultFloatReturn returned if function fails
     * @return double result
     */
    double __getFloat__(std::string commandStr, double defaultFloatReturn);

    /**
     * @brief calls getVariable and formats to an int
     * 
     * @param commandStr command string for getting an int
     * @param defaultIntReturn returned if function fails
     * @return int result
     */
    int __getInt__(std::string commandStr, int defaultIntReturn);

    /**
     * @brief calls getVariable and formats to a vector of double
     * 
     * @param commandStr command string for getting a vector of doubles
     * @param defaultVectorReturn returned if function fails - standard is an empty vector with length equal to expected
     * @return std::vector<double> result 
     */
    std::vector<double> __getFloatList__(std::string commandStr, std::vector<double> defaultVectorReturn);

    /**
     * @brief function used for calling sending, recieving and acknowledging functions for setting a single variable in the gripper
     * 
     * @param commandStr command string for setting a varaible
     * @param argumentList arguments to send along with commandStr
     * @return CMD_STATUS - int error code
     */
    CMD_STATUS __setVariable__(std::string commandStr, std::vector<std::string> argumentList);

    /**
     * @brief sends a command to the grupper with no arguments, or return values
     * 
     * @param commandStr command to be sent
     * @return CMD_STATUS - int error code
     */
    CMD_STATUS __emptyCommand__(std::string commandStr);

    /**
     * @brief constructs the command string to be sent over serial to the gripper
     * 
     * @param commandName the name of the command to be sent - see GripperConstants.h 
     * @param argStrs arguments to be sent along with the commandName
     * @return std::string the structured command ready to be sent to the gripper
     */
    std::string buildGripperCommand(std::string commandName, std::vector<std::string> argStrs);

    // Private member variables

    /**
     * @brief bebug flag
     * 
     */
    bool DEBUG_;

    /**
     * @brief pointer to the serial manager object stored within the gripper class, used for calling functions in the serial class
     * 
     */
    std::unique_ptr<SerialManager> serialManager_;

    /**
     * @brief thread object which is running the serial manager class instance
     * 
     */
    std::thread serialThread_;

    //variables shared with the serial manager

    /**
     * @brief shared access with the serial manager, protected by std::mutex mtx_ used for transferring commands from the gripper class to serial class
     * 
     */
    std::queue<std::string> command_q_;

    /**
     * @brief shared access with the serial manager, protected by std::mutex mtx_ used for transferring responses from the serial manager class to the gripper class
     * 
     */
    std::queue<responseStruct> response_q_;

    /**
     * @brief shared access with the serial manager, protected by std::mutex mtx_ used for transferring data (retVals) from the serial manager class to the gripper class
     * 
     */
    std::queue<std::vector<double>> data_q_;

    /**
     * @brief mutex to protect the shared resources between the gripper and serial class
     * 
     */
    std::mutex mtx_;

    /**
     * @brief Mutex for protecting this class from multi-thread access causing multiple commands to be sent or expected at the same time
     * 
     */
    std::mutex commandMtx_;

    //used for testing
    friend class GripperClassTest;
};

#endif
