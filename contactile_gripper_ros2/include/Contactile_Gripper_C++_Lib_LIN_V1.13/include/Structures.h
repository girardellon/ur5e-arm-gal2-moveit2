/**
 * @mainpage C++ and PyBind Angular Grippper Library
 *
 * @section intro_sec Introduction
 * Documentation for the C++ library for contactile grippers. Included along with the C++ library is a PyBind11 binding and .pyi stub file for python access
 *
 * @section install_sec Installation
 * For C++ build using cmakelists \n
 * mkdir build \n
 * cd build \n
 * cmake .. \n
 * make \n
 * 
 * For python usage fist build: \n python -m build \n
 * then inside the /dist folder \n
 * pip install contactile_gripper_lib-X.X.whl
 * 
 * For C++ usage link the relevant static library file either libContactileGripper.a or ContactileGripper.lib and include the required header .h files.
 * 
 */

#ifndef STRUCTURES_H
#define STRUCTURES_H
 
#include <string>
#include <vector>
#include <iostream>

/**
 * @brief Command Status emun is used for representing error codes/success flag for easy readability
 * 
 */
enum CMD_STATUS{
    COMMAND_SUCCESS = 0,
    COMMAND_FAIL = -999999,
    ERR_COMMAND_NAME = -1,
    ERR_RESP_TYPE = -2,
    ERR_N_RET = -3,
    ERR_SEND = -4,
    ERR_RECIEVE = -5,
    ERR_INVALID_ARG = -6,
    ERR_BUTTON_INTERRUPT = -7,
    ERR_TIMEOUT = -8,    
};

/**
 * @brief reponseStruct is used to group data corresponding to a singular gripper response together and create an easy way to print a response
 * 
 */
struct responseStruct {
    /**
     * @brief command string which generated this response (see GripperConstants)
     * 
     */
    std::string cmdStr;

    /**
     * @brief Command ID recceived from gripper - otherwise often defaults to -1
     * 
     */
    int cmdId;

    /**
     * @brief response type from gripper either see GripperConstants.h for all response types
     * 
     */
    std::string respType;

    /**
     * @brief the data returned from the gripper in this response
     * 
     */
    std::vector<double> retVals;

    /**
     * @brief the string returned from the gripper - stored for debug purposes
     * 
     */
    std::string baseString = "";

    /**
     * @brief Print method to display all information in struct
     * 
     */
    void print() const {
        std::cout << "Response Structure Details:" << std::endl;
        std::cout << "------------------------" << std::endl;
        std::cout << "Command String: " << cmdStr << std::endl;
        std::cout << "Command ID:     " << cmdId << std::endl;
        std::cout << "Response Type:  " << respType << std::endl;
        
        std::cout << "Return Values:  ";
        if (retVals.empty()) {
            std::cout << "None";
        } else {
            for (size_t i = 0; i < retVals.size(); ++i) {
                std::cout << retVals[i];
                if (i < retVals.size() - 1) {
                    std::cout << ", ";
                }
            }
        }
        std::cout << std::endl;
        std::cout << "Base String:    " << baseString << std::endl;
    }
};

#endif