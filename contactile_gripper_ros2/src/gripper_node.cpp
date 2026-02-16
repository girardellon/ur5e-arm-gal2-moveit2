/**
 * @mainpage Contactiles Gripper ROS2 node - Wrapping Contactile_Gripper_C++_Lib_LIN_V1.4
 * 
 * @section intro_sec Introduction
 * documentation for the ROS2 node that gives access to control Contactile grippers
 * 
 * @section Operation
 * to run this package use the luanch file provided with the following command: 
 * ros2 launch contactile_gripper_ros2 GRIPPER_ROS2.launch.py
 * 
 * @section Installation
    copy contactile_gripper_ros2 folder into your ros2 ws - ros2_ws/src
    in ros2_ws run "colcon build"
 * 
 * @section ROS2 Services and Topics
 * @section contactile_gripper_comms
    "contactile_gripper_comms" Service that is used to communicate commands to the gripper and return responses from the gripper

    Request part = string command_name, float command_argument
    Response part = int32 result_enum, float64 return_values

    send commands in string form list of possible commands:
    "DATA_STREAM"
    "DATA_STOP"
    "BIAS"
    "GET_WIDTH"
    "GET_VEL"
    "GET_IMU"
    "GET_TACTILE"
    "PC_SET_VEL"
    "PC_MOVE_TO_WIDTH"
    "FF_SET_VEL"
    "FF_SET_FORCE"
    "FF_GRIP"
    "DF_SET_VEL"
    "DF_SET_EXF"
    "DF_DYN_EXF_EN"
    "DF_DYN_EXF_DIS"
    "DF_GRIP"
    "STOP"
    "BRAKE"
    "RELEASE"
    "GET_STATE"
    "RESET_PARAMS"
    "CLEAR_LAST_ERROR"
    "GET_LAST_ERROR"
    "PC_GET_VEL"
    "FF_GET_VEL"
    "FF_GET_FORCE"
    "DF_GET_VEL"
    "DF_GET_EXF"
    "DF_GET_DYN_EXF"
    "DF_GET_SHEARG"
    "DF_GET_TORQG"
    "DF_SET_SHEARG"
    "DF_SET_TORQG"
    "GET_TACTILE_GLOBAL"
    "DATA_STREAM_GLOBAL"
    "DATA_STOP_GLOBAL"

 @section contactile_gripper_port_set
    "contactile_gripper_port_set" service sets the serial port for connecting to the gripper format example "ACM0".
    
    Request part = string port
    Response part = bool success

    success true if the gripper has been connect false if not

 @section Data stream topic: contactile_data_stream
    The topic that streams the data from the gripper. If data_stop command has been called the topic will not be published to untill data_stream is called. The topic is published at 50hz. 
    see manual for additional information on data stream data.
    For ROS2 Node documentation see GripperNode
 *
 */

#include <filesystem>
#include <regex>

#include <iostream>
#include <string>
#include <unordered_map>
#include <functional>

//basic ros
#include <rclcpp/rclcpp.hpp>

//data types
#include "contactile_gripper_ros2/srv/gripper_command.hpp"
#include "contactile_gripper_ros2/srv/gripper_serial_port.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

//Gripper library
#include "GripperClass.h"
#include "Structures.h"

/**
 * @brief node class for contactile's angular gripper- wrapper for the c++ gripper communications class
 */

class GripperNode : public rclcpp::Node {
public:
    /**
     * @brief Construct gripper_node object start serial service, start command service and start data stream topic 
     */
    GripperNode() : Node("gripper_node")
    {
        RCLCPP_INFO(this->get_logger(), "[GripperNode] Node Started");

        //create gripper library gripper instance
        gripper_ = std::make_unique<GripperClass>(true);

        // Create service for commands
        commandService_ = create_service<contactile_gripper_ros2::srv::GripperCommand>(
            "contactile_gripper_comms",
            std::bind(&GripperNode::commandServiceCallback, 
                      this, 
                      std::placeholders::_1, 
                      std::placeholders::_2)
        );

        //Create service or serial port setting
        portService_ = create_service<contactile_gripper_ros2::srv::GripperSerialPort>(
            "contactile_gripper_port_set",
            std::bind(&GripperNode::portServiceCallback,
                        this,
                        std::placeholders::_1,
                        std::placeholders::_2)
        );

        //create data stream topic
        dataStreamPublisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("contactile_data_stream", 10);

        //create a timer for publishing data stream data - 50hz
        dataStreamTimer_= this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&GripperNode::publishDataStream, this));
    }

    /**
     * @brief Destroy the Gripper Node object - calls shutdown procedure in gripper library
     * 
     */
    ~GripperNode(){
        gripper_->terminate();
    }

private:
    /**
     * @brief gets data stream data from the gripper, parrots the most recent one onto a /contactile_data_stream
     * 
     */
    void publishDataStream(){
        std::vector<std::vector<double>> data = gripper_->getDataStream();
        if(data.size() > 0){
            auto message = std_msgs::msg::Float64MultiArray();
            message.data = data.at(0);
            dataStreamPublisher_->publish(message);
        }
    }

    /**
     * @brief callback for the serial port service ("contactile_gripper_port_set"), calls serialportset within the gripper object and responds to the service with if it was successful or not
     * see /srv/GripperSerialPort.srv for definition of service format
     * 
     * @param request contains serial port in port e.g. "ACM0" see /srv/GripperSerialPort.srv for definition of service format
     * @param response ->sucess = true for port set successfully
     */
    void portServiceCallback(
	  const std::shared_ptr<contactile_gripper_ros2::srv::GripperSerialPort::Request> request,
	  std::shared_ptr<contactile_gripper_ros2::srv::GripperSerialPort::Response> response)
	{
	  RCLCPP_INFO(this->get_logger(), "[GripperNode] port service request received: '%s'", request->port.c_str());

	  std::string port = request->port;

	  // If user passes /dev/... (e.g. /dev/contactile_gripper), resolve to real device and convert to ACMx/USBx
	  if (port.rfind("/dev/", 0) == 0) {
	    try {
	      auto resolved = std::filesystem::canonical(port).string();   // e.g. /dev/ttyACM0
	      auto base = std::filesystem::path(resolved).filename().string(); // ttyACM0

	      // Convert ttyACM0 -> ACM0, ttyUSB1 -> USB1
	      std::smatch m;
	      if (std::regex_match(base, m, std::regex(R"(tty(ACM|USB)(\d+))"))) {
		port = m[1].str() + m[2].str();  // "ACM" + "0" => "ACM0"
	      } else {
		RCLCPP_ERROR(this->get_logger(), "[GripperNode] Resolved '%s' but cannot parse device name '%s'",
		             resolved.c_str(), base.c_str());
		response->success = false;
		return;
	      }
	    } catch (const std::exception& e) {
	      RCLCPP_ERROR(this->get_logger(), "[GripperNode] Failed to resolve '%s': %s", port.c_str(), e.what());
	      response->success = false;
	      return;
	    }
	  }

	  // Call library with expected format (ACM0/USB0)
	  response->success = gripper_->setComPort(port);

	  if (!response->success) {
	    RCLCPP_ERROR(this->get_logger(), "[GripperNode] setComPort('%s') failed", port.c_str());
	    return;
	  }

	  // Wait for ready, but don't hang forever
	  for (int i = 0; i < 100 && !gripper_->sendReceiveReady(); i++) { // 100*50ms = 5s
	    std::this_thread::sleep_for(std::chrono::milliseconds(50));
	  }

	  if (!gripper_->sendReceiveReady()) {
	    RCLCPP_ERROR(this->get_logger(), "[GripperNode] Serial not ready after timeout");
	    response->success = false;
	    return;
	  }

	  RCLCPP_INFO(this->get_logger(), "[GripperNode] Serial connected OK (port token: '%s')", port.c_str());
	}


    /**
     * @brief callback for the command service ("contactile_gripper_comms") takes the request command and argument and calls the corresponding 
     * see /srv/GripperCommand.srv for definition of service format
     * 
     * @param request 
     *          ->command_name: string representing command - aligns with contactile manual
     *          ->command_argument: float representing argument to go along with command - used for setters
     * @param response 
     *          ->return_enum: will either be the value returned by the getter called or the error code or success code see include/c-pybind-angular-gripper-library/include/Structures.h
     *          ->return_values: will be an empty vector if there are no return values for specified command, if command returns 1 value then this will be a vector of length 1, otherwise vector of return values
     */
    void commandServiceCallback(const std::shared_ptr<contactile_gripper_ros2::srv::GripperCommand::Request> request, std::shared_ptr<contactile_gripper_ros2::srv::GripperCommand::Response> response){
        // Function dispatch based on function_name
        RCLCPP_INFO(this->get_logger(), "[GripperNode] Command service request received");

        //Default values
        std::vector<double> dummyVector;
        response->return_values = dummyVector;
        response->result_enum = COMMAND_FAIL;
        double ret = 0;
        std::vector<double> retVec;

        if (request->command_name == "DATA_STREAM"){
            response->result_enum=gripper_->data_stream();
        }else if(request->command_name == "DATA_STOP"){
            response->result_enum=gripper_->data_stop();
        }else if(request->command_name == "BIAS"){
            response->result_enum=gripper_->bias();
        }else if(request->command_name == "GET_WIDTH"){
            ret = gripper_->get_width();
            if(ret < 0){ //error code
                response->result_enum=ret;
            }else{ //non error code
                response->return_values.push_back(ret);
                response->result_enum=COMMAND_SUCCESS;
            }
        }else if(request->command_name == "GET_VEL"){
            ret = gripper_->get_vel();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "GET_IMU"){
            retVec = gripper_->get_imu();
            if(retVec.size() > 0){
                response->result_enum = COMMAND_SUCCESS;
                for(size_t i = 0; i < retVec.size(); i++){ //had to do this because of weird data conversion caused by the service header creation library return_values = retVec doesn't work
                    response->return_values.push_back(retVec.at(i));
                }
            }else{
                response->result_enum=COMMAND_FAIL;
            }
        }else if(request->command_name == "GET_TACTILE"){
            retVec = gripper_->get_tactile();
            if(retVec.size() > 0){
                response->result_enum = COMMAND_SUCCESS;
                for(size_t i = 0; i < retVec.size(); i++){ //had to do this because of weird data conversion caused by the service header creation library return_values = retVec doesn't work
                    response->return_values.push_back(retVec.at(i));
                }
            }else{
                response->result_enum=COMMAND_FAIL;
            }
        }else if(request->command_name == "PC_SET_VEL"){
            response->result_enum = gripper_->pc_set_vel(request->command_argument);
        }else if(request->command_name == "PC_MOVE_TO_WIDTH"){
            response->result_enum = gripper_->pc_move_to_width(request->command_argument);
        }else if(request->command_name == "FF_SET_VEL"){
            response->result_enum = gripper_->ff_set_vel(request->command_argument);
        }else if(request->command_name == "FF_SET_FORCE"){
            response->result_enum = gripper_->ff_set_force(request->command_argument);
        }else if(request->command_name == "FF_GRIP"){
            response->result_enum = gripper_->ff_grip();
        }else if(request->command_name == "DF_SET_VEL"){
            response->result_enum = gripper_->df_set_vel(request->command_argument);
        }else if(request->command_name == "DF_SET_EXF"){
            response->result_enum = gripper_->df_set_exf(request->command_argument);
        }else if(request->command_name == "DF_DYN_EXF_EN"){
            response->result_enum = gripper_->df_dyn_exf_en();
        }else if(request->command_name == "DF_DYN_EXF_DIS"){
            response->result_enum = gripper_->df_dyn_exf_dis();
        }else if(request->command_name == "DF_GRIP"){
            response->result_enum = gripper_->df_grip();
        }else if(request->command_name == "STOP"){
            response->result_enum = gripper_->stop();
        }else if(request->command_name == "BRAKE"){
            response->result_enum = gripper_->brake();
        }else if(request->command_name == "RELEASE"){
            response->result_enum=gripper_->release();
        }else if(request->command_name == "GET_STATE"){
            response->result_enum=gripper_->get_state();
        }else if(request->command_name == "RESET_PARAMS"){
            response->result_enum=gripper_->reset_params();
        }else if(request->command_name == "CLEAR_LAST_ERROR"){
            response->result_enum=gripper_->clear_last_error();
        }else if(request->command_name == "GET_LAST_ERROR"){
            ret = gripper_->get_last_error();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL; 
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "PC_GET_VEL"){
            ret = gripper_->pc_get_vel();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "FF_GET_VEL"){
            ret = gripper_->ff_get_vel();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "FF_GET_FORCE"){
            ret = gripper_->ff_get_force();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_GET_VEL"){
            ret = gripper_->df_get_vel();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_GET_EXF"){
            ret = gripper_->df_get_exf();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_GET_DYN_EXF"){
            ret = gripper_->df_get_dyn_exf();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_GET_SHEARG"){
            ret = gripper_->df_get_shearg();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_GET_TORQG"){
            ret = gripper_->df_get_torqg();
            response->result_enum = (ret != COMMAND_FAIL) ? COMMAND_SUCCESS : COMMAND_FAIL;
            if(ret != COMMAND_FAIL) response->return_values.push_back(ret);
        }else if(request->command_name == "DF_SET_SHEARG"){
            response->result_enum = gripper_->df_set_shearg(request->command_argument);
        }else if(request->command_name == "DF_SET_TORQG"){
            response->result_enum = gripper_->df_set_torqg(request->command_argument);
        }else if(request->command_name == "GET_TACTILE_GLOBAL"){
            retVec = gripper_->get_tactile_global();
            if(retVec.size() > 0){
                response->result_enum = COMMAND_SUCCESS;
                for(size_t i = 0; i < retVec.size(); i++){ //had to do this because of weird data conversion caused by the service header creation library return_values = retVec doesn't work
                    response->return_values.push_back(retVec.at(i));
                }
            }else{
                response->result_enum=COMMAND_FAIL;
            }
        }else if(request->command_name == "DATA_STREAM_GLOBAL"){
            response->result_enum=gripper_->data_stream_global();
        }else if(request->command_name == "DATA_STOP_GLOBAL"){
            response->result_enum=gripper_->data_stop_global();
        }else{
            response->result_enum=ERR_COMMAND_NAME;
        }
    }

private:
    /**
     * @brief gripper object from angular gripper library this node wraps
     * 
     */
    std::unique_ptr<GripperClass> gripper_;

    /**
     * @brief Ptr to command serice
     * 
     */
    rclcpp::Service<contactile_gripper_ros2::srv::GripperCommand>::SharedPtr commandService_;
    
    /**
     * @brief pointer to port service
     * 
     */
    rclcpp::Service<contactile_gripper_ros2::srv::GripperSerialPort>::SharedPtr portService_;
    
    /**
     * @brief pointer to data stream publisher
     * 
     */
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dataStreamPublisher_;
    
    /**
     * @brief timer which calls the data stream publisher at 50hz
     * 
     */
    rclcpp::TimerBase::SharedPtr dataStreamTimer_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
