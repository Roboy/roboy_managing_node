#pragma once

#include "roboy_managing_node/roboy.hpp"
#include "roboy_managing_node/myoMaster.hpp"
#include "common_utilities/CommonDefinitions.h"
#include "roboy_communication_middleware/Initialize.h"
#include "roboy_communication_middleware/EmergencyStop.h"
#include "roboy_communication_middleware/Record.h"
#include <roboy_communication_middleware/Steer.h>
#include "roboy_communication_middleware/Trajectory.h"
#include "roboy_communication_middleware/RoboyState.h"
#include "roboy_communication_middleware/JointAngle.h"
#include "roboy_communication_middleware/JointStatus.h"
#include "roboy_communication_middleware/MotorStatus.h"
#include "roboy_communication_middleware/ArucoPose.h"
#include "geometry_msgs/Pose.h"

// ros
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/LoadController.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

// std
#include <thread>
#include <vector>
#include <mutex>
#include <thread>
#include <map>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//! enum for state machine
typedef enum {
    WaitForInitialize,
    LoadControllers,
    Controlloop,
    ResetPowerlinkStack,
    Recording,
    Dancing
} ActionState;

class Roboy : public hardware_interface::RobotHW, MyoMaster {
public:
    /**
     * Constructor
     */
    Roboy(int argc, char* argv[]);

    /**
     * Destructor
     */
    ~Roboy();

    /**
     * This function initialises the requested motors
     */
    bool initializeControllers(roboy_communication_middleware::Initialize::Request &req,
                               roboy_communication_middleware::Initialize::Response &res);
    /**
	 * Read from hardware
	 */
    void read();

    /**
     * Write to Hardware
     */
    void write();

    /**
     * This is the main loop
     */
    void main_loop(controller_manager::ControllerManager *ControllerManager);
//    /**
//     * Handles signals and shuts down everything
//     * @param sig signals
//     */
//    static void sigintHandler(int sig);

private:
    /**
     * Subscriber callback for joint status
     * @param msg
     */
    void JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg);
    /**
     * Subscriber callback for motor status
     * @param msg
     */
    void MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg);
    /**
     * Subscriber callback for motor status
     * @param msg
     */
    void ArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr &msg);

    /**
     * calculates the angle between two lines defined by four aruco markers
     * @param aruco0
     * @param aruco1
     * @param aruco2
     * @param aruco3
     * @return angle in degree
     */
    float calculateAngleBetween(int aruco0, int aruco1, int aruco2, int aruco3);
    /*
     * This function loads the controllers registered to the individual joint interfaces
     * @param controllers names of controllers
     * @return success
     */
    bool loadControllers(vector<string> controllers);

    /*
     * This function unloads the controllers registered to the individual joint interfaces
     * @param controllers names of controllers
     * @return success
     */
    bool unloadControllers(vector<string> controllers);

    /*
	 * This function starts the controllers registered to the individual joint interfaces
	 * @param controllers names of controllers
	 * @return success
	 */
    bool startControllers(vector<string> controllers);

    /*
	 * This function stops the controllers registered to the individual joint interfaces
	 * @param controllers names of controllers
	 * @return success
	 */
    bool stopControllers(vector<string> controllers);

    /**
     * SERVICE This function record the trajectories of the requested motors
     * @param req vector<int8> containing requested motor ids
     * @param res vector<ControllerStates> cf. CommonDefinitions.h
     */
    bool record(roboy_communication_middleware::Record::Request &req,
                roboy_communication_middleware::Record::Response &res);

    /**
     * SUBSCRIBER enables pause/resume and stop recording
     */
    void steer_record(const roboy_communication_middleware::Steer::ConstPtr &msg);

    ros::NodeHandlePtr nh;
    double *cmd;
    double *pos;
    double *vel;
    double *eff;
    ros::Time prevTime;
    int8_t recording;
    bool initialized = false;
    static bool shutdown_flag;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber jointStatus_sub, motorStatus_sub;
    vector<float> jointAngles;
    mutex mux;
    bool initializeJointAngles = false;
    float jointAngleOffset[4] = {0,0,0,0};

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    controller_manager::ControllerManager *cm = nullptr;
    ros::Subscriber steer_recording_sub, arucoMarker_sub;
    map<int, Vector3f> arucoMarkerPosition;
    ros::ServiceServer init_srv, record_srv, resetSpring_srv;
    ros::Publisher recordResult_pub, jointAnglesOffset_pub, hipCenter_pub;

    roboy_communication_middleware::RoboyState roboyStateMsg;

    vector<ros::Publisher> displacement_pub;

    //! current state of roboy
    ActionState currentState;

    /**
     * Statemachine function for next state
     * @param s current State
     * @return next state
     */
    ActionState NextState(ActionState s);

    //! state strings describing each state
    std::map<ActionState, std::string> state_strings = {
            {WaitForInitialize, "Waiting for initialization of controllers"},
            {Controlloop,       "Control loop"},
            {Recording,         "Recording"},
            {ResetPowerlinkStack,         "Resetting Powerlink Stack"},
            {Dancing,         "Look at me, I'm dancing"}
    };
};

