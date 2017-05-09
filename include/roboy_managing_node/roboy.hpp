#pragma once

#include "roboy_managing_node/roboy.hpp"
#include "roboy_managing_node/myoMaster.hpp"
#include "common_utilities/CommonDefinitions.h"
#include "common_utilities/Initialize.h"
#include "common_utilities/EmergencyStop.h"
#include "common_utilities/Record.h"
#include <common_utilities/Steer.h>
#include "common_utilities/Trajectory.h"
#include "common_utilities/RoboyState.h"

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

using namespace std;

//! enum for state machine
typedef enum {
    WaitForInitialize,
    LoadControllers,
    Controlloop,
    PublishState,
    Recording
} ActionState;

class Roboy : public hardware_interface::RobotHW {
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
    bool initializeControllers(common_utilities::Initialize::Request &req,
                               common_utilities::Initialize::Response &res);
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
    bool record(common_utilities::Record::Request &req,
                common_utilities::Record::Response &res);

    /**
     * SUBSCRIBER enables pause/resume and stop recording
     */
    void steer_record(const common_utilities::Steer::ConstPtr &msg);

    ros::NodeHandle nh;
    double *cmd;
    double *pos;
    double *vel;
    double *eff;
    ros::Time prevTime;
    int8_t recording;
    bool initialized = false;
    static bool shutdown_flag;

    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    controller_manager::ControllerManager *cm = nullptr;
    ros::Subscriber steer_recording_sub;
    ros::ServiceServer init_srv, record_srv, resetSpring_srv;
    ros::Publisher recordResult_pub;

    MyoMaster *myoMaster;
    common_utilities::RoboyState roboyStateMsg;

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
            {Recording,         "Recording"}
    };
};

