#include "roboy_managing_node/roboy.hpp"

bool Roboy::shutdown_flag = false;

Roboy::Roboy(int argc, char* argv[])
{
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy",
                  ros::init_options::NoSigintHandler |
                  ros::init_options::AnonymousName|
                  ros::init_options::NoRosout);
    }

    init_srv = nh->advertiseService("/roboy/initialize", &Roboy::initializeControllers, this);
	record_srv = nh->advertiseService("/roboy/record", &Roboy::record, this);
	steer_recording_sub = nh->subscribe("/roboy/steer_record",1000, &Roboy::steer_record, this);

    initialize(argc, argv);

	cmd = new double[14];
	pos = new double[14];
	vel = new double[14];
	eff = new double[14];

    motorStatus_sub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &Roboy::MotorStatus, this);
    jointStatus_sub = nh->subscribe("/roboy/middleware/JointStatus", 1, &Roboy::JointStatus, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));
    spinner->start();
}

Roboy::~Roboy()
{
}

void Roboy::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    ROS_INFO_THROTTLE(5, "receiving joint status");
    jointAngles.resize(msg->relAngles.size());
    for (uint joint = 0; joint < jointAngles.size(); joint++) {
        jointAngles[joint] = msg->relAngles[joint]/4096.0f * 360.0f;
    }
}

void Roboy::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
	ROS_INFO_THROTTLE(5, "receiving motor status");
	for (uint motor = 0; motor < msg->position.size(); motor++) {
		pos[motor] = msg->position[motor];
		vel[motor] = msg->velocity[motor];
		eff[motor] = msg->displacement[motor];
	}
}

bool Roboy::initializeControllers( roboy_communication_middleware::Initialize::Request &req,
								   roboy_communication_middleware::Initialize::Response &res )
{
    initialized = false;

    vector<string> start_controllers;
    for (uint i=0; i<req.idList.size(); i++){
        char resource[100];
        if((uint)req.controlmode[i]!=3) {
            sprintf(resource, "motor%d", req.idList[i]);
            uint ganglion = req.idList[i] / 4;
            uint motor = req.idList[i] % 4;
            ROS_INFO("motor%d control_mode %d", motor, req.controlmode[i]);

            // connect and register the joint state interface
            start_controllers.push_back(resource);
            hardware_interface::JointStateHandle state_handle(resource, &pos[req.idList[i]], &vel[req.idList[i]],
                                                              &eff[req.idList[i]]);
            jnt_state_interface.registerHandle(state_handle);
        }

		switch((uint)req.controlmode[i]){
			case 0: {
				ROS_INFO("%s position controller", resource);
				// connect and register the joint position interface
				hardware_interface::JointHandle pos_handle(jnt_state_interface.getHandle(resource), &cmd[req.idList[i]]);
				jnt_pos_interface.registerHandle(pos_handle);
                changeControl(req.idList[i],POSITION);
				break;
			}
			case 1: {
				ROS_INFO("%s velocity controller", resource);
				// connect and register the joint position interface
				hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(resource), &cmd[req.idList[i]]);
				jnt_vel_interface.registerHandle(vel_handle);
                changeControl(req.idList[i],VELOCITY);
				break;
			}
			case 2: {
				ROS_INFO("%s force controller", resource);
				// connect and register the joint position interface
				hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(resource), &cmd[req.idList[i]]);
				jnt_eff_interface.registerHandle(eff_handle);
                changeControl(req.idList[i],DISPLACEMENT);
				break;
			}
			case 3: {
				// connect and register the joint position interface
                vector<int> motorIDs;
                nh->getParam("danceMotors", motorIDs);
                for(int motor:motorIDs) {
                    sprintf(resource, "motor%d", motor);
                    start_controllers.push_back(resource);
                    hardware_interface::JointStateHandle state_handle(resource, &pos[motor], &vel[motor], &eff[motor]);
                    jnt_state_interface.registerHandle(state_handle);
                    hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(resource),
                                                               &cmd[motor]);
                    jnt_eff_interface.registerHandle(eff_handle);
                    changeControl(motor, DISPLACEMENT);
                    ROS_INFO("%s dance controller", resource);
                }
				break;
			}
			default:
				ROS_WARN("The requested controlMode is not available, choose [0]PositionController [1]VelocityController "
								 "[2]DisplacementController [3]DanceController");
				break;
		}
    }

    registerInterface(&jnt_state_interface);
	string str;
	registerInterface(&jnt_pos_interface);
	vector<string> resources = jnt_pos_interface.getNames();
    if(!resources.empty())
        str.append("position controllers:\n");
	for(uint i=0; i<resources.size();i++){
		str.append(resources[i]);
		str.append(" ");
	}

	registerInterface(&jnt_vel_interface);
	resources = jnt_vel_interface.getNames();
    if(!resources.empty())
        str.append("velocity controllers:\n");
	for(uint i=0; i<resources.size();i++){
		str.append(resources[i]);
		str.append(" ");
	}

	registerInterface(&jnt_eff_interface);
	resources = jnt_eff_interface.getNames();
    if(!resources.empty())
        str.append("force controllers:\n");
	for (uint i = 0; i < resources.size(); i++) {
		str.append(resources[i]);
		str.append(" ");
	}

    ROS_INFO("Stopping and unloading the controllers that are already running");
    stopControllers(start_controllers);
    unloadControllers(start_controllers);

    ROS_INFO("Resources registered to hardware interface:\n%s", str.c_str());
    if(!loadControllers(start_controllers))
        return false;

    ROS_INFO("Starting controllers now...");
    if(!startControllers(start_controllers))
        ROS_WARN("could not start controllers, try starting via /controller_manager/switch_controller service");

	initialized = true;
    return true;
}

void Roboy::read(){
    ROS_DEBUG("read");
    // nothing to be done here, since the motor status comes via ros topic /roboy/MotorStatus
}
void Roboy::write(){
    ROS_DEBUG("write");
    // lock the mux then write new setPoints, they are continously exchanged with the fpgas via openPowerLink
    std::lock_guard<std::mutex> lock(mux);
    for(uint motor; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        changeSetPoint(motor, cmd[motor]);
    }
}

void Roboy::main_loop(controller_manager::ControllerManager *ControllerManager)
{
    cm = ControllerManager;

    // Control loop
	ros::Time prev_time = ros::Time::now();

    currentState = WaitForInitialize;

    while (ros::ok() && !fExit){
		switch(currentState){
			case WaitForInitialize: {
                ROS_INFO_THROTTLE(5, "%s", state_strings[WaitForInitialize].c_str());
                if(!initialized) {
                    // idle
                    continue;
                }else {
                    // go to next state
                    prev_time = ros::Time::now();
                    break;
                }
			}
			case Controlloop: {
				if (system_getTermSignalState() == TRUE) {
					fExit = true;
					eventlog_printMessage(kEventlogLevelInfo,
										  kEventlogCategoryControl,
										  "Received termination signal, exiting...");
					return;
				}

				if (oplk_checkKernelStack() == FALSE) {
					fExit = true;
					eventlog_printMessage(kEventlogLevelFatal,
										  kEventlogCategoryControl,
										  "Kernel stack has gone! Exiting...");
					return;
				}
				processSync();

				ROS_INFO_THROTTLE(5, "%s", state_strings[Controlloop].c_str());
				const ros::Time time = ros::Time::now();
				const ros::Duration period = time - prev_time;

				read();
				write();

				prev_time = time;
				break;
			}
			case Recording: {
				ROS_INFO_THROTTLE(5, "%s", state_strings[Recording].c_str());
				ros::Duration d(1);
				d.sleep();
				break;
			}
			case ResetPowerlinkStack: {
				ROS_INFO( "%s", state_strings[ResetPowerlinkStack].c_str());
				tOplkError ret = oplk_execNmtCommand(kNmtEventSwReset);
				if (ret != kErrorOk) {
					ROS_ERROR( "oplk_execNmtCommand() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
				}
				break;
			}
		}
		// get next state from state machine
		currentState = NextState(currentState);
    }
}

bool Roboy::loadControllers(vector<string> controllers){
	bool controller_loaded = true;
	for (auto controller : controllers) {
		if(!cm->loadController(controller)) {
			controller_loaded = false;
		}
	}
	return controller_loaded;
}

bool Roboy::unloadControllers(vector<string> controllers){
    bool controller_loaded = true;
    for (auto controller : controllers) {
        if(!cm->unloadController(controller)) {
            controller_loaded = false;
        }
    }
	return controller_loaded;
}

bool Roboy::startControllers(vector<string> controllers) {
    vector<string> stop_controllers;
    int strictness = 1; // best effort
    return cm->switchController(controllers, stop_controllers, strictness);
}

bool Roboy::stopControllers(vector<string> controllers) {
    vector<string> start_controllers;
    int strictness = 1; // best effort
    return cm->switchController(start_controllers, controllers, strictness);
}

ActionState Roboy::NextState(ActionState s)
{
	ActionState newstate;
	switch (s)
	{
		case WaitForInitialize:
			newstate = ResetPowerlinkStack;
			break;
		case Controlloop:
			newstate = Controlloop;
			break;
		case ResetPowerlinkStack:
			newstate = Controlloop;
			break;
		case Recording:
			newstate = Recording;
			break;
	}
	return newstate;
}

bool Roboy::record( roboy_communication_middleware::Record::Request &req,
					roboy_communication_middleware::Record::Response &res) {
//	currentState = Recording;
//	std::vector<std::vector<float>> trajectories;
//	recording = PLAY_TRAJECTORY;
//	vector<int> controllers;
//	vector<int> controlmode;
//	for(auto controller:req.controllers){
//		controllers.push_back(controller.id);
//		controlmode.push_back(controller.controlmode);
//	}
//	float averageSamplingTime = flexray.recordTrajectories(req.sampleRate, trajectories, controllers, controlmode, &recording);
//	res.trajectories.resize(req.controllers.size());
//	for(uint m=0; m<req.controllers.size(); m++){
//		res.trajectories[m].id = req.controllers[m].id;
//		res.trajectories[m].waypoints = trajectories[req.controllers[m].id];
//		res.trajectories[m].samplerate = averageSamplingTime;
//	}
//	currentState = Controlloop;
//    return true;
}

void Roboy::steer_record(const roboy_communication_middleware::Steer::ConstPtr& msg){
	switch (msg->steeringCommand){
		case STOP_TRAJECTORY:
			recording = STOP_TRAJECTORY;
			ROS_INFO("Received STOP recording");
			break;
		case PAUSE_TRAJECTORY:
			if (recording==PAUSE_TRAJECTORY) {
				recording = PLAY_TRAJECTORY;
				ROS_INFO("Received RESUME recording");
			}else {
				recording = PAUSE_TRAJECTORY;
				ROS_INFO("Received PAUSE recording");
			}
			break;
	}
}
