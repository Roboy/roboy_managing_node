#include "roboy_managing_node/roboy.hpp"

bool Roboy::shutdown_flag = false;

Roboy::Roboy(int argc, char* argv[])
{
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "roboy", ros::init_options::NoRosout);
    }

    init_srv = nh->advertiseService("/roboy/initialize", &Roboy::initializeControllers, this);
	record_srv = nh->advertiseService("/roboy/record", &Roboy::record, this);
	steer_recording_sub = nh->subscribe("/roboy/steer_record",1000, &Roboy::steer_record, this);

    initialize(argc, argv);

	cmd = new double[NUMBER_OF_MOTORS_PER_FPGA];
	pos = new double[NUMBER_OF_MOTORS_PER_FPGA];
	vel = new double[NUMBER_OF_MOTORS_PER_FPGA];
	eff = new double[NUMBER_OF_MOTORS_PER_FPGA];

    motorStatus_sub = nh->subscribe("/roboy/middleware/MotorStatus", 1, &Roboy::MotorStatus, this);
    jointStatus_sub = nh->subscribe("/roboy/middleware/JointStatus", 1, &Roboy::JointStatus, this);
	arucoMarker_sub = nh->subscribe("/roboy/middleware/ArucoPose", 1, &Roboy::ArucoPose, this);
    jointAnglesOffset_pub = nh->advertise<roboy_communication_middleware::JointAngle>("/roboy/middleware/JointAngleOffset", 1);
    hipCenter_pub = nh->advertise<geometry_msgs::Pose>("/roboy/middleware/HipCenter", 1);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(4));
    spinner->start();

    jointAngles.resize(4);

    bool initializeJointAngles = true;

}

Roboy::~Roboy()
{
}

void Roboy::JointStatus(const roboy_communication_middleware::JointStatus::ConstPtr &msg) {
    static int counter = 0;
    ROS_INFO_THROTTLE(10, "receiving joint status");
    jointAngles.resize(msg->relAngles.size());
    for (uint joint = 0; joint < jointAngles.size(); joint++) {
        jointAngles[joint] = msg->relAngles[joint]/4096.0f * 360.0f;
    }
    if(!initializeJointAngles){ // if we are initialized publish the corrected joint angles

        ROS_INFO_THROTTLE(1,"joint angles corrected: \n%f\t%f\t%f\t%f", jointAngles[0]+jointAngleOffset[0],
                          jointAngles[1]+jointAngleOffset[1], jointAngles[2]+jointAngleOffset[2],
                          jointAngles[3]+jointAngleOffset[3]);
    }

    if(initializeJointAngles && jointAngleOffset[0]!=0 && jointAngleOffset[1]!=0 &&
            jointAngleOffset[2]!=0 && jointAngleOffset[3]!=0 && counter>20) { // wait until 20 messages have arrived
        initializeJointAngles = !initializeJointAngles;
        roboy_communication_middleware::JointAngle angleOffset_msg;
        for (uint joint = 0; joint < jointAngles.size(); joint++) {
            angleOffset_msg.angle.push_back(jointAngleOffset[joint]);
        }
        jointAnglesOffset_pub.publish(angleOffset_msg);
    }else
        counter++;
}

void Roboy::MotorStatus(const roboy_communication_middleware::MotorStatus::ConstPtr &msg) {
	ROS_INFO_THROTTLE(10, "receiving motor status");
	for (uint motor = 0; motor < msg->position.size(); motor++) {
		pos[motor] = msg->position[motor];
		vel[motor] = msg->velocity[motor];
		eff[motor] = msg->displacement[motor];
	}
}

void Roboy::ArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr &msg) {
	ROS_INFO_THROTTLE(10, "receiving aruco pose");
	for(uint i=0;i<msg->id.size();i++)
	    arucoMarkerPosition[msg->id[i]] = Vector3f(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z);

    Vector3f hipCenter = (arucoMarkerPosition[282]+(arucoMarkerPosition[754]-arucoMarkerPosition[282])/2.0f)-arucoMarkerPosition[429];
    geometry_msgs::Pose hipCenterPose;
    hipCenterPose.position.x = hipCenter[0];
    hipCenterPose.position.y = -hipCenter[1];
    hipCenterPose.position.z = hipCenter[2];
    hipCenter_pub.publish(hipCenterPose);

    if(initializeJointAngles) {
        float anglebetween[4];
        anglebetween[1] = 180.0f - calculateAngleBetween(429, 260, 868, 576);
        anglebetween[0] = 180.0f - calculateAngleBetween(868, 576, 282, 754);
        anglebetween[2] = calculateAngleBetween(282, 754, 1007, 422);
        anglebetween[3] = 360.0f - calculateAngleBetween(1007, 422, 120, 100);
        jointAngleOffset[1] = anglebetween[1] -jointAngles[1];
        jointAngleOffset[0] = anglebetween[0] -jointAngles[0];
        jointAngleOffset[2] = anglebetween[2] -jointAngles[2];
        jointAngleOffset[3] = anglebetween[3] -jointAngles[3];
        ROS_INFO_THROTTLE(1,"angle between 868, 576, 282, 754:    %f \t\tsensor measures %f", anglebetween[0],jointAngles[0]);
        ROS_INFO_THROTTLE(1,"angle between 429, 260, 868, 576:    %f \t\tsensor measures %f", anglebetween[1],jointAngles[1]);
        ROS_INFO_THROTTLE(1,"angle between 282, 754, 1007, 422:   %f \t\tsensor measures %f", anglebetween[2],jointAngles[2]);
        ROS_INFO_THROTTLE(1,"angle between 1007, 422, 120, 100:   %f \t\tsensor measures %f", anglebetween[3],jointAngles[3]);
    }
}

float Roboy::calculateAngleBetween(int aruco0, int aruco1, int aruco2, int aruco3){
    Vector3f line0 = arucoMarkerPosition[aruco1]-arucoMarkerPosition[aruco0];
    Vector3f line1 = arucoMarkerPosition[aruco3]-arucoMarkerPosition[aruco2];

    float len1 = sqrtf(line0[0] * line0[0] + line0[1] * line0[1] + line0[2] * line0[2]);
    float len2 = sqrtf(line1[0] * line1[0] + line1[1] * line1[1] + line1[2] * line1[2]);

    float dot = line0[0] * line1[0] + line0[1] * line1[1] + line0[2] * line1[2];

    float a = dot / (len1 * len2);

    return acos(a)*180.0f/(float)M_PI; // 0..PI
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
                for(uint joint=0;joint<4;joint++) {
					vector<int> motors;
					char str[100];
					sprintf(str, "motors%d", joint);
					nh->getParam(str, motors);
					for(auto id:motors) {
						sprintf(resource, "motor%d", id);
						start_controllers.push_back(resource);
						hardware_interface::JointStateHandle state_handle(resource, &pos[id], &vel[id], &eff[id]);
						jnt_state_interface.registerHandle(state_handle);
						hardware_interface::JointHandle eff_handle(jnt_state_interface.getHandle(resource), &cmd[id]);
						jnt_eff_interface.registerHandle(eff_handle);
						changeControl(id, DISPLACEMENT);
						ROS_INFO("%s dance controller", resource);
					}
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

void Roboy::calibrate(){
	ROS_DEBUG("calibrate");

}

void Roboy::read(){
    ROS_DEBUG("read");
    // nothing to be done here, since the motor status comes via ros topic /roboy/MotorStatus
}
void Roboy::write(){
    ROS_DEBUG("write");
    // lock the mux then write new setPoints, they are continously exchanged with the fpgas via openPowerLink
    std::lock_guard<std::mutex> lock(mux);
    for(uint motor=0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++) {
        changeSetPoint(motor, (int)cmd[motor]);
    }
//    static uint counter = 0;
//    if((counter++)%1000==0){
//        cout << "setPoints: ";
//        for(uint motor=0; motor<NUMBER_OF_MOTORS_PER_FPGA; motor++) {
//            changeSetPoint(motor, (int)cmd[motor]);
//            cout << (int)cmd[motor] << " ";
//        }
//        cout << endl;
//    }
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
				ROS_INFO_THROTTLE(5, "%s", state_strings[Controlloop].c_str());
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

				const ros::Time time = ros::Time::now();
				const ros::Duration period = time - prev_time;

				read();
				write();

				prev_time = time;
				break;
			}
			case Calibration: {
				ROS_INFO_THROTTLE(5, "%s", state_strings[Controlloop].c_str());


				break;
			}
			case Recording: {
				ROS_INFO_THROTTLE(10, "%s", state_strings[Recording].c_str());
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
			case Dancing:{
				ROS_INFO_THROTTLE(10, "%s", state_strings[Dancing].c_str());
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

                const ros::Time time = ros::Time::now();
                const ros::Duration period = time - prev_time;

                read();
                write();

                prev_time = time;
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
			newstate = Calibration;
		case Calibration:
			newstate = Controlloop;
			break;
		case Recording:
			newstate = Recording;
			break;
        case Dancing:
			newstate = Dancing;
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
