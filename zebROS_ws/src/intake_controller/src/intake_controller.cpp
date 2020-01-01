#include "intake_controller/intake_controller.h"

namespace intake_controller
{

bool IntakeController::init(hardware_interface::RobotHW *hw,
                            ros::NodeHandle &/*root_nh*/,
                            ros::NodeHandle &controller_nh)
{
    hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
    hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

    //initialize intake_joint (spinny thing)
	//read intake name from config file
    XmlRpc::XmlRpcValue intake_params;
    if (!controller_nh.getParam("intake_joint", intake_params))
    {
        ROS_ERROR_STREAM("Can not read intake name");
        return false;
    }

	//initialize intake joint
    if (!intake_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_params))
    {
        ROS_ERROR("Cannot initialize intake joint!");
        return false;
    }
    else
    {
        ROS_INFO("Initialized intake joint!");
    }

    intake_service_ = controller_nh.advertiseService("intake_command", &IntakeController::cmdService, this);

	return true;
}

void IntakeController::starting(const ros::Time &/*time*/) 
{
    // set the command to the spinny part of the intake and the command to the in/out part of the intake
    intake_cmd_.writeFromNonRT(IntakeCommand(0, false));
}

void IntakeController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
	//process input for the up/down part of the intake (pneumatic piston)
	const IntakeCommand intake_cmd = *(intake_cmd_.readFromRT());
	double intake_open_cmd_double; //to store processed input
	if(intake_cmd.intake_open_cmd_ == true) {
		intake_open_cmd_double = 1;
	}
	else {
		intake_open_cmd_double = 0;
	}
	//ROS_WARN_STREAM("intake open command: " << intake_open_cmd_double);

	//read spin command
	intake_joint_.setCommand(intake_cmd.spin_cmd_); // set the command to the spinny part of the intake
	intake_open_joint_.setCommand(intake_open_cmd_double); // set the in/out command to the up/down part of the intake
}

void IntakeController::stopping(const ros::Time &/*time*/) {
}

bool IntakeController::cmdService(intake_controller::IntakeSrv::Request &req, intake_controller::IntakeSrv::Response &/*res*/)
{
    if(isRunning())
    {
        //take the service request for a certain amount of power (-1 to 1) and write it to the command variable
		//take the service request for in/out (true/false???) and write to a command variable
		intake_cmd_.writeFromNonRT(IntakeCommand(req.power, req.open));
	}
    else
    {
        ROS_ERROR_STREAM("Can't accept new commands. IntakeController is not running.");
        return false;
    }
    return true;
}
}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(intake_controller::IntakeController, controller_interface::ControllerBase)
