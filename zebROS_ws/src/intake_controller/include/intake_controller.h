#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/multi_interface_controller.h>
#include <talon_controllers/talon_controller_interface.h> // "
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <cargo_intake_controller/CargoIntakeSrv.h>

namespace intake_controller
{
class IntakeCommand //define class to hold command data - so we only need 1 realtime buffer
{
    public:
        IntakeCommand()
            : spin_cmd_(0.0)
            , open_arm_cmd_(false)
        {
        }
        IntakeCommand(double spin_cmd, bool open_arm_cmd)
        {
            spin_cmd_ = spin_cmd;
                open_arm_cmd_ = open_arm_cmd;
        }
        double spin_cmd_;
        bool open_arm_cmd_;
}; //class

//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class IntakeController : public controller_interface::MultiInterfaceController<hardware_interface::TalonCommandInterface, hardware_interface::JointStateInterface, hardware_interface::PositionJointInterface>
{
    public:
        IntakeController()
        {
        }

        //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
        //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
        virtual bool init(hardware_interface::RobotHW *hw,
                          ros::NodeHandle             &root_nh,
                          ros::NodeHandle             &controller_nh);
        virtual void starting(const ros::Time &time);
        virtual void update(const ros::Time & time, const ros::Duration& period);
        virtual void stopping(const ros::Time &time);

        virtual bool cmdService(intake_controller::IntakeSrv::Request &req,
                                intake_controller::IntakeSrv::Response &res);
        
    private:
        talon_controllers::TalonPercentOutputControllerInterface intake_joint_; //interface for the spinny part of the intake
			hardware_interface::JointHandle intake_open_joint_; //interface for the up/down arm of the intake

			realtime_tools::RealtimeBuffer<CargoIntakeCommand> intake_cmd_;

            ros::ServiceServer intake_service_; //service for receiving commands

}; //class


} //namespace