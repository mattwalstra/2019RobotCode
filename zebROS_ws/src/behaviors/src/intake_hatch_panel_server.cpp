#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <panel_intake_controller/PanelIntakeSrv.h>
#include <behaviors/IntakeAction.h>
#include <behaviors/ElevatorAction.h>
#include "behaviors/enumerated_elevator_indices.h"
#include <behaviors/FinishActionlib.h>

//define global variables that will be defined based on config values
double elevator_timeout = 2;
double pause_time_after_extend = 0.25;
double pause_time_after_clamp = 1;
double wait_for_server_timeout = 5;

class IntakeHatchPanelAction
{
	protected:
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<behaviors::IntakeAction> as_;
		std::string action_name_;

		//Create actionlib client for the elevator server
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;
		//Service client for hatch panel mech
		ros::ServiceClient panel_controller_client_;

		ros::ServiceServer finish_actionlib_server_;
		bool finish_command_sent_; //stores true when callback to finish_actionlib_server_ runs, to false when the command has been processed by executeCB

	public:
		IntakeHatchPanelAction(const std::string &name) :
			as_(nh_, name, boost::bind(&IntakeHatchPanelAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true)
	{
		as_.start();

		//do networking stuff?
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize the client being used to call the controller
		panel_controller_client_ = nh_.serviceClient<panel_intake_controller::PanelIntakeSrv>("/frcrobot_jetson/panel_intake_controller/panel_command", false, service_connection_header);

		//initialize the server that will accept a command to finish the actionlib server (upon button release)
		finish_actionlib_server_ = nh_.advertiseService("finish_actionlib", &IntakeHatchPanelAction::finishActionlibCB, this); //namespaces mean that the name doesn't have to be as specific
		finish_command_sent_ = false;
	}

		~IntakeHatchPanelAction(void) {}

		void executeCB(const behaviors::IntakeGoalConstPtr &/*goal*/)
		{
			ROS_INFO("Hatch Panel Intake Server Running");

			//make sure the elevator server exists
			bool elevator_server_found = ac_elevator_.waitForServer(ros::Duration(wait_for_server_timeout));

			if(!elevator_server_found)
			{
				ROS_ERROR_STREAM("The elevator server was not loaded before the panel intake server needed it");
				as_.setPreempted();
				return;
			}

			ros::Rate r(10);

			//define variables that will be re-used for each call to a controller

			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false; // TODO : never set?

			finish_command_sent_ = false; //make sure this is what we think it should be

			//release claw
			panel_intake_controller::PanelIntakeSrv srv;
			srv.request.claw_release = true;
			srv.request.push_extend = false;
			//send request to controller
			if(!panel_controller_client_.call(srv))
			{
				ROS_ERROR("Panel controller call failed in panel intake server");
				preempted = true;
			}
			ros::spinOnce(); //update everything


			//move elevator to intake location
			behaviors::ElevatorGoal elev_goal;
			elev_goal.setpoint_index = INTAKE;
			elev_goal.place_cargo = false;
			elev_goal.raise_intake_after_success = true;
			ac_elevator_.sendGoal(elev_goal);

			//determine if elevator server timed out or did not time out
			const double start_time = ros::Time::now().toSec();
			bool finished_before_timeout = ac_elevator_.waitForResult(ros::Duration(elevator_timeout - (ros::Time::now().toSec() - start_time)));
			//if finished before timeout then it has either succeeded or failed 
			if(finished_before_timeout) {
				//determine final state of elevator server
				actionlib::SimpleClientGoalState state = ac_elevator_.getState();
				//test if failed
				if(state.toString() != "SUCCEEDED") {
					ROS_ERROR("%s: Elevator Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
					preempted = true;
				}
				else {
					ROS_WARN("%s: Elevator Server ACTION SUCCEEDED",action_name_.c_str());
				}
			}
			else {
				ROS_ERROR("%s: Elevator Server ACTION TIMED OUT",action_name_.c_str());
				timed_out = true;
				//make sure elevator server finishes
				ac_elevator_.cancelAllGoals();
			}

			//test if we got a preempt while waiting
			if(as_.isPreemptRequested())
			{
				preempted = true;
			}
			//send commands to panel_intake_controller to grab the panel ---------------------------------------
			if(!preempted && ros::ok())
			{
				//release claw (NOOT NOOT) and extend panel mechanism
				srv.request.claw_release = true;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv))
				{
					ROS_ERROR("Panel controller call failed in panel intake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything

				//wait until the panel intake button is released before retracting mech and lowering elevator
				while(!finish_command_sent_ && ros::ok() && !preempted)
				{
					//check if preempted
					if(as_.isPreemptRequested())
					{
						preempted = true;
					}
					else
					{
						//wait a bit before iterating the loop again
						ros::spinOnce();
						r.sleep();
					}
				}
				//pause for a bit
				ros::Duration(pause_time_after_extend).sleep();

				//grab the panel - we can reuse the srv variable
				srv.request.claw_release = false;
				srv.request.push_extend = true;
				//send request to controller
				if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
				{
					ROS_ERROR("Panel controller call failed in panel intake server");
					preempted = true;
				}
				ros::spinOnce(); //update everything

				ros::Duration(pause_time_after_clamp).sleep();
			}

			//Set final state - retract the panel mechanism and clamp (to stay within frame perimeter)
			//it doesn't matter if preempted or timed out, do this anyway
			srv.request.claw_release = false;
			srv.request.push_extend = false;
			//send request to controller
			if(!panel_controller_client_.call(srv)) //note: the call won't happen if preempted was true, because of how && operator works
			{
				ROS_ERROR("Panel controller call failed in panel intake server");
				preempted = true;
			}
			ros::spinOnce(); //update everything

			//log state of action and set result of action
			behaviors::IntakeResult result;
			result.timed_out = timed_out;

			if(timed_out)
			{
				ROS_WARN("%s: Timed Out", action_name_.c_str());
				result.success = false;
				as_.setSucceeded(result);
			}
			else if(preempted)
			{
				ROS_WARN("%s: Preempted", action_name_.c_str());
				result.success = false;
				as_.setPreempted(result);
			}
			else //implies succeeded
			{
				ROS_WARN("%s: Succeeded", action_name_.c_str());
				result.success = true;
				as_.setSucceeded(result);
			}
			return;
		}
		bool finishActionlibCB(behaviors::FinishActionlib::Request &req, behaviors::FinishActionlib::Response &/*res*/)
		{
			if(req.finish)
			{
				finish_command_sent_ = true;
			}
			return true;
		}
};


int main(int argc, char** argv)
{
	//create node
	ros::init(argc, argv, "panel_intake_server");

	IntakeHatchPanelAction intake_hatch_panel_server("intake_hatch_panel_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR("Could not read wait_for_server_timeout in panel_intake_server");
	if (!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR("Could not read elevator_timeout in panel_intake_server");
		if (!n_panel_params.getParam("pause_time_after_clamp", pause_time_after_clamp))
		ROS_ERROR("Could not read pause_time_after_clamp in panel_intake_server");

	ros::spin();

	return 0;
}
