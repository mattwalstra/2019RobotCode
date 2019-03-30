#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/ClimbAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/ElevatorGoal.h"
#include "std_srvs/SetBool.h" //for the climber controller
#include "geometry_msgs/Twist.h" //for the drivebase
#include <atomic>
#include <ros/console.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "frc_msgs/MatchSpecificData.h"
#include <thread>
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "cargo_intake_controller/CargoIntakeSrv.h"
#include <talon_state_controller/TalonState.h>

//define global variables that will be defined based on config values

//durations and timeouts
double elevator_deploy_timeout;
double elevator_climb_timeout;
double running_forward_timeout;
double elevator_climb_low_timeout;
double match_time_lock;
double wait_for_server_timeout;
double wait_at_top_for_engage;
double pull_leg_up_pause_time;

//other config variables
double drive_forward_speed;
double climber_engage_pos;

class ClimbAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::ClimbAction> as_; //create the actionlib server
		std::string action_name_;

		//create clients to call other actionlib servers
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ae_; //to call the elevator

		//create clients/subscribers to activate controllers
		ros::ServiceClient climber_controller_client_; //create a ros client to send requests to the climber controller (piston in the leg)
		ros::ServiceClient climber_engage_client_; //ros client to engage the climber via the elevator controller
		ros::Publisher cmd_vel_publisher_;

		//create subscribers to get data
		ros::Subscriber match_data_sub_;
		ros::Subscriber navX_sub_;
		ros::Subscriber talon_states_sub_;

		// Data from subscribers
		double match_time_remaining_;
		double navX_roll_;
		double navX_pitch_;
		double elev_cur_position_;

		std::atomic<double> cmd_vel_forward_speed_;
		std::atomic<bool> stopped_;

		ros::ServiceClient cargo_intake_controller_client_; //create a ros client to send requests to the controller

		// Try to safely wait for an elevator action to succeed
		void waitForElevator(bool &timed_out, bool &preempted, int step, ros::Rate &r, double timeout)
		{
			bool waiting_for_elevator = true;
			const double start_time = ros::Time::now().toSec();
			while (waiting_for_elevator && ros::ok())
			{
				auto state = ae_.getState();
				if ((state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) ||
					(state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED))
				{
					waiting_for_elevator = false;
					if (state == actionlib::SimpleClientGoalState::StateEnum::PREEMPTED)
					{
						ROS_INFO_STREAM("elevator " << step << " state returned preempted");
						preempted = true;
					}
					if (!ae_.getResult()->success)
					{
						ROS_INFO_STREAM("elevator " << step << " did not succeed -- climber_server");
						preempted = true;
					}
				}
				else if(as_.isPreemptRequested())
				{
					waiting_for_elevator = false;
					preempted = true;
				}
				else if((ros::Time::now().toSec() - start_time) > timeout)
				{
					ROS_ERROR_STREAM("climber server step " << step << " : timed out");
					timed_out = true;
					waiting_for_elevator = false;
				}
				else
				{
					ros::spinOnce();
					r.sleep();
				}
			}
		}

		// Basic thread which spams cmd_vel to the drive base to
		// continually drive forward during the climb
		void cmdVelThread()
		{
			ROS_INFO_STREAM("the callback is being called");
			geometry_msgs::Twist cmd_vel_msg;
			stopped_ = false;

			ros::Rate r(20);

			while(ros::ok() && !stopped_)
			{
				cmd_vel_msg.linear.x = cmd_vel_forward_speed_;
				cmd_vel_msg.linear.y = 0.0;
				cmd_vel_msg.linear.z = 0.0;
				cmd_vel_msg.angular.x = 0.0;
				cmd_vel_msg.angular.y = 0.0;
				cmd_vel_msg.angular.z = 0.0;

				cmd_vel_publisher_.publish(cmd_vel_msg);
				r.sleep();
			}
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::ClimbGoalConstPtr &goal) {
			if(match_time_remaining_ > match_time_lock)
			{
				ROS_ERROR_STREAM("can not climb, too much time remaining in match");
				return;
			}

			if(!ae_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The elevator server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}
			if(!climber_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The climber foot server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}
			if(!climber_engage_client_.waitForExistence(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("The climber engage server was not loaded before the climber server needed it");
				as_.setPreempted();
				return;
			}

			/* Steps to climb:
			 * Climber Server Step 0
			 * 1. Deploy foot via climber controller
			 * 2. Raise elevator to right height via elevator actionlib server
			 * 3. Deploy climber engagement piston via elevator controller
			 * 4. Lower elevator to right height to make the robot climb (this should be slow... change pid somehow?)
			 * 5. Retract foot to drop robot onto platform
			 * (driver drives forward a bit)
			 *
			 * Climber Server Step 1
			 * 6. Pull climber all the way up
			 * (driver drives forward a bit)
			 *
			 * Climber Server Step 2
			 * 7. Push climber a bit down to prop back wheels up
			 */
			ros::Rate r(20);

			std::thread cmdVelThread(std::bind(&ClimbAction::cmdVelThread, this));

			//define variables that will be reused for each controller call/actionlib server call
			//define variables that will be set true if the actionlib action is to be ended
			//this will cause subsequent controller calls to be skipped, if the template below is copy-pasted
			//if both of these are false, we assume the action succeeded
			bool preempted = false;
			bool timed_out = false;

			if(goal->step == 0)
			{
				ROS_INFO("Running climber server step 0");
				cmd_vel_forward_speed_ = 0;
				//deploy foot using climber controller -----------------------------------------------
				//define service to send
				std_srvs::SetBool srv;
				srv.request.data = false; //shouldn't do anything, this is default
				//call controller
				if(!climber_controller_client_.call(srv))
				{
					ROS_ERROR("Foot deploy failed in climber controller");
					preempted = true;
				}
				else
				{
					//only spin if we're not going to error out
					// TODO - Why spin here?
					ros::spinOnce();
				}

				cargo_intake_controller::CargoIntakeSrv cargo_srv;
				cargo_srv.request.power = 0;
				cargo_srv.request.intake_arm = false;
				//send request to controller
				if(!cargo_intake_controller_client_.call(cargo_srv))
				{
					ROS_ERROR("Cargo outtake server: controller call failed to lift arm up");
					preempted = true;
				}

				// raise elevator to right height so we can engage the climber ------------------------------------------------
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 0: raising elevator before climber is engaged");
					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_DEPLOY;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					elevator_goal.raise_intake_after_success = true;
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_deploy_timeout);
				} //end of raise elevator to right height before engaging
			}
			else if(goal->step == 1)
			{
				//engage climber with elevator controller -----------------------------------------------------------------
				if(!preempted && ros::ok())
				{
					ROS_INFO("climber server step 1: engaging the climber with the elvator");

					//call the elevator controller
					//define the goal to send
					std_srvs::SetBool srv;
					srv.request.data = true;
					//call controller
					if(!climber_engage_client_.call(srv))
					{
						ROS_ERROR("climber server step 1: Climber engage failed in climber controller");
						preempted = true;
					}
					// TODO - Why spin here?
					ros::spinOnce();

					// delay to make sure that we engaged properly
					// TODO - if we need to spin, spin in a loop here for 1 second here?
					ros::Duration(wait_at_top_for_engage).sleep();
				}

				cmd_vel_forward_speed_ = drive_forward_speed;

				//lower elevator to make robot rise off ground
				if(!preempted && !timed_out && ros::ok())
				{
					ROS_INFO("climber server step 1: lowering elevator to make robot climb");

					//call the elevator actionlib server
					//define the goal to send
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_CLIMB;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					elevator_goal.raise_intake_after_success = true;
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_deploy_timeout);
				} //end of lowering elevator to make robot climb

				ROS_INFO_STREAM("preempted = " << preempted);

				//handle preempting/timed out
				if(preempted || timed_out)
				{
					ROS_WARN_STREAM("Climber server timed out or was preempted");
					ae_.cancelAllGoals();
					std_srvs::SetBool srv;
					srv.request.data = false;

					if(!climber_engage_client_.call(srv))
					{
						ROS_ERROR("climber server step 1: Climber PANIC failed in climber controller");
					}
				}

				ROS_INFO_STREAM("preempted = " << preempted);

				if(!preempted && !timed_out)
				{
					//retract climber foot to make robot fall
					//define service to send
					std_srvs::SetBool srv;
					srv.request.data = true;
					//call controller
					if(!climber_controller_client_.call(srv))
					{
						ROS_ERROR("climber server step 1: Foot retract failed in climber controller");
						preempted = true;
					}
					else {
						//only spin if we're not going to error out
						ros::spinOnce();
					}
				}

				ROS_INFO_STREAM("Driving forward");
				double start_time = ros::Time::now().toSec();
				while(ros::ok() && !preempted && !timed_out)
				{
					timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
					preempted = as_.isPreemptRequested();
					r.sleep();
				}
				if(timed_out)
					ROS_INFO_STREAM("Driving forward after fall has timed out");
				if(preempted)
					ROS_INFO_STREAM("Driving forward after fall was preempted");

				//preempt handling: do nothing
			}
			else if(goal->step == 2)
			{
				if(!preempted && !timed_out && ros::ok())
				{
					// TODO - is this info correct?
					ROS_INFO("climber server step 2: raising elevator to pull climber leg up a bit");

					cargo_intake_controller::CargoIntakeSrv srv;
					srv.request.power = 0;
					srv.request.intake_arm = true;
					//send request to controller
					if(!cargo_intake_controller_client_.call(srv))
					{
						ROS_ERROR("Cargo outtake server: controller call failed to lift arm up");
						preempted = true;
					}

					//pull the climber leg up a bit
					//define the goal to send
					behaviors::ElevatorGoal elevator_goal;
					elevator_goal.setpoint_index = ELEVATOR_CLIMB_LOW;
					elevator_goal.place_cargo = 0; //doesn't actually do anything
					elevator_goal.raise_intake_after_success = true;
					//send the elevator_goal
					ae_.sendGoal(elevator_goal);
					waitForElevator(timed_out, preempted, goal->step, r, elevator_climb_timeout);
				} //end of raising elevator to make robot climb


				//pause a bit, driver can preempt now if something looks wrong
				double start_time = ros::Time::now().toSec();
				while(ros::ok() && !preempted)
				{
					if(as_.isPreemptRequested())
					{
						preempted = true;
					}
					if((ros::Time::now().toSec() - start_time) > pull_leg_up_pause_time)
					{
						break;
					}
				}


				//pull climber leg all the way up - move elevator to climber deploy setpoint
				ROS_INFO("climber server step 2: raising elevator to pull climber leg rest of the way up");

				//call the elevator actionlib server
				//define the goal to send
				behaviors::ElevatorGoal elevator_goal;
				elevator_goal.setpoint_index = ELEVATOR_RAISE;
				elevator_goal.place_cargo = 0; //doesn't actually do anything
				elevator_goal.raise_intake_after_success = true;
				//send the elevator_goal
				ae_.sendGoal(elevator_goal);
				waitForElevator(timed_out, preempted, goal->step, r, elevator_climb_timeout);

				//determine the outcome of the goal
				if (!preempted)
				{
					//Drive forward until drive forward timeout or end of game
					ROS_INFO_STREAM("Driving forward at end of climb");
					const double start_time = ros::Time::now().toSec();

					while(ros::ok() && !preempted && !timed_out)
					{
						timed_out = (ros::Time::now().toSec() -  start_time) > running_forward_timeout;
						preempted = as_.isPreemptRequested();
						r.sleep();
					}
				}

				//preempt handling: preempt elevator server to freeze the elevator
				if(preempted || timed_out)
				{
					ROS_INFO("Running climber server step 2 preempt/timeout handling - preempting elevator server");
					ae_.cancelGoalsAtAndBeforeTime(ros::Time::now());
				}
			}

			//log state of action and set result of action

			// TODO : timed_out is never set
			behaviors::ClimbResult result; //variable to store result of the actionlib action
			if(timed_out)
			{
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);

				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			else //implies succeeded
			{
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			stopped_ = true;
			cmdVelThread.join();

			return;
		}

		void navXCallback(const sensor_msgs::Imu &navXState)
		{
			const tf2::Quaternion navQuat(navXState.orientation.x, navXState.orientation.y, navXState.orientation.z, navXState.orientation.w);
			double roll;
			double pitch;
			double yaw;
			tf2::Matrix3x3(navQuat).getRPY(roll, pitch, yaw);

			if (roll == roll) // ignore NaN results
				navX_roll_ = roll;

			if (pitch == pitch) // ignore NaN results
				navX_pitch_ = pitch;
		}


		void talonStateCallback(const talon_state_controller::TalonState &talon_state)
		{
			static size_t elevator_master_idx = std::numeric_limits<size_t>::max();
			if (elevator_master_idx >= talon_state.name.size())
			{
				for (size_t i = 0; i < talon_state.name.size(); i++)
				{
					if (talon_state.name[i] == "elevator_master")
					{
						elevator_master_idx = i;
						break;
					}
				}
			}
			else {
				elev_cur_position_ = talon_state.position[elevator_master_idx];
			}
		}


		void matchStateCallback(const frc_msgs::MatchSpecificData &msg)
		{
			match_time_remaining_ = msg.matchTimeRemaining;
		}

	public:
		//make the executeCB function run every time the actionlib server is called
		ClimbAction(const std::string &name) :
			as_(nh_, name, boost::bind(&ClimbAction::executeCB, this, _1), false),
			action_name_(name),
			ae_("/elevator/elevator_server", true)
		{
			as_.start(); //start the actionlib server

			//do networking stuff?
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			//get the match timer
			match_data_sub_ = nh_.subscribe("/frcrobot_rio/match_data", 1, &ClimbAction::matchStateCallback,this);
			//initialize the client being used to call the climber controller
			climber_controller_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_feet_retract", false, service_connection_header);
			//initialize the client being used to call the climber controller to engage the climber
			climber_engage_client_ = nh_.serviceClient<std_srvs::SetBool>("/frcrobot_jetson/climber_controller/climber_release_endgame", false, service_connection_header);

			navX_sub_ = nh_.subscribe("/frcrobot_rio/navx_mxp", 1, &ClimbAction::navXCallback, this);
			talon_states_sub_ = nh_.subscribe("/frcrobot_jetson/talon_states",1,&ClimbAction::talonStateCallback, this);

			cargo_intake_controller_client_ = nh_.serviceClient<cargo_intake_controller::CargoIntakeSrv>("/frcrobot_jetson/cargo_intake_controller/cargo_intake_command", false, service_connection_header);

			//initialize the publisher used to send messages to the drive base
			cmd_vel_publisher_ = nh_.advertise<geometry_msgs::Twist>("swerve_drive_controller/cmd_vel", 1);
			//start subscribers subscribing
			//joint_states_sub_ = nh_.subscribe("/frcrobot_jetson/joint_states", 1, &ClimbAction::jointStateCallback, this);
		}

		~ClimbAction(void)
		{
		}

};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "climb_server");

	//create the cargo intake actionlib server
	ClimbAction climb_action("climber_server");

	//get config values
	ros::NodeHandle n;
	ros::NodeHandle n_climb_params(n, "climber_server");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
	{
		ROS_ERROR("Could not read wait_for_server_timeout in climber_server");
		wait_for_server_timeout = 10;
	}

	if (!n_climb_params.getParam("wait_at_top_for_engage", wait_at_top_for_engage))
	{
		ROS_ERROR("Could not read wait_at_top_for_engage in climber_server");
		wait_at_top_for_engage = 1;
	}

	if (!n_climb_params.getParam("deploy_timeout", elevator_deploy_timeout))
	{
		ROS_ERROR("Could not read elevator_deploy_timeout in climber_server");
		elevator_deploy_timeout = 6;
	}

	if (!n_climb_params.getParam("climb_timeout", elevator_climb_timeout))
	{
		ROS_ERROR("Could not read elevator_climb_timeout in climber_server");
		elevator_climb_timeout = 20;
	}

	if (!n_climb_params.getParam("running_forward_timeout", running_forward_timeout))
	{
		ROS_ERROR("Could not read running_forward_timeout in climber_server");
		running_forward_timeout= 2;
	}

	if (!n_climb_params.getParam("climb_low_timeout", elevator_climb_low_timeout))
	{
		ROS_ERROR("Could not read climb_low_timeout in climber_server");
		elevator_climb_low_timeout = 6;
	}

	if (!n_climb_params.getParam("match_time_lock", match_time_lock))
	{
		ROS_ERROR("Could not read match_time_lock in climber_server");
		match_time_lock = 135;
	}

	if (!n_climb_params.getParam("pull_leg_up_pause_time", pull_leg_up_pause_time))
	{
		ROS_ERROR("Could not read pull_leg_up_pause_time in climber_server");
		pull_leg_up_pause_time = 0.5;
	}

	if (!n_climb_params.getParam("drive_forward_speed", drive_forward_speed))
	{
		ROS_ERROR("Could not read drive_forward_speed in climber_server");
		drive_forward_speed = 0.2;
	}

	ros::spin();
	return 0;
}
