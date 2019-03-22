#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_srvs/Empty.h"
#include "behaviors/AlignGoal.h"
#include "behaviors/AlignAction.h"
#include "behaviors/ElevatorAction.h"
#include "behaviors/enumerated_elevator_indices.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "behaviors/PathGoal.h"
#include "behaviors/PathAction.h"
#include "goal_detection/GoalDetection.h"
//tf stuff
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


double align_timeout;
double orient_timeout;
double orient_error_threshold;
double x_error_threshold;
double cargo_error_threshold;
double time_to_path;
double wait_for_server_timeout;

double elevator_timeout;
double path_timeout;

bool goals_found;

//bool startup = true; //disable all pid nodes on startup
class AlignAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behaviors::AlignAction> as_; //create the actionlib server
		std::string action_name_;
		behaviors::AlignResult result_; //variable to store result of the actionlib action
		actionlib::SimpleActionClient<behaviors::ElevatorAction> ac_elevator_;
		actionlib::SimpleActionClient<behaviors::PathAction> ac_path_;
        
		std::shared_ptr<ros::Publisher> enable_navx_pub_;
		std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_;
		std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_;
		std::shared_ptr<ros::Publisher> enable_y_pub_;
		std::shared_ptr<ros::Publisher> enable_align_hatch_pub_;
		std::shared_ptr<ros::Publisher> enable_align_cargo_pub_;
		std::shared_ptr<ros::Publisher> enable_cargo_pub_;

		ros::Subscriber navx_error_sub_;
		ros::Subscriber hatch_panel_distance_error_sub_;
		ros::Subscriber cargo_distance_error_sub_;
		ros::Subscriber y_error_sub_;
		ros::Subscriber cargo_error_sub_;
		message_filters::Subscriber<geometry_msgs::PointStamped> zed_msg_sub_;

        ros::ServiceClient BrakeSrv;

		bool hatch_panel_distance_aligned_ = false;
		bool cargo_distance_aligned_ = false;
		bool y_aligned_ = false;
		bool cargo_aligned_ = false;
		bool orient_aligned_ = false;
		bool distance_aligned_ = false;

		//tf stuff
		std::string target_frame_;
		tf2_ros::Buffer buffer_;
		tf2_ros::TransformListener tf2_;
		tf2_ros::MessageFilter<geometry_msgs::PointStamped> tf2_filter_;

		geometry_msgs::PointStamped relative_goal_location_;

	public:
		//make the executeCB function run every time the actionlib server is called
		AlignAction(const std::string &name,
			const std::shared_ptr<ros::Publisher>& enable_navx_pub_,
			const std::shared_ptr<ros::Publisher>& hatch_panel_enable_distance_pub_,
			const std::shared_ptr<ros::Publisher>& cargo_enable_distance_pub_,
			const std::shared_ptr<ros::Publisher>& enable_y_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_hatch_pub_,
			const std::shared_ptr<ros::Publisher>& enable_align_cargo_pub_,
			const std::shared_ptr<ros::Publisher>& enable_cargo_pub_):
			as_(nh_, name, boost::bind(&AlignAction::executeCB, this, _1), false),
			action_name_(name),
			ac_elevator_("/elevator/elevator_server", true),
			ac_path_("/path_to_goal/path_server", true),
			enable_navx_pub_(enable_navx_pub_),
			hatch_panel_enable_distance_pub_(hatch_panel_enable_distance_pub_),
			cargo_enable_distance_pub_(cargo_enable_distance_pub_),
			enable_y_pub_(enable_y_pub_),
			enable_align_hatch_pub_(enable_align_hatch_pub_),
			enable_align_cargo_pub_(enable_align_cargo_pub_),
			enable_cargo_pub_(enable_cargo_pub_),
			tf2_(buffer_),
			target_frame_("panel_outtake"),
			tf2_filter_(zed_msg_sub_, buffer_, target_frame_, 10, 0)
		{
            as_.start();

            std::map<std::string, std::string> service_connection_header;
            service_connection_header["tcp_nodelay"] = "1";
			BrakeSrv = nh_.serviceClient<std_srvs::Empty>("/frcrobot_jetson/swerve_drive_controller/brake", false, service_connection_header);

			navx_error_sub_ = nh_.subscribe("navX_pid/pid_debug", 1, &AlignAction::navx_error_cb, this);
			hatch_panel_distance_error_sub_ = nh_.subscribe("hatch_panel_distance_pid/pid_debug", 1, &AlignAction::hatch_panel_distance_error_cb, this);
			cargo_distance_error_sub_ = nh_.subscribe("cargo_distance_pid/pid_debug", 1, &AlignAction::cargo_distance_error_cb, this);
			cargo_error_sub_ = nh_.subscribe("cargo_pid/pid_debug", 1, &AlignAction::cargo_error_cb, this);
			y_error_sub_ = nh_.subscribe("align_with_terabee/y_aligned", 1, &AlignAction::y_error_cb, this);
			zed_msg_sub_.subscribe(nh_, "/zed_data/pointstamped_goal_msg", 1);

			//tf stuff
			tf2_filter_.registerCallback(boost::bind(&AlignAction::ZEDMsgCB, this, _1));
		}

		~AlignAction(void)
		{
		}

		//Error callbacks
		void navx_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			orient_aligned_ = (fabs(msg.data[0]) < orient_error_threshold);
			//ROS_WARN_STREAM_THROTTLE(1, "navX error: " << fabs(msg.data[0]));
		}
		void hatch_panel_distance_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			hatch_panel_distance_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			//ROS_WARN_STREAM("distance error: " << msg.data[0]);
		}
		void cargo_distance_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			cargo_distance_aligned_ = (fabs(msg.data[0]) < x_error_threshold);
			ROS_ERROR_STREAM("distance error: " << msg.data[0]);
		}
		void y_error_cb(const std_msgs::Bool &msg)
		{
			y_aligned_ = msg.data;
		}
		void cargo_error_cb(const std_msgs::Float64MultiArray &msg)
		{
			cargo_aligned_ = (fabs(msg.data[0]) < cargo_error_threshold);
			//ROS_WARN_STREAM_THROTTLE(1, "cargo error: " << msg.data[0]);
		}
		void ZEDMsgCB(const geometry_msgs::PointStampedConstPtr &raw_goal_location)
		{
			goals_found = true;
			try
			{
				buffer_.transform(*raw_goal_location, relative_goal_location_, target_frame_);
			}
			catch (tf2::TransformException &ex)
			{
				ROS_WARN("Failed %s\n", ex.what());
			}
		}

		//define the function to be executed when the actionlib server is called
		void executeCB(const behaviors::AlignGoalConstPtr &goal) {
			ROS_INFO_STREAM("align server callback called");

			if(!ac_path_.waitForServer(ros::Duration(wait_for_server_timeout)))
			{
				ROS_ERROR_STREAM("Align server couldn't find pathing server");
				as_.setPreempted();
				return;
			}

			ros::Rate r(30);

            static bool elevator_moved = false;

			double start_time = ros::Time::now().toSec();
			bool preempted = false;
			bool timed_out = false;
			bool orient_timed_out = false;
			bool aligned = false;
			bool success = false;

			orient_aligned_ = false;
			distance_aligned_ = false;
			hatch_panel_distance_aligned_ = false;
			cargo_distance_aligned_ = false;
			y_aligned_ = false;

			if(goal->has_cargo)
			{
			}
			else
			{
				ROS_INFO_STREAM("Running align with hatch panel");
				while(!orient_aligned_ && !orient_timed_out && ros::ok())
				{
					ROS_INFO_STREAM_THROTTLE(0.25, "Running the orient align in hatch_panel align");
					ROS_INFO_STREAM("orient aligned = " << orient_aligned_ << " orient_timed_out = " << orient_timed_out);
					ros::spinOnce();
					r.sleep();

					std_msgs::Bool orient_msg;
					orient_msg.data = !orient_timed_out && !orient_aligned_;
					enable_navx_pub_->publish(orient_msg);

					orient_timed_out = (ros::Time::now().toSec() - start_time) > orient_timeout;
				}
				//Just make sure the orient stops
				std_msgs::Bool false_msg;
				false_msg.data = false;
				enable_navx_pub_->publish(false_msg);

				if(goals_found)
				{
					ROS_INFO_STREAM("Sending the pathing goal");
					behaviors::PathGoal path_location_goal;
					path_location_goal.x = relative_goal_location_.point.x;
					path_location_goal.y = relative_goal_location_.point.y;
					path_location_goal.rotation = 0;
					path_location_goal.time_to_run = time_to_path;
					ac_path_.sendGoal(path_location_goal);
					bool finished_before_timeout = ac_path_.waitForResult(ros::Duration(path_timeout)); //TODO VERY IMPORTANT -- CHANGE TIMEOUTS IN PATH SERVER
					if(finished_before_timeout) {
						actionlib::SimpleClientGoalState state = ac_path_.getState();
						if(state.toString() != "SUCCEEDED") {
							ROS_ERROR("%s: Path Server ACTION FAILED: %s",action_name_.c_str(), state.toString().c_str());
							preempted = true;
						}
						else {
							ROS_WARN("%s: Path Server ACTION SUCCEEDED",action_name_.c_str());
							success = true;
						}
					}
					else {
						ROS_ERROR("%s: Path Server ACTION TIMED OUT",action_name_.c_str());
						timed_out = true;
					}
				}
				else
				{
					ROS_INFO_STREAM("Not sending the pathing goal because nothing was detected");
				}

			}

			if(timed_out)
			{
				result_.timed_out = true;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Timed Out", action_name_.c_str());
			}
			else if(preempted)
			{
				result_.timed_out = false;
				result_.success = false;
				as_.setPreempted(result_);
				ROS_INFO("%s: Preempted", action_name_.c_str());
			}
			// Only for the case where the BrakeSrv call failed
			// Does this make sense?
			else if (!success)
			{
				result_.timed_out = false;
				result_.success = false;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Not Successful", action_name_.c_str());
			}
			else //implies succeeded
			{
				result_.timed_out = false;
				result_.success = true;
				as_.setSucceeded(result_);

				ROS_INFO("%s: Succeeded", action_name_.c_str());
			}

			return;
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "align_server");

	ros::NodeHandle n;
	ros::NodeHandle n_params(n, "align_server_params");
    ros::NodeHandle n_panel_params(n, "actionlib_hatch_panel_intake_params");

	if (!n.getParam("/actionlib_params/wait_for_server_timeout", wait_for_server_timeout))
		ROS_ERROR_STREAM("Could not read wait_for_server_timeout in align_sever");
	if(!n_params.getParam("align_timeout", align_timeout))
		ROS_ERROR_STREAM("Could not read align_timeout in align_server");
	if(!n_params.getParam("orient_timeout", orient_timeout))
		ROS_ERROR_STREAM("Could not read orient_timeout in align_server");
	if(!n_params.getParam("x_error_threshold", x_error_threshold))
		ROS_ERROR_STREAM("Could not read x_error_threshold in align_server");
	if(!n_params.getParam("orient_error_threshold", orient_error_threshold))
		ROS_ERROR_STREAM("Could not read orient_error_threshold in align_server");
	if(!n_params.getParam("cargo_error_threshold", cargo_error_threshold))
		ROS_ERROR_STREAM("Could not read cargo_error_threshold in align_server");
	if(!n_params.getParam("path_timeout", path_timeout))
		ROS_ERROR_STREAM("Could not read path_timeout in align_server");
	if(!n_params.getParam("time_to_path", time_to_path))
		ROS_ERROR_STREAM("Could not read time_to_path in align_server");

	if(!n_panel_params.getParam("elevator_timeout", elevator_timeout))
		ROS_ERROR_STREAM("Could not read elevator_timeout in align_server");

	std::shared_ptr<ros::Publisher> enable_navx_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> hatch_panel_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> cargo_enable_distance_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_y_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_hatch_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_align_cargo_pub_ = std::make_shared<ros::Publisher>();
	std::shared_ptr<ros::Publisher> enable_cargo_pub_ = std::make_shared<ros::Publisher>();

	*enable_navx_pub_ = n.advertise<std_msgs::Bool>("navX_pid/pid_enable", 1,  true);
	*hatch_panel_enable_distance_pub_ = n.advertise<std_msgs::Bool>("hatch_panel_distance_pid/pid_enable", 1,  true);
	*cargo_enable_distance_pub_ = n.advertise<std_msgs::Bool>("cargo_distance_pid/pid_enable", 1,  true);
	*enable_y_pub_ = n.advertise<std_msgs::Bool>("align_with_terabee/enable_y_pub", 1,  true);
	*enable_cargo_pub_ = n.advertise<std_msgs::Bool>("cargo_pid/pid_enable", 1,  true);
	*enable_align_hatch_pub_ = n.advertise<std_msgs::Bool>("align_hatch_pid/pid_enable", 1,  true);
	*enable_align_cargo_pub_ = n.advertise<std_msgs::Bool>("align_cargo_pid/pid_enable", 1,  true);

	AlignAction align_action("align_server", enable_navx_pub_, hatch_panel_enable_distance_pub_, cargo_enable_distance_pub_, enable_y_pub_, enable_align_hatch_pub_, enable_align_cargo_pub_, enable_cargo_pub_);

	ros::Rate r(20);
	//Stop PID nodes from defaulting true
	std_msgs::Bool false_msg;
	false_msg.data = false;
	enable_navx_pub_->publish(false_msg);
	hatch_panel_enable_distance_pub_->publish(false_msg);
	cargo_enable_distance_pub_->publish(false_msg);
	enable_y_pub_->publish(false_msg);
	enable_cargo_pub_->publish(false_msg);
	enable_align_hatch_pub_->publish(false_msg);
	enable_align_cargo_pub_->publish(false_msg);

    ros::spin();
	return 0;
}
