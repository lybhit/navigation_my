#include <pluginlib/class_list_macros.h>
 #include "global_planner_line/global_planner_line.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner_line::GlobalPlannerLine, nav_core::BaseGlobalPlanner)

 using namespace std;

 //Default Constructor
 namespace global_planner_line {

 GlobalPlannerLine::GlobalPlannerLine(): costmap_ros_(NULL), initialized_(false){}

 GlobalPlannerLine::GlobalPlannerLine(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void GlobalPlannerLine::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	 
	 if (initialized_){
		 costmap_ros_ = costmap_ros;
		 costmap_ = costmap_ros_ ->getCostmap();
		 
		 ros::NodeHandle private_nh("~/" + name);
		 private_nh.param("step_size", step_size_, costmap_ ->getResolution());
		 private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
		 world_model_ = new base_local_planner::CostmapModel(*costmap_);
		 
		 initialized_ = true;
 }
     else
	   ROS_WARN("This planner has already been initialized... doing nothing");
 }
 
 
 double GlobalPlannerLine::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }

 bool GlobalPlannerLine::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    plan.push_back(start);
	
	double x0, y0, x1, y1, len;
	x0 = start.pose.position.x;
	y0 = start.pose.position.y;
	x1 = goal.pose.position.x;
	y1 = goal.pose.position.y;
	len = sqrt((x1 -x0) * (x1 - x0) + (y1 -y0) * (y1 - y0));
	
	double delta_len = 0.02;
	int step_num = ceil(len / delta_len - 0.5);
	
	double theta = atan2((y1 - y0),(x1 -x0));
	
   for (int i=0; i<step_num; i++){
     geometry_msgs::PoseStamped new_goal = start;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(theta);

      new_goal.pose.position.x += delta_len * cos(theta);
      new_goal.pose.position.y += delta_len * sin(theta);

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

   plan.push_back(new_goal);
   }
   plan.push_back(goal);
  return true;
 }
 };