/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/


#include "../include/base_local_planner/trajectory_planner_ros.h"

#include <sys/time.h>
#include <boost/tokenizer.hpp>

#include <eigen3/Eigen/Core>
#include <cmath>

//#include <ros/console.h>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <iostream>
#include <string>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::TrajectoryPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner {

  void TrajectoryPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level) {
      if (setup_ && config.restore_defaults) {
        config = default_config_;
        //Avoid looping
        config.restore_defaults = false;
      }
      if ( ! setup_) {
        default_config_ = config;
        setup_ = true;
      }
      tc_->reconfigure(config);
      reached_goal_ = false;
  }


  TrajectoryPlannerROS::TrajectoryPlannerROS() :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom"),_nh("TrajectoryPlannerROS") {}

  TrajectoryPlannerROS::TrajectoryPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
      world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom"),_nh("TrajectoryPlannerROS") {

      //initialize the planner
      initialize(name, tf, costmap_ros);
  }

  void TrajectoryPlannerROS::initialize(
      std::string name,
      tf::TransformListener* tf,
      costmap_2d::Costmap2DROS* costmap_ros){
    if (! isInitialized()) {

      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

		  this->turn1_num = 1;
      BC_Status = false;
	    task_type = 1;

      tf_ = tf;
      costmap_ros_ = costmap_ros;
      rot_stopped_velocity_ = 1e-2;
      trans_stopped_velocity_ = 1e-2;
      double sim_time, sim_granularity, angular_sim_granularity;
      int vx_samples, vtheta_samples;
      double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
      bool holonomic_robot, dwa, simple_attractor, heading_scoring;
      double heading_scoring_timestep;
      //double max_vel_x, min_vel_x;
      double backup_vel;
      double stop_time_buffer;
      std::string world_model_type;
      rotating_to_goal_ = false;

      //initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();

      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      private_nh.param("prune_plan", prune_plan_, true);

      private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
      private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      //---------------------------------------
      private_nh.param("Kp_move", Kp_move_, 20.0);
      private_nh.param("Kd_move", Kd_move_, 5.0);
      private_nh.param("Ki_move", Ki_move_, 0.0);
      private_nh.param("Ko_move", Ko_move_, 40.0);
      private_nh.param("Ka_move", Ka_move_, 1.0);
      private_nh.param("Kb_move", Kb_move_, 1.5);
      
      private_nh.param("slowdown_goal_tolerance", slowdown_goal_tolerance_, 2.0);
      private_nh.param("stop_goal_tolerance", stop_goal_tolerance_, 0.2);
      private_nh.param("escape_vel", escape_vel_, -0.1);
      //---------------------------------------



      private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
      private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

      private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

      private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);


      //Since I screwed up nicely in my documentation, I'm going to add errors
      //informing the user if they've set one of the wrong parameters
      if(private_nh.hasParam("acc_limit_x"))
        ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_y"))
        ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      if(private_nh.hasParam("acc_limit_th"))
        ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

      //Assuming this planner is being run within the navigation stack, we can
      //just do an upward search for the frequency at which its being run. This
      //also allows the frequency to be overwritten locally.
      std::string controller_frequency_param_name;
      if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
        sim_period_ = 0.05;
      else
      {
        double controller_frequency = 0;
        private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        if(controller_frequency > 0)
          sim_period_ = 1.0 / controller_frequency;
        else
        {
          ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          sim_period_ = 0.05;
        }
      }

      ROS_INFO("Sim period is set to %.2f", sim_period_);

      private_nh.param("sim_time", sim_time, 1.0);
      private_nh.param("sim_granularity", sim_granularity, 0.025);
      private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
      private_nh.param("vx_samples", vx_samples, 3);
      private_nh.param("vtheta_samples", vtheta_samples, 20);

      private_nh.param("path_distance_bias", pdist_scale, 0.6);
      private_nh.param("goal_distance_bias", gdist_scale, 0.8);
      private_nh.param("occdist_scale", occdist_scale, 0.01);

      bool meter_scoring;
      if ( ! private_nh.hasParam("meter_scoring")) {
        ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settins robust against changes of costmap resolution.");
      } else {
        private_nh.param("meter_scoring", meter_scoring, false);

        if(meter_scoring) {
          //if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          double resolution = costmap_->getResolution();
          gdist_scale *= resolution;
          pdist_scale *= resolution;
          occdist_scale *= resolution;
        } else {
          ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
        }
      }

      private_nh.param("heading_lookahead", heading_lookahead, 0.325);
      private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
      private_nh.param("escape_reset_dist", escape_reset_dist, 0.10);
      private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
      private_nh.param("holonomic_robot", holonomic_robot, true);
      private_nh.param("max_vel_x", max_vel_x, 1.0);
      private_nh.param("min_vel_x", min_vel_x, 0.1);


      double max_rotational_vel;
      private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
      max_vel_th_ = max_rotational_vel;
      min_vel_th_ = -1.0 * max_rotational_vel;

      private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.1);
      //private_nh.param("min_out_place_rotational_vel", min_out_place_vel_th_, 0.1);

      reached_goal_ = false;

      backup_vel = -0.1;

      if(private_nh.getParam("backup_vel", backup_vel))
        ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

      //if both backup_vel and escape_vel are set... we'll use escape_vel
      private_nh.getParam("escape_vel", backup_vel);

      if(backup_vel >= 0.0)
        ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      private_nh.param("world_model", world_model_type, std::string("costmap"));
      private_nh.param("dwa", dwa, true);
      private_nh.param("heading_scoring", heading_scoring, false);
      private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      simple_attractor = false;

      //parameters for using the freespace controller
      double min_pt_separation, max_obstacle_height, grid_resolution;
      private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");

      world_model_ = new CostmapModel(*costmap_);

      std::vector<double> y_vels = loadYVels(private_nh);

      footprint_spec_ = costmap_ros_->getRobotFootprint();

      tc_ = new TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);


      map_viz_.initialize(name, global_frame_, boost::bind(&TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
      initialized_ = true;

      dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&TrajectoryPlannerROS::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);

    } else {
      ROS_WARN("This planner has already been initialized, doing nothing");
    }
    ros::NodeHandle simple_nh("TrajectoryPlannerROS_obstacle");

    //batteryInfo_sub = _nh.subscribe("/BatteryInfo",1,&TrajectoryPlannerROS::batteryInfo_rec, this);
  	feedbackMsg_sub = _nh.subscribe("/FeedbackMsg",1,&TrajectoryPlannerROS::feedbackMsg_recv, this);
  	
    task_current_sub = _nh.subscribe("/task_current", 1,&TrajectoryPlannerROS::task_current_recv, this);  
    
    distance_pid.set_k(Kp_move_/Ko_move_,0.01*Ki_move_/Ko_move_,Kd_move_/Ko_move_,acc_lim_theta_);
    yaw_pid.set_k(Kp_move_/Ko_move_,0.01*Ki_move_/Ko_move_,Kd_move_/Ko_move_,acc_lim_theta_);
  }

  //void TrajectoryPlannerROS::batteryInfo_rec(const carbot_msgs::BatteryInfo& batteryInfo)
  //{
	//  _batteryInfo = batteryInfo;
  //}
void TrajectoryPlannerROS::feedbackMsg_recv(const carbot_msgs::FeedbackMsg& feedbackMsg)
{
	//_batteryInfo = batteryInfo;
	BC_Status = feedbackMsg.BC_Status;
}


  void TrajectoryPlannerROS::task_current_recv(const carbot_msgs::Taskarrived& task_current)
  {
	  task_type = task_current.task_type;
  }

  std::vector<double> TrajectoryPlannerROS::loadYVels(ros::NodeHandle node){
    std::vector<double> y_vels;
    std::string y_vel_list;
    if(node.getParam("y_vels", y_vel_list)){
      typedef boost::tokenizer< boost::char_separator<char> > tokenizer;
      boost::char_separator<char> sep("[], ");
      tokenizer tokens(y_vel_list, sep);
      for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++){
        y_vels.push_back(atof((*i).c_str()));
      }
    }
    else{
      //if no values are passed in, we'll provide defaults
      y_vels.push_back(-0.3);
      y_vels.push_back(-0.1);
      y_vels.push_back(0.1);
      y_vels.push_back(0.3);
    }
    return y_vels;
  }

  TrajectoryPlannerROS::~TrajectoryPlannerROS() {
    //make sure to clean things up
    delete dsrv_;
    if(tc_ != NULL)
      delete tc_;
    if(world_model_ != NULL)
      delete world_model_;
  }

  bool TrajectoryPlannerROS::stopWithAccLimits(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, geometry_msgs::Twist& cmd_vel){
    //slow down with the maximum possible acceleration... we should really use the frequency that we're running at to determine what is feasible
    //but we'll use a tenth of a second to be consistent with the implementation of the local planner.
    double vx = sign(robot_vel.getOrigin().x()) * std::max(0.0, (fabs(robot_vel.getOrigin().x()) - acc_lim_x_ * sim_period_));
    double vy = sign(robot_vel.getOrigin().y()) * std::max(0.0, (fabs(robot_vel.getOrigin().y()) - acc_lim_y_ * sim_period_));

    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    double vth = sign(vel_yaw) * std::max(0.0, (fabs(vel_yaw) - acc_lim_theta_ * sim_period_));

    //we do want to check whether or not the command is valid
    double yaw = tf::getYaw(global_pose.getRotation());
    bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, 
        robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, vx, vy, vth);

    //if we have a valid command, we'll pass it on, otherwise we'll command all zeros
    if(valid_cmd){
      ROS_DEBUG("Slowing down... using vx, vy, vth: %.2f, %.2f, %.2f", vx, vy, vth);
      cmd_vel.linear.x = vx;
      cmd_vel.linear.y = vy;
      cmd_vel.angular.z = vth;
      return true;
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::rotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel){
    double yaw = tf::getYaw(global_pose.getRotation());
    double vel_yaw = tf::getYaw(robot_vel.getRotation());
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

    double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
        std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
        std::min(-1.0 * min_in_place_vel_th_, ang_diff));

    //take the acceleration limits of the robot into account
    double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_ * sim_period_;
    double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_ * sim_period_;

    v_theta_samp = sign(v_theta_samp) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

    //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
    double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

    v_theta_samp = sign(v_theta_samp) * std::min(max_speed_to_stop, fabs(v_theta_samp));

    // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
    v_theta_samp = v_theta_samp > 0.0
      ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
      : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

    //we still want to lay down the footprint of the robot and check if the action is legal
    bool valid_cmd = tc_->checkTrajectory(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw, 
        robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw, 0.0, 0.0, v_theta_samp);

    ROS_DEBUG("Moving to desired goal orientation, th cmd: %.2f, valid_cmd: %d", v_theta_samp, valid_cmd);

    if(valid_cmd)
    {
      cmd_vel.angular.z = v_theta_samp;
      //cmd_vel.angular.z = normalize_angle_me(cmd_vel.angular.z);
      return true;
    }

    cmd_vel.angular.z = 0.0;
    return false;
  }

  bool TrajectoryPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //reset the global plan
    global_plan_.clear();
    global_plan_ = orig_global_plan;
    
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    xy_tolerance_latch_ = false;
    //reset the at goal flag
    reached_goal_ = false;
    return true;
  }

  //计算当前点与路径直线间的距离
  double TrajectoryPlannerROS::cal_distance(geometry_msgs::Point cur_pos, geometry_msgs::Point pre_pose, geometry_msgs::Point next_pose)
  {
	  double a_x = next_pose.x- pre_pose.x;
	  double a_y = next_pose.y- pre_pose.y;
	  double ptl;
	  if(a_x == 0){
		  if(a_y >= 0){
			  ptl = cur_pos.x - next_pose.x;
		  }else{
			  ptl = next_pose.x - cur_pos.x;
		  }
	  }else{
		  double K = (next_pose.y - pre_pose.y)/(next_pose.x- pre_pose.x);
		  double B = (next_pose.x * pre_pose.y - pre_pose.x * next_pose.y)/(next_pose.x - pre_pose.x);
		  if(a_x > 0){
			  if ((-K * cur_pos.x + cur_pos.y - B) > 0)
				 ptl = -fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
			 else
				 ptl = fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
		  }else{
			  if ((-K*cur_pos.x + cur_pos.y - B) > 0)
				  ptl = fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
			  else
				  ptl = -fabs(-K*cur_pos.x + cur_pos.y - B)/sqrt(pow(-K,2) + 1);
		  }
	  }
	  return ptl;
  }

  double TrajectoryPlannerROS::cal_path_rotation(geometry_msgs::Point cur_pos,geometry_msgs::Point next_pose){
	  double path_th;
	  //计算下一条路径直线和全局正方间的夹角 (规定正方向向量（xcur+1,ycur）)
	  double prodis = sqrt(pow(next_pose.x - cur_pos.x,2) + pow(next_pose.y - cur_pos.y,2));
	  double curdis = 1;
	  double dot_product = (next_pose.x - cur_pos.x);
	  double dif_product = (next_pose.y - cur_pos.y);
	  double cosm, sinm;

	  if (prodis==0){
		  cosm = cos(cur_pos.z);
		  sinm = sin(cur_pos.z);
	  }else{
		  cosm = dot_product / (prodis*curdis);
		  sinm = dif_product / (prodis*curdis);
	  }
	  if (cosm >= 0)
		  path_th = asin(sinm);
	  else{
		  if (sinm >=0)
			  path_th = M_PI - asin(sinm);
		  else
			  path_th = -M_PI - asin(sinm);
	  }
	  return path_th;
  }
  /*
  double TrajectoryPlannerROS::normalize_angle_me(double angle){
      double res = angle;
      while (res > M_PI)
          res -= 2.0 * M_PI;
      while (res < -M_PI)
          res += 2.0 * M_PI;
      return res;
  }*/

  void TrajectoryPlannerROS::task_done(geometry_msgs::Twist& cmd_vel, double goal_x, double goal_y, double goal_th)
  {
				cmd_vel.linear.x = 0.0;
				cmd_vel.linear.y = 0.0;
				cmd_vel.angular.z = 0.0;
				this->rotating_to_goal_ = false;
				this->xy_tolerance_latch_ = false;
				this->reached_goal_ = true;

        this->distance_pid.reset_error();
        this->yaw_pid.reset_error();
				this->p_end.x = goal_x;
				this->p_end.y = goal_y;
				this->p_end.z = goal_th;

				this->goal_distance = 0.0;
  }


bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (! isInitialized()) {
	  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
	  return false;
  }

  std::vector<geometry_msgs::PoseStamped> local_plan;
  tf::Stamped<tf::Pose> global_pose;
  if (!costmap_ros_->getRobotPose(global_pose)) {
	  return false;
  }

  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  //get the global plan in our frame
  if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
	  ROS_WARN("Could not transform the global plan to the frame of the controller");
	  return false;
  }
  //now we'll prune the plan based on the position of the robot
  if(prune_plan_)
  prunePlan(global_pose, transformed_plan, global_plan_);

  tf::Stamped<tf::Pose> drive_cmds;
  drive_cmds.frame_id_ = robot_base_frame_;

  tf::Stamped<tf::Pose> robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  //if the global plan passed in is empty... we won't do anything
  if(transformed_plan.empty())
	  return false;

  tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);

  //we assume the global goal is the last point in the global plan
  double goal_x = goal_point.getOrigin().getX();
  double goal_y = goal_point.getOrigin().getY();
  double yaw = tf::getYaw(goal_point.getRotation());
  double goal_th = yaw;

  tf::Stamped<tf::Pose> start_pose;
  tf::poseStampedMsgToTF(transformed_plan.front(), start_pose);
  p_start.x = start_pose.getOrigin().getX();
  p_start.y = start_pose.getOrigin().getY();
  p_start.z = tf::getYaw(start_pose.getRotation());

  goal_distance = getGoalPositionDistance(start_pose, goal_x, goal_y);
  double start_distance = getGoalPositionDistance(global_pose, p_start.x, p_start.y);

  p_end.x = goal_x;
  p_end.y = goal_y;
  p_end.z = goal_th;



  //path_rot = normalize_angle_me(cal_path_rotation(p_start,p_end));
  path_rot = angles::shortest_angular_distance(0.0,cal_path_rotation(p_start,p_end));

  p_cur.x = global_pose.getOrigin().getX();
  p_cur.y = global_pose.getOrigin().getY();
  p_cur.z = tf::getYaw(global_pose.getRotation());


	//compute what trajectory to drive along
	//copy over the odometry information
  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  cmd_vel.linear.x = base_odom.twist.twist.linear.x;
  cmd_vel.angular.z = base_odom.twist.twist.angular.z;


  //ROS_INFO("A valid velocity command of (%.4f, %.4f, %.4f) was found for this cycle.",
	//		cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
	  
	if(task_type == 2) //to charge pile
 	{
    double diff_dis = getGoalPositionDistance(global_pose, goal_x, goal_y); 

    if((!BC_Status) && ( diff_dis > xy_goal_tolerance_) && (start_distance < goal_distance))
	 	{

      //double diff_rot = normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot)-M_PI);  
      
      //double diff_rot = angles::shortest_angular_distance(-M_PI, getGoalOrientationAngleDifference(global_pose, path_rot));
      double diff_rot = getGoalOrientationAngleDifference(global_pose, p_end.z);
      
      if((fabs(diff_rot) > (0.5*yaw_goal_tolerance_)) && (fabs(cmd_vel.linear.x) < acc_lim_x_))
			{
          //ROS_WARN("charge turn %0.4f",diff_rot);

					cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
					cmd_vel.angular.z = sign(diff_rot)*min_in_place_vel_th_*0.2;
					return true;

			}else
      {
        if(diff_dis < 10.0*xy_goal_tolerance_)
        {
          cmd_vel.linear.x = 0.5*escape_vel_;  
        }else
        {
          cmd_vel.linear.x = escape_vel_;
        }
        
        
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;

        return true;

      }

		}else
		{
			this->task_done(cmd_vel, goal_x, goal_y, goal_th);
 			return true;
		}

	}else if(task_type == 3) //charge pile to charge point
	{
		if((getGoalPositionDistance(global_pose, goal_x, goal_y) > xy_goal_tolerance_) && (start_distance < goal_distance))
	 	{
	  	cmd_vel.linear.x = -escape_vel_;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
			return true;

		}else
		{
			this->task_done(cmd_vel, goal_x, goal_y, goal_th);
 			return true;
		}

	}else //other task type
	{

	  //check to see if we've reached the goal position
    double diff_dis = getGoalPositionDistance(global_pose, goal_x, goal_y);

	  if((diff_dis < (xy_goal_tolerance_*this->turn1_num)) || (start_distance >= goal_distance))
    {

		  //double diff_rot = normalize_angle_me(getGoalOrientationAngleDifference(global_pose, goal_th));
		  double diff_rot = getGoalOrientationAngleDifference(global_pose, goal_th);
		  
      //check to see if the goal orientation has been reached
		  if((fabs(diff_rot) > yaw_goal_tolerance_) && (task_type == 1))
		  {

				cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
				cmd_vel.angular.z = sign(diff_rot) * min_in_place_vel_th_;
				this->turn1_num = 2;
				//ROS_WARN("end turn");
				return true;

		  }else
		  {
        this->turn1_num = 1;
        //ROS_WARN("task done");
        this->task_done(cmd_vel, goal_x, goal_y, goal_th);
        return true;
		  }
	  }else //move to
	  {
	  
	  	this->turn1_num = 1;

      
		  // slow down 
      if(diff_dis <= slowdown_goal_tolerance_)  //&& start_distance <= (goal_distance -xy_goal_tolerance_)
		  {

    	  tc_->updatePlan(transformed_plan);


        //double diff_rot = normalize_angle_me(getGoalOrientationAngleDifference(global_pose, path_rot));
        double diff_rot = getGoalOrientationAngleDifference(global_pose, path_rot);

			  
			  if((fabs(diff_rot) > yaw_goal_tolerance_) && (cmd_vel.linear.x < acc_lim_x_))
			  {

					cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
					cmd_vel.angular.z = sign(diff_rot)*min_in_place_vel_th_;
					//ROS_WARN("begin turn slowdown");
					return true;

			  }else
			  {

					if((diff_dis < stop_goal_tolerance_))// && (_task_current.task_type == 1))
					{
            //ROS_WARN("deacc 2");
						cmd_vel.linear.x = std::max(acc_lim_x_, cmd_vel.linear.x - acc_lim_x_);
					}else
					{
            if(cmd_vel.linear.x < min_vel_x)
            {
              cmd_vel.linear.x = std::min(min_vel_x, cmd_vel.linear.x + acc_lim_x_);//acc to acc_lim_x_
            }else
            {
              cmd_vel.linear.x = std::max(min_vel_x, cmd_vel.linear.x - acc_lim_x_*3.0);//slowdown
            }
          }

          geometry_msgs::Point cur_pos;
          cur_pos.x = global_pose.getOrigin().getX();
          cur_pos.y = global_pose.getOrigin().getY();

          double error_th = 0.0;
          double dis_pid_tmp = 0.0;
          double th_pid_tmp = 0.0;


          dis_pid_tmp = Ka_move_*distance_pid.calculate(cal_distance(cur_pos, p_start, p_end));
          th_pid_tmp = Kb_move_*yaw_pid.calculate(diff_rot);
          error_th = dis_pid_tmp + th_pid_tmp;
          //ROS_INFO("dis_pid_tmp = %.3f,th_pid_tmp=%.3f",dis_pid_tmp,dis_pid_tmp2);
          //ROS_INFO("dis_pid_tmp = %.3f,th_pid_tmp=%.3f,error_th=%.3f",dis_pid_tmp,th_pid_tmp,error_th);
          cmd_vel.angular.z = error_th;

          if(cmd_vel.angular.z > max_vel_th_){
            cmd_vel.angular.z = max_vel_th_;
          }
          if(cmd_vel.angular.z < min_vel_th_)
          {
            cmd_vel.angular.z = min_vel_th_;
          }

          //publish an empty plan because we've reached our goal position
          publishPlan(transformed_plan, g_plan_pub_);
          publishPlan(local_plan, l_plan_pub_);



          return true;
		    }
	    }else  //accel 
	    {

        tc_->updatePlan(transformed_plan);

        geometry_msgs::Point cur_pos;
        cur_pos.x = global_pose.getOrigin().getX();
        cur_pos.y = global_pose.getOrigin().getY();


        double diff_rot = getGoalOrientationAngleDifference(global_pose, path_rot);

        if((fabs(diff_rot) > (yaw_goal_tolerance_)) && cmd_vel.linear.x < acc_lim_x_) //begin turn
        {
          cmd_vel.linear.x = 0.0;
          cmd_vel.linear.y = 0.0;
          cmd_vel.angular.z = sign(diff_rot)*min_in_place_vel_th_;
          //ROS_WARN("begin turn accel");
          return true;
        }else
        {
          //ROS_WARN("acc 2");
          cmd_vel.linear.x = std::min(max_vel_x, cmd_vel.linear.x + acc_lim_x_); //acc

          p_end.x = goal_x;
          p_end.y = goal_y;
          p_end.z = goal_th;

          double error_th = 0.0;
          double dis_pid_tmp = 0.0;
          double th_pid_tmp = 0.0;

          dis_pid_tmp = Ka_move_*distance_pid.calculate(cal_distance(cur_pos, p_start, p_end));
          th_pid_tmp = Kb_move_*yaw_pid.calculate(diff_rot);
          error_th = dis_pid_tmp + th_pid_tmp;

          cmd_vel.angular.z = error_th;

          if(cmd_vel.angular.z > max_vel_th_)
          {
            cmd_vel.angular.z = max_vel_th_;
          }
          if(cmd_vel.angular.z < min_vel_th_)
          {
            cmd_vel.angular.z = min_vel_th_;
          }

          //publish information to the visualizer
          publishPlan(transformed_plan, g_plan_pub_);
          publishPlan(local_plan, l_plan_pub_);

          return true;
		    }
	  	}//accel
	  }//move
	} //end other task type
}

  bool TrajectoryPlannerROS::checkTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->checkTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);
    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return false;
  }


  double TrajectoryPlannerROS::scoreTrajectory(double vx_samp, double vy_samp, double vtheta_samp, bool update_map){
    // Copy of checkTrajectory that returns a score instead of True / False
    tf::Stamped<tf::Pose> global_pose;
    if(costmap_ros_->getRobotPose(global_pose)){
      if(update_map){
        //we need to give the planne some sort of global plan, since we're only checking for legality
        //we'll just give the robots current position
        std::vector<geometry_msgs::PoseStamped> plan;
        geometry_msgs::PoseStamped pose_msg;
        tf::poseStampedTFToMsg(global_pose, pose_msg);
        plan.push_back(pose_msg);
        tc_->updatePlan(plan, true);
      }

      //copy over the odometry information
      nav_msgs::Odometry base_odom;
      {
        boost::recursive_mutex::scoped_lock lock(odom_lock_);
        base_odom = base_odom_;
      }

      return tc_->scoreTrajectory(global_pose.getOrigin().x(), global_pose.getOrigin().y(), tf::getYaw(global_pose.getRotation()),
          base_odom.twist.twist.linear.x,
          base_odom.twist.twist.linear.y,
          base_odom.twist.twist.angular.z, vx_samp, vy_samp, vtheta_samp);

    }
    ROS_WARN("Failed to get the pose of the robot. No trajectories will pass as legal in this case.");
    return -1.0;
  }

  bool TrajectoryPlannerROS::isGoalReached() {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //return flag set in controller
    return reached_goal_; 
  }
};
