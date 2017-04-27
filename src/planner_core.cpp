/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *         David V. Lu!!
 *********************************************************************/
#include <global_replanner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <global_replanner/dijkstra.h>
#include <global_replanner/astar.h>
#include <global_replanner/grid_path.h>
#include <global_replanner/gradient_path.h>
#include <global_replanner/quadratic_calculator.h>
#include <global_replanner/potential_calculator.h>
#include <navfn/navfn_ros.h>
#include <carrot_planner/carrot_planner.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace navfn;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_replanner::GlobalReplanner, nav_core::BaseGlobalPlanner)

namespace global_replanner {

void GlobalReplanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

GlobalReplanner::GlobalReplanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

  GlobalReplanner::GlobalReplanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id, costmap_2d::Costmap2DROS* costmap_ros) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id, costmap_ros);
}

GlobalReplanner::~GlobalReplanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void GlobalReplanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID(), costmap_ros);
}

  void GlobalReplanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap; // This is costmap_ros_->getCostmap();
	costmap_ros_ = costmap_ros;
        frame_id_ = frame_id;
	count_ = 0;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
	
	replanner_ = boost::shared_ptr<NavFn>(new NavFn(cx,cy));
	// replan_->initialize(frame_id_,costmap_ros_);
	// navfn::NavfnROS::initialize(frame_id_,costmap_ros_);

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);
            
        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 5.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        double costmap_pub_freq;
        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalReplanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_replanner::GlobalReplannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_replanner::GlobalReplannerConfig>::CallbackType cb = boost::bind(
                &GlobalReplanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");

}

void GlobalReplanner::reconfigureCB(global_replanner::GlobalReplannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
}

void GlobalReplanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool GlobalReplanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void GlobalReplanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool GlobalReplanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool GlobalReplanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool GlobalReplanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    //////////////////// Changes here /////////////////////////////
    double sx = wx;
    double sy = wy;
    ////////////////////// To here ////////////////////////////////

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    // if(!old_navfn_behavior_)
    //     planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    // if(publish_potential_)
    //     publishPotential(potential_array_);


    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    /////////////////////// STARTING REPLANNING ALGORITHMS ////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////
    //             FASTER REPLANNING BASED ON GET POINT POTENTIAL            //
    ///////////////////////////////////////////////////////////////////////////

    // Start time for loop time
    double start_time = ros::Time::now().toSec();

    // Need functions from navfn planner for this
    replanner_->setNavArr(nx,ny);
    replanner_->setCostmap(costmap_->getCharMap(),true,allow_unknown_);

    // Start loop for if goal is not the same as previous one
    if((prev_goal_.x != goal.pose.position.x || prev_goal_.y != goal.pose.position.y)){
      // Set map start and goal in new planner
      int map_start[2];
      map_start[0] = start_x;
      map_start[1] = start_y;
      int map_goal[2];
      map_goal[0] = goal_x;
      map_goal[1] = goal_y;
      replanner_->setStart(map_goal);
      replanner_->setGoal(map_start);
      
      // Calculate potentials along path using Astar
      // replanner_->calcNavFnAstar();
      replanner_->setupNavFn(true);

      int astarint1 = replanner_->nx*replanner_->ny/1000;
      int astarint2 = replanner_->nx+replanner_->ny;
      // replanner_->propNavFnAstar(std::max(astarint1,astarint2));
      replanner_->propNavFnAstar(astarint2/2);

      int len = replanner_->calcPath(replanner_->nx*4);

      if (len > 0){
	  ROS_DEBUG("path found");
	}
      else{
	ROS_DEBUG("no path found");
      }
      

      // Get resolution, tolerance (from parameters) and initialize some vars
      double resolution = costmap_->getResolution();
      double tol = tolerance;
      geometry_msgs::PoseStamped p,check_best_change;
      p = goal;
      check_best_change = best_pose;
      double best_sdist = DBL_MAX;

      // This will make the algorithm restart as long as it hasn't found a goal by increasing the tolerance
      bool tolerance_check = false;

      // Potential searching algorithm -- finds lowest potential closest to goal by cycling through a tolerance x tolerance sized square around the goal (if it fails, it increases the tolerance and goes through the loop again)
      while (!tolerance_check){
	p.pose.position.y = goal.pose.position.y - tol;
	while (p.pose.position.y <= goal.pose.position.y + tol){
	  p.pose.position.x = goal.pose.position.x - tol;
	  while (p.pose.position.x <= goal.pose.position.x + tol){
	    unsigned int mx,my;
	    double potential;
	    if(!costmap_->worldToMap(p.pose.position.x,p.pose.position.y,mx,my)){
	      potential = DBL_MAX;
	    }
	    else {
	      unsigned int index = my * nx + mx;
	      potential = replanner_->potarr[index];
	    }
	    double sdist = sq_distance(p,goal);
	    if (potential < POT_HIGH && sdist < best_sdist){
	      best_sdist = sdist;
	      best_pose = p;
	    }
	    p.pose.position.x += resolution;
	  }
	  p.pose.position.y += resolution;
	}
	// Check to see if best_pose changed, if it didn't, set best pose as current position
	if(check_best_change.pose.position.x == best_pose.pose.position.x && check_best_change.pose.position.y == best_pose.pose.position.y){
	  tol += 1.0;
	  ROS_WARN("Goal still in obstacle, increasing tolerance to: %.2f",tol);
	}
	else{
	  tolerance_check = true;
	}
      }
      // Push goal further away from obstacle by a factor of scale * distance between goal and best pose -- This is just because I felt it was getting too close to obstacles
      double diff_x,diff_y,scale;
      scale = 0.3;
      diff_x = best_pose.pose.position.x - goal.pose.position.x;
      diff_y = best_pose.pose.position.y - goal.pose.position.y;
      best_pose.pose.position.x += diff_x * scale;
      best_pose.pose.position.y += diff_y * scale;

      // If gone through this loop, that means you had a new goal
      prev_goal_.x = goal.pose.position.x;
      prev_goal_.y = goal.pose.position.y;
    }

    // Convert to cell coordinates
    double gx,gy;
    worldToMap(best_pose.pose.position.x,best_pose.pose.position.y,gx,gy);
    goal_x = gx;
    goal_y = gy;

    // Clear the plan, just in case
    plan.clear();

    // Set new goal
    geometry_msgs::PoseStamped new_goal = goal;
    new_goal.pose.position.x = best_pose.pose.position.x;
    new_goal.pose.position.y = best_pose.pose.position.y;

    // Check if calculated potentials are legal and make plan if they are
    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, nx * ny * 2, potential_array_);
    if (found_legal){
      if (getPlanFromPotential(start_x,start_y, goal_x, goal_y,new_goal,plan)){
	// geometry_msgs::PoseStamped goal_copy = goal;
	// goal_copy.header.stamp = ros::Time::now();
	// plan.push_back(goal_copy);
      }
      else{
	ROS_ERROR("Plan potential not legal. Will not plan path.");
      }
    }

    // Inform of total time loop took
    double end_time = ros::Time::now().toSec();
    double total_time = end_time - start_time;
    if (total_time > 1.0){
      ROS_WARN("Total replan-loop time exceeded 1.0 s. Time was: %.2f",total_time);
    }

    // // If you want to publish the new pose uncomment this -- Note, causes problems, so use only for debugging really
    // ros::NodeHandle nre;
    // ros::Publisher nrepub = nre.advertise<geometry_msgs::PoseStamped>("goalswitching",1000);
    // double pub_time = ros::Time::now().toSec() + 1.0;
    // while(ros::Time::now().toSec() < pub_time){
    //   nrepub.publish(best_pose);
    // }
    
    
    ///////////////////////////////////////////////////////////////////////////
    //               SLOW REPLANNING BASED ON COMPUTE POTENTIAL              //
    ///////////////////////////////////////////////////////////////////////////
    // double start_time = ros::Time::now().toSec();
    // geometry_msgs::Point wp;
    // geometry_msgs::Point sp;
    // wp.x = wx;
    // wp.y = wy;
    // sp.x = sx;
    // sp.y = sy;

    // bool a = computePotentialtoo(wp,sp);
    // computePotential(wp);
    // double diffx = goal_x - start_x;
    // double diffy = goal_y - start_y;
    // double scale = 1.0;
    // double dScale = 0.2;
    // geometry_msgs::PoseStamped p;
    // ros::NodeHandle nre;
    // ros::Publisher pubby = nre.advertise<geometry_msgs::PoseStamped>("goalswitching",1000);

    // while (!a){
    //   ROS_WARN("Potential is bad");  
    //   goal_x = start_x + scale * diffx;
    //   goal_y = start_y + scale * diffy;
    //   mapToWorld(goal_x,goal_y,wp.x,wp.y);
    //   scale -= dScale;
      
    //   p.pose.position.x = wp.x;
    //   p.pose.position.y = wp.y;
    //   p.pose.orientation.w = 1.0;
    //   p.header.frame_id = "map";
    //   p.header.stamp = ros::Time::now();
    //   pubby.publish(p);

    //   if (scale < 0){
    // 	ROS_ERROR("Cannot find Path");
    // 	break;
    //   }
    //   a = computePotentialtoo(wp,sp);
    // }
    // // ROS_WARN("Potential is A Ok ================");

    // plan.clear();
    // geometry_msgs::PoseStamped new_goal;
    // new_goal.header.frame_id = "map";
    // new_goal.header.stamp = ros::Time::now();
    // new_goal.pose.position.x = wp.x;
    // new_goal.pose.position.y = wp.y;
    // new_goal.pose.orientation.w = 1.0;
    // publishPotential(potential_array_);
    // // double start_time_first = ros::Time::now().toSec();
    // bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y, nx * ny * 2, potential_array_);
    // // double end_time_first = ros::Time::now().toSec();
    // // double total_time_first = end_time_first - start_time_first;
    // // ROS_WARN("Total calculatePotentials function time %.2f",total_time_first);

    // if (getPlanFromPotential(start_x,start_y,goal_x,goal_y,new_goal,plan)){
    //   // geometry_msgs::PoseStamped goal_copy = goal;
    //   // goal_copy.header.stamp = ros::Time::now();
    //   // plan.push_back(goal_copy);
    // }
    // else{
    //   ROS_ERROR("Something went wrong");
    // }
    // double end_time = ros::Time::now().toSec();
    // double total_time = end_time - start_time;
    // ROS_INFO("Total time for loop %.2f",total_time);
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////// ENDING REPLANNING ALGORITHMS ////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////


    // ORIGINAL PLANNING ALGORITHM
/*    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
	  //make sure the goal we push on has the same timestamp as the rest of the plan
	  geometry_msgs::PoseStamped goal_copy = goal;
	  goal_copy.header.stamp = ros::Time::now();
	  plan.push_back(goal_copy);
        } else {
	  ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
      ROS_ERROR("Failed to get a plan.");
    }
*/
    // add orientations if needed
    orientation_filter_->processPath(start, plan);
    
    //publish the plan for visualization purposes
    publishPlan(plan);
    delete potential_array_;
    return !plan.empty();
}

void GlobalReplanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()) {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool GlobalReplanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void GlobalReplanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

bool GlobalReplanner::computePotential(const geometry_msgs::Point& world_point){
  if(!initialized_){
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  //make sure to resize the underlying array that Navfn uses
  
  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  planner_->setSize(nx,ny);

  unsigned int mx, my;
  if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my)){
    ROS_ERROR("This shouldn't stop here");
    return false;
  }

  int map_start[2];
  map_start[0] = 0;
  map_start[1] = 0;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_computer_ = boost::shared_ptr<navfn::NavFn>(new navfn::NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
  planner_computer_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
  planner_computer_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  planner_computer_->setStart(map_start);
  planner_computer_->setGoal(map_goal);

  return planner_computer_->calcNavFnAstar();
  
  }
  bool GlobalReplanner::computePotentialtoo(const geometry_msgs::Point& goal_point,const geometry_msgs::Point& start_point){

  if(!initialized_){
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  //make sure to resize the underlying array that Navfn uses
  
  int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
  planner_->setSize(nx,ny);

  unsigned int mx, my,sx,sy;
  if(!costmap_->worldToMap(goal_point.x, goal_point.y, mx, my)){
    ROS_ERROR("This shouldn't stop here");
    return false;
  }

  int map_start[2];
  if(!costmap_->worldToMap(start_point.x,start_point.y,sx,sy)){
    ROS_ERROR("This shouldn't stop here");
    return false;
  }
  map_start[0] = sx;
  map_start[1] = sy;

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_computer_ = boost::shared_ptr<navfn::NavFn>(new navfn::NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
  planner_computer_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
  planner_computer_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

  planner_computer_->setStart(map_start);
  planner_computer_->setGoal(map_goal);

  return planner_computer_->calcNavFnAstar();


  }
} //end namespace global_planner

