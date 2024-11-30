///////////////////////////////////////////////////////////////////////////////
//      Title     : ik_solver.hpp
//      Project   : ap_planning
//      Created   : 07/27/2022
//      Author    : Adam Pettinger
//      Edited    : Akita 
//      Copyright : Copyright© The University of Texas at Austin, 2014-2022. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <ros/ros.h>
#include <ap_planning/ik_solver_base.hpp>
#include <ap_planning/state_sampling.hpp>

namespace ap_planning {

// Some defaults if parameters are not found
const double JOINT_TOLERANCE = 0.05;  // radians
const double WAYPOINT_DIST = 0.005;   // meters
const double WAYPOINT_ANG = 0.01;     // radians, ~0.5 degrees
const double CONDITION_NUM_LIMIT = 100;

/**
 * Estimates the state of a task
 */
class IKSolver : public IKSolverBase {
 public:
  IKSolver(){};
  ~IKSolver(){};

  /** Initializes the solver by looking up ROS parameters for:
   *
   * Required: move_group_name
   *
   * Optional: robot_description_name, joint_tolerance, waypoint_dist,
   * waypoint_ang, condition_num_limit
   *
   * @param nh Parameters are considered to be namespaced to this node
   * @return False if the parameters couldn't be found, true otherwise
   */
  bool initialize(const ros::NodeHandle& nh, const std::string& move_group_name,
                  const std::string& robot_description_name) override;

  /** Solves 1 IK request
   *
   * @param jmg Valid JointModelGroup
   * @param target_pose The pose to solve for
   * @param ee_frame The frame to move to target_pose
   * @param robot_state The robot state. It is updated so the positions match
   * the solution
   * @param point The trajectory point to fill out
   * @return True if a solution was found, false otherwise
   */
  bool solveIK(const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
               const geometry_msgs::Pose& target_pose,
               const std::string& ee_frame,
               moveit::core::RobotState& robot_state,
               trajectory_msgs::JointTrajectoryPoint& point) override;

  /** Verifies a single joint state transition
   *
   * Checks each joint doesn't move too much
   *
   * @param point_a The start point (joint state)
   * @param point_b The end point (joint state)
   * @param jmg Joint model group for checking velocity limits
   * @param state_b The end robot state
   * @return The result
   */
  ap_planning::Result verifyTransition(
      const trajectory_msgs::JointTrajectoryPoint& point_a,
      const trajectory_msgs::JointTrajectoryPoint& point_b,
      const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
      const moveit::core::RobotState& state_b) override;

  /** Plans a joint trajectory based on an affordance trajectory
   *
   * @param req The planning request
   * @param res The planning response
   * @return The result
   */
  ap_planning::Result plan(const APPlanningRequest& req,
                           APPlanningResponse& res) override;

 protected:
  /** Plans a joint trajectory based on an affordance trajectory
   *
   *
   * @param affordance_traj The Cartesian trajectory to plan for
   * @param start_state The starting state of the robot
   * @param ee_name The name of the EE link
   * @param res The planning response
   * @return The result
   */
  ap_planning::Result plan(
      const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
      const std::vector<double>& start_state, const std::string& ee_name,
      APPlanningResponse& res) override;

  // This holds the kinematics solver
  kinematics::KinematicsBasePtr ik_solver_;

  planning_scene_monitor::PlanningSceneMonitorPtr psm_;  

  // Planning parameters
  double joint_tolerance_;
  double waypoint_dist_, waypoint_ang_;
  double condition_num_limit_;

  void setUp(APPlanningResponse& res);

  double calculateSegmentSpacing(const ScrewSegment& segment);

  /** General check to make sure 2 joint states are relatively close together
   *
   * @param point_a The start point (joint state)
   * @param point_b The end point (joint state)
   * @return The result
   */
  bool checkPointsAreClose(
      const trajectory_msgs::JointTrajectoryPoint& point_a,
      const trajectory_msgs::JointTrajectoryPoint& point_b);
};

}  // namespace ap_planning
