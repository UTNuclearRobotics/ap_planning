#include <affordance_primitives/screw_model/screw_execution.hpp>
#include <ap_planning/ik_solver.hpp>

#include <pluginlib/class_list_macros.h>

namespace ap_planning {
bool IKSolver::initialize(const ros::NodeHandle& nh,
                          const std::string& move_group_name,
                          const std::string& robot_description_name) {
  nh_ = nh;
  const std::string n_name = ros::this_node::getName();

  // Get planning parameters
  nh_.param<double>(n_name + "/waypoint_dist", waypoint_dist_, WAYPOINT_DIST);
  nh_.param<double>(n_name + "/waypoint_ang", waypoint_ang_, WAYPOINT_ANG);
  nh_.param<double>(n_name + "/joint_tolerance", joint_tolerance_,
                    JOINT_TOLERANCE);
  nh_.param<double>(n_name + "/condition_num_limit", condition_num_limit_,
                    CONDITION_NUM_LIMIT);

  robot_model_loader::RobotModelLoader robot_model_loader(
      robot_description_name);
  kinematic_model_ = robot_model_loader.getModel();
  if (!kinematic_model_) {
    ROS_ERROR("Could not load RobotModel");
    return false;
  }
  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup(move_group_name));
  if (!joint_model_group_) {
    ROS_ERROR_STREAM("Could not find joint model group: " << move_group_name);
    return false;
  }

  ik_solver_ = joint_model_group_->getSolverInstance();
  if (!ik_solver_) {
    ROS_ERROR("Could not get IK Solver");
    return false;
  } else if (!ik_solver_->supportsGroup(joint_model_group_.get())) {
    ROS_ERROR_STREAM(
        "IK Solver doesn't support group: " << joint_model_group_->getName());
    return false;
  }

  // Set up planning scene monitor
  psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      robot_description_name);
  psm_->startSceneMonitor();
  psm_->startStateMonitor();

  return true;
}

void IKSolver::setUp(APPlanningResponse& res) {
  // Set response to failing case
  res.joint_trajectory.joint_names.clear();
  res.joint_trajectory.points.clear();
  res.percentage_complete = 0.0;
  res.trajectory_is_valid = false;
  res.path_length = -1;
}

bool IKSolver::checkPointsAreClose(
    const trajectory_msgs::JointTrajectoryPoint& point_a,
    const trajectory_msgs::JointTrajectoryPoint& point_b) {
  if (point_a.positions.size() != point_b.positions.size()) {
    return ap_planning::INVALID_TRANSITION;
  }

  // Check joint diffs for large deltas (joint reconfigurations)
  for (size_t i = 0; i < point_a.positions.size(); i++) {
    if (fabs(point_a.positions.at(i) - point_b.positions.at(i)) >
        joint_tolerance_) {
      return false;
    }
  }
  return true;
}

ap_planning::Result IKSolver::verifyTransition(
    const trajectory_msgs::JointTrajectoryPoint& point_a,
    const trajectory_msgs::JointTrajectoryPoint& point_b,
    const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
    const moveit::core::RobotState& state_b) {
  // Do broad face check to avoid joint reconfigurations
  if (!checkPointsAreClose(point_a, point_b)) {
    return ap_planning::INVALID_TRANSITION;
  }

  // Check joint velocities
  const double wp_duration =
      (point_b.time_from_start - point_a.time_from_start).toSec();
  if (wp_duration <= 0) {
    ROS_WARN_STREAM_THROTTLE(5, "Time between waypoints must be positive");
    return ap_planning::INVALID_TRANSITION;
  }

  // Go through each joint
  size_t i = 0;
  for (const moveit::core::JointModel* joint : jmg->getActiveJointModels()) {
    const auto& bounds = joint->getVariableBounds(joint->getName());
    if (bounds.velocity_bounded_) {
      const double vel =
          (point_b.positions.at(i) - point_a.positions.at(i)) / wp_duration;
      if (vel > bounds.max_velocity_ || vel < bounds.min_velocity_) {
        ROS_WARN_STREAM_THROTTLE(5, "Velocity limit exceeded");
        return ap_planning::INVALID_TRANSITION;
      }
    }
    ++i;
  }

  // Check for a singular position
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(state_b.getJacobian(jmg.get()));
  const double cond_number =
      svd.singularValues()[0] /
      svd.singularValues()[svd.singularValues().size() - 1];
  if (cond_number > condition_num_limit_) {
    ROS_WARN_STREAM_THROTTLE(5, "Singularity: " << cond_number);
    return ap_planning::INVALID_TRANSITION;
  }

  return ap_planning::SUCCESS;
}

bool IKSolver::solveIK(
    const std::shared_ptr<moveit::core::JointModelGroup>& jmg,
    const geometry_msgs::Pose& target_pose, const std::string& ee_frame,
    moveit::core::RobotState& robot_state,
    trajectory_msgs::JointTrajectoryPoint& point) {
  auto planning_scene = planning_scene_monitor::LockedPlanningSceneRW(psm_);
  // Set up the validation callback to make sure we don't collide with the
  // environment
  kinematics::KinematicsBase::IKCallbackFn ik_callback_fn =
      [this, &jmg, &robot_state, &planning_scene](
          const geometry_msgs::Pose& pose, const std::vector<double>& joints,
          moveit_msgs::MoveItErrorCodes& error_code) {
        robot_state.setJointGroupPositions(jmg.get(), joints);
        robot_state.update(true);
        collision_detection::CollisionResult::ContactMap contacts;
        planning_scene->getCollidingPairs(contacts, robot_state);

        if (contacts.size() == 0) {
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        } else {
          error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
        }
      };

  // Solve the IK
  std::vector<double> ik_solution, seed_state;
  robot_state.copyJointGroupPositions(jmg.get(), seed_state);
  moveit_msgs::MoveItErrorCodes err;
  if (!ik_solver_->searchPositionIK(target_pose, seed_state, 0.05, ik_solution,
                                    ik_callback_fn, err)) {
    ROS_WARN_STREAM_THROTTLE(5, "Could not solve IK");
    return false;
  }

  // Update the robot state
  robot_state.setJointGroupPositions(jmg.get(), ik_solution);
  robot_state.update();

  // Copy to the point
  robot_state.copyJointGroupPositions(jmg.get(), point.positions);

  return true;
}

ap_planning::Result IKSolver::plan(
    const affordance_primitive_msgs::AffordanceTrajectory& affordance_traj,
    const std::vector<double>& start_state, const std::string& ee_name,
    APPlanningResponse& res) {
  // Only supported input is a starting joint state
  if (start_state.size() != joint_model_group_->getVariableCount()) {
    ROS_WARN_STREAM("Starting joint state was size: "
                    << start_state.size() << ", expected size: "
                    << joint_model_group_->getVariableCount());
    return ap_planning::INVALID_GOAL;
  }

  setUp(res);

  // Make a new robot state and copy the starting state
  moveit::core::RobotState current_state =
      *(psm_->getStateMonitor()->getCurrentState());
  current_state.setJointGroupPositions(joint_model_group_.get(), start_state);
  current_state.update(true);

  // We will check the first IK solution is close to the starting state
  trajectory_msgs::JointTrajectoryPoint starting_point;
  current_state.copyJointGroupPositions(joint_model_group_.get(),
                                        starting_point.positions);

  // Figure out how many waypoints there are
  const size_t num_waypoints = affordance_traj.trajectory.size();
  const double wp_percent = 1 / double(num_waypoints);

  // Rip through trajectory and plan
  // Note: these waypoints are defined in the screw's (PLANNING) frame
  for (auto& wp : affordance_traj.trajectory) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = wp.time_from_start;
    if (!solveIK(joint_model_group_, wp.pose, ee_name, current_state, point)) {
      return ap_planning::NO_IK_SOLUTION;
    }
    if (res.joint_trajectory.points.size() < 1) {
      if (!checkPointsAreClose(starting_point, point)) {
        ROS_ERROR_STREAM("Points are not close!\n"
                         << starting_point << "\n\n"
                         << point);
        return ap_planning::INVALID_TRANSITION;
      }
    } else {
      auto transition_result =
          verifyTransition(res.joint_trajectory.points.back(), point,
                           joint_model_group_, current_state);
      if (transition_result != ap_planning::SUCCESS) {
        return transition_result;
      }
    }
    res.percentage_complete += wp_percent;
    res.joint_trajectory.points.push_back(point);
  }

  // Set up the output
  res.joint_trajectory.header.frame_id = kinematic_model_->getModelFrame();
  res.joint_trajectory.joint_names = joint_model_group_->getVariableNames();
  res.trajectory_is_valid = true;
  res.path_length = -1;  // Not implemented

  return ap_planning::SUCCESS;
}

ap_planning::Result IKSolver::plan(const APPlanningRequest& req,
                                   APPlanningResponse& res) {
  if (req.screw_path.size() < 1) {
    ROS_WARN_STREAM("Screw path is empty");
    return INVALID_GOAL;
  }

  psm_->requestPlanningSceneState();

  setUp(res);

  // Set up a constraints class
  auto constraints_base = req.toConstraint();

  // TODO make general for non-chained case
  affordance_primitives::ChainedScrews& constraints =
      dynamic_cast<affordance_primitives::ChainedScrews&>(*constraints_base);

  // Make a new robot state
  moveit::core::RobotStatePtr current_state =
      std::make_shared<moveit::core::RobotState>(
          *(psm_->getStateMonitor()->getCurrentState()));
  std::vector<double> starting_joint_config = req.start_joint_state;
  std::vector<std::vector<double>> starts;

  geometry_msgs::Pose first_pose;
  const bool passed_start_joint_state =
      req.start_joint_state.size() == joint_model_group_->getVariableCount();
  if (passed_start_joint_state) {
    // Copy the starting position
    current_state->setJointGroupPositions(joint_model_group_.get(),
                                          req.start_joint_state);
    if (!current_state->knowsFrameTransform(req.ee_frame_name)) {
      ROS_WARN_STREAM("Unknown EE name");
      return INVALID_GOAL;
    }
    constraints.setReferenceFrame(
        current_state->getFrameTransform(req.ee_frame_name));
    first_pose = tf2::toMsg(constraints.referenceFrame());
  } else {
    // Calculate the starting pose
    const auto tf_start_pose = constraints.getPose(constraints.startPhi());
    first_pose = tf2::toMsg(tf_start_pose);

    // Calculate a bunch of starting joint configs
    size_t i = 0;
    constexpr size_t num_starts = 20;
    auto planning_scene = planning_scene_monitor::LockedPlanningSceneRW(psm_);
    while (starts.size() < num_starts && i < 2 * num_starts && ros::ok()) {
      // Every time, we set to random states to get variety in solutions
      current_state->setToRandomPositions(joint_model_group_.get());
      current_state->update(true);

      // Try to add a goal configuration
      increaseStateList(joint_model_group_, current_state, planning_scene,
                        ik_solver_, first_pose, starts);

      i++;
    }
    if (starts.size() == 0) {
      ROS_WARN_STREAM("No initial IK solution found");
      return ap_planning::NO_IK_SOLUTION;
    }
  }

  // Create the trajectory
  affordance_primitive_msgs::AffordanceTrajectory affordance_traj;
  affordance_traj.header.frame_id =
      req.screw_path.front().screw_msg.header.frame_id;
  double time_now = 0;

  affordance_primitive_msgs::AffordanceWaypoint first_wp;
  first_wp.time_from_start = ros::Duration().fromSec(time_now);
  first_wp.pose = first_pose;
  affordance_traj.trajectory.push_back(first_wp);

  // Go through the screw segments
  const double theta_dot = 0.1;  // TODO do this better
  double lambda = 0;
  auto phi = constraints.lowerBounds();
  for (size_t i = 0; i < constraints.size(); ++i) {
    // Figure out lambda spacing on this axis
    const double lambda_step = calculateSegmentSpacing(req.screw_path.at(i));
    lambda += lambda_step;
    time_now += lambda_step / theta_dot;

    // Calculate when we should stop for this segment
    phi.at(i) = constraints.upperBounds().at(i);
    const double lambda_max = constraints.getLambda(phi);

    // Go through and calculate all the poses for this point
    while (lambda <= lambda_max && ros::ok()) {
      const auto waypoint_now = constraints.getPose(lambda);
      lambda += lambda_step;
      time_now += lambda_step / theta_dot;

      // Stuff into ROS type
      affordance_primitive_msgs::AffordanceWaypoint this_wp;
      this_wp.time_from_start = ros::Duration().fromSec(time_now);
      this_wp.pose = tf2::toMsg(waypoint_now);
      affordance_traj.trajectory.push_back(this_wp);
    }
  }

  // If pass a joint state, just plan with that one
  if (passed_start_joint_state) {
    return plan(affordance_traj, starting_joint_config, req.ee_frame_name, res);
  }

  // Otherwise, plan from multiple
  ap_planning::APPlanningResponse this_response;
  while (ros::ok() && !starts.empty()) {
    auto this_start = starts.back();
    starts.pop_back();

    // Do the plan
    auto result =
        plan(affordance_traj, this_start, req.ee_frame_name, this_response);

    // If success, just get out ASAP
    if (result == ap_planning::SUCCESS) {
      res = this_response;
      return result;
    }

    // If better than previous, update it
    if (this_response.percentage_complete > res.percentage_complete) {
      res = this_response;
    }
  }

  // If we are here, we did not find a valid plan, but res is the best found
  return ap_planning::PLANNING_FAIL;
}

double IKSolver::calculateSegmentSpacing(const ScrewSegment& segment) {
  const double theta = segment.end_theta - segment.start_theta;

  // Pure translation case
  if (segment.screw_msg.is_pure_translation) {
    const double num_waypoints = ceil(theta / waypoint_dist_);
    return theta / num_waypoints;
  }

  Eigen::Vector3d screw_dist, axis;
  tf2::fromMsg(segment.screw_msg.origin, screw_dist);
  tf2::fromMsg(segment.screw_msg.axis, axis);

  // Project origin distance onto axis plane, to get distance to axis
  screw_dist -= (screw_dist.dot(axis)) / (axis.squaredNorm()) * axis;

  // Calculate the number of waypoints we need for angular and linear
  const double wps_ang = ceil(theta / waypoint_ang_);
  const double wps_lin = ceil(theta * screw_dist.norm() / waypoint_ang_);

  // Use whichever needed more
  const double num_waypoints = std::max(wps_ang, wps_lin);
  return theta / num_waypoints;
}
}  // namespace ap_planning

PLUGINLIB_EXPORT_CLASS(ap_planning::IKSolver, ap_planning::IKSolverBase);
