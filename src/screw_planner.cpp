#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ap_planning/screw_planner.hpp>

namespace ap_planning {
ScrewPlanner::ScrewPlanner(const std::string& move_group_name,
                           const std::string& robot_description_name) {
  // Load the robot model
  robot_model_loader::RobotModelLoader robot_model_loader(
      robot_description_name);
  kinematic_model_ = robot_model_loader.getModel();

  // Get information about the robot
  joint_model_group_ = std::make_shared<moveit::core::JointModelGroup>(
      *kinematic_model_->getJointModelGroup(move_group_name));

  // Set kinematic model for classes that will need it
  ScrewSampler::kinematic_model = kinematic_model_;
  ScrewValidityChecker::kinematic_model = kinematic_model_;
  ScrewValidSampler::kinematic_model = kinematic_model_;
}

ap_planning::Result ScrewPlanner::plan(const APPlanningRequest& req,
                                       APPlanningResponse& res) {
  // Set response to failing case
  res.joint_trajectory.joint_names.clear();
  res.joint_trajectory.points.clear();
  res.percentage_complete = 0.0;
  res.trajectory_is_valid = false;

  kinematic_state_ =
      std::make_shared<moveit::core::RobotState>(kinematic_model_);
  kinematic_state_->setToDefaultValues();

  // Set up the state space for this plan
  if (!setupStateSpace(req)) {
    return INITIALIZATION_FAIL;
  }

  // Set the parameters for the state space
  if (!setSpaceParameters(req, state_space_)) {
    return INITIALIZATION_FAIL;
  }

  // Set the sampler and lock
  state_space_->setStateSamplerAllocator(ap_planning::allocScrewSampler);
  state_space_->as<ob::CompoundStateSpace>()->lock();

  // Set up... the SimpleSetup
  if (!setSimpleSetup(state_space_)) {
    return INITIALIZATION_FAIL;
  }

  // Create start and goal states
  std::vector<std::vector<double>> start_configs, goal_configs;
  if (passed_start_config_) {
    if (!findGoalStates(req, 10, start_configs, goal_configs)) {
      return NO_IK_SOLUTION;
    }
  } else {
    if (!findStartGoalStates(req, 5, 10, start_configs, goal_configs)) {
      return NO_IK_SOLUTION;
    }
  }

  // Set the start states
  for (const auto& start_state : start_configs) {
    ss_->addStartState(
        vectorToState(state_space_, std::vector<double>(1, 0), start_state));
  }

  // Create and populate the goal object
  auto goal_obj = std::make_shared<ScrewGoal>(ss_->getSpaceInformation());
  for (const auto& goal_state : goal_configs) {
    goal_obj->addState(vectorToState(
        state_space_, std::vector<double>(1, req.theta), goal_state));
  }
  ss_->setGoal(goal_obj);

  // Plan
  ob::PlannerStatus solved = ss_->solve(5.0);
  if (solved) {
    ss_->simplifySolution(1.0);

    populateResponse(ss_->getSolutionPath(), req, res);
    return SUCCESS;
  }

  return PLANNING_FAIL;
}

bool ScrewPlanner::setupStateSpace(const APPlanningRequest& req) {
  state_space_.reset();

  // construct the state space we are planning in
  auto screw_space = std::make_shared<ob::RealVectorStateSpace>();
  auto joint_space = std::make_shared<ob::RealVectorStateSpace>();

  // Add screw dimension
  // TODO: multi-DoF problems?
  // TODO: make sure the theta is positive
  screw_space->addDimension(0, req.theta);

  // Go through joint group and add bounds for each joint
  for (const moveit::core::JointModel* joint :
       joint_model_group_->getActiveJointModels()) {
    const auto jt = joint->getType();

    if (jt == joint->PLANAR) {
      joint_space->addDimension(-1e3, 1e3);
      joint_space->addDimension(-1e3, 1e3);
      joint_space->addDimension(-M_PI, M_PI);
    } else if (jt == joint->REVOLUTE || jt == joint->PRISMATIC) {
      const auto& bounds = joint->getVariableBounds(joint->getName());
      if (bounds.position_bounded_) {
        joint_space->addDimension(bounds.min_position_, bounds.max_position_);
      } else {
        // TODO: Add warning or throw?
        return false;
      }
    } else {
      // TODO: Add warning or throw?
    }
  }

  // combine the state space
  state_space_ = screw_space + joint_space;
  return true;
}

bool ScrewPlanner::setSpaceParameters(const APPlanningRequest& req,
                                      ompl::base::StateSpacePtr& space) {
  // We need to transform the screw to be in the starting frame
  geometry_msgs::TransformStamped tf_msg = getStartTF(req);
  auto transformed_screw =
      affordance_primitives::transformScrew(req.screw_msg, tf_msg);

  // Set the screw axis from transformed screw
  screw_axis_.setScrewAxis(transformed_screw);

  // Calculate the goal pose and set
  const Eigen::Isometry3d planning_to_start = tf2::transformToEigen(tf_msg);
  goal_pose_ = planning_to_start * screw_axis_.getTF(req.theta);

  // Add screw param (from starting pose screw)
  auto screw_param = std::make_shared<ap_planning::ScrewParam>("screw_param");
  screw_param->setValue(
      affordance_primitives::screwMsgToStr(transformed_screw));
  space->params().add(screw_param);

  // Add starting pose
  auto pose_param = std::make_shared<ap_planning::PoseParam>("pose_param");
  auto pose_msg = req.start_pose;
  pose_msg.pose = tf2::toMsg(start_pose_);
  pose_param->setValue(affordance_primitives::poseToStr(pose_msg));
  space->params().add(pose_param);

  // Add EE frame name and move group parameters
  auto ee_name_param =
      std::make_shared<ap_planning::StringParam>("ee_frame_name");
  ee_name_param->setValue(req.ee_frame_name);
  space->params().add(ee_name_param);
  auto move_group_param =
      std::make_shared<ap_planning::StringParam>("move_group");
  move_group_param->setValue(joint_model_group_->getName());
  space->params().add(move_group_param);

  return true;
}

affordance_primitives::TransformStamped ScrewPlanner::getStartTF(
    const APPlanningRequest& req) {
  geometry_msgs::TransformStamped tf_msg;

  // Check if a starting joint configuration was given
  if (req.start_joint_state.size() == joint_model_group_->getVariableCount()) {
    // Extract the start pose
    kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                             req.start_joint_state);
    kinematic_state_->update(true);
    auto pose_eig = kinematic_state_->getFrameTransform(req.ee_frame_name);
    tf_msg = tf2::eigenToTransform(pose_eig);
    passed_start_config_ = true;
  } else {
    // Set start pose directly
    tf_msg.transform.rotation = req.start_pose.pose.orientation;
    tf_msg.transform.translation.x = req.start_pose.pose.position.x;
    tf_msg.transform.translation.y = req.start_pose.pose.position.y;
    tf_msg.transform.translation.z = req.start_pose.pose.position.z;
    passed_start_config_ = false;
  }

  // Set the start pose
  tf_msg.header.frame_id = req.screw_msg.header.frame_id;
  tf_msg.child_frame_id = req.ee_frame_name;
  start_pose_ = tf2::transformToEigen(tf_msg);

  return tf_msg;
}

bool ScrewPlanner::setSimpleSetup(const ompl::base::StateSpacePtr& space) {
  // Create the SimpleSetup class
  ss_ = std::make_shared<og::SimpleSetup>(space);

  // Set state validity checking
  ss_->setStateValidityChecker(
      std::make_shared<ScrewValidityChecker>(ss_->getSpaceInformation()));

  // Set valid state sampler
  ss_->getSpaceInformation()->setValidStateSamplerAllocator(
      ap_planning::allocScrewValidSampler);

  // Set planner
  auto planner = std::make_shared<og::PRM>(ss_->getSpaceInformation());
  ss_->setPlanner(planner);

  return true;
}

bool ScrewPlanner::findStartGoalStates(
    const APPlanningRequest& req, const size_t num_start, const size_t num_goal,
    std::vector<std::vector<double>>& start_configs,
    std::vector<std::vector<double>>& goal_configs) {
  if (num_goal < 1 || num_start < 1) {
    return false;
  }

  start_configs.clear();
  start_configs.reserve(num_start);
  goal_configs.clear();
  goal_configs.reserve(num_goal);

  geometry_msgs::Pose goal_pose_msg = tf2::toMsg(goal_pose_);

  // Go through and make configurations
  size_t i = 0;
  while ((start_configs.size() < num_start || goal_configs.size() < num_goal) &&
         i < 2 * (num_goal + num_start)) {
    // Every time, we set to random states to get variety in solutions
    kinematic_state_->setToRandomPositions();
    i++;

    // Try to add a start configuration
    if (start_configs.size() < num_start) {
      increaseStateList(req.start_pose.pose, start_configs);
    }

    // Try to add a goal configuration
    if (goal_configs.size() < num_goal) {
      increaseStateList(goal_pose_msg, goal_configs);
    }
  }

  return start_configs.size() > 0 && goal_configs.size() > 0;
}

bool ScrewPlanner::findGoalStates(
    const APPlanningRequest& req, const size_t num_goal,
    std::vector<std::vector<double>>& start_configs,
    std::vector<std::vector<double>>& goal_configs) {
  start_configs.clear();
  goal_configs.clear();
  goal_configs.reserve(num_goal);

  if (req.start_joint_state.size() != joint_model_group_->getVariableCount()) {
    return false;
  }
  start_configs.push_back(req.start_joint_state);
  kinematic_state_->setJointGroupPositions(joint_model_group_.get(),
                                           req.start_joint_state);

  geometry_msgs::Pose goal_pose_msg = tf2::toMsg(goal_pose_);

  // Go through and make configurations
  size_t i = 0;
  while (goal_configs.size() < num_goal && i < 2 * num_goal) {
    // Try to add a goal configuration
    increaseStateList(goal_pose_msg, goal_configs);

    // Every time, we set to random states to get variety in solutions
    kinematic_state_->setToRandomPositions();
    i++;
  }

  return goal_configs.size() > 0;
}

void ScrewPlanner::increaseStateList(
    const affordance_primitives::Pose& pose,
    std::vector<std::vector<double>>& state_list) {
  // Try to solve the IK
  if (!kinematic_state_->setFromIK(joint_model_group_.get(), pose)) {
    return;
  }

  // Copy found solution to vector
  std::vector<double> joint_values;
  kinematic_state_->copyJointGroupPositions(joint_model_group_.get(),
                                            joint_values);

  // If the solution is valid, add it to the list
  if (checkDuplicateState(state_list, joint_values)) {
    state_list.push_back(joint_values);
  }
}

void ScrewPlanner::populateResponse(ompl::geometric::PathGeometric& solution,
                                    const APPlanningRequest& req,
                                    APPlanningResponse& res) {
  // We can stop if the trajectory doesn't have points
  if (solution.getStateCount() < 2) {
    return;
  }

  // Interpolate the solution path
  solution.interpolate();

  // We will populate the trajectory
  res.joint_trajectory.joint_names = joint_model_group_->getVariableNames();
  const size_t num_joints = res.joint_trajectory.joint_names.size();
  res.joint_trajectory.points.reserve(solution.getStateCount());

  // Go through each point and check it for validity
  for (const auto& state : solution.getStates()) {
    // Extract the state info
    const ob::CompoundStateSpace::StateType& compound_state =
        *state->as<ob::CompoundStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType& screw_state =
        *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
    const ob::RealVectorStateSpace::StateType& robot_state =
        *compound_state[1]->as<ob::RealVectorStateSpace::StateType>();

    // First check for validity
    if (!ss_->getSpaceInformation()->isValid(state)) {
      // If a state is invalid, we don't want to continue the trajectory
      res.trajectory_is_valid = false;

      // Calculate the percent through the trajectory we made it
      res.percentage_complete = screw_state[0] / req.theta;
      return;
    }

    // Now add this state to the trajectory
    trajectory_msgs::JointTrajectoryPoint output;
    output.positions.reserve(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
      output.positions.push_back(robot_state[i]);
    }

    // TODO: figure out what to do about vel / time
    res.joint_trajectory.points.push_back(output);
  }

  // Finally, we check the last point to make sure it is at the goal
  const ob::CompoundStateSpace::StateType& compound_state =
      *solution.getStates().back()->as<ob::CompoundStateSpace::StateType>();
  const ob::RealVectorStateSpace::StateType& screw_state =
      *compound_state[0]->as<ob::RealVectorStateSpace::StateType>();
  const double err = fabs(req.theta - screw_state[0]);
  if (err > 0.01) {
    res.trajectory_is_valid = false;
  } else {
    res.trajectory_is_valid = true;
  }
  res.percentage_complete = screw_state[0] / req.theta;
  res.path_length = solution.length();
}
}  // namespace ap_planning
