///////////////////////////////////////////////////////////////////////////////
//      Title     : ap_planning_common.hpp
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

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <affordance_primitives/screw_planning/chained_screws.hpp>
#include <affordance_primitives/screw_planning/unchained_screws.hpp>

#include <optional>
#include <string>
#include <vector>

namespace ap_planning {
enum Result {
  SUCCESS,
  INITIALIZATION_FAIL,
  INVALID_GOAL,
  NO_IK_SOLUTION,
  INVALID_TRANSITION,
  PLANNING_FAIL,
  PARTIAL_SUCCESS
};

enum ScrewPathType { CHAINED, UNCHAINED };

inline std::string toStr(const Result result) {
  switch (result) {
    case SUCCESS:
      return "Success";
    case INITIALIZATION_FAIL:
      return "Failed initialization";
    case INVALID_GOAL:
      return "Recieved invalid goal";
    case NO_IK_SOLUTION:
      return "Could not solve IK";
    case INVALID_TRANSITION:
      return "Invalid transition";
    case PLANNING_FAIL:
      return "Failed planning";
  }
  return "Unknown code";
}

// Enumeration for the underlying planner
enum PlannerType { PRM, PRMstar, RRT, RRTconnect };

/**
 * A struct for describing a single segment of a screw path
 */
struct ScrewSegment {
  affordance_primitive_msgs::ScrewStamped screw_msg;

  // User CAN set bounds. If they do not, we start/end
  // TODO: fancy work to allow relaxing start/end
  double start_theta;
  double end_theta;
  std::optional<double> lower_bound{std::nullopt};
  std::optional<double> upper_bound{std::nullopt};
};

/**
 * A struct for holding all the information needed to make an AP planning
 * request
 *
 * Note: the screw axes (in segments) and starting pose (start_pose) should be
 * given with respect to the planning frame
 */
struct APPlanningRequest {
  ScrewPathType screw_path_type{CHAINED};
  std::vector<ScrewSegment> screw_path;
  std::string ee_frame_name;

  PlannerType planner{PRM};
  double planning_time;

  // Only set one of these
  std::vector<double> start_joint_state;
  geometry_msgs::PoseStamped start_pose;

  /** Converts the request to a Constraint object
   */
  std::shared_ptr<affordance_primitives::ScrewConstraint> toConstraint() const;
};

/**
 * A struct for holding all the information returned from planning
 */
struct APPlanningResponse {
  trajectory_msgs::JointTrajectory joint_trajectory;
  double percentage_complete;
  bool trajectory_is_valid;
  double path_length;
};
}  // namespace ap_planning
