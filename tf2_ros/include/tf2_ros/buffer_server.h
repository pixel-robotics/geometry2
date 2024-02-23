/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*********************************************************************/

#ifndef TF2_ROS__BUFFER_SERVER_H_
#define TF2_ROS__BUFFER_SERVER_H_

#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>

#include "tf2/time.h"
#include "tf2/buffer_core_interface.h"
#include "tf2_ros/visibility_control.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_msgs/action/lookup_transform.hpp"
#include "tf2_msgs/srv/lookup_transform.hpp"
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>


namespace tf2_ros
{
/** \brief Action server for the action-based implementation of tf2::BufferCoreInterface.
 *
 * Use this class with a tf2_ros::TransformListener in the same process.
 * You can use this class with a tf2_ros::BufferClient in a different process.
 */
class BufferServer : public rclcpp::Node
{
  using LookupTransformAction = tf2_msgs::action::LookupTransform;
  using LookupTransformService = tf2_msgs::srv::LookupTransform;
  using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<LookupTransformAction>>;

public:
  /** \brief Constructor
   * \param buffer The Buffer that this BufferServer will wrap.
   * \param node The node to add the buffer server to.
   * \param ns The namespace in which to look for action clients.
   * \param check_period How often to check for changes to known transforms (via a timer event).
   */
  explicit BufferServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  struct GoalInfo
  {
    GoalHandle handle;
    tf2::TimePoint end_time;
  };
  
  TF2_ROS_PUBLIC
  void serviceCB(const std::shared_ptr<LookupTransformService::Request> request,
          std::shared_ptr<LookupTransformService::Response> response);

  TF2_ROS_PUBLIC
  rclcpp_action::GoalResponse goalCB(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const LookupTransformAction::Goal> goal);

  TF2_ROS_PUBLIC
  void acceptedCB(GoalHandle gh);

  TF2_ROS_PUBLIC
  rclcpp_action::CancelResponse cancelCB(GoalHandle gh);

  TF2_ROS_PUBLIC
  void checkTransforms();

  TF2_ROS_PUBLIC
  bool canTransform(GoalHandle gh);

  TF2_ROS_PUBLIC
  geometry_msgs::msg::TransformStamped lookupTransform(GoalHandle gh);

  TF2_ROS_PUBLIC
  geometry_msgs::msg::TransformStamped lookupTransform(const std::shared_ptr<LookupTransformService::Request> request);

  rclcpp::Logger logger_ {rclcpp::get_logger("BufferServer")};
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp_action::Server<LookupTransformAction>::SharedPtr server_;
  rclcpp::Service<LookupTransformService>::SharedPtr service_server_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::list<GoalInfo> active_goals_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr check_timer_;
};

}  // namespace tf2_ros

#endif  // TF2_ROS__BUFFER_SERVER_H_
