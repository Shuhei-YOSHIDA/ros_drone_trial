/**
 * @file hector_quadrotor_ex_node.cpp
 * @brief After hector_quadrotor_demo/outdoor_flight_gazebo.launch, try this node
 * @details To use Takeoff and Landing server, prepare PoseAction server at the same time.
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_uav_msgs/LandingAction.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<hector_uav_msgs::LandingAction> LandingClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::TakeoffAction> TakeoffClient;
typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> PoseClient;

ros::Publisher velocity_publisher_; //, attitude_publisher_, yawrate_publisher_, thrust_publisher_;
ros::ServiceClient motor_enable_service_;
std::shared_ptr<LandingClient> landing_client_;
std::shared_ptr<TakeoffClient> takeoff_client_;
std::shared_ptr<PoseClient> pose_client_;

std::string base_link_frame_, world_frame_, base_stabilized_frame_;

// position control
geometry_msgs::PoseStamped pose_;

void init()
{
  ros::NodeHandle robot_nh;
  robot_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  robot_nh.param<std::string>("world_frame", world_frame_, "world");
  robot_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

  // control_mode == 'position'
  pose_.pose.position.x = 0;
  pose_.pose.position.y = 0;
  pose_.pose.position.z = 0;
  pose_.pose.orientation.x = 0;
  pose_.pose.orientation.y = 0;;
  pose_.pose.orientation.z = 0;
  pose_.pose.orientation.w = 1;

  motor_enable_service_ = robot_nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
  takeoff_client_ = std::make_shared<TakeoffClient>(robot_nh, "action/takeoff");
  landing_client_ = std::make_shared<LandingClient>(robot_nh, "action/landing");
  pose_client_ = std::make_shared<PoseClient>(robot_nh, "action/pose");
}

// Takeoff Action Serverは起動時のtakeoff_height値で飛び上がり高さが決まる
// Landing Action Serverは指令の方法によって着陸高さが変わる

void orderTakeoff()
{
  // motor起動して，パラメータtakeoff_height(default 0.3m, params.yaml:0.1)で指定した高さに移動
  // world->baselinkが地面z=0にdroneが設置している状態でz=0.18..であるのでパラメータによっては実質動かない場合もある
  takeoff_client_->sendGoalAndWait(hector_uav_msgs::TakeoffGoal(), ros::Duration(10.0), ros::Duration(10.0));
}

void orderMovement()
{
  ROS_INFO("test, wait 3sec");
  ros::Duration(3.0).sleep();

  hector_uav_msgs::PoseGoal p0;
  p0.target_pose.header.frame_id = "world";
  p0.target_pose.header.stamp = ros::Time::now();
  p0.target_pose.pose.orientation.w = 1;
  p0.target_pose.pose.position.z = 10.0;
  ROS_INFO("move 0");
  pose_client_->sendGoalAndWait(p0, ros::Duration(10.0), ros::Duration(10.0));

  hector_uav_msgs::PoseGoal p1 = p0;
  p1.target_pose.pose.orientation.w = 1;
  p1.target_pose.pose.position.x = 10.0;
  p1.target_pose.pose.position.y = 10.0;
  ROS_INFO("move 1");
  pose_client_->sendGoalAndWait(p1, ros::Duration(10.0), ros::Duration(10.0));

  hector_uav_msgs::PoseGoal p2 = p1;
  p2.target_pose.header.stamp = ros::Time::now();
  p2.target_pose.pose.position.x = 0;
  p2.target_pose.pose.position.y = 0;
  ROS_INFO("move 2");
  pose_client_->sendGoalAndWait(p2, ros::Duration(10.0), ros::Duration(10.0));

  hector_uav_msgs::PoseGoal p3 = p2;
  p3.target_pose.header.stamp = ros::Time::now();
  p3.target_pose.pose.position.z = 0.5;
  ROS_INFO("move 3");
  pose_client_->sendGoalAndWait(p3, ros::Duration(10.0), ros::Duration(10.0));

}

void orderLanding()
{
  // frame_id未設定の場合その場着陸(default z=0.3m位置[params.yaml:0.1]に移動，その後motor停止)
  // 指定の位置が地面の位置より低い場合，uavがどうしても目標達成できないのでserverは失敗で終わる
  landing_client_->sendGoalAndWait(hector_uav_msgs::LandingGoal(), ros::Duration(10.0), ros::Duration(10.0));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hector_quadrotor_ex_node");
  ros::NodeHandle nh;

  ROS_INFO("start %s, initialze", "hector_quadrotor_ex_node");
  init();

  // Takeoff
  ROS_INFO("Start takeoff");
  orderTakeoff();

  // Movement
  ROS_INFO("Start movement");
  orderMovement();

  // Landing
  ROS_INFO("Start landing");
  orderLanding();

  ROS_INFO("finished");
  return 0;
}
