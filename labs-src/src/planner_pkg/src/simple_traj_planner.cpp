#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <iostream>
#include <sstream>

#include <math.h>

ros::Subscriber desire_traj_vertices_sub;
ros::Publisher desired_state_pub;
tf::TransformBroadcaster* br;

/**
 * Callback function for listening to the desired vertices msgs
 */
void trajCB(const geometry_msgs::PoseArray& traj_msg) {
  // sanity check for traj_msg size
  if (traj_msg.poses.size() == 0) {
    ROS_ERROR_THROTTLE(1, "Empty trajectory vertices msg.");
    return;
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 0 |  16.485 - Fall 2020  - Lab 4 coding assignment  (10 pts)
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  //  As a simple warm up exercise before we get to the actual 'real deal',
  //  let's just make our quadcopter fly to the first gate in the course.
  //  In this section:
  //   1. Extract the first vertex of the trajectory
  //   2. Set the acceleration and velocities to zero
  //   3. Publish the desired MultiDOFJointTrajectoryPoint
  //   4. Create and publish TF transform of the desired pose
  // ~~~~ begin solution
  // Get the first gate pose
  geometry_msgs::Pose first_gate_pose = traj_msg.poses[0];
  geometry_msgs::Twist velocity;
  velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;

  geometry_msgs::Twist acceleration;
  acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
  // Publish
  trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
  msg.transforms.resize(1);
  // msg.transforms[0] = desired_pose.transform;
  msg.velocities.resize(1);
  msg.velocities[0] = velocity;
  msg.accelerations.resize(1);
  msg.accelerations[0] = acceleration;
  desired_state_pub.publish(msg);

  // Create and publish TF transform of the desired pose
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(first_gate_pose.position.x,
                                  first_gate_pose.position.y,
                                  first_gate_pose.position.z));
  tf::Quaternion q(first_gate_pose.orientation.x, first_gate_pose.orientation.y,
                    first_gate_pose.orientation.z, first_gate_pose.orientation.w);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world",
                                         "desired_state"));

  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 0
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_traj_planner");
  ros::NodeHandle n;
  ros::Rate loop_rate(500);
  ros::Time start(ros::Time::now());

  // deired traj vertices subscriber
  desire_traj_vertices_sub = n.subscribe("desired_traj_vertices", 10, trajCB);

  // publisher for desired states for controller
  desired_state_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          "desired_state", 1);

  br = new tf::TransformBroadcaster();

  ros::spin();
}
