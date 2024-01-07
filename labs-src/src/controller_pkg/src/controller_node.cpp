#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <cmath>

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  16.485 - Fall 2019  - Lab 3 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a C++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

#include <eigen3/Eigen/Dense>

// If you choose to use Eigen, tf2 provides useful functions to convert tf2
// messages to eigen types and vice versa.
#include <tf2_eigen/tf2_eigen.h>

// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class controllerNode{
  ros::NodeHandle nh;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  ros::Subscriber desired_state_sub;
  ros::Subscriber current_state_sub;
  ros::Publisher prop_speeds_pub;
  ros::Timer control_loop;

    

  //
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  Eigen::Vector3d omegad;// desired angular velocity of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle
  bool controller_active_curr_state;
  bool controller_active_des_state;
  double hz;             // frequency of the main control loop


  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  controllerNode():e3(0,0,1),F2W(4,4),hz(1000.0){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 2 |  Initialize ROS callback handlers
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // In this section, you need to initialize your handlers from part 1.
      // Specifically:
      //  - bind controllerNode::onDesiredState() to the topic "desired_state"
      //  - bind controllerNode::onCurrentState() to the topic "current_state"
      //  - bind controllerNode::controlLoop() to the created timer, at frequency
      //    given by the "hz" variable
      //
      // Hints: 
      //  - use the nh variable already available as a class member
      //  - read the lab 3 handout to fnd the message type
      //
      // ~~~~ begin solution
      //

      
        desired_state_sub = nh.subscribe("desired_state",1, &controllerNode::onDesiredState, this);
        current_state_sub = nh.subscribe("current_state",1, &controllerNode::onCurrentState, this);
        control_loop = nh.createTimer(ros::Duration(1/hz), &controllerNode::controlLoop, this);
        prop_speeds_pub = nh.advertise<mav_msgs::Actuators>("rotor_speed_cmds", 1);
          
        control_loop.start();


      //
      // ~~~~ end solution

      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 2
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 6 [NOTE: save this for last] |  Tune your gains!
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Live the life of a control engineer! Tune these parameters for a fast
      // and accurate controller.
      //
      // Controller gains
      //
      //
      // **** FIDDLE WITH THESE! ***
      // Change them in the provided launch file.
      nh.getParam("kx", kx);
      nh.getParam("kv", kv);
      nh.getParam("kr", kr);
      nh.getParam("komega", komega);
      ROS_INFO("Gain values:\nkx: %f \nkv: %f \nkr: %f \nkomega: %f\n", kx, kv, kr, komega);

      //
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 6
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

      // Initialize constants
      m = 1.0;
      controller_active_curr_state = false;
      controller_active_des_state = false;
      cd = 1e-5;
      cf = 1e-3;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
      // Make F2W matrix for converting from wrench to rotor speeds with 45 degree offset
      double cd_cf = cd/cf;   
      double c45 = cos(PI/4);
      double s45 = sin(PI/4);
      F2W << 1.0*cf, 1.0*cf, 1.0*cf, 1.0*cf,
              cf*d*s45, cf*d*s45, -cf*d*s45, -cf*d*s45,
              -cf*d*c45, cf*d*c45, cf*d*c45, -cf*d*c45,
              cd, -cd, cd, -cd;
    
      // F2W << -1.0, -1.0, -1.0, -1.0,
      //        -d, -d, d, d,
      //        d, -d, -d, d,
      //        -cd/cf, cd/cf, -cd/cf, cd/cf;
  }

  void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 3 | Objective: fill in xd, vd, ad, yawd
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.
      //
      // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
      //
      // ~~~~ begin solution

      //


      xd << des_state.transforms[0].translation.x, des_state.transforms[0].translation.y, des_state.transforms[0].translation.z;
      // xd = Eigen::Vector3d(0.0, 0.0, des_state.transforms[0].translation.z);
      // vd = Eigen::Vector3d(0, 0, des_state.velocities[0].linear.z);
      // ad = Eigen::Vector3d(0, 0, des_state.accelerations[0].linear.z);
      vd << des_state.velocities[0].linear.x, des_state.velocities[0].linear.y, des_state.velocities[0].linear.z;
      ad << des_state.accelerations[0].linear.x, des_state.accelerations[0].linear.y, des_state.accelerations[0].linear.z;
      yawd = tf2::getYaw(des_state.transforms[0].rotation);
      ROS_INFO("Yawd: %f\n", yawd);

      Eigen::Vector3d omegad = Eigen::Vector3d(0, 0, des_state.velocities[0].angular.z);
      controller_active_des_state = true;

      // 
      //
      // ~~~~ end solution
      //
      // 3.2 Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable
      //
      //  Hints:
      //    - use the methods tf2::getYaw(...)
      //    - maybe you want to use also tf2::fromMsg(...)
      //
      // ~~~~ begin solution
      //
      //     **** FILL IN HERE ***
      //
      // ~~~~ end solution
      //
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 3
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void onCurrentState(const nav_msgs::Odometry& cur_state){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 4 | Objective: fill in x, v, R and omega
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //    
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!
      //
      // ~~~~ begin solution
      //
      //     **** FILL IN HERE ***
      x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
      v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;
      //Convert the cur_state.pose.pose.orientiation from quaternion to rotation matrix

      Eigen::Quaterniond quat;
      Eigen::fromMsg(cur_state.pose.pose.orientation, quat);
      R = quat.toRotationMatrix();
      // ROS info this matrix
      std::stringstream ss;
      ss << "Trajectory Position"
         << " R:" << R;
      ROS_INFO("%s", ss.str().c_str());
      ROS_INFO_STREAM(R);

      omega = R.transpose()*Eigen::Vector3d(cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z);
      controller_active_curr_state = true;
      //
      // ~~~~ end solution
      //
      // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
      //                                 end part 4
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }

  void controlLoop(const ros::TimerEvent& t){
    Eigen::Vector3d ex, ev, er, eomega;

    // // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // //  PART 5 | Objective: Implement the controller!
    // // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    // //
    // // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    // //  Hint: [1], eq. (6), (7)
    // //
    // // ~~~~ begin solution
    // //
    // //     **** FILL IN HERE ***
    ex = x - xd;
    ev = v - vd;
    // //
    // // ~~~~ end solution

    // // 5.2 Compute the Rd matrix.
    // //
    // //  Hint: break it down in 3 parts:
    // //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    // //    - check out [1] fig. (3) for the remaining axes [use cross product]
    // //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    // //
    // //  CAVEATS:
    // //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    // //      paper. The z-axes are flipped, which affects the signs of:
    // //         i) the gravity term and
    // //        ii) the overall sign (in front of the fraction) in equation (12)
    // //            of the paper
    // //    - remember to normalize your axes!
    // //
    // // ~~~~ begin solution
    // //
    // //     **** FILL IN HERE ***
    Eigen::Vector3d b3d = (-kx*ex - kv*ev + m*g*e3 + m*ad)/((-kx*ex - kv*ev + m*g*e3 + m*ad).norm()); // Might need to replace the - in front
    Eigen::Vector3d x1d_uv = Eigen::Vector3d(cos(yawd), sin(yawd), 0);
    x1d_uv = x1d_uv/(x1d_uv.norm());
    // Error could be here
    Eigen::Vector3d b1d = ((b3d.cross(x1d_uv)).cross(b3d))/(((b3d.cross(x1d_uv)).cross(b3d)).norm());

    // Cross product
    Eigen::Vector3d b2d = b3d.cross(x1d_uv)/(b3d.cross(x1d_uv).norm());

    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;


    // //
    // // ~~~~ end solution
    // //
    // // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)
    // //  Hints:
    // //     - [1] eq. (10) and (11)
    // //     - you can use the Vee() static method implemented above
    // //
    // //  CAVEAT: feel free to ignore the second addend in eq (11), since it
    // //          requires numerical differentiation of Rd and it has negligible
    // //          effects on the closed-loop dynamics.
    // //
    // // ~~~~ begin solution
    // //
    // //     **** FILL IN HERE ***
    er = 0.5*Vee(Rd.transpose()*R-R.transpose()*Rd);
    eomega = omega - R.transpose()*Rd*omegad; // This isnt right because we need omega_d
    
    // //
    // // ~~~~ end solution
    // //
    // // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    // //  Hints:
    // //     - [1] eq. (15), (16)

    // // CAVEATS:
    // //    - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    // //      paper. The z-axes are flipped, which affects the signs of:
    // //         i) the gravity term
    // //        ii) the overall sign (in front of the bracket) in equation (15)
    // //            of the paper
    // //
    // //    - feel free to ignore all the terms involving \Omega_d and its time
    // //      derivative as they are of the second order and have negligible
    // //      effects on the closed-loop dynamics.
    // //
    // // ~~~~ begin solution
    // //
    // //     **** FILL IN HERE ***
    double f = (-kx*ex - kv*ev + m*g*e3 + m*ad).dot(R*e3); 
    // double f = 4000;
    Eigen::Vector3d M = -kr*er - komega*eomega + omega.cross(J*omega);
    // Eigen::Vector3d M = Eigen::Vector3d(0,0,0);
    Eigen::Vector4d wrench;
    wrench << f, M;
    // std::stringstream ss;
    // ss << "Trajectory Position"
    //     << " f:" << f
    //     << " M:" << M;
    // ROS_INFO("%s", ss.str().c_str());
    // //
    // // ~~~~ end solution
    // // 5.5 Recover the rotor speeds from the wrench computed above
    // //
    // //  Hints:
    // //     - [1] eq. (1)
    // //
    // // CAVEATs:
    // //     - we have different conventions for the arodynamic coefficients,
    // //       Namely: C_{\tau f} = c_d / c_f
    // //               (LHS paper [1], RHS our conventions [lecture notes])
    // //
    // //     - Compare the reference frames in the Lab 3 handout with Fig. 1 in the
    // //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    // //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    // //       between b1 and b2). To resolve this, check out equation 6.9 in the
    // //       lecture notes!
    // //
    // //     - The thrust forces are **in absolute value** proportional to the
    // //       square of the propeller speeds. Negative propeller speeds - although
    // //       uncommon - should be a possible outcome of the controller when
    // //       appropriate. Note that this is the case in unity but not in real
    // //       life, where propellers are aerodynamically optimized to spin in one
    // //       direction!
    // //
    // // ~~~~ begin solution
    // //
    // //     **** FILL IN HERE ***
    Eigen::VectorXd prop_speeds;
    prop_speeds = F2W.inverse()*wrench;
    // Take the signed sqrt
    for (int i = 0; i < 4; i++){
        prop_speeds(i) = signed_sqrt(prop_speeds(i));
    }
    // prop_speeds.cwiseMax(Eigen::VectorXd::Zero(4));
    // //
    // ~~~~ end solution
    //
    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //
    // ~~~~ begin solution
    //
    //     **** FILL IN HERE ***



    mav_msgs::Actuators prop_speeds_msg;

    // // Convert the prop_speeds to float64[]
    // std::vector<double> stdVector(prop_speeds.data(), prop_speeds.data() + prop_speeds.size());
    // prop_speeds_msg.angular_velocities.resize(4);
    if (!(controller_active_curr_state && controller_active_des_state)){
      prop_speeds_msg.angular_velocities = {0.0,0.0, 0, 0.0};
      return;
    }
    prop_speeds_msg.angular_velocities.clear();
    for (int i = 0; i<4; i++)
    {
      // prop_speeds_msg.angular_velocities.push_back(200);
      prop_speeds_msg.angular_velocities.push_back(prop_speeds[i]);
      // prop_speeds[i]
    }
    prop_speeds_msg.header.stamp = ros::Time::now();

    prop_speeds_pub.publish(prop_speeds_msg);
    // Publish ros info

    std::stringstream ss;
    ss << "Trajectory Position"
      << " prop1:" << prop_speeds[0]
      << " prop2:" << prop_speeds[1]
      << " prop3:" << prop_speeds[2]
      << " prop4:" << prop_speeds[3];
    ROS_INFO("%s", ss.str().c_str());
    ROS_INFO_STREAM(prop_speeds_msg);



    // ros::spinOnce();
    // // // Sleep 1 timesteep
    // ros::Duration(1/hz).sleep();



    //
    // ~~~~ end solution
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //           end part 5, congrats! Start tuning your gains (part 6)
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  controllerNode n;
  ros::spin();
}
