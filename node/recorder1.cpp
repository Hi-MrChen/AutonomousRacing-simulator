// Function 
// input: /joy /odom /gt_pose
// output: 

#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace racecar_simulator;

class Recorder {
private:
    
    ros::NodeHandle n;
    
    ros::Subscriber drive_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber key_sub;
    ros::Subscriber mux_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber gt_pose_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber odom_sub;

    // Mux indices
    int joy_mux_idx;
    int key_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;
    std::vector<bool> prev_mux;

    // Params for joystick calculations
    int joy_speed_axis, joy_angle_axis;


public:

    // Data to record ***************************************
    double x=0.0, y=0.0, theta=0.0;
    double gt_x=0.0, gt_y=0.0, gt_theta=0.0;
    double throttle=0.0, steering=0.0;

    Recorder(){
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // Get the topic names
        std::string drive_topic, pose_topic, gt_pose_topic, odom_topic, imu_topic, joy_topic, key_topic;

        n.getParam("drive_topic", drive_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("imu_topic", imu_topic);
        n.getParam("pose_topic", pose_topic);
        n.getParam("ground_truth_pose_topic", gt_pose_topic);
        n.getParam("joy_topic", joy_topic);
        n.getParam("keyboard_topic", key_topic);

        // get mux indices
        n.getParam("joy_mux_idx", joy_mux_idx);
        n.getParam("key_mux_idx", key_mux_idx);

                // get size of mux
        n.getParam("mux_size", mux_size);

        // initialize mux controller
        mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        drive_sub = n.subscribe(drive_topic, 1, &Recorder::drive_callback, this);
        joy_sub = n.subscribe(joy_topic, 1, &Recorder::joy_callback, this);
        key_sub = n.subscribe(key_topic, 1, &Recorder::key_callback, this);
        pose_sub = n.subscribe(pose_topic, 1, &Recorder::pose_callback, this);
        gt_pose_sub = n.subscribe(gt_pose_topic, 1, &Recorder::gt_pose_callback, this);
        odom_sub = n.subscribe(odom_topic, 1, &Recorder::odom_callback, this);

        
    }

    void pose_callback(const geometry_msgs::PoseStamped & msg) {
        x = msg.pose.position.x;
        y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        theta = tf2::impl::getYaw(quat);
    }

    void gt_pose_callback(const geometry_msgs::PoseStamped & msg) {
        gt_x = msg.pose.position.x;
        gt_y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        gt_theta = tf2::impl::getYaw(quat);
    }
    void odom_callback(const nav_msgs::Odometry & msg) {
        // x = msg.pose.pose.position.x;
        // y = msg.pose.pose.position.y;
        // geometry_msgs::Quaternion q = msg.pose.pose.orientation;
        // tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        // theta = tf2::impl::getYaw(quat);
        ;
    }

    void joy_callback(const sensor_msgs::Joy & msg) {
        // make drive message from joystick if turned on
        if (mux_controller[joy_mux_idx]) {
            throttle = msg.axes[joy_speed_axis];
            steering = msg.axes[joy_angle_axis];

        }
    }

    void key_callback(const std_msgs::String & msg) {
        ;
    }
    
    void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
        // desired_speed = msg.drive.speed;
        // desired_steer_ang = msg.drive.steering_angle;
        ;
    }


    
};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "recorder");
    Recorder rc;     
            
    std::ofstream file("new.csv");

    ros::Rate loop_rate(50); 
    
    while(ros::ok())
    {
        ros:: spinOnce();
        
        file << rc.x << "," << rc.y << "," << rc.theta << "," 
        << rc.throttle << "," << rc.steering << "/n" ;

        loop_rate.sleep();
    }
    

    file.close();

    return 0;
}


