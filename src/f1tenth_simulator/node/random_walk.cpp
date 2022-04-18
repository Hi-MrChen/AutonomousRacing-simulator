// Function 
// input: Odometry  
// output: drive_st_msg 
// random_walk allow car to drive randomly at a fixed speed but changing steering angle

#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

bool isRandomWalker; 
int mode = 1;
bool init = false;

class RandomWalker {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, max_throttle, max_steering;
    double max_incre;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;
    // For printing
    std::vector<bool> prev_mux;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Listen for mux messages 
    ros::Subscriber mux_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // Publish control command 
    ros::Publisher ctrl_pub;

    // previous desired steering angle
    double prev_angle=0.0;
    double prev_throttle=0.0;
    double prev_speed=0.0;

    // 
    int throttle_num = 0;
    int steering_num = 0;
    double throttle = 0;
    double steering = 0;


public:

    RandomWalker() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, mux_topic;
        n.getParam("rand_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("mux_topic", mux_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        n.getParam("max_throttle", max_throttle);
        n.getParam("max_steering", max_steering);
        n.getParam("max_incre", max_incre);

        // get size of mux
        n.getParam("mux_size", mux_size);
        
        int random_walker_mux_idx;
        n.getParam("random_walker_mux_idx", random_walker_mux_idx);

        // initialize mux controller
        mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Make a publisher for control commands 
        ctrl_pub = n.advertise<std_msgs::Float32MultiArray>("/ctrl", 10);

        // Start a subscriber to listen to odom messages
        // odom_sub = n.subscribe(odom_topic, 1, &RandomWalker::odom_callback, this);

        // Start a subscriber to listen to mux messages
        mux_sub = n.subscribe(mux_topic, 1, &RandomWalker::mux_callback, this);
    }

    void mux_callback(const std_msgs::Int32MultiArray & msg) {
        // reset mux member variable every time it's published
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = bool(msg.data[i]);
        }
        if( mux_controller[2]){
            isRandomWalker = true;
        }
        else{
            isRandomWalker = false;
        }
    }

    void mode1() {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        if (!init){
            throttle = 0;steering = 0;
            throttle_num=1;steering_num=0;
            init = true;
        }

        // only change the steering 
        else if (steering_num <= 100 && throttle_num < 100){
            steering_num += 1;

            // get random 
            double random = ((double) rand() / RAND_MAX);
            double range = max_steering / 2.0;
            double rand_ang = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            if ((random > 0.8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
            }   

            // set angle (add random change to previous angle)
            steering = std::min(std::max(prev_angle + rand_ang, -max_steering), max_steering);

             // reset previous desired angle
            prev_angle = steering;
        }
        
        else if (throttle_num < 100){
            steering_num =0;
            throttle_num += 1; 

            double random = ((double) rand() / RAND_MAX);
            double range = max_throttle / 2.0; 
            double rand_throttle  = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            if ((random > 0.8) && (prev_throttle != 0)) {
            double sign_rand = rand_throttle / std::abs(rand_throttle);
            double sign_prev = prev_throttle / std::abs(prev_throttle);
            rand_throttle *= sign_rand * sign_prev;
            }

            // set angle (add random change to previous angle)
            throttle = std::min(std::max(prev_throttle + rand_throttle, -max_throttle), max_throttle);
            
            // reset prev_throttle
            prev_throttle = throttle;
        }
        else{
            init = false;
            ROS_INFO("Mode1 Over !!!!!!!!!!!!!!!!!!!!!!\n");
            mode += 1;
        }



        // publish control command
        std_msgs::Float32MultiArray ctrl_msg;
        ctrl_msg.data.clear();
        ctrl_msg.data.push_back(throttle);
        ctrl_msg.data.push_back(steering);
        ctrl_msg.data.push_back(throttle_num);
        ctrl_msg.data.push_back(steering_num);
        ctrl_pub.publish(ctrl_msg);


        // Calculate desired velocity and steering angle 
        double desired_velocity= prev_speed * 0.8;
        if (throttle * desired_velocity >= 0 ){
            desired_velocity = desired_velocity + max_incre * throttle;
        }
        else {
            desired_velocity = desired_velocity + 1.5 * max_incre * throttle;
        }
        desired_velocity = std::min(std::max(desired_velocity, -max_speed), max_speed);
        prev_speed = desired_velocity;

        // publish drive_msg
        double desired_steer = steering * max_steering_angle;

        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);

    }

    void mode2() {

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        if (!init){
            throttle = 0;steering = 0;
            throttle_num=0;steering_num=1;
            init = true;
        }

        // only change the throttle
        else if (throttle_num <= 100 && steering_num < 100){
            throttle_num += 1;

            double random = ((double) rand() / RAND_MAX);
            double range = max_throttle / 4.0; 
            double rand_throttle  = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            if ((random > 0.8) && (prev_throttle != 0)) {
            double sign_rand = rand_throttle / std::abs(rand_throttle);
            double sign_prev = prev_throttle / std::abs(prev_throttle);
            rand_throttle *= sign_rand * sign_prev;
            }

            // set angle (add random change to previous angle)
            throttle = std::min(std::max(prev_throttle + rand_throttle, -max_throttle), max_throttle);
            
            // reset prev_throttle
            prev_throttle = throttle;

        
        }
        else if (steering_num < 100){
            throttle_num =0;
            steering_num += 1; 

            // get random 
            double random = ((double) rand() / RAND_MAX);
            double range = max_steering / 1.5;
            double rand_ang = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            if ((random > 0.8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
            }   

            // set angle (add random change to previous angle)
            steering = std::min(std::max(prev_angle + rand_ang, -max_steering), max_steering);

                // reset previous desired angle
            prev_angle = steering;
        }
        else{
            throttle = 0;
            steering = 0;
            ROS_INFO("Mode2 Over !!!!!!!!!!!!!!!!!!!!!!\n");
            mode += 1;
        }


        // publish control command
        std_msgs::Float32MultiArray ctrl_msg;
        ctrl_msg.data.clear();
        ctrl_msg.data.push_back(throttle);
        ctrl_msg.data.push_back(steering);
        ctrl_msg.data.push_back(throttle_num);
        ctrl_msg.data.push_back(steering_num);
        ctrl_pub.publish(ctrl_msg);


        // Calculate desired velocity 
        double desired_velocity= prev_speed * 0.8;
        if (throttle * desired_velocity >= 0 ){
            desired_velocity = desired_velocity + max_incre * throttle;
        }
        else {
            desired_velocity = desired_velocity + 1.5 * max_incre * throttle;
        }
        desired_velocity = std::min(std::max(desired_velocity, -max_speed), max_speed);
        prev_speed = desired_velocity;

        // publish drive_msg
        double desired_steer = steering * max_steering_angle;

        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);

    }


}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "random_walker");
    RandomWalker rw;

    ros::Rate loop_rate(50); 

    while(ros::ok())
    {   
        ros::spinOnce();
        if(isRandomWalker){
            
            if(mode == 1) rw.mode1();
            else if(mode == 2) rw.mode2();
            else break;
        }
    
        loop_rate.sleep();
    }
    
    return 0;
}