#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher velocity_pub_; // Publisher for velocity
ros::Publisher error_pub_;    // Publisher for error

geometry_msgs::PoseStamped kyle_pose_; // Pose of top turtle
ros::Time last_msg_time_;              // Last time callback was called (to calculate delta t)

double integral_, pre_error_;

// PID Global Variables
double Kp_ = 6;
double Ki_ = 0.3;
double Kd_ = 0.2;

/**
 * Callback for Kyle (top turtle). Saves the position of the top turtle into the global
 * variable kyle_pose_
 * @param msg message containing Kyle's pose
 */
void kylePoseCallback(geometry_msgs::PoseStamped msg)
{
    kyle_pose_ = msg;
}


/**
 * Uses PID terms (Kp, Ki, Kd) to comput output given error and delta time
 * @param error error feed into PID (goal.x - current.x)
 * @param dt delta time from last update to this one
 */
double pid(double error, double dt) {

    double p = Kp_ * error;

    integral_ += error * dt;
    double i = Ki_ * integral_;
    
    double d = Kd_ * (error - pre_error_) / dt;
    pre_error_ = error;

    return p + i + d;
}


/**
 * Callback for Oswin (bottom turtle). Calculates the error in x between the two turtles,
 * and then uses a PID Controller to calculate a control to publish
 * @param msg message containing Oswin's pose
 */
void oswinPoseCallback(geometry_msgs::PoseStamped msg)
{
    // Check if last_msg_time_ is populated (not first callback call)
    if (last_msg_time_.sec == 0)
    {
    	last_msg_time_ = msg.header.stamp;
    	return;
    }

    // Call PID
    double error = kyle_pose_.pose.position.x - msg.pose.position.x;
    double dt = (msg.header.stamp - last_msg_time_).toSec();
    double control = pid(error, dt);

    last_msg_time_ = msg.header.stamp;

    // publish a geometry_msgs::Twist message so that the turtle will move
    geometry_msgs::Twist control_output;
    control_output.linear.x = control;
    velocity_pub_.publish(control_output);

    // publish a std_msgs::Float64 message to be able to graph the error in rqt_plot
    std_msgs::Float64 error_output;
    error_output.data = error;
    error_pub_.publish(error_output);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_node");

    ros::NodeHandle nh;

    // Advertise "/oswin/velocity" to control the bottom turtle and "/error" for visualization
    velocity_pub_ = nh.advertise<geometry_msgs::Twist>("oswin/velocity", 1);
    error_pub_ = nh.advertise<std_msgs::Float64>("error", 1);

    // Subscriber to both ground truth topics to get their positions
    ros::Subscriber kyle_sub = nh.subscribe("kyle/ground_truth", 1, kylePoseCallback);
    ros::Subscriber oswin_sub = nh.subscribe("oswin/ground_truth", 1, oswinPoseCallback);

    ros::spin();
}
