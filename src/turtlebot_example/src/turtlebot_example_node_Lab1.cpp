#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#define WINDOW_LEN 40

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
    //This function is called when a new position message is received
    double X = 0;
    double Y = 0;
    int seq = 0;
    double Yaw = 0;
    double saveYaw = 0;
    double saveX = 0;
    double saveY = 0;
    geometry_msgs::Twist vel;
//}

void pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //This function is called when a new position message is received
    X = msg->pose.pose.position.x; // Robot X psotition
    Y = msg->pose.pose.position.y; // Robot Y psotition
    Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
    if (seq == 0) {
        saveYaw = Yaw;
        saveX = X;
        saveY = Y;
    }
    seq = msg->header.seq;
}

// array of absolute error values
bool CheckWindow(double* nums, int len, double threshold) {
    bool settled = true;
    for (int i = 0; i < len; i++){
        if (nums[i] > threshold){
            settled = false;
            break;
        }
    }
    return settled;
}


int main(int argc, char **argv){
    //Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    int side = 1;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/odom", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable

    //Set the loop rate
    ros::Rate loop_rate(80);
    double error_window[WINDOW_LEN];
    for (int i = 0; i < WINDOW_LEN; i++) {
        error_window[i] = 1;
    }
    int error_index = 0;

    double target_yaw = 0;
    int time_counter = 0;
    double dist = 0;
    const double square_length = 1, speed_offset = 0.05;
    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    
        //Main loop code goes here:
        switch(side) {
            case 1:
                dist = sqrt(pow((X - saveX), 2) + pow((Y - (saveY)), 2));
                //ROS_INFO("dist: %lf", dist);
                if( dist < square_length) {
                    vel.linear.x = -pow(dist, 2) + square_length*dist + speed_offset;
                    vel.angular.z = 0.0;
                }
                else {
                    vel.linear.x = 0.0;
                    vel.angular.z = 0.0;
                    side = 2;
                    saveYaw = Yaw;
                }
                break;
            case 2:
                //set new target yaw value
                target_yaw = saveYaw - M_PI/2;
                target_yaw = target_yaw < -M_PI ? 2*M_PI + target_yaw : target_yaw;
                
                ROS_INFO("current Yaw: %f goal Yaw: %f", Yaw, target_yaw);
                error_window[error_index] = std::min(fabs(Yaw - target_yaw), 2*M_PI - fabs(Yaw - target_yaw));
                error_index++;
                if (error_index >= WINDOW_LEN)
                    error_index = 0;
                if (!CheckWindow(error_window, WINDOW_LEN, 0.02)) {
                    vel.linear.x = 0.0;
                    vel.angular.z = -1 * atan2(sin(Yaw-target_yaw), cos(Yaw-target_yaw)); 
                }
                else {
                    saveX = X;
                    saveY = Y;
                    side = 1;
                }
                break;
        }
        velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
