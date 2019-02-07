//  ///////////////////////////////////////////////////////////
//
// Pure Pursuit Path Following algorithm
// Assumes that robot starting position is the same as in the PRM calculated path plan
//
// Author: Michal Kaca
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

ros::Publisher pose_publisher;
ros::Publisher velocity_publisher;
ros::Publisher path_pub;
ros::Publisher given_path_pub;

geometry_msgs::Twist vel;
nav_msgs::Path givenPathMsg;
nav_msgs::Path pathMsg;

// init current index (to start following)
int current_index = 0;

// robot speed
double linearSpeed = 0.05; // m/s
double angleSpeed = 0.2;  // m/s

//init path vectors    ..... MODIFY to obtain vectors from PRM
std::vector<float> pathVectorX = {0.0, 0.3, 0.5, 0.5, 0.8, 1.0,
 1.5, 2.0, 2.5, 3.0, 3.2, 3.3,  2.9, 3.4, 4.0, 4.5, 4.5, 4.5};
std::vector<float> pathVectorY = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0,
 1.1, 0.0, 1,0, 0.0, 0.0, 0.2, -0.5, 0.1, 0.0, 1.0, 2.0, 1.5};
std::vector<geometry_msgs::PoseStamped> givenVectorPath; //given input


// current robot position
float ips_yaw;
float ips_x;
float ips_y;

// Custom lookahead distance
float lookahead = 1; 

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    //ROS_INFO_STREAM(" UPDATED POSES");

}

/*void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
    	geometry_msgs::PoseWithCovarianceStamped curpose;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id="/map";
	curpose.pose.pose.position = msg.pose.pose.position;
	curpose.pose.pose.orientation = msg.pose.pose.orientation;
	//republish pose for rviz
	pose_publisher.publish(curpose);
}*/

//pesudo code
/*
1. Get vector of points to follow
2. Get distances from your point to first point in vector, check if great than lookup, if not, get distance to next point and amalgamate, else set point as target and move there, record index

Small lookahead --moves quickly
Large lookahead --overshoots corners more severly
*/

// Returns linear distance between two points
float getLinearDistance (float x0, float y0, float x1, float y1){
    return pow(pow(y0 - y1, 2) + pow(x0 - x1, 2), 0.5);
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"try_moving_bitch");
    ros::NodeHandle n;

    int step = 1;

    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    //Subscribe to the desired topics and assign callbacks
    //ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);

    //pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vis_pos", 1, true);

    // For visualizing path
    path_pub = n.advertise<nav_msgs::Path>("/pathFlex",1,true);
    given_path_pub = n.advertise<nav_msgs::Path>("/givenPath",1,true);


    //// Generate vector path for testing:
    for (int i = 0; i < pathVectorX.size(); i++){
        geometry_msgs::PoseStamped pathPose;
        pathPose.pose.position.x = pathVectorX[i];
        pathPose.pose.position.y = pathVectorY[i];
        givenVectorPath.push_back(pathPose);
    }
    
    givenPathMsg.header.frame_id = "odom";
    givenPathMsg.poses = givenVectorPath;
    /// Section for testing path ends


    geometry_msgs::PoseStamped pathPose;
    std::vector<geometry_msgs::PoseStamped> vectorPath;
    pathMsg.header.frame_id = "odom";

    given_path_pub.publish(givenPathMsg); // publishes given position (provided by path planner)

/*
    // set speeds to zero
    vel.linear.x = 0; //m /s
    vel.angular.z = 0;
    velocity_publisher.publish(vel); // Publish the command velocity
*/
    //Set the loop rate
    ros::Rate loop_rate(50);    // update rate in Hz
	

    while (ros::ok())
    {

        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages --> processes callbacks

        // init transitional variables
        float radius;
        float newVehicleDirection;
        int rotateCounter = 0;

        // MAIN LOOP HERE 
        switch(step) {
            // Calculates optimal path and point to follow for Robot
            case 1:{
                // Gets total path 
                // Basically algo below checks for distance between points, (between point 0 to point 1, point 1 to point 2, etc... and once lookahead distance is reached, we go to that point)
                float totDistance = 0;
                float workingDistance;
                bool first_loop = true;
                while (1){
                    if (first_loop){
                        // ends path follower
                        if (current_index == pathVectorY.size() - 1){
                            ROS_INFO_STREAM("ENDING PROGRAM... Index: " << current_index);
                            return 0;
                        }
                        workingDistance = getLinearDistance(ips_x, ips_y, pathVectorX[current_index], pathVectorY[current_index]);
                        ROS_INFO_STREAM("First Loop working distance:" << workingDistance);
                    }
                    else{
                        workingDistance = getLinearDistance(pathVectorX[current_index-1], pathVectorY[current_index-1], pathVectorX[current_index], pathVectorY[current_index]);
                    }
                    if (totDistance + workingDistance < lookahead) {
                        totDistance = totDistance + workingDistance;
                        current_index++;
                    }
                    else{
                        if (first_loop){
                            ROS_INFO_STREAM("THIS IS SOME BULLLLLLLSHIT: lookahead distance is TOO SMALL");
                        }
                        break;
                    }
                    first_loop = false;
                }

                // ends path follower
                if (current_index == pathVectorY.size()){
                    ROS_INFO_STREAM("ENDING PROGRAM... Index: " << current_index);
                    return 0;
                }
                // get curvature  (1/radius)
                float linDist = getLinearDistance(ips_x, ips_y, pathVectorX[current_index],pathVectorY[current_index]);
                float curvature = 2*(pathVectorY[current_index] - ips_y)/ (linDist*linDist);
                radius = 1.0/curvature;

                ROS_INFO_STREAM("targetY:" << pathVectorY[current_index] 
                    << " targetX: " << pathVectorX[current_index]);
                ROS_INFO_STREAM("current Y Pos: " << ips_y << "current X Pos: " << ips_x);
                
                //ROS_INFO_STREAM("current_index:" << current_index);

                //get circle center
                float xMed = (ips_x + pathVectorX[current_index])/2;
                float yMed = (ips_y + pathVectorY[current_index])/2;
                //ROS_INFO_STREAM("xMed:" << xMed << " yMed: " << yMed);

                // here we obtain the larger circle (so we add medians to trig method)
                float xCir = xMed + std::sqrt(radius*radius - pow(linDist/2,2))*
                (ips_y-pathVectorY[current_index])/linDist; 
                float yCir = yMed + std::sqrt(radius*radius - pow(linDist/2,2))*
                (pathVectorX[current_index] - ips_x)/linDist;

                ROS_INFO_STREAM("xCir:" << xCir << " yCir: " << yCir);
                ROS_INFO_STREAM("radius:" << radius);

                //get radius direction (angle)
                float radiusDir =  atan2((yCir - ips_y),(xCir - ips_x));
                float newVehicleDirection = radiusDir - M_PI/2; 

                /*// Find starting angle based on cosine law and trigonometry
                // cos law: a^2 = b^2 +c^2 -2abcosA
                float middleAngle = radiusDir - M_PI/2; 
                float a = getLinearDistance(xCir, yCir, ips_x, ips_y);
                float b = getLinearDistance(xCir, yCir, pathVectorX[current_index], pathVectorY[current_index]);
                float differenceAngle = acos( (linDist*linDist - a*a - b*b)/(-2.0*a*b) );
                ROS_INFO_STREAM("diff Angle: " << differenceAngle);
                newVehicleDirection = middleAngle - differenceAngle/2.0;*/
                ROS_INFO_STREAM("radiusDir:" << radiusDir); 
                ROS_INFO_STREAM("newVehicleDirection:" << newVehicleDirection);
                ROS_INFO_STREAM("currentVehicleOrientation:" << ips_yaw);
                ROS_INFO_STREAM(" ");

                // Set angular direction 
                if (ips_yaw > newVehicleDirection){
                    angleSpeed = angleSpeed*(-1);
                }
                step = 2;
                break;
            }

            case 2:{
                // set robot in correct starting orientation
                if (std::abs(ips_yaw - newVehicleDirection) > 0.08){
                    // Set angular direction 
                    if (ips_yaw < newVehicleDirection){
                        angleSpeed = angleSpeed*(-1);
                    } 
                    vel.angular.z = angleSpeed; // set angular speed for rotating robot to its target
                    ROS_INFO_STREAM("anglespeed IS SET:" << angleSpeed);
                }
                step = 3;
                break;
            }

            case 3:{
                if (std::abs(ips_yaw - newVehicleDirection) > 0.08){
                    vel.angular.z = angleSpeed;
                }
                else{
                    ROS_INFO_STREAM("ANGLE SET TO: " << ips_yaw);
                    vel.angular.z = 0;
                    rotateCounter = 0;
                    step = 4;
                }
                rotateCounter++;
                if (rotateCounter > 600000)
                {
                    ROS_INFO_STREAM(" COUNTER HIT 60, exiting");
                    return 0;
                }
                break;
            }

            case 4:{
                ROS_INFO_STREAM("Mans stopped spinning. Moving towards target");

                // Set robot to start following circular path for allotted amount of time
                vel.angular.z = linearSpeed/radius; // stop spinning                             MIGHT HAVE TO BE NEGATIVE OR ABSOLUTE  ---> NEEDS TESTING
                vel.linear.x = linearSpeed;
                //ROS_INFO_STREAM("PUBLISHED ANGULAR SPEED IS: " << linearSpeed/radius);
                step = 5;
                break;
            }

            case 5:{
                if (getLinearDistance(ips_x, ips_y, pathVectorX[current_index], pathVectorY[current_index]) < 0.15){
                    // stop robot to prepare for next calculation                                         
                vel.angular.z = 0; // stop spinning
                vel.linear.x = 0;
                ROS_INFO_STREAM("STEP 5 COMPLETE");
                step = 1;
                }
                /*else
                    ROS_INFO_STREAM("STEP 5 IS HUSTTTTTTLING");*/
            }
        }
                        ROS_INFO_STREAM(step);

        velocity_publisher.publish(vel); // Publish the command velocity*/

        
        //ROS_INFO_STREAM("Re-running loop");


        // update pose path for rviz VISUALIZATION
        pathPose.pose.position.x = ips_x;
        pathPose.pose.position.y = ips_y;
        // create quaternion from yaw angle
        geometry_msgs::Quaternion msgQuat;
        msgQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, ips_yaw); 
        pathPose.pose.orientation = msgQuat;
        vectorPath.push_back(pathPose);
        pathMsg.poses = vectorPath;
        path_pub.publish(pathMsg);
    }

    return 0;
}


 /*
    // QUICK TEST:
    int supernoodecount = 0;
    geometry_msgs::Twist velTest;
    while (supernoodecount < 50){
        velTest.linear.x = linearSpeed*2;
        velTest.angular.z = angleSpeed*2;
        velocity_publisher.publish(velTest);
        ros::Duration(0.2).sleep();
    }
    return 0;*/