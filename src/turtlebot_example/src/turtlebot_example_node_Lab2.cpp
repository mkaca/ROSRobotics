//  ///////////////////////////////////////////////////////////
//
// turtlebot_example_node_Lab2.cpp
// This file is used for the mapping section of Lab 2 for MTE 544
//
// Author: Michael Kaca
// Edited: Sebastian Thrun, jkjkjkjk
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "/opt/ros/kinetic/include/sensor_msgs/LaserScan.h"
#include <math.h>
#include <typeinfo>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher map_pub;
ros::Publisher path_pub;
ros::Publisher laser_init_pub;

// IPS positions
double ips_x;
double ips_y;
double ips_yaw;

// laser values
float angle_incr;
float max_angle;
float min_angle;
float scanner_min_range;
//float[] ranges; can't do float[] cuz c++ is dumb
std::vector<float> ranges;

//Occupancy grid value for initialization
float resolutionGlobal;
float realHeight = 20; //was 20
float targetResolution = 0.05;  // was 0.01
int gridSize = realHeight/targetResolution; //# of cells


short sgn(int x) { return x >= 0 ? 1 : -1; }

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and should be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    /**ROS_INFO_STREAM("x0: " << x0);
    ROS_INFO_STREAM("y0: " << y0);
    ROS_INFO_STREAM("x1: " << x1);
    ROS_INFO_STREAM("y1: " << y1);*/

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
        //ROS_INFO_STREAM("adding X: " << x0);
        //ROS_INFO_STREAM("adding Y: " << y0);
    }
}

/*
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
}*/

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	//ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
    ROS_INFO_STREAM("poseX: " << ips_x);
    ROS_INFO_STREAM("poseY: " << ips_y);
    ROS_INFO_STREAM("poseYaw: " << ips_yaw);
}

//   DONT forget to call function with &x, and &y , but declare x,y normally
// NOTE that it scans it from min angle to max angle
void convertRangeToRelativeXY(float distance, float* x, float* y, float angle_increment, 
    int step, float angle_min, float angle_max){

    // sanity check
    if (angle_min + angle_increment*step > angle_max){
         throw std::invalid_argument( "Your function DOES NOT WORK");
    }
    *x = distance * cos(angle_min + angle_increment*step);
    *y = distance * sin(angle_min + angle_increment*step);
}

// converts probability to logit value (from -1 to +1)
// input: probability
// output: logit (ln)
 float logit(float probability){
    //return log(probability/(1.0000-probability));  // ln
    return log10(probability/(1.0000-probability));  // log
 }

 // UNUSED
 float measurementModel(std::vector <float> &probabilityVector, 
    std::vector <int> vectorX, std::vector <int> vectorY, int length){
        for (int i = 0; i < length; i++ ){
        }
 }


// shitty measurement model --> but it will do :D
 float measModel(int length, int i){
    if (length - i > 1)  // sets last 1 point(s)
        return 0.2;  // flipped these twoa around //made it super small
    else
        return 0.8; // flipped these two around
 }

//x = odds
 int tfLogToROSOccGrid(float x){
    if (x>1)
        x = 1;
    else if (x < -1)
        x = -1;
    return int(50*(1 + x));
 }

 // gets inverse of logit
 // input: logit value
 //output: probability
 float inverseLogit (float x){
    return pow(10,x)/(pow(10,x)+1); // used for log
    //return (exp(x)/(exp(x)+1));   // used for ln
 }

bool gotten_map = false;
// happens everytime a message is called
// updates laser values per complete scan
void laser_callback(const sensor_msgs::LaserScan &msg){
    //ROS_INFO_STREAM("Updated Laser: " << msg.range_min);

    // NOTE: these should be constant
    angle_incr =  msg.angle_increment;
    min_angle =  msg.angle_min;
    max_angle =  msg.angle_max;
    ranges = msg.ranges;
    scanner_min_range = msg.range_min;
    ROS_INFO_STREAM("min range Laser: " << msg.range_min);
    gotten_map = true;
}


int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_mapping");
    ros::NodeHandle n;

    //Init vector for appending vectorPath
    std::vector<geometry_msgs::PoseStamped> vectorPath;

    //Subscribe to the desired topics and assign callbacks
    //ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    // may have to change name "map"
    //added latch and changed queue size from 10 to 1
    map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map",1, true);

    // For visualizing path
    path_pub = n.advertise<nav_msgs::Path>("/pathFlex",1,true);
    geometry_msgs::PoseStamped pathPose;

    // For initializing min range of laser scanner
    //#################################################################
    // Only changes it once... then it goes back to 0.45 (default) :( 
    laser_init_pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1, true);
    sensor_msgs::LaserScan laserScanner; 
    //#################################################################

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Occupancy Grid control
    nav_msgs::OccupancyGrid gridMsg;
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = "odom";  //was map


    //Set the loop rate
    ros::Rate loop_rate(2); //10 update rate --> fast enough to keep up with scans  // try 7 after

    //initOccupancyGrid(map_sub);

    ROS_INFO_STREAM("gridSize: " << gridSize);
    
    gridMsg.data.clear();
    gridMsg.header.frame_id = "odom";   // was map
    gridMsg.info.resolution = targetResolution;   // 5 cm resolution
    gridMsg.info.width = gridSize;  // 300 cells
    gridMsg.info.height = gridSize; // 300 cells
    geometry_msgs::Pose origin;
    origin.position.x = -realHeight/2;
    origin.position.y = -realHeight/2;  // was 0
    origin.position.z = 0;

    origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    gridMsg.info.origin = origin; //real world pose (x,y,theta) of cell 0,0 in mappie-boi
    gridMsg.data.resize(gridSize*gridSize);  //resize gridOccupancy

    int defaultValue = int(50);
    for (int a = 0; a < gridSize*gridSize; a++ ){
        //init occGrid to 50 SINCE it goes from 0 to 100 !!!!
        gridMsg.data[a] = int(defaultValue);
    }
    ROS_INFO_STREAM("testDONE:" << gridMsg.data[69] );

    resolutionGlobal = gridMsg.info.resolution;
    int countSanity = 0; // for debugging

    map_pub.publish(gridMsg);  //publish values for occupancy Grid message
    while (ros::ok()) {
        // debugging
        //ROS_INFO_STREAM("Msg: " << ips_x);
        //ROS_INFO_STREAM("Msg: " << ips_y);
        //ROS_INFO_STREAM("Msg: " << ips_yaw);

        //sanity check
        //ROS_INFO_STREAM("resolution: " << resolutionGlobal);


        //Main loop code goes here:

        loop_rate.sleep(); //Maintain the loop rate
        // first we update scan to get ranges parameter
        ros::spinOnce();   //Check for new messages

        laserScanner.range_min = 0.2;
        laser_init_pub.publish(laserScanner);
        //if (gotten_map == true) {
        for (unsigned int a = 0; a < ranges.size(); a++ ){
            if (!std::isnan(ranges[a])) {
                float relativeX; //relative X position of end of sensor
                float relativeY; //relative Y position of end of sensor

                convertRangeToRelativeXY(ranges[a],&relativeX, &relativeY, angle_incr,
                    a, min_angle, max_angle);
                

                // assume that turtlebot is facing its relative x-axis
                // next we convert to absolute x,y
                float absPointX = relativeX * cos(ips_yaw) - relativeY * sin(ips_yaw) + ips_x;
                float absPointY = relativeX * sin(ips_yaw) + relativeY * cos(ips_yaw) + ips_y;
                //ROS_INFO_STREAM("absX: " <<  absPointX);
                //ROS_INFO_STREAM("absY: " <<  absPointY);

                //next we map the x,y coordinates to an array --> implement algo
                //initiate empty vectors to store bresenham results in
                 //vectors of X,Y coordinates SCALED TO GRID to update with p = low except for last 3 ...
                //   See measurementModel Function
                std::vector <int> vectorX;
                std::vector <int> vectorY; 

                //check if within bounds
                if (absPointY > -realHeight/2 && absPointY < realHeight/2 && absPointX < realHeight/2 && absPointX > -realHeight/2) {
                    //conver to grid coordinates and use bresenham
                    int breX0 = (gridSize/realHeight) * ips_x;
                    int breY0 = (gridSize/realHeight) * ips_y;
                    int breX1 = (gridSize/realHeight) * absPointX;
                    int breY1 = (gridSize/realHeight) * absPointY;
                    bresenham(breX0, breY0, breX1, breY1, vectorX, vectorY);   
                    
                    int length = vectorX.size();
                    for (int i = 0; i < length; i++ ){
                        // Data is 1D ... attempt: array format = [row1Col1,row1Col2, row2Col1,row2Col2]
                        // OLD INDEX: int index = int(gridSize/2 + vectorX[i] + vectorY[i]*gridSize); 
                        int index = int(gridSize/2 + vectorX[i] + (vectorY[i] + gridSize/2) * gridSize); 
                        if (index < 0){
                            //ROS_INFO_STREAM("X: " << vectorX[i]);
                            //ROS_INFO_STREAM("Y: " << vectorY[i]);
                            //ROS_INFO_STREAM("index: " << index);

                        }
                        float minLimit = -99;
                        float maxLimit = 99;
                        bool debugOn = false;
                        if (i == 2 && debugOn){  // for debugging
                            
                             ROS_INFO_STREAM("dataValuePre: " << int(gridMsg.data[index]));
                             ROS_INFO_STREAM("measModel: " << measModel(length,i));
                             ROS_INFO_STREAM("logitMeasModel: " << logit(measModel(length,i)));
                             ROS_INFO_STREAM("logitGridValue: " << logit(int(gridMsg.data[index])/100.00));
                             ROS_INFO_STREAM("std::max " << std::max(minLimit,std::min(maxLimit,
                                (logit(int(gridMsg.data[index])/100.00) + 
                                    logit(measModel(length,i)) ) ) 
                            ));
                             ROS_INFO_STREAM("inverseModel: " << inverseLogit(
                            std::max(minLimit,std::min(maxLimit,
                                (logit(int(gridMsg.data[index])/100.00) + 
                                    logit(measModel(length,i)) ) ) 
                            )));

                        }
                        // gridMsg.data is a probability multipled by 100 !! 
                        // then convert logit back to probability and multiple by 100
                        gridMsg.data[index] = int(100.00*inverseLogit(
                            std::max(minLimit,std::min(maxLimit,
                                (logit(int(gridMsg.data[index])/100.00) + 
                                    logit(measModel(length,i)) ) ) 
                            )));

                        if (i == 2 && debugOn){  // for debugging
                            ROS_INFO_STREAM("dataValuePost: " << int(gridMsg.data[index]));

                        }
                        //used for debugging to see if anypart of grid crosses 50
                        /*if (gridMsg.data[index] > 50)
                        {
                            ROS_INFO_STREAM("Index: "<< index);
                            ROS_INFO_STREAM("Value: "<< int(gridMsg.data[index]));
                        }*/

                    }
                } else {
                    ROS_INFO_STREAM("out of range...skipping...AbsY: "<< absPointY);
                    ROS_INFO_STREAM("out of range...skipping...AbsX: "<< absPointX);
                }
            } //end if statement about nan
        }
        map_pub.publish(gridMsg);

        //  Flex Path For Lalit 
        pathPose.pose.position.x = ips_x;
        pathPose.pose.position.y = ips_y;
        //geometry_msgs::Quaternion q;
        //q.setRPY(0, 0, ips_yaw);
        //pathPose.pose.orientation = q;
        //pose_publisher.publish(pathPose);

        // create quaternion from yaw angle
        geometry_msgs::Quaternion msgQuat;
        msgQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, ips_yaw); 
        //geometry_msgs::Quaternion msgQuat; //init geometry quaternion 
        //myQuat.setRPY(0, 0, ips_yaw);
        //tf::quaternionTFToMsg(myQuat, msgQuat)
        pathPose.pose.orientation = msgQuat;

        vectorPath.push_back(pathPose);
        pathMsg.poses = vectorPath;
        path_pub.publish(pathMsg);

        countSanity++;

        // feel free to remove sanity if you want to run loop forever
        if (countSanity > 2070){
            return 0;
        }
        //vel.linear.x = 0.1;  // set linear speed
        //vel.angular.z = 0.3; // set angular speed

        //velocity_publisher.publish(vel); // Publish the command velocity
    }
    return 0;
}



// OLD garbage


// updates occupancy grid

/*
// NOTE: may have to only have laser_callback, and include map_callback as the function 
// being called by laser call back additionally....
void updateOccupancyGrid(const nav_msgs::OccupancyGrid &gridMsg){
    resolutionGlobal = msgGrid.info.resolution;
    for (unsigned int a = 0; a < sizeof(ranges)/sizeof(ranges[0]); a++ ){
        float relativeX;
        float relativeY;
        convertRangeToRelativeXY(ranges[a],&relativeX, &relativeY, angle_incr,
            a, min_angle, max_angle);

        // assume that turtlebot is facing its relative x-axis
        // next we convert to absolute x,y, assuming bot starts at 0,0
        int startingX = 0;
        int startingY = 0;
        float absPointX = relativeX * cos(ips_yaw) - relativeY * sin(ips_yaw) - ips_x;
        float absPointY = relativeX * sin(ips_yaw) + relativeY * cos(ips_yaw) - ips_y;
        
        //next we map the x,y coordinates to an array --> implement algo
        //initiate empty vectors to store bresenham results in
        std::vector <int> vectorX;
        std::vector <int> vectorY;
        bresenham(ips_x, ips_y, absPointX, absPointY,vectorX, vectorY);   
        
        //NOTE: the step size is fucked!!!

        int length = sizeof(vectorX)/sizeof(vectorX[0]);
        for (int i = 0; i < length; i++ ){
            ROS_INFO_STREAM("Bresenham Vector length: " << length);
            // Data is 1D ... attempt: array format =
            //   [row1Col1,row1Col2, row2Col1,row2Col2]
            int width = 300;
            int index = int(width/2 + vectorX[i] + vectorY[i]*width); 
            gridMsg.data[index] = msgGrid.data[index] + logit(measModel(length,i));
        }
    }
    map_pub.publish(gridMsg);
 }

// Initialize occupancy Grid
 // NOTE: this isn't working because I'm not using ROS properly
void initOccupancyGrid(const nav_msgs::OccupancyGrid &msg){
    msg.header.frame_id = "map";
    msg.info.resolution = 0.05;   // 5 cm resolution
    msg.info.width = 15/0.05;  // 300 cells
    msg.info.height = 15/0.05; // 300 cells
    msg.info.origin = (-7.5, 0, 0); //real world pose (x,y,theta) of cell 0,0 in map

    int mapData[300][300] = { 0 };  //assumes data is 2D
    msg.data = mapData;  //storing all values of array as logit of p = 0.5

    //map_pub.publish(*msg)  // pretty sure this should be publishing an OccupancyGrid
}
*/
