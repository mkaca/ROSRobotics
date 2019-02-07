//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: Lalit Lal
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <eigen3/Eigen/Core>

//using namespace Eigen;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

//IPS Data
double ips_x;
double ips_y;
double ips_yaw;

//Laser Data
float angle_min;        // start angle of the scan [rad]
float angle_max;        // end angle of the scan [rad]
float angle_increment;  // angular distance between measurements [rad]
float scan_time; //time between scans (we can check for this to modify the rate of updates)
float range_min;
float range_max;

//laser rangedata:
std::vector <float> r;

//laser constants
//float alpha = 1; 
//float beta = 0.05; 

//Map Properties
static const int RES_X = 10; //10m x direction map
static const int RES_Y = 10; //10m y direction map
static const float cell_size_in_m = 0.01; //10 cm per cell
static float cell_not_occ = 0.4;
static float cell_occ = 0.6;

static const int M = RES_X/cell_size_in_m;
static const int N = RES_Y/cell_size_in_m;

double m[M][N] = {50.0};
double L0[M][N] = {0};

/*for(int i = 0; i < M; i++)
{
    for(int j = 0; j < N; j++)
    {
        L0[i][j] = log((m[i][j])/(1-(m[i][j])));
    }
}*/

double L[M][N] = {0}; 

//store position of robot
std::vector <double> robot_pose_x;
std::vector <double> robot_pose_y;
std::vector <double> robot_pose_yaw;

std::vector<int8_t> final_array;

//vector iterator


short sgn(int x) { return x >= 0 ? 1 : -1; }


//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//    vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

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
    }
}

void inverse_meas_bres(float theta, float r,
 std::vector <int> &vector_x, std::vector <int> &vector_y, std::vector <float> &vector_prob)
{
    //Steps: 
    // Get Range, bearing
    // convert to global coordinates
    //Run occupancy grid bayes filter
    if(std::isnan(r)) return; 
    int x0 = 0;
    int y0 = 0; 
    int x1 = 0; 
    int y1 = 0;

    //int r_max = 40/cell_size_in_m;
    //int r_min = 0/cell_size_in_m;
    r = r/cell_size_in_m;



    x0 = ips_x/cell_size_in_m + (M/2); 
    y0 = ips_y/cell_size_in_m + (N/2); 

    if(x0 > M-1 || x0 < 0)
    {
        //ROS_INFO("X out of bounds: %d", x0);
        return;
    }
    if(y0 > N-1 || y0 < 0)
    {
        //ROS_INFO("Y out of bounds: %d", y0);
        return; 
    }   

    if(r > (range_max/cell_size_in_m) || (r < range_min/cell_size_in_m))
    {
        //ROS_INFO("Range out of bounds: %f", r);
        return; 
    }

    //ROS_INFO("x y: %f %f ", x, y);
    //ROS_INFO("dist: %f max = %f ", r, r_max);

    /*if(x < 0)
    {
        int x0 = abs(std::round(x)) > (M/2) ? 0 : std::round(x) + M/2;
    }
    else
    {
        int x0 = std::round(x) < (M/2) ? std::round(x) : (M/2); 
    }*/

    //int x0 = std::round(x) < (M-1) ? std::round(x) : (M-1); //max(1,min(M,round(x)));
    //x0 = x0 > 1 ? x0 : 1;

    /*if(y < 0)
    {
        int y0 = abs(std::round(y)) > (N/2) ? 0 : std::round(x) + N/2;
    }
    else
    {
        int y0 = std::round(y) < (N/2) ? std::round(y) : (N/2)-1; 
    }*/  
    //int y0 = abs(std::round(x)) < (N-1) ? std::round(y) : (N-1); //max(1,min(M,round(x)));
    //y0 = y0 > 1? y0 : 1;

    float rel_x = r * cos(theta); 
    float rel_y = r * sin(theta); 

    float endpt_x = (rel_x * cos(ips_yaw)) - (rel_y * sin(ips_yaw)) + x0;
    float endpt_y = (rel_x * sin(ips_yaw)) + (rel_y * cos(ips_yaw)) + y0;

    x1 = endpt_x;// + M/2;
    y1 = endpt_y;// + N/2;

    if(x1 > M-1 || x1 < 0) 
    {
        ROS_INFO("x1 out of bounds: %d", x1);
        return;
    }
    if(y1 > N-1 || y1 < 0) 
    {
        ROS_INFO("y1 out of bounds: %d", y1);
        return;
    }   
  

    /*if(endpt_x < 0)
    {
        int x1 = abs(std::round(endpt_x)) > (M/2) ? 0 : std::round(endpt_x) + M/2;
    }
    else
    {
        int x1 = std::round(endpt_x) < (M/2) ? std::round(endpt_x) : (M/2)-1; 
    }

    if(endpt_y < 0)
    {
        int y1 = abs(std::round(endpt_y)) > (N/2) ? 0 : std::round(endpt_y) + N/2;
    }
    else
    {
        int y1 = std::round(endpt_y) < (N/2) ? std::round(endpt_y) : (N/2)-1; 
    }*/
    //int x1 = std::round(endpt_x) < (M-1) ? std::round(endpt_x) : (M-1); //max(1,min(M,round(endpt_x)))
    //x1 = x1 > 1 ? x1 : 1; 

    //int y1 = std::round(endpt_y) < (N-1) ? std::round(endpt_y) : (N-1); //max(1,min(N,round(endpt_y)))
    //y1 = y1 > 1 ? y1 : 1;


    //ROS_INFO("x0 y0: %d %d ", x0, y0);
    //ROS_INFO("x1 y1: %d %d", x1, y1); 

    bresenham(x0, y0, x1, y1, vector_x, vector_y);
    //ROS_INFO("VALID DATA"); 

    for(std::vector<int>::size_type i = 0; i < vector_x.size(); i++)
    {
        if(i == (vector_x.size() - 1) && r > (range_min/cell_size_in_m) && r < (range_max/cell_size_in_m))
        {
            //ROS_INFO("Occupied Cell!"); 
            vector_prob.push_back(cell_occ);   
        }
        else
        {
            vector_prob.push_back(cell_not_occ); 
        }
    }   

}
void mapping()
{
    ROS_INFO("IN MAPPING");
    //GO THROUGH EACH CELL
    // CHECK IF CELL's (thorugh relative measurements) are in bearing field
    int measurement_size = (angle_max - angle_min)/angle_increment;
    float measL[M][N] = {0};

    std::vector <int> vector_x;
    std::vector <int> vector_y;
    std::vector <float> vector_prob;
    // find which cells to update

    for(int i = 0; i < measurement_size; i++)
    {
        vector_x.clear();
        vector_y.clear();
        vector_prob.clear(); 
        //ROS_INFO("GOT HERE 1");
        float theta = i*angle_increment;
        if(!std::isnan(r[i]))
        {
            inverse_meas_bres(theta,r[i], vector_x, vector_y, vector_prob);  // returns points AND their probabilities (0-1)
            //ROS_INFO("Measurement size: %ld %ld %ld", vector_x.size(), vector_y.size(), vector_prob.size());
            int start = vector_x.size() > 70 ? vector_x.size()-60 : 0; 
            for(int j = start; j < vector_x.size(); j++)
            {
                int ix = vector_x[j];
                int iy = vector_y[j]; 
                float il = vector_prob[j];
                //ROS_INFO("GOT HERE 3"); 

               // ROS_INFO("Occupied?: %f", il); 

                //update log odds
                L[ix][iy] = L[ix][iy] + log(il/(1-il)) - L0[ix][iy];
                double new_prob = exp(L[ix][iy])/(1 + exp(L[ix][iy]))*100;

                //ROS_INFO("New L: %f New Prob: %f", L[ix][iy], new_prob); 

                //measL[ix][iy] = measL[ix][iy] + log(il/(1-il)) - L0[ix][iy];
                m[ix][iy] = new_prob;
                if(new_prob > 100) new_prob = 100;
                if(new_prob < 0) new_prob = 0; 

                int insert_index = (ix * N) + iy;
                final_array[insert_index] = new_prob; 
                //ROS_INFO("Assigned Prob: %f", m[ix][iy]);
                //ROS_INFO("x: %d y: %d prob: %f", ix, iy, m[ix][iy]);
            }
 
        }
        
    }

    //calculate probabilities
    /*for(int i = 0; i < M; i++)
    {
        for(int j = 0; j < N; j++)
        {
            
            m[i][j] = 1 - 1/(1 + exp(L[i][j]));
            //ROS_INFO("Calculated probability: %f", m[i][j]);

        }
    }*/
    //m = (L.array().exp()).cwiseQuotient(1+L.array().exp());
    //invmod_T = (measL.array().exp()).cwiseQuotient(1+measL.array().exp());

    //clear data from previous inputs
    vector_x.clear();
    vector_y.clear();
    vector_prob.clear();
    r.clear(); 
    //ROS_INFO("GOT HERE 3");

          
}

void init_final_occupancy_grid(nav_msgs::OccupancyGrid &msg)
{
    msg.header.frame_id = "odom";
    msg.info.resolution  = cell_size_in_m; // each cell is 0.1m [m/cell]
    msg.info.width = M; // in cells
    msg.info.height = N; // in cells
    geometry_msgs::Pose pose;
    pose.position.x = -RES_X/2;
    pose.position.y = -RES_Y/2;
    pose.position.z = 0;
    msg.info.origin = pose; // [m, m, rad] 
}

void update_final_grid(nav_msgs::OccupancyGrid &msg)
{

    msg.data = final_array;
    //final_array.clear(); 

    //r.clear(); 
}

void laser_callback(const sensor_msgs::LaserScan &msg)
{
    size_t RANGE_SIZE = sizeof(msg.ranges)/sizeof(msg.ranges[0]); 

    angle_min = msg.angle_min; 
    angle_max = msg.angle_max; 
    angle_increment = msg.angle_increment;
    range_max = msg.range_max;
    range_min = msg.range_min; 
    //ROS_INFO("increment: %f", angle_increment);

    for(std::vector<int>::size_type i = 0; i < RANGE_SIZE; i++)
    {
        //ROS_INFO("Getting range: %f", msg.ranges[i]);
        //ROS_INFO("min r: %f, max r: %f", range_min, range_max); 
        r.push_back(msg.ranges[i]); 
    }
    

    //ROS_INFO("Range size: %ld\n", RANGE_SIZE);
    //ROS_INFO("Min Angle: %f\n", angle_min); 
    //ROS_INFO("Max Angle: %f\n", angle_max);
    ROS_INFO("TEST GOING TO MAPPING");
    if(RANGE_SIZE != 0)
    {
        mapping(); 
    }
    //mapping();

}

//Callback function for the Position topic (SIMULATION)
/*void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x; //cell_size_in_m;
    ips_y = msg.pose[i].position.y; //cell_size_in_m;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    //ROS_INFO("X: %lf\n", ips_x);
    //ROS_INFO("Y: %lf\n", ips_y); 
    //ROS_INFO("Theta: %lf\n", ips_yaw); 

    //store robot poses for plotting path
    //robot_pose_x.push_back(ips_x); 
    //robot_pose_y.push_back(ips_y);
    //robot_pose_yaw.push_back(ips_yaw); 
}*/

//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{

    ips_x = msg.pose.pose.position.x; // Robot X psotition
    ips_y = msg.pose.pose.position.y; // Robot Y psotition
    ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
    ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with

}

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    //ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    //ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber laser_sub = n.subscribe("/scan", 1, laser_callback); 

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    //pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    //marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true); 

    //Velocity control variable
    geometry_msgs::Twist vel;
    nav_msgs::OccupancyGrid occ_grid;
    init_final_occupancy_grid(occ_grid);
    //m = {50}; 

    //Set the loop rate
    ros::Rate loop_rate(1); //20Hz update rate
    for(int a = 0; a < M; a++)
    {
        for(int b = 0; b < N; b++)
        {
            final_array.push_back(-1); 
        }
    }

    while (ros::ok()) {

        //ROS_INFO("SENDING DATA");
        //Main loop code goes here:
        //vel.linear.x = 0.1;  // set linear speed
        //vel.angular.z = 0.3; // set angular speed
        //ROS_INFO("final Index value %f", m[85][128]); 


        //mapping(); // this is called by laser callback
        // update map
        update_final_grid(occ_grid); // generate occupancy grid message
        //ROS_INFO("We HERE");

        //velocity_publisher.publish(vel); // Publish the command velocity
        map_publisher.publish(occ_grid);  //publish grid to /map topic
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    }

    return 0;
}
