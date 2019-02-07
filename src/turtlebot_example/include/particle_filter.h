#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues> // header file
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <stdlib.h>
#include "tf/transform_datatypes.h"
#include <tf2/LinearMath/Quaternion.h>
#include <queue>

typedef struct {
  double v[3];
  double weight;
} particle_t;

// Description for a single map cell.
typedef struct {
  // Occupancy state (true = unknown or occupied, false = empty)
  bool occupied;
} cells_t;

// Description for a map
typedef struct {
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  // Map scale (m/cell)
  double scale;
  // Map dimensions (number of cells)
  int size_x, size_y;
  // The map data, stored as a grid
  cells_t *cells;
} map_t;

class ParticleFilter {
  private:
    ros::Publisher pf_cloud_publisher_;
    ros::Publisher pf_pose_publisher_;
    ros::Publisher path_publisher_;
    ros::Subscriber odom_sub_;
    ros::Subscriber command_sub_;
    ros::Subscriber sim_ips_sub_;
    ros::Subscriber ips_sub_;
    ros::Subscriber map_sub_;

    nav_msgs::Path robot_path_;
    double ips_x_;
    double ips_y_;
    double ips_yaw_;
    bool map_received_;
    map_t map_;
    std::queue<std::array<double, 3> > pose_deltas_;
    double last_command_time_;
    double last_measurement_time_;
    bool measurement_ready_;
    std::vector<particle_t> particles_;
    geometry_msgs::PoseArray particles_msg_;
    int num_of_particles_;
    Eigen::Vector3d measurements_, measurements_raw_;
    double dt_;
    std::default_random_engine generator_;
    double total_[3];
    double count_;

  public:
    ros::NodeHandle nh_;
    ParticleFilter(); 
    void ConvertMap(const nav_msgs::OccupancyGrid &map_msg);
    void UniformPoseGenerator();
    bool PropagateParticles();
    bool WeighParticles();
    void ResampleParticles();

    void CommandCb(const geometry_msgs::Twist::ConstPtr &msg);
    void MapCb(const nav_msgs::OccupancyGrid &msg);
    void PoseCb(const geometry_msgs::PoseWithCovarianceStamped &msg);
    void SimIPSCb(const gazebo_msgs::ModelStates::ConstPtr &msg);
    void OdomCb(const nav_msgs::Odometry::ConstPtr& msg);

    void PublishPoseArray();
    void PublishPosePath();
    void Estimate();
};
#endif