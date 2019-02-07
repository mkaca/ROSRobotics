#include "particle_filter.h"

// Mapping helper functions.
inline int GetMapIndex(map_t map, int x, int y) {
  return ((x) + (y) * map.size_x);
}

double GetProb(const Eigen::Vector3d& x, const Eigen::Vector3d& mean, 
               const Eigen::Matrix3d& covar) {
  // Multivariate Gaussian distribution
  using namespace Eigen;
  //std::cout << x << std::endl <<  mean << std::endl << covar << std::endl;
  VectorXd quadform = (x - mean).transpose() * covar.inverse() * (x - mean);
  double quad = quadform(0);
  return std::exp(-0.5 * quad);
}

void ParticleFilter::ConvertMap(const nav_msgs::OccupancyGrid &map_msg) {
  map_.size_x = map_msg.info.width;
  map_.size_y = map_msg.info.height;
  map_.scale = map_msg.info.resolution;
  map_.origin_x = map_msg.info.origin.position.x
                  + (map_.size_x / 2) * map_.scale;
  map_.origin_y = map_msg.info.origin.position.y
                  + (map_.size_y / 2) * map_.scale;

  // Convert to player format
  map_.cells = (cells_t*)malloc(sizeof(cells_t)*map_.size_x*map_.size_y);
  for(int i=0;i<map_.size_x * map_.size_y;i++) {
    if(map_msg.data[i] == 0)
      map_.cells[i].occupied = false;
    else
      map_.cells[i].occupied = true;
  }
}

// Generates random pose guesses uniformly across map
void ParticleFilter::UniformPoseGenerator() {
  double min_x, max_x, min_y, max_y;

  min_x = -8;//(map_.size_x * map_.scale)/2.0 + map_.origin_x;
  max_x = 8;//(map_.size_x * map_.scale)/2.0 + map_.origin_x;
  
  min_y = -8;//(map_.size_y * map_.scale)/2.0 + map_.origin_y;
  max_y = 8;//(map_.size_y * map_.scale)/2.0 + map_.origin_y;

  particle_t p;
  p.weight = 1.0/(float)num_of_particles_;
  int n = 0;
  ROS_INFO("Generating new uniform particles");
  while(n < num_of_particles_) {
    p.v[0] = min_x + ((double)rand() / (double)RAND_MAX) * (max_x - min_x);
    p.v[1] = min_y + ((double)rand() / (double)RAND_MAX) * (max_y - min_y);
    p.v[2] = ((double)rand() / (double)RAND_MAX) * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int x_pos,y_pos;
    //x_pos = (floor((p.v[0] - map_.origin_x) / map_.scale + 0.5) + map_.size_x / 2);
    //y_pos = (floor((p.v[1] - map_.origin_y) / map_.scale + 0.5) + map_.size_y / 2);
    /*if(map_.cells[GetMapIndex(map_, x_pos,y_pos)].occupied == false)*/ {
      particles_.push_back(p);
      n++;
    }
  }
  ROS_INFO("Generated new uniform particles");
}

void ParticleFilter::PublishPoseArray() {
    for (int i = 0; i < num_of_particles_; i++) {
        particles_msg_.poses[i].position.x = particles_[i].v[0];
        particles_msg_.poses[i].position.y = particles_[i].v[1];
        particles_msg_.poses[i].position.z = 0;  
        tf2::Quaternion q;
        q.setRPY(0, 0, particles_[i].v[2]);
        particles_msg_.poses[i].orientation.w = q.getW();
        particles_msg_.poses[i].orientation.x = q.getX();
        particles_msg_.poses[i].orientation.y = q.getY();
        particles_msg_.poses[i].orientation.z = q.getZ();
        // ROS_INFO("particles: %lf %lf %lf ",poses[i].v[0], poses[i].v[1], poses[i].v[2]);
    }
    pf_cloud_publisher_.publish(particles_msg_);
}

bool ParticleFilter::PropagateParticles() {
  std::normal_distribution<double> x_noise(0, 2);
  std::normal_distribution<double> theta_noise(0, 1);
  double pose_delta[3] = {};

  if (pose_deltas_.empty()) {
    return false;
  }

  while (!pose_deltas_.empty()) {
    pose_delta[0] = pose_deltas_.front()[0];
    pose_delta[1] = pose_deltas_.front()[1];
    pose_delta[2] = pose_deltas_.front()[2];
    pose_deltas_.pop();
    for (int i = 0; i < particles_.size(); i++) {
      double v  = pose_delta[0] + x_noise(generator_) * dt_;
      particles_[i].v[2] += pose_delta[2] + theta_noise(generator_) * dt_;
      particles_[i].v[0] += v * cos(particles_[i].v[2]);
      particles_[i].v[1] += v * sin(particles_[i].v[2]);
      if (particles_[i].v[2] > M_PI) {
        particles_[i].v[2] -= 2*M_PI;
      }
      else if (particles_[i].v[2] < -M_PI) {
        particles_[i].v[2] += 2*M_PI;
      }
    }
  }
  return true;
}

void ParticleFilter::PublishPosePath() {
  geometry_msgs::PoseStamped average;
  average.header.frame_id = "/odom";
  std::vector<double> angles;
  double yaw = 0;

  for (int i = 0; i < particles_.size(); i++) {
    average.pose.position.x += particles_[i].v[0];
    average.pose.position.y += particles_[i].v[1];
    angles.push_back(particles_[i].v[2]);
  }
  average.pose.position.x /= num_of_particles_;
  average.pose.position.y /= num_of_particles_;

  if(angles.size() > 2) {
    std::sort(angles.begin(), angles.end());
    tf2::Quaternion q;
    yaw = angles[angles.size()/2];
    q.setRPY(0, 0, angles[angles.size()/2]);
    average.pose.orientation.w = q.getW();
    average.pose.orientation.x = q.getX();
    average.pose.orientation.y = q.getY();
    average.pose.orientation.z = q.getZ();
  }
  total_[0] += pow(fabs(measurements_raw_(0) - average.pose.position.x), 2);
  total_[1] += pow(abs(measurements_raw_(1) - average.pose.position.y), 2);
  total_[2] += pow(fabs(measurements_raw_(2) - yaw), 2);
  count_++;
  ROS_INFO("average error: x= %lf y=%lf yaw=%lf", total_[0]/count_, total_[1]/count_, total_[2]/count_);
  robot_path_.poses.push_back(average);
  pf_pose_publisher_.publish(average);
  path_publisher_.publish(robot_path_);
}

bool ParticleFilter::WeighParticles() {
  double total  = 0.0;
  double weight_total  = 0.0;

  if (fabs(measurements_(2)) <= 0.00001) {
    ROS_INFO("Jumped ship");
    return false;
  }
  if (measurement_ready_ == false) {
    return false;
  }
  Eigen::Matrix3d cov;
  cov << 0.1, 0, 0,
          0, 0.1, 0,
          0, 0, 0.1;
  for (int i = 0; i < particles_.size(); i++) {
    Eigen::Vector3d mean;
    mean << particles_[i].v[0], particles_[i].v[1], particles_[i].v[2];
    // ROS_INFO("WEIGHED particles: %lf %lf %lf ",particles[i].v[0], 
    //          particles[i].v[1], particles[i].v[2]);
    //ROS_INFO("before: %lf %lf ", measurements_(2), mean(2));
    /*
    double ang_diff = measurements_(2) - mean(2);
    mean(2) = std::min(fabs(ang_diff), 2*M_PI -fabs(ang_diff));
    if (ang_diff < 0) {
      mean(2) *= -1;
    }
    measurements_(2) = 0;
    */
    particles_[i].weight = GetProb(measurements_, mean, cov) * 1000000;
    //ROS_INFO("mean: %lf %lf %lf ", mean(0), mean(1), mean(2));
    //ROS_INFO("mesu: %lf %lf %lf ", measurements_(0), measurements_(1),
    //         measurements_(2));
    total += particles_[i].weight;
  }

  for (int i = 0; i < particles_.size(); i++) {
    particles_[i].weight /= total;
    weight_total += particles_[i].weight;
    //ROS_INFO("Weight of particles = %.17g", particles_[i].weight);
  }
  //ROS_INFO("total: %f", weight_total);
  measurement_ready_ = false;
  return true;
}

void ParticleFilter::ResampleParticles() {
  double r;
  double count_inv;
  double c[num_of_particles_] = {};
  std::vector<particle_t> resampled_poses;
  resampled_poses.clear();
  particle_t single;
  ROS_INFO("resample");

  c[0] = particles_[0].weight;
  for(int i=1; i < num_of_particles_; i++) {
    c[i] = c[i-1] + particles_[i].weight;
  }
  for(int i = 0; i < num_of_particles_; i++) {
    r = rand() / double(RAND_MAX);

    for (int j = 0; j < num_of_particles_; j++) {
      if (r <= c[j]) {
        single.v[0] = particles_[j].v[0];
        single.v[1] = particles_[j].v[1];
        single.v[2] = particles_[j].v[2];
        single.weight = 1.0 / (double)num_of_particles_;
        resampled_poses.push_back(single);
        //ROS_INFO("select %lf %lf", r, );d
        break;
      }
    }
  }

  particles_.clear();
  for(int l = 0; l < num_of_particles_; l++)
    particles_.push_back(resampled_poses[l]);
}

void ParticleFilter::SimIPSCb(const gazebo_msgs::ModelStates::ConstPtr &msg) {
  double now = ros::Time::now().toSec();
  double yaw = tf::getYaw(msg->pose[8].orientation); // Robot Yaw
  measurements_raw_ << msg->pose[8].position.x, msg->pose[8].position.y, yaw;

  if (last_measurement_time_ == -1 || now - last_measurement_time_ > 1) {
    std::normal_distribution<double> x_noise(0, 0.1);
    std::normal_distribution<double> theta_noise(0, 0.01);
    //TODO find which model has the right index for the lookup below
    measurements_ << msg->pose[8].position.x + x_noise(generator_), 
    msg->pose[8].position.y + x_noise(generator_), yaw + theta_noise(generator_);

    //ROS_INFO("%s %f %f %f", msg->name[8].c_str(), msg->pose[8].position.x, 
    //          msg->pose[8].position.y, yaw);
    //ROS_INFO("%lf %lf %lf ", measurements_(0), measurements_(1), measurements_(2));
    last_measurement_time_ = now;
    measurement_ready_ = true;
  }
}

//Callback for the command data
void ParticleFilter::CommandCb(const geometry_msgs::Twist::ConstPtr &msg) {
  double now = ros::Time::now().toSec();
  dt_ = now - last_command_time_;
  std::array<double,3> pose_delta;

  if (last_command_time_ == -1){
    last_command_time_ = now;
    return;
  }

  if (map_received_) {
    pose_delta[0] = msg->linear.x * dt_;
    pose_delta[1] = msg->linear.y * dt_;
    pose_delta[2] = msg->angular.z * dt_;
    pose_deltas_.push(pose_delta);
  }
  last_command_time_ = now;
}

void ParticleFilter::OdomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  double now = ros::Time::now().toSec();
  dt_ = now - last_command_time_;
  std::array<double,3> pose_delta;

  if (last_command_time_ == -1){
    last_command_time_ = now;
    return;
  }

  if (map_received_) {
    pose_delta[0] = msg->twist.twist.linear.x * dt_;
    pose_delta[1] = msg->twist.twist.linear.y * dt_;
    pose_delta[2] = msg->twist.twist.angular.z * dt_;
    pose_deltas_.push(pose_delta);
  }
  last_command_time_ = now;
  ROS_INFO("odom");
}

//Callback function for the Position topic (LIVE)
void ParticleFilter::PoseCb(
  const geometry_msgs::PoseWithCovarianceStamped &msg) {
  measurements_ << msg.pose.pose.position.x, msg.pose.pose.position.y,
                   tf::getYaw(msg.pose.pose.orientation);
  measurements_raw_ << msg.pose.pose.position.x, msg.pose.pose.position.y,
                   tf::getYaw(msg.pose.pose.orientation);
  ROS_INFO("pose");
  measurement_ready_ = true;
  //ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", measurements_(0), 
  //          measurements_(1), measurements_(2));
}

//Callback function for the map
void ParticleFilter::MapCb(const nav_msgs::OccupancyGrid &msg) {
  //This function is called when a new map is received
  //ConvertMap(msg);
  UniformPoseGenerator();
  particles_msg_.header.frame_id = "/odom";
  particles_msg_.poses.resize(num_of_particles_);
  ROS_INFO("Made the particles!");
  PublishPoseArray();
  map_received_ = true;
}

ParticleFilter::ParticleFilter () {
  //nh_.param<int>("no_of_particles", num_of_particles_, 20);
  //if (!nh_.hasParam("no_of_particles")) {
  //  ROS_INFO("no_of_particles is not defined.");
  //}
  num_of_particles_ = 3000;
  //Subscribe to the desired topics and assign callbacks
  odom_sub_ = nh_.subscribe("/odom", 1, &ParticleFilter::OdomCb, this);
  //command_sub_ = nh_.subscribe("/mobile_base/commands/velocity", 1, 
                                            //&ParticleFilter::CommandCb, this);
  //sim_ips_sub_ = nh_.subscribe("/gazebo/model_states", 1, 
                                               //&ParticleFilter::SimIPSCb, this);
  ips_sub_ = nh_.subscribe("/indoor_pos", 1, &ParticleFilter::PoseCb, this);
  map_sub_ = nh_.subscribe("/map", 1, &ParticleFilter::MapCb, 
                                           this);
  //Setup topics to Publish from this node
  pf_cloud_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("/pf_cloud", 1,
                                                                true);
  pf_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/pf_pose", 1, 
                                                                 true);
  path_publisher_ = nh_.advertise<nav_msgs::Path>("/pf_path", 1, true);
  robot_path_.header.frame_id = "/odom";
  map_received_ = true;
  last_command_time_ = -1;
  last_measurement_time_ = -1;
  measurement_ready_ = false;
  total_[0] = 0;
  total_[1] = 0;
  total_[2] = 0;
  count_ = 0;
  UniformPoseGenerator();
  particles_msg_.header.frame_id = "/odom";
  particles_msg_.poses.resize(num_of_particles_);
  ROS_INFO("Made the particles!");
  PublishPoseArray();
}

void ParticleFilter::Estimate() {
  /*
  Load map
  Generate Particles
  Propagate the particles using the commanded velocity.
  Weight each of the particles
  Normalize the weights
  Resample using a process called stochastic universal sampling(Low variance resampling)
  */
  if (map_received_) {
    if (PropagateParticles()) {
      if (WeighParticles()) {
        ResampleParticles();
        PublishPosePath();
      }
    }
    PublishPoseArray();
  }
}
