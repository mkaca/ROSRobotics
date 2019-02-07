#include <ros/ros.h>
#include "particle_filter.h"

int main(int argc, char **argv) {
  //Initialize the ROS framework
  ros::init(argc, argv, "particle_filter");

  ParticleFilter state_estimator;
  //Set the loop rate
  ros::Rate loop_rate(20); //20Hz update rate

  while (ros::ok()) {
    ros::spinOnce();   //Check for new messages
    state_estimator.Estimate();
    loop_rate.sleep(); //Maintain the loop rate
  }
  return 0;
}