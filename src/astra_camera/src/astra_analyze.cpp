
//   define obstacle function --> outputs distance of obstacle of ___ (inputed) size

/* 
	AUTHOR: Michal Kaca
	January 16, 2019
	Arguments: None required
	Optional: v --> will turn on visualizer for filtered Point Cloud
*/

#include <ros//ros.h>
//#include <tf/transform_datatypes.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>

//fucking around with PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

// for RANSAC
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

std::vector<float> visibleX;
std::vector<float> visibleY;
std::vector<float> visibleZ;
bool firstFrame = true;
bool VISUALIZE = false;

// REGARDLESS OF RVIZ, upon horizontal movement of camera, Z points what I'm interested in
//     z = depth of camera (how far away objects are)

// Gets average of vector (mostly for debugging purposes)
float getAverageOfVector(std::vector<float> vec){
	float size = vec.size();
	float sum = 0.0;
	for (std::vector<float>::iterator it = vec.begin(); it != vec.end(); ++it)
		sum += *it;
	float avrg = sum/size;
	return avrg;
}

// Visualizer with PCL
boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}


// ROS Callback --> is called everytime the camera picks up a new frame 
void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

	// Store as relative x,z vectors for now
	std::vector<float> vectorXstate2DMap;
	std::vector<float> vectorZstate2DMap;


	pcl::PCLPointCloud2 pcl_pc2;
	pcl_conversions::toPCL(*input, pcl_pc2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ransac_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud); //converts to XYZ cloud

	// do stuff with temp cloud here 
	ROS_INFO_STREAM("sizeOG: " << temp_cloud->points.size());

	// clear my working x,y,z vectors
	visibleX.clear();
	visibleY.clear();
	visibleZ.clear();

	//populate x,y,z vectors of visible points
	for (int i = 0; i < temp_cloud->points.size(); i++){
		if (!std::isnan(temp_cloud->points[i].x)){
				visibleY.push_back(temp_cloud->points[i].y);
			visibleX.push_back(temp_cloud->points[i].x);
			visibleZ.push_back(temp_cloud->points[i].z);
		}
	}
	ROS_INFO_STREAM("sizeActual OG: " << visibleX.size());
	ROS_INFO_STREAM("dataxAvrgX OG:" << getAverageOfVector(visibleX));
	ROS_INFO_STREAM("datayAvrgY OG:" << getAverageOfVector(visibleY));
	ROS_INFO_STREAM("datazAvrgZ OG:" << getAverageOfVector(visibleZ));

	// remember: -> == (*foo).

    //init inlier vector
	std::vector <int> inliers;
	//Create RANSAC (Random Sample Consensus) object and compute model
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (
		new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (temp_cloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
	ransac.setDistanceThreshold (0.01);
	ransac.computeModel();
	ransac.getInliers(inliers);

	 // copies all inliers of the model computed to another PointCloud
	pcl::copyPointCloud<pcl::PointXYZ>(*temp_cloud, inliers, *ransac_cloud);


	// output filtered averages

	// clear my working x,y,z vectors
	visibleX.clear();
	visibleY.clear();
	visibleZ.clear();

	//populate x,y,z vectors of visible points
	for (int i = 0; i < ransac_cloud->points.size(); i++){
		if (!std::isnan(ransac_cloud->points[i].x)){
				visibleY.push_back(ransac_cloud->points[i].y);
			visibleX.push_back(ransac_cloud->points[i].x);
			visibleZ.push_back(ransac_cloud->points[i].z);
		}
	}
	ROS_INFO_STREAM("sizeActual filtered: " << visibleX.size());
	ROS_INFO_STREAM("dataxAvrgX filtered:" << getAverageOfVector(visibleX));
	ROS_INFO_STREAM("datayAvrgY filtered:" << getAverageOfVector(visibleY));
	ROS_INFO_STREAM("datazAvrgZ filtered:" << getAverageOfVector(visibleZ));

	ROS_INFO_STREAM(typeid(std::min_element(std::begin(visibleX), std::end(visibleX))).name());

	//ROS_INFO_STREAM(std::min_element(std::begin(visibleX), std::end(visibleX)));
	//ROS_INFO_STREAM(std::max_element(std::begin(visibleX), std::end(visibleX)));
	ROS_INFO_STREAM( " ");


	// TODO:
	// find min x, max x..... this gets you range
	// find min z in ransac --> this gets you distance from wall / target
	// throw into occupancy grip for visualization
	// find distance of small objects in the way (not just walls)
	//    walls are necessary for speed of robot + position estimation


    ////////////////////////////////////////////////////////

    // THIS IS THE VISUALIZER FOR MY Pointcloud

    //////////////////////////////////////////////////////
    if (VISUALIZE){
    	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		viewer = simpleVis(ransac_cloud);
		while (!viewer->wasStopped ())
		{
		  viewer->spinOnce (100);
		  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
    }

	//initialize init wall distance
	if (firstFrame){
		//dont check expectation (based on movement)
		firstFrame = 0;
	}
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"replicate_pointCloud");
    ros::NodeHandle n;

    ros::Subscriber pcld = n.subscribe("/camera/depth_registered/points", 1, cloud_cb);

    ros::Rate loop_rate(1); // popping off at 1 Hz

    // Check if PointCloud Visualizer is desired
    for (int k=0; k < argc; k++)
    	if (argv[k] == "v"){
    		VISUALIZE = true;
    		ROS_INFO_STREAM("argk:" << argv[k]);
    	}

    while (ros::ok())
    {
    	ROS_INFO_STREAM( "chck grade");

    	loop_rate.sleep(); // maintains loop rate
    	ros::spinOnce();
    	//ROS_INFO_STREAM("check out me pointcloud:" << pcl2.data[0]);
    }

    return 0;

}

/*  OLDER SENSOR MSGS CODE THAT WAS USELESS
//
float xTest;
float yTest;
float zTest;
sensor_msgs::PointCloud2 globalPCL; 

// size of pointField data is: (row_step * height)
void image_callback(const sensor_msgs::PointCloud2& astra_cloud_msg){
	globalPCL.data = astra_cloud_msg.data;
	ROS_INFO_STREAM("check out me rowstep:" << astra_cloud_msg.row_step);
	ROS_INFO_STREAM("check out me point step:" << astra_cloud_msg.point_step);
	// no idea how to access this data lemao
	ROS_INFO_STREAM("check out me pointcld:" << astra_cloud_msg.data[45]);
// 

}

// conversion function ::: not used atm
// ????
void conversion(const sensor_msgs::PointCloud2ConstPtr& input){
	
	// create container for data
	sensor_msgs::PointCloud2 output;

	//data processing here
	output = *input;


}



*/