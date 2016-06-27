#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/package.h>
#include <duckietown_msgs/BoolStamped.h>
#include <duckietown_msgs/Pose2DStamped.h>
#include <duckietown_msgs/Twist2DStamped.h>
#include <sensor_msgs/Joy.h>



int main(int argc, char **argv) {
    ros::init(argc, argv, "play_prim");

    ros::NodeHandle n;

  /*  
    ros::Publisher d_info_pub = n.advertise<sensor_msgs::CameraInfo>("scene/depth/camera_info", 1);
    ros::Publisher d_points_pub = n.advertise<sensor_msgs::PointCloud2>("scene/depth/points", 1);
    ros::Publisher rgb_info_pub = n.advertise<sensor_msgs::CameraInfo>("scene/rgb/camera_info", 1);
    ros::Publisher rgb_image_pub = n.advertise<sensor_msgs::Image>("scene/rgb/image", 1);
*/
  ros::Publisher joy_node_pub = n.advertise<sensor_msgs::Joy>("/grasseaterpi3/joy", 1);
   ros::Publisher veloc_to_pose_pub = n.advertise<duckietown_msgs::Pose2DStamped>("/grasseaterpi3/velocity_to_pose_node/pose", 1);
     ros::Publisher joy_mapper_node_pub = n.advertise<duckietown_msgs::Twist2DStamped>("/grasseaterpi3/joy_mapper_node/car_cmd", 1);
 
    ros::Publisher fwd_kinematics_pub = n.advertise<duckietown_msgs::Twist2DStamped>("/grasseaterpi3/forward_kinematics_node/velocity", 1);
 

     ros::Publisher wheels_cmd_exec_pub = n.advertise<duckietown_msgs::WheelsCmdStamped>("/grasseaterpi3/wheels_driver_node/wheels_cmd_executed", 1);
 
    ros::Publisher wheels_emerg_pub = n.advertise<duckietown_msgs::BoolStamped>("/grasseaterpi3/wheels_driver_node/emergency_stop", 1);
    ros::Publisher d_info_pub = n.advertise<duckietown_msgs::WheelsCmdStamped>("/grasseaterpi3/wheels_driver_node/wheels_cmd", 1);
 
   ros::Rate loop_rate(10);


/////// Obtain the path automatically
/*    std::string package_path = ros::package::getPath("move_primitives");
    std::string scene_path;

    int primitive_number=1;
    sstm << package_path << "/primitives/prim_" << primitive_number << ".bag";
*/  
 //   std::string primitive_path = "/home/daniel/str2.bag";
     std::string primitive_path = "/mnt/logs/straight2_2016-05-28-01-03-15_0.bag";
  

// Declare and open bag 
    rosbag::Bag bag;
    bag.open(primitive_path, rosbag::bagmode::Read);

//set topic to be played from the bag
    std::string wheels_rec_cmd = "/grasseaterpi3/wheels_driver_node/wheels_cmd";
    std::string wheels_cmd_exec = "/grasseaterpi3/wheels_driver_node/wheels_cmd_exec";
    std::string wheels_emerg = "/grasseaterpi3/wheels_driver_node/emergency_stop";
    std::string fwd_kinematics = "/grasseaterpi3/forward_kinematics_node/velocity";
    std::string veloc_to_pose = "/grasseaterpi3/velocity_to_pose_node/pose";
    std::string joy_node = "/grasseaterpi3/joy";
    std::string joy_mapper_node = "/grasseaterpi3/joy_mapper_node/car_cmd";
    std::vector<std::string> topics;
    topics.push_back(joy_node);
    topics.push_back(joy_mapper_node);
    topics.push_back(veloc_to_pose);
    topics.push_back(fwd_kinematics);
    topics.push_back(wheels_emerg);
    topics.push_back(wheels_rec_cmd);
    topics.push_back(wheels_cmd_exec);
// bool first=false;
//Play the selected topics
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
      if (m.getTopic() == joy_node) 
      {
//	first=true;
        sensor_msgs::Joy::ConstPtr joy_node_ptr = m.instantiate<sensor_msgs::Joy>();
        if (joy_node_ptr != NULL)
         {
	   joy_node_pub.publish(*joy_node_ptr);
           ROS_INFO_ONCE("joy node published");
         }  	 
      }
      if (m.getTopic() == joy_mapper_node) 
      {
//	first=false;
        duckietown_msgs::Twist2DStamped::ConstPtr joy_mapper_node_ptr = m.instantiate<duckietown_msgs::Twist2DStamped>();
        if (joy_mapper_node_ptr != NULL)
         {
	   joy_mapper_node_pub.publish(*joy_mapper_node_ptr);
           ROS_INFO_ONCE("joy mapper published");
         }  	 
      }
        if (m.getTopic() == wheels_cmd_exec) 
      {
        duckietown_msgs::WheelsCmdStamped::ConstPtr wceptr = m.instantiate<duckietown_msgs::WheelsCmdStamped>();
        if (wceptr != NULL)
         {
	   wheels_cmd_exec_pub.publish(*wceptr);
           ROS_INFO_ONCE("wheel command exec  published");
         }  	 
      }
      if (m.getTopic() == wheels_emerg) 
      {
        duckietown_msgs::BoolStamped::ConstPtr wemptr = m.instantiate<duckietown_msgs::BoolStamped>();
        if (wemptr != NULL)
         {
	   wheels_emerg_pub.publish(*wemptr);
           ROS_INFO_ONCE("wheel emerg published");
         }  	 
      }
      if (m.getTopic() == fwd_kinematics) 
      {
        duckietown_msgs::Twist2DStamped::ConstPtr fwdkinemptr = m.instantiate<duckietown_msgs::Twist2DStamped>();
        if (fwdkinemptr != NULL)
         {
	   fwd_kinematics_pub.publish(*fwdkinemptr);
           ROS_INFO_ONCE("forward kinematics published");
         }  	 
      }
  
     if (m.getTopic() == veloc_to_pose) 
      {
        duckietown_msgs::Pose2DStamped::ConstPtr veloc2poseptr = m.instantiate<duckietown_msgs::Pose2DStamped>();
        if (veloc2poseptr != NULL)
         {
	   veloc_to_pose_pub.publish(*veloc2poseptr);
           ROS_INFO_ONCE("wheel command published");
         }  	 
      }
  




    }
/*
      if (m.getTopic() == depth_points) 
      {
        sensor_msgs::PointCloud2::ConstPtr d_points_ptr = m.instantiate<sensor_msgs::PointCloud2>();
        if (d_points_ptr != NULL)
          d_points_pub.publish(*d_points_ptr);
      }

      if (m.getTopic() == rgb_cam_info) 
      {
        sensor_msgs::CameraInfo::ConstPtr rgb_info_ptr = m.instantiate<sensor_msgs::CameraInfo>();
        if (rgb_info_ptr != NULL)
          rgb_info_pub.publish(*rgb_info_ptr);
      }

      if (m.getTopic() == rgb_cam_image) 
      {
        sensor_msgs::Image::ConstPtr rgb_image_ptr = m.instantiate<sensor_msgs::Image>();
        if (rgb_image_ptr != NULL)
          rgb_image_pub.publish(*rgb_image_ptr);
      }

    }
*/
  bag.close();
  ros::spinOnce();
  
// loop_rate.sleep();
}
