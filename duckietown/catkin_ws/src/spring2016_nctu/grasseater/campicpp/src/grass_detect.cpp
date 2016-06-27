#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv2/opencv.hpp>
//#include <conio.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>


#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"




using namespace std;
using namespace cv;

ros::Publisher array_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
		//Receive undistorted image
		Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

		//Resize received image 
		resize(image, image, Size(), 0.5, 0.5);


		//Control squares define when the robot has to rotate. 
		int control_w = image.cols*0.25;
		int control_h = image.rows *0.25;

		int x = image.cols*0.5;
		int y = image.rows *0.5;

		//Define control squares based on the height and widthe of the input image
		Rect r1r = Rect(x, y, control_w, control_h);
		Rect l1r = Rect(x - control_w, y, control_w, control_h);
		Rect r2r = Rect(x + control_w*0.5, y + control_h, control_w, control_h);
		Rect c2r = Rect(x - control_w*0.5, y + control_h, control_w, control_h);
		Rect l2r = Rect(x - control_w*1.5, y + control_h, control_w, control_h);

		//Transfer the image samples into the control squares 
		Mat r1(image, r1r); r1 = r1.clone();
		Mat l1(image, l1r); l1 = l1.clone();
		Mat r2(image, r2r); r2 = r2.clone();
		Mat c2(image, c2r); c2 = c2.clone();
		Mat l2(image, l2r); l2 = l2.clone();

		//Percentage_of_green is a output with percentage of grass in each control square 
		//    l1 r1
		//  l2  c2  r2       
		//  percentage_of_green[0] = l1
		//  percentage_of_green[1] = r1
		//  percentage_of_green[2] = l2
		//  percentage_of_green[3] = c2
		//  percentage_of_green[4] = r2

		double number_of_green = 0;
		int percentage_of_green[5];
		Vec3b color;

		//Counting for r1
		for (int y = 0; y < r1.rows; y++)
		{
			for (int x = 0; x<r1.cols; x++)
			{
				//Get the BGR color from the image
				color = r1.at<Vec3b>(Point(x, y));
				//Check if the green color prevails in this pixel 
				if ((color[1] >= color[0] + 10) && (color[1] >= color[2] + 10) && (color[1] > 50))
				{
					//If yes, keep only green color, other channels set to zero 
					color[0] = 0;
					color[2] = 0;
					r1.at<Vec3b>(Point(x, y)) = color;
					//Count number of green pixels in the control square
					number_of_green++;
				}
				else
				{
					//If the pixel is not green set it to black 
					color.val[0] = 0;
					color.val[1] = 0;
					color.val[2] = 0;
					r1.at<Vec3b>(Point(x, y)) = color;

				}
			}
		}
		//Get the percentage of grass (green color ) in the control square 
		percentage_of_green[1] = 100 * number_of_green / (double)(r1.rows*r1.cols);
		
		//Counting for l1
		number_of_green = 0;
		for (int y = 0; y < l1.rows; y++)
		{
			for (int x = 0; x<l1.cols; x++)
			{
				color = l1.at<Vec3b>(Point(x, y));
				if ((color[1] >= color[0] + 10) && (color[1] >= color[2] + 10) && (color[1] > 50))
				{
					color[0] = 0;
					color[2] = 0;
					l1.at<Vec3b>(Point(x, y)) = color;
					number_of_green++;
				}
				else
				{
					color.val[0] = 0;
					color.val[1] = 0;
					color.val[2] = 0;
					l1.at<Vec3b>(Point(x, y)) = color;

				}
			}
		}

		percentage_of_green[0] = 100 * number_of_green / (double)(r1.rows*r1.cols);

		//Counting for r2
		number_of_green = 0;
		for (int y = 0; y < r2.rows; y++)
		{
			for (int x = 0; x<r2.cols; x++)
			{
				color = r2.at<Vec3b>(Point(x, y));
				if ((color[1] >= color[0] + 10) && (color[1] >= color[2] + 10) && (color[1] > 50))
				{
					color[0] = 0;
					color[2] = 0;
					r2.at<Vec3b>(Point(x, y)) = color;
					number_of_green++;
				}
				else
				{
					color.val[0] = 0;
					color.val[1] = 0;
					color.val[2] = 0;
					r2.at<Vec3b>(Point(x, y)) = color;

				}
			}
		}

		percentage_of_green[4] = 100 * number_of_green / (double)(r1.rows*r1.cols);

		//Counting for l2
		number_of_green = 0;
		for (int y = 0; y < l2.rows; y++)
		{
			for (int x = 0; x<l2.cols; x++)
			{
				color = l2.at<Vec3b>(Point(x, y));
				if ((color[1] >= color[0] + 10) && (color[1] >= color[2] + 10) && (color[1] > 50))
				{
					color[0] = 0;
					color[2] = 0;
					l2.at<Vec3b>(Point(x, y)) = color;
					number_of_green++;
				}
				else
				{
					color.val[0] = 0;
					color.val[1] = 0;
					color.val[2] = 0;
					l2.at<Vec3b>(Point(x, y)) = color;

				}
			}
		}

		percentage_of_green[2] = 100 * number_of_green / (double)(r1.rows*r1.cols);

		//Counting for c2
		number_of_green = 0;
		for (int y = 0; y < c2.rows; y++)
		{
			for (int x = 0; x<c2.cols; x++)
			{
				color = c2.at<Vec3b>(Point(x, y));
				if ((color[1] >= color[0] + 10) && (color[1] >= color[2] + 10) && (color[1] > 50))
				{
					color[0] = 0;
					color[2] = 0;
					c2.at<Vec3b>(Point(x, y)) = color;
					number_of_green++;
				}
				else
				{
					color.val[0] = 0;
					color.val[1] = 0;
					color.val[2] = 0;
					c2.at<Vec3b>(Point(x, y)) = color;

				}
			}
		}
		percentage_of_green[3] = 100 * number_of_green / (double)(r1.rows*r1.cols);
	



		//Initialize array 
		std_msgs::Int32MultiArray grass_array;
		
		grass_array.data.clear();

		for (int i = 0; i < 5; i++)
		{
			//assign array grass percentage number
			grass_array.data.push_back(percentage_of_green[i]);
		}

		//Publish array
		array_pub.publish(grass_array);

		//Let the world know
		ROS_INFO("Grass array published");
	
	}



	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "grass_detect");
		ros::NodeHandle nh;


		image_transport::ImageTransport it(nh);
		image_transport::Subscriber sub = it.subscribe("/imgpreproc/undist", 1, imageCallback);

		array_pub = nh.advertise<std_msgs::Int32MultiArray>("grass_detect_array", 5);

		ros::spin();

	}
