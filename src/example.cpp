#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// Note: for colorized point cloud, use PointXYZRGBA
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher pointcloud_pub;

void callback(const ImageConstPtr& image_color_msg,
		const ImageConstPtr& image_depth_msg,
		const CameraInfoConstPtr& info_color_msg,
		const CameraInfoConstPtr& info_depth_msg) {
	// Solve all of perception here...
	cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
	cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;

	cvtColor(image_color,image_color, CV_RGB2BGR);

	// colorize the image
	cv::Vec3b color(255,0,0);
	for(int y=0;y<image_color.rows;y++) {
		for(int x=0;x<image_color.cols;x++) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;

			if( depth == 0) {
				image_color.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);
			}
			if( depth > 1.00) {
				image_color.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
			}
		}
	}

	//cout <<"depth[320,240]="<< image_depth.at<short int>(cv::Point(320,240)) / 1000.0 <<"m"<< endl;
	image_color.at<cv::Vec3b>(cv::Point(320,240)) = cv::Vec3b(255,255,255);

	// get camera intrinsics
	float fx = info_depth_msg->K[0];
	float fy = info_depth_msg->K[4];
	float cx = info_depth_msg->K[2];
	float cy = info_depth_msg->K[5];
	//cout << "fx="<< fx << " fy="<<fy<<" cx="<<cx<<" cy="<<cy<<endl;

	// produce a point cloud
	PointCloud::Ptr pointcloud_msg (new PointCloud);
	pointcloud_msg->header = image_depth_msg->header;

	pcl::PointXYZ pt;
	for(int y=0;y<image_color.rows;y+=4) {
		for(int x=0;x<image_color.cols;x+=4) {
			float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;

			if(depth>0) {
				pt.x = (x - cx) * depth / fx;
				pt.y = (y - cy) * depth / fy;
				pt.z = depth;
				//cout << pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
				pointcloud_msg->points.push_back (pt);
			}
		}
	}
	pointcloud_msg->height = 1;
	pointcloud_msg->width = pointcloud_msg->points.size();
	pointcloud_pub.publish (pointcloud_msg);

	cv::imshow("color", image_color);

	cv::waitKey(3);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;

	message_filters::Subscriber<Image> image_color_sub(nh,"/camera/rgb/image_raw", 1);
	message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/depth_registered/image_raw", 1);
	message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/camera/rgb/camera_info", 1);
	message_filters::Subscriber<CameraInfo> info_depth_sub(nh,"/camera/depth_registered/camera_info", 1);
	pointcloud_pub = nh.advertise<PointCloud> ("mypoints", 1);

	typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, image_depth_sub, info_color_sub, info_depth_sub);

	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

	ros::spin();

	return 0;
}
