#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

enum Filter {
	CANNY,
	FLOOD,
	THRESHOLD
};

void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage) {

	cv::Mat grayImage;
	cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);

	double threshold1 = 20;
	double threshold2 = 50;
	int apertureSize = 3;

	cv::Canny(grayImage, outputImage, threshold1, threshold2, apertureSize);
}

void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage, Filter filter) {
	switch (filter) {
		case CANNY:getCannyImage(inputImage, outputImage); break;
		//case FLOOD:getFloodFillImage(inputImage, outputImage);break;
		//case THRESHOLD:getThresholdImage(inputImage, outputImage);break;
	}
}


void processImage(const sensor_msgs::ImageConstPtr &msg) {

	cv_bridge::CvImageConstPtr imageMsgPtr = cv_bridge::toCvShare(msg, "bgr8");
	cv::Mat outputImage;
	
	callFilter(imageMsgPtr->image, outputImage, CANNY);
	
	cv::imshow("Output", outputImage);
	cv::imshow("Input", imageMsgPtr->image);
}

/*
void imageCallback(){
		
	sensor_msgs::Cvbridge bridge;
	try{
		cvShowImage("view", bridge.imgMsgToCv(msg,"bgr8"));
	}catch(){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

	}


}*/

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "Capture2");
	
	cv::namedWindow("Input");
	cvResizeWindow("Input", 320, 240);
	cv::namedWindow("Output");
	cvResizeWindow("Output", 320, 240);
	
	cvStartWindowThread();
	
	ros::NodeHandle node;
	
	image_transport::ImageTransport it(node);
	
	std::string image_topic = node.resolveName("usb_cam/image_raw");

	image_transport::Subscriber center_camera = it.subscribe(image_topic,1, &processImage);
	
	ROS_INFO("starting main loop");
	ros::spin();
	
	// handle incoming data
	ROS_INFO("exiting main loop");
	return 0;
}

