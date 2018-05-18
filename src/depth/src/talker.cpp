//ZED includes
#include <sl/Camera.hpp>
//OpenCV includes
#include <opencv2/opencv.hpp>
//Opencv gpuversion

//ROS
#include "ros/ros.h"
//stdlib
#include <iostream>
#include <cstdio>


using namespace cv;

cv::Mat slMat2cvMat(sl::Mat& input);
int main(int argc,char** argv)
{
    //Create a ZED camera object
    sl::Camera zed;

    //Set configuration parameters
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_HD1080;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;

    //Set ros initialize
    ros::init(argc,argv,"talker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    //Open the camera
    sl::ERROR_CODE zerr = zed.open(init_params);
    if(zerr != sl::SUCCESS)
    {
        std::cout<<zerr<<std::endl;
        zed.close();
        return 1;// Quit if an error occurred
    }

    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;

    // Prepare new image size to retrieve half-resolution images
    //sl::Resolution image_size = zed.getResolution();
    //int new_width = image_size.width;
    //int new_height = image_size.height;

    //sl::Mat image_zed(new_width,new_height,sl::MAT_TYPE_8U_C4);
    //Mat image_ocv = slMat2cvMat(image_zed);
    //sl::Mat depth_image_zed(new_width,new_height,sl::MAT_TYPE_8U_C4);
    //Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    //sl::Mat point_cloud;
	sl::Mat image_ocv, image_zed, depth_image_zed, point_cloud; 


    int delay=1;
	int i = 0;
    while(i<1000)
    {
        if(zed.grab(runtime_parameters) == sl::SUCCESS)
        {
			//Retrieve left image
            zed.retrieveImage(image_zed, sl::VIEW_LEFT, sl::MEM_CPU);
            //Retrieve depth map. Depth is aligned on the left image
			zed.retrieveMeasure(depth_image_zed, sl::MEASURE_DEPTH);
			//Retrieve colored point cloud. Point cloud is aligned on the left image.
			zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA);
            //image_ocv = slMat2cvMat(image_zed);
            //depth_image_ocv = slMat2cvMat(depth_image_zed);
            //imshow("Image", image_ocv);
            //imshow("Depth", depth_image_ocv);

 			// Get and print distance value in mm at the center of the image
            // We measure the distance camera - object using Euclidean distance
            int x = image_zed.getWidth() / 2;
            int y = image_zed.getHeight() / 2;
            sl::float4 point_cloud_value;
            point_cloud.getValue(x, y, &point_cloud_value);

            float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
            printf("Distance to Camera at (%d, %d): %f mm\n", x, y, distance);
        }
        int ckey = waitKey(delay);
        if(ckey == 27)
            break;
		//Increment the loop
		i++;
    }
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
