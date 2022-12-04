#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

int main(int, char**) {
    cv::VideoCapture inputVideo;
    cv::Mat image;
    
    const std::string videoStreamAddress = "http://192.168.140.2:8080/video";
    //open the video stream and make sure it's opened
    if(!inputVideo.open(videoStreamAddress)) {
        std::cerr << "Error opening video stream or file" << std::endl;
        return -1;
    }

    for(;;) {
        if(!inputVideo.read(image)) {
            std::cout << "No frame" << std::endl;
            cv::waitKey();
        }
        cv::imshow("Output Window", image);

        if(cv::waitKey(1) == 27) break;
    }   

    return 0;
}