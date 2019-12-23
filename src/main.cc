#include<iostream>
#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<SystemHandler.h>
using namespace std; 

void LoadImages(const string &PathToSequence, vector<cv::String> &LeftStringVector,
                vector<cv::String> &RightStringVector);

int main(int argc, char** argv)
{
    if(argc < 3) 
    {
        cerr << endl << "Usage: ./MY_SLAM path_to_squence path_to_VOC path_to_settings" << endl; 
        return 1;
    }

    std::vector<cv::String> LeftStringVector;
    std::vector<cv::String> RightStringVector;
    std::string PathToSequence = string(argv[1]);
    LoadImages(PathToSequence, LeftStringVector, RightStringVector);
    MY_SLAM::SystemHandler SLAM = MY_SLAM::SystemHandler(string(argv[2]), string(argv[3]));
    for(int i=0; i<LeftStringVector.size(); i++)
    {
        cv::Mat imLeft = cv::imread(LeftStringVector[i], CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat imRight = cv::imread(RightStringVector[i], CV_LOAD_IMAGE_UNCHANGED);
    }
}

void LoadImages(const string &PathToSequence, vector<cv::String> &LeftStringVector,
                vector<cv::String> &RightStringVector)
{
    // std::string Timestamps_str = PathToSequence + "/cam_timestamps.txt";
    std::string Left_dir = PathToSequence + "/image_2/"; 
    std::string Right_dir = PathToSequence + "/image_3/";
    cv::glob(Left_dir, LeftStringVector, true); 
    cv::glob(Right_dir, RightStringVector, true); 
}

