#include <Tracker.h> 

using namespace std; 

namespace MY_SLAM{
    Tracker::Tracker(const string& strSettingPath)
    {
        cv::FileStorage slam_settings(strSettingPath, cv::FileStorage::READ);
        float fx = slam_settings["Camera.fx"];
        float fy = slam_settings["Camera.fy"];
        float cx = slam_settings["Camera.cx"];
        float cy = slam_settings["Camera.cy"];
        
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0,0) = fx; 
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx; 
        K.at<float>(1,3) = cy;
        K.copyTo(mK);
        cout << mK << endl;
    }
}

