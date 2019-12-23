#ifndef TRACKER_H
#define TRACKER_H
#include<iostream>
#include<string>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

namespace MY_SLAM
{
    class Tracker
    {
        public:
            Tracker(const std::string& strSettingPath);
        private:
            float fx; 
            float fy; 
            float cx; 
            float cy; 
            cv::Mat mK;
    };
}
#endif
