#include<string>
#include<iostream>
#include<opencv2/core/core.hpp>
#include<Tracker.h>

namespace MY_SLAM
{
    using namespace std;
    class SystemHandler
    {
    public: 
        SystemHandler(const std::string &strVocFile, const std::string &strSettingFile);
        void Tracking(const cv::Mat& left_image, const cv::Mat& right_image);

    private:
        Tracker* pTracker;
    };
}
