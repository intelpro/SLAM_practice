#include <SystemHandler.h>

namespace MY_SLAM
{
    SystemHandler::SystemHandler(const std::string &strVocFile, const std::string &strSettingFile)
    {
        //Check settings file
        cv::FileStorage slamSettings(strSettingFile.c_str(), cv::FileStorage::READ);
        if(!slamSettings.isOpened())
        {
           cerr << "Failed to open settings file at: " << strSettingFile << endl;
           exit(-1);
        }
        pTracker = new Tracker(strSettingFile);
    }

    void SystemHandler::Tracking(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        pTracker->StereoProcessing(left_image, right_image);
    }
}
