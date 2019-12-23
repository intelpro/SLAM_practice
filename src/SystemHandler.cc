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
}
