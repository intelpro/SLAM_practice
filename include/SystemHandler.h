#include<string>
#include<iostream>
#include<opencv2/core/core.hpp>
#include <Tracker.h>

namespace MY_SLAM
{
    using namespace std;
    class SystemHandler
    {
    public: 
        SystemHandler(const std::string &strVocFile, const std::string &strSettingFile);
    
    private:
        Tracker* pTracker;
    };
}
