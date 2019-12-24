#include<thread>
#include<Frame.h>

namespace MY_SLAM
{
    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
                 cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    {
        mpORBextractorLeft = extractorLeft;
        mpORBextractorRight = extractorRight;
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = std::log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetScaleFactors();

        // thread for ORB extraction 
        std::thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        std::thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();
    }

    void Frame::ExtractORB(int lr_flag, const cv::Mat& im)
    {
        if(lr_flag==0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptor);
        else if(lr_flag==1)
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorRight);

    }
}
