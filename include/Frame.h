#ifndef FRAME_H
#define FRAME_H
#include<vector>
#include<ORBextractor.h>
namespace MY_SLAM
{
    class Frame
    {
        public:
            Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
                  cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
            void ExtractORB(int lr_flag, const cv::Mat& imLeft);

        public: 
            ORBextractor* mpORBextractorLeft;
            ORBextractor* mpORBextractorRight;
            // scale pyramid information
            int mnScaleLevels;
            float mfScaleFactor;
            float mfLogScaleFactor;
            std::vector<float> mvScaleFactors;
            std::vector<float> mvInvScaleFactors;
            std::vector<float> mvLevelSigma2;
            std::vector<float> mvInvLevelSigma2;

            // vector of keypoints 
            std::vector<cv::KeyPoint> mvKeys;
            std::vector<cv::KeyPoint> mvKeysRight;
            // vector of descriptor
            cv::Mat mDescriptor;
            cv::Mat mDescriptorRight;
    };
}
#endif
