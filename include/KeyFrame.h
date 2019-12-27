#ifndef KEYFRAME_H
#define KEYFRAME_H

#include<mutex>
#include<Frame.h>
#include<ORBextractor.h>
#include<ORBmatcher.h>

namespace MY_SLAM
{
    class KeyFrame
    {
        public: 
            KeyFrame(Frame &F);
            // new one after mappoint class and keyframe db class
            void SetPose(const cv::Mat &Tcw_);
        
        public:
            static long unsigned int mNextId;
            long unsigned int mnId;
            const long unsigned int mnFrameId;

            // calibration matrix
            const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

            // Number of keypoints 
            const int N_keypoints;
            
            // KeyPoints, stereo coordinate and descriptors (all associated by an index)
            const std::vector<cv::KeyPoint> mvKeys;
            const std::vector<cv::KeyPoint> mvKeysUn;
            const std::vector<float> mvuRight; // negative value for monocular points
            const std::vector<float> mvDepth; // negative value for monocular points
            const cv::Mat mDescriptors;

            // Scale
            const int mnScaleLevels;
            const float mfScaleFactor;
            const float mfLogScaleFactor;
            const std::vector<float> mvScaleFactors;
            const std::vector<float> mvLevelSigma2;
            const std::vector<float> mvInvLevelSigma2;

            // Image bounds and calibration
            const int mnMinX;
            const int mnMinY;
            const int mnMaxX;
            const int mnMaxY;

        private:
            // SE(3) pose and camera center
            cv::Mat Tcw;
            cv::Mat Twc;
            cv::Mat Ow;
            cv::Mat Cw;
            // mutex
            std::mutex mMutexPose;
            std::mutex mMutexConnections;
            std::mutex mMutexFeatures;
            // half baseline
            float mHalfBaseline;
    };
}
#endif
