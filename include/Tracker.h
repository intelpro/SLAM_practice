#ifndef TRACKER_H
#define TRACKER_H
#include<iostream>
#include<string>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<ORBextractor.h>
#include<Frame.h>

namespace MY_SLAM
{
    class Tracker
    {
        public:
            Tracker(const std::string& strSettingPath);
            void StereoProcessing(const cv::Mat& left_image, const cv::Mat& right_image);
            void Track();

        public:
            cv::Mat mImGray;
            
            // current Frame class
            Frame mCurrentFrame;

            // Tracking state
            enum TrackingState{
                SYSTEM_NOT_READY=-1,
                NO_IAMGES_YET=0,
                NOT_INITIALZIED=1,
                OK=2,
                LOST=3
            };
            TrackingState mState;
            TrackingState mLastState;
            void StereoInitialization();
            
        private:
            // intrinsic parameters
            float fx; 
            float fy; 
            float cx; 
            float cy;
            // calibration matrix
            cv::Mat mK;
            cv::Mat mDistCoef;
            float mbf;
            // Threshold for close/far points
            float mThDepth;
            // Motion model 
            cv::Mat mVelocity;
            // keyframe rules
            int mMinFrames;
            int mMaxFrames;
            ORBextractor* mpORBextractorLeft;
            ORBextractor* mpORBextractorRight;
    };
}
#endif
