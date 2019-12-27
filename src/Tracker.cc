#include<Tracker.h> 
#include<opencv2/opencv.hpp>

using namespace std; 
using namespace cv;

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

        cv::Mat DistCoef(4,1,CV_32F); 
        DistCoef.at<float>(0) = slam_settings["Camera.k1"];
        DistCoef.at<float>(1) = slam_settings["Camera.k2"];
        DistCoef.at<float>(2) = slam_settings["Camera.p1"];
        DistCoef.at<float>(3) = slam_settings["Camera.p2"];
        const float k3 = slam_settings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3; 
        }
        DistCoef.copyTo(mDistCoef);

        float fps = slam_settings["Camera.fps"];
        if(fps==0)
            fps=30; 

        // Max&Min frames to insert keyframes and to check relocalzation
        mMinFrames = 0;
        mMaxFrames = fps;

        int nFeatures = slam_settings["ORBextractor.nFeatures"];
        float fScaleFactor = slam_settings["ORBextractor.scaleFactor"];
        int nLevels = slam_settings["ORBextractor.nLevels"];
        int fIniThFAST = slam_settings["ORBextractor.iniThFAST"];
        int fMinThFAST = slam_settings["ORBextractor.minThFAST"];

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        mThDepth = mbf*(float)slam_settings["ThDepth"]/fx;

        // initialize tracking state 
        mState = NO_IAMGES_YET;
    }

    void Tracker::StereoProcessing(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        mImGray = left_image;
        cv::Mat imGrayRight = right_image;
        cvtColor(mImGray, mImGray, CV_BGR2GRAY);
        cvtColor(imGrayRight, imGrayRight, CV_BGR2GRAY);
        mCurrentFrame = Frame(mImGray, imGrayRight, mpORBextractorLeft, mpORBextractorRight, mK, mDistCoef, mbf, mThDepth);

        Track();
    }

    void Tracker::Track()
    {
        if(mState==NO_IAMGES_YET)
        {
            mState = NOT_INITIALZIED;
        }

        mLastState = mState;

        if(mState==NOT_INITIALZIED)
        {
            StereoInitialization();
        }
    }

    void Tracker::StereoInitialization()
    {
        if(mCurrentFrame.N_keypoints > 500)
        {
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            
            // create keyframe
            /*
             
             */

        }

    }
}
