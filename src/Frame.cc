#include<thread>
#include<Frame.h>
#include<ORBmatcher.h>
#include<opencv2/opencv.hpp>

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

        // distortion coefficient
        mDistCoef = distCoef;
        
        // Camera matrix
        mK = K;
        // thread for ORB extraction 
        std::thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        std::thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();
        
        N_keypoints = mvKeys.size();
    }

    void Frame::ExtractORB(int lr_flag, const cv::Mat& im)
    {
        if(lr_flag==0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptor);
        else if(lr_flag==1)
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorRight);
    }

    void Frame::UndistortKeyPoints()
    {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        cv::Mat KeyPoint_location(N_keypoints, 2, CV_32F);
        for(int i=0; i < N_keypoints; i++)
        {
            KeyPoint_location.at<float>(i,0)=mvKeys[i].pt.x;
            KeyPoint_location.at<float>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort keypoints 
        KeyPoint_location.reshape(2);
        cv::undistortPoints(KeyPoint_location,KeyPoint_location,mK,mDistCoef,cv::Mat(),mK);
        KeyPoint_location.reshape(1);

        // Fill Undistort Keypoints
        mvKeysUn.resize(N_keypoints);
        for(int i=0; i<N_keypoints; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = KeyPoint_location.at<float>(i,0);

            kp.pt.y = KeyPoint_location.at<float>(i,1);
            mvKeysUn[i] = kp;
        }
    }
   
    void Frame::StereoMatch()
    {
        mvuRight = std::vector<float>(N_keypoints, -1.0f);
        mvDepth = std::vector<float>(N_keypoints, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW)/2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;
        //Assign keypoints to row table
        std::vector<std::vector<size_t> > vRowIndices(nRows,std::vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = std::ceil(kpY+r);
            const int minr = std::floor(kpY+r);
            for(int yi=minr; yi; yi++)
                vRowIndices[yi].push_back(yiR);
        }
    }
}
