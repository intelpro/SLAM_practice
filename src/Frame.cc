#include<thread>
#include<Frame.h>
#include<ORBmatcher.h>
#include<opencv2/opencv.hpp>

namespace MY_SLAM
{
    float Frame::cx; float Frame::cy; float Frame::fx; float Frame::fy; float Frame::invfx; float Frame::invfy;
    float Frame::mnMinX; float Frame::mnMaxX; float Frame::mnMinY; float Frame::mnMaxY;
    float Frame::mfGridElementWidthInv; float Frame::mfGridElementHeightInv;
    bool Frame::mbInitialComputation; 
    long unsigned int Frame::nNextId = 0;
    Frame::Frame()
    {}
    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor* extractorLeft, ORBextractor* extractorRight,
                 cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    {
        mThDepth = thDepth;
        mnId=nNextId++;
        mpORBextractorLeft = extractorLeft;
        mpORBextractorRight = extractorRight;
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = std::log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        
        // baseline times 
        mbf = bf;

        // distortion coefficient
        mDistCoef = distCoef;
        
        // Camera matrix
        mK = K;
        // thread for ORB extraction 
        std::thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        std::thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();

        // Number of keypoints
        N_keypoints = mvKeys.size();

        // for first mb computation 
        if(mbInitialComputation)
        {
            ComputeImageBound(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputation = false;
        }

        mb = mbf/fx;
        
        AssignFeaturesToGrid();
    }

    void Frame::AssignFeaturesToGrid() 
    {
        int nReserve = 0.5f * N_keypoints/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(uint16_t i = 0; i<FRAME_GRID_COLS; i++)
            for(uint16_t j = 0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);

        for(int i=0; i<N_keypoints; i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
        
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = std::round((kp.pt.x - mnMinX)*mfGridElementWidthInv);
        posY = std::round((kp.pt.y - mnMinY)*mfGridElementHeightInv);

        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        else 
            return true;
    }

    void Frame::ExtractORB(int lr_flag, const cv::Mat& im)
    {
        if(lr_flag==0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptor);
        else if(lr_flag==1)
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorRight);
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRwc.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }

    void Frame::ComputeImageBound(const cv::Mat& left_image)
    {
        if(mDistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
            mat.at<float>(1,0)=left_image.cols; mat.at<float>(1,1)=0.0;
            mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=left_image.rows;
            mat.at<float>(3,0)=left_image.cols;
            mat.at<float>(3,1)=left_image.rows;

            // Undistort corners
            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            mnMinX = std::min(mat.at<float>(0,0),mat.at<float>(2,0));
            mnMaxX = std::max(mat.at<float>(1,0),mat.at<float>(3,0));
            mnMinY = std::min(mat.at<float>(0,1),mat.at<float>(1,1));
            mnMaxY = std::max(mat.at<float>(2,1),mat.at<float>(3,1));

        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = left_image.cols;
            mnMinY = 0.0f;
            mnMaxY = left_image.rows;
        }
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
                vRowIndices[yi].push_back(iR);
        }

        // set limits for search
    }

    cv::Mat Frame::BackProjectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
            return mRwc*x3Dc+mOw;
        }
        else 
            return cv::Mat();
    }
}
