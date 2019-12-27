#ifndef FRAME_H
#define FRAME_H
#include<vector>
#include<ORBextractor.h>
namespace MY_SLAM
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
    class Frame
    {
        public:
            Frame();
            Frame(const cv::Mat &imLeft, const cv::Mat &imRight, ORBextractor* extractorLeft, ORBextractor* extractorRight, 
                  cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
            void ExtractORB(int lr_flag, const cv::Mat& imLeft);
            bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
            void SetPose(cv::Mat Tcw);
            void UpdatePoseMatrices();
            void StereoMatch();
            cv::Mat BackProjectStereo(const int &i);

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

            // number of keypoints 
            int N_keypoints;
            // vector of keypoints 
            std::vector<cv::KeyPoint> mvKeys;
            std::vector<cv::KeyPoint> mvKeysRight;
            // vector of descriptor
            cv::Mat mDescriptor;
            cv::Mat mDescriptorRight;
            // undistorted keypoints
            std::vector<cv::KeyPoint> mvKeysUn;
            // Camera pose
            cv::Mat mTcw;
            // Frame id 
            static long unsigned int nNextId;
            long unsigned int mnId;
            // Calibration matrix
            cv::Mat mK;
            static float fx;
            static float fy;
            static float cx;
            static float cy;
            static float invfx;
            static float invfy;
            cv::Mat mDistCoef;
            // stereo matching 
            std::vector<float> mvuRight;
            std::vector<float> mvDepth;

            // mb (stero baseline)
            static bool mbInitialComputation;
            // baseline times fx
            float mbf;
            // stereo baseline in meters
            float mb;
            // Threshold for close and far point
            float mThDepth;
            // Undistorted image bound 
            static float mnMinX;
            static float mnMaxX;
            static float mnMinY;
            static float mnMaxY;

            // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
            static float mfGridElementWidthInv;
            static float mfGridElementHeightInv;
            std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        private:
            void ComputeImageBound(const cv::Mat& left_image);
            void UndistortKeyPoints();
            void AssignFeaturesToGrid();
            // Rotation, translation and camera center
            cv::Mat mRcw;
            cv::Mat mtcw;
            cv::Mat mRwc;
            cv::Mat mOw;
    };
}
#endif
