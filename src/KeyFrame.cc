#include<KeyFrame.h>

namespace MY_SLAM
{
    long unsigned int KeyFrame::mNextId = 0;
    KeyFrame::KeyFrame(Frame& F):fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy), mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth),
    mnFrameId(F.mnId), N_keypoints(F.N_keypoints), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn), mvuRight(F.mvuRight), mvDepth(F.mvDepth),
    mDescriptors(F.mDescriptor.clone()), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor), mfLogScaleFactor(F.mfLogScaleFactor),
    mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2), mvInvLevelSigma2(F.mvInvLevelSigma2),
    mnMaxX(F.mnMaxX), mnMinX(F.mnMinX), mnMaxY(F.mnMaxY), mnMinY(F.mnMinY)
    {
        mnId = mNextId;
    }

    void KeyFrame::SetPose(const cv::Mat &Tcw_)
    {
        std::unique_lock<std::mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;

        Tcw = cv::Mat::eye(4,4, Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));
        cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0, 0, 1);
        Cw = Twc*center;
    }
}
