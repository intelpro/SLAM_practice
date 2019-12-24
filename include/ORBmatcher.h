#ifndef ORBMATCHER_H
#define ORBMATCHER_H
#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
namespace MY_SLAM
{
    class ORBmatcher
    {
        public:
            ORBmatcher(float nnratio=0.6, bool checkori=true);
        public:
            static const int TH_LOW;
            static const int TH_HIGH;
            static const int HISTO_LENGTH;
    };
}
#endif
