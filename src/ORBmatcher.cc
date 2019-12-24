#include<ORBmatcher.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

using namespace std;

namespace MY_SLAM
{
    const int ORBmatcher::TH_LOW = 50;
    const int ORBmatcher::TH_HIGH = 100;
    const int ORBmatcher::HISTO_LENGTH = 30;
}

