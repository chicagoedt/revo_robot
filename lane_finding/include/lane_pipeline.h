#ifndef Pipeline_h
#define Pipeline_h

#include <stdio.h>
#include <opencv2/opencv.hpp>

class Pipeline
{

    public:

        void Initialize();

        cv::Mat hue;
        cv::Mat saturation;
        cv::Mat value;
        cv::Mat intensity;

        cv::Mat saturation_gradientMag;
        cv::Mat saturation_gradientDir;
        cv::Mat intensity_gradientMag;
        cv::Mat intensity_gradientDir;
        
        void pipe( cv::Mat& );
        void getHSVI( cv::Mat& );
        void getSobelMagDir( cv::Mat& );
        void CannyThresh( cv::Mat& );
};

#endif

