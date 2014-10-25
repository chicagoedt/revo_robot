/*
* @author Bash, Vijay
*
* TODO handle memory correctly!
*/

#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"

using namespace std;
using namespace cv;
using namespace cv::gpu;

//global variables
int cannyThresh1 = 100;
int cannyThresh2 = 200;
int cannyToggle = 0;
int houghRho = 1;
int houghTheta = 1;
int houghMinLineLength = 50;
int houghMaxLineGap = 5;
int houghMaxNumberOfLines = 100;
int thresholdThresh = 20;
int thresholdSize = 0;
int blurAmount = 0;
// int erodeSize;
int erodeIterations;

//global constants
int cannyThresh1Max = 300;
int cannyThresh2Max = 600;
int cannyToggleMax = 1;
int houghRhoMax = 200;
int houghThetaMax = 359;
int houghMinLineLengthMax = 600;
int houghMaxLineGapMax = 50;
int houghMaxNumberOfLinesMax = 500;
int thresholdThreshMax = 40;
int thresholdSizeMax = 500;
int blurAmountMax = 50;
// int erodeSizeMax= 1000;
int erodeIterationsMax = 100;


int rows=400;
int columns=1280;
int roi_x=0;
int roi_y=320;

int imageWidth=1280;
int imageHeight=720;

    cv::VideoCapture cap(0); // open the default camera
    


// prints a line (array of 4 ints)
// for debugging
 void printLine(vector<int> line){
 	std::cout << "here's a line\n [ ( " <<  line[0] << ", " << line[1] << ") , ( " << line[2] << ", " << line[3] << " ) ]\n" << endl;
}


// int* oneDee;

// void createOneDee(int rows, int columns){
//     oneDee = new int[rows * columns];

//     cap.set(CV_CAP_PROP_FRAME_WIDTH, imageWidth ); //TODO pick appropriate dimensions so that the image is big enough for ROI
//     cap.set(CV_CAP_PROP_FRAME_HEIGHT, imageHeight );
// }


// //call this when you're done with line detection
// void freeOneDee(){
//     delete[] oneDee;
// }

// int oneDeeLength(){
//     return sizeof(oneDee);
// }

/**
1. reads from camera
2. prepares image for hough
3. finds hough lines
4. return lines

args
cannyThresh1 (Threshold1 for Canny edge detector)
cannyThresh2 (Threshold2 for Canny edge detector)
cannyApertureSize (aperture size of canny edge detector)
cannyL2 (whether to use the L2 Gradient method when running canny edge detector)
houghRho ()
houghMinLineLength ()
houghMaxLineGap ()
houghMaxNumberOfLines (set it to a negative number to set to unlimited)


returns 1-D array (really a 2-D bool array of binary skeletonized image) as int pointer
*/
int* findHough(/*double cannyThresh1, double cannyThresh2, double cannyApertureSize, bool cannyToggle, *//*double houghRho, double houghTheta, double houghMinLineLength, double houghMaxLineGap, int houghMaxNumberOfLines, int thresholdThresh/*, int thresholdType*//*, int blurAmount, int erodeSize, int erodeIterations */)
{
    const int64 start = getTickCount();

    //create a 2-D array and set it up to hold all lines
    vector<vector<int> > allTheLines;

/**
*   GRAB INPUT
*/
//grab from image
    // Mat src = imread("/home/scipio/Pictures/Webcam/12.jpg");
     // Mat src = imread("calib/46.jpg");

// 1. Grabbing from webcam
     
     //NOTE: uncomment this out (and the declaration of cap above) to use camera as input!
    
     if(!cap.isOpened())  { // check if we succeeded
         std::cout << "Camera not open!\n";
         cap.release(); //Instead of completely disabling it, we will turn it off instead.
     }
     Mat src;
     cap >> src; //get a new frame from the camera

     cout << "Cam is open!" << endl;
// 1. done
/**
*   end GRAB INPUT
*/


// 2. Preparing image for Hough
    
    // grab an roi and only look at that
    cv::Mat img (src); // this is x1, y1, width and height
cout << "img rows is " << img.rows << " and img cols is " << img.cols << endl;
/** 
*   UNDISTORTION
*   removing distortion effects from webcam
*   we create two matrices then fill them with the camera matrix and distortion coefficients data (obtained from camera calibration)
*   finally, we run the undistort method with the two matrices as parameters
*/

Mat camMatrix = Mat(3,3, CV_32FC1);
Mat distCoeffs = Mat(5,1, CV_32FC1);

float camArr[][3] = {{415.18336614522843,0,639.5},{0,415.18336614522843,359.5},{0,0,1}};
float distArr[] = {-0.026328136117034617,-0.017595589895546278,0,0,0.0052553271895269720};

//fill in camMatrix
  for(int i=0;i<3;i++){
    for(int j=0; j<3;j++){
      camMatrix.at<float>(i,j) = camArr[i][j];  
    }
  }

//fill in distCoeffs
  for (int i = 0; i < 5; ++i)
  {
      distCoeffs.at<float>(i) = distArr[i];
  }


//NOTE: uncomment this to apply undistort

   //      Mat undistorted = src.clone();
 //undistort(src, undistorted, camMatrix, distCoeffs);
     //imshow("undistorted", undistorted);
/**
*   end UNDISTORTION
*/


/**
*   IMAGE PREPARATION
*   we prepare image and filter it to obtain the pixels that contain lines, and nothing else.
*   first we convert to grayscale, then we add blur, then we apply adaptive threshold, then we obtain the skeleton of that.
*/

    //grayscale
    cvtColor(img, img, CV_RGB2GRAY);


    //  blurring
    if(blurAmount > 0 )
        blur( img, img, Size(blurAmount*2 + 1,blurAmount*2 + 1) );

    //adaptive threshold
    cv::adaptiveThreshold(img, img, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3+thresholdSize*2, -1 * thresholdThresh);
     
    // imshow("source", img);

    //setting up Mats for skeletonizing
    cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
    cv::Mat temp;
    cv::Mat eroded;
    std::vector< std::vector<Point> > contours;

    //this is the type of element used in the morphological transformation (MORPH_CROSS means a Mat that looks like a plus sign)
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
 
    int count = 0;

    bool done;      

    //keep applying these transformations (erode, dilate then subtract then bitwise_OR) until there's only the skeleton of the image left (the thinnest possible squiggly line)
    do
    {
        cv::erode(img, eroded, element);
        cv::dilate(eroded, temp, element); // temp = open(img)
        cv::subtract(img, temp, temp);
        cv::bitwise_or(skel, temp, skel);
        eroded.copyTo(img);
 
        done = (cv::countNonZero(img) == 0);
        count++;
    } while (!done && count < 50 + erodeIterations);    //erodeIterations basically sets a limit to how many times it should loop (in case the image is empty, this will stop it from looping infinitely.) Usually, looping about 30 times should be enough to get the skeleton.
    cout << "erosion count has reached " << count << endl;




    //displays images
    //imshow("source", src);
    //imshow("after", dst_gpu);
    //imshow("mainWindow", src);

    

// TODO make sure skel.rows is same as rows parameter and skel.cols is same as columns parameter!!!

    //build a 1D int array out of the skeletonized image

    // oneDee = new int[skel.rows * skel.cols]; // the array might be way too large this way (can we return something this big?)
    //                                             // note: had to allocate oneDee on the heap (it was too large for the stack and caused segfaults)
 //    for(int i=0;i<skel.rows;i++){
 //        for(int j=0;j<skel.cols;j++){            
 //            if (skel.at<float>(i,j) > 200) // WHY 200? white pixel
 //            {
 //                oneDee[j + i * skel.cols] = 1; //if we find a white pixel, store it as 1
 //            }
 //            else{
 //                oneDee[j + i * skel.cols] = 0; //otherwise store it as 0/change it to 0
 //            }
 //        }
 //    }

 //    for(int v = 0; v < sizeof(oneDee); v++){
	// cout << oneDee[v] << endl;
 //    }

    // waitKey();
    // return the 1D array (as a pointer)
    //TODO check if we need to release oneDee from the heap before we return it... or make Java release it when it gets it or sumshit...
    // return oneDee;

/**
*   end IMAGE PREPARATION
*/




/**
*   LINE DETECTION (simple)
*   detect the lines from the processed skeleton image (this should replace the Hough transform stuff)
*   for now, a simple fitLines method is applied (basically a least-squares method). This currently isn't working well.
*/

// convert the skeleton image to 32-bit float single-channel Mat. (fitLine is very picky and BITCHY about that... I spent a whole day trying to get it to work...) :/
    //findContours(skel,contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    //skel.convertTo(skel, CV_32FC1);



//fill in imgVec (basically, find each pixel in the image that's not black, and add it as a vector of Vec2f's [an array of 2-float arrays])
    //TODO bug could be here
    //I will apply the code from the stack overflow here I guess - Vijay
    /*
    create a 2D boolean array, as we are finding pixels in white in our transformation image
    (a "binary image" of only black and white pixels) we decided that we should just store
    these as true and false values that the Pixel to Coordinate a lot easier than drawing a line
    */

    /*
    int lineMat[src.rows][src.cols]; 
    int lineMatarray[src.cols * src.rows];

    //std::vector<Vec2f> imgVec; //why does this give us an error, do we still need this?

    for(int i=0;i<skel.rows;i++){
        for(int j=0;j<skel.cols;j++){            
            if (skel.at<float>(i,j) > 200) // WHY 200? bug is probably here!
            {
                //Vec2f vcf = Vec2f(i,j);
                //imgVec.push_back(vcf);
                lineMat[i][j] = 1; //if we find a white pixel, store it as 1
            }
            else{
                lineMat[i][j] = 0; //otherwise store it as 0/change it to 0
            }
        }
    }

    //from the 2D array map it to a 1D array to be sent Java side
    for (int i = 0; i < sizeof(lineMat); i++){
        for(int j = 0; j < sizeof(lineMat[0]); j++){
            if(lineMat[i][j] == 1)
                lineMatarray[ sizeof(lineMat) * j + i ] = 1;
            else
                lineMatarray[ sizeof(lineMat) * j + i ] = 0;
        }
    }

/*
//now make imgVec into a Mat, then copy it and make a const Mat out of it (another one of fitLine's bitchy parameter requirements)
    Mat newMat = Mat(imgVec);
    Mat leastSquares;
    const Mat tempMat = newMat.clone();

    if (tempMat.rows > 0)   //checks if the image isn't empty before attempting to run fitLines (very bitchy and it crashes)...
    {
    //Basheers code
    fitLine(tempMat, leastSquares, CV_DIST_L2, 0.0, 100, 0.1);
    line(src, Point(leastSquares.at<float>(0,0), leastSquares.at<float>(0,1)), Point(leastSquares.at<float>(0,2), leastSquares.at<float>(0,3)), Scalar(0, 0, 255), 3, CV_AA);
    //cout << newMat.rows << " and " << newMat.cols << endl;
    
    //Vijays Code from stack overflow
    fitLine(Mat(contours[0]), leastSquares, 2,0,0.01,0.01);

    int lefty = (-tempMat.at<float>(0,2)*tempMat.at<float>(0,1)/tempMat.at<float>(0,0))+tempMat.at<float>(0,3);
    int righty = ((leastSquares.cols-tempMat.at<float>(0,2))*tempMat.at<float>(0,1)/tempMat.at<float>(0,0))+tempMat.at<float>(0,3);

    line(src, Point(leastSquares.cols-1, righty), Point(0,lefty),Scalar(0, 0, 255), 3, CV_AA);
    
    } else
    {
        cout << "fitLines can't be applied because the given skeleton image is empty!!!" << endl;
    }
    //cout << leastSquares.at<float>(0,0) << " " << leastSquares.at<float>(0,1) << " " << leastSquares.at<float>(0,2) << " " << leastSquares.at<float>(0,3) << endl;
    //cout << tempMat.at<float>(0,0) << " " << tempMat.at<float>(0,1) << " " << tempMat.at<float>(0,2) << " " << tempMat.at<float>(0,3) << endl;
    // cout << leastSquares.size() << endl;
    */
    

/**
*   end LINE DETECTION (simple)
*/




/**
*   LINE DETECTION (Hough in GPU)
*   this is the old line detection using Hough transforms that run super-fast on the GPU...
*   it currently doesn't work with the skeleton pre-processed images we get, but there might be a way to make it work...
*/
     Mat mask = skel;
     Mat dst_cpu;

    //convert back to RGB
     cvtColor(mask, dst_cpu, CV_GRAY2BGR);
     Mat dst_gpu = dst_cpu.clone();
// 2. done

 // 3. find hough lines and save them to device memory
    GpuMat d_src(mask);
    GpuMat d_lines;
    HoughLinesBuf d_buf;
    // {

 //        // don't include houghMaxNumberOfLines parameter
        if (houghMaxNumberOfLines < 0)
        {
       		gpu::HoughLinesP(d_src, d_lines, d_buf, houghRho, houghTheta, houghMinLineLength, houghMaxLineGap);
        } else {
        	gpu::HoughLinesP(d_src, d_lines, d_buf,houghRho, houghTheta, houghMinLineLength, houghMaxLineGap, houghMaxNumberOfLines);
		}


 //    }

    // now that lines are saved on device memory, copy them to host memory (to CPU)
    vector<Vec4i> lines_gpu;
    if (!d_lines.empty())
    {
        lines_gpu.resize(d_lines.cols);
        Mat h_lines(1, d_lines.cols, CV_32SC4, &lines_gpu[0]);
        d_lines.download(h_lines);

    } //by now, the lines are on the CPU memory and stored in lines_gpu


 //    // prepare our array to hold all the lines (set its size)
    // allTheLines.resize(lines_gpu.size());
    // for (int i = 0; i < lines_gpu.size(); ++i)
    //     allTheLines[i].resize(4);


 //    // this will have each new line appended to it (then finally convert it to a CString and return that)
 //    //std::string allTheLinesAsString = "";

	// // go through each line u found
    for (size_t i = 0; i < lines_gpu.size(); ++i)
    {
    	// draw it
        Vec4i l = lines_gpu[i];
        line(/*dst_gpu*/src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);


 //        //get this line and store it in our "array" 
 //        allTheLines[i][0] = l[0];
 //        allTheLines[i][1] = l[1];
 //        allTheLines[i][2] = l[2];
 //        allTheLines[i][3] = l[3];
 //        //print this line (for debugging)
 //        // printLine(allTheLines[i]);


 //        // store all lines plus this line (with parantheses etc) into a stream
 //        //std::ostringstream s;

 //        //s << "< (" << l[0] << ", " << l[1] << "),(" << l[2] << ", " << l[3] << ") >" << endl;

    cout << "< (" << l[0] << ", " << l[1] << "),(" << l[2] << ", " << l[3] << ") >" << endl;        
 //        //append that line
 //        //allTheLinesAsString.append(s.str());

 //        //for debugging
 //        //cout << "currently, we're at " << i << ": " << " and the lines are: " << allTheLinesAsString; 
        
    }
    
 //    //for debugging
 //    //cout << "that's all the lines I found: " << allTheLinesAsString << endl;
 //    //    3. done
    


 //    //create a CString from the String
 //    //char *allTheLinesAsCString = new char[allTheLinesAsString.length() + 1];
 //    //std::strcpy(allTheLinesAsCString, allTheLinesAsString.c_str());

 //    const double timeSec = (getTickCount() - start) / getTickFrequency();
 //    cout << "GPU Time : " << timeSec * 1000 << " ms" << endl;
 //    cout << "GPU Found : " << d_lines.cols << endl;
    
    imshow("source", src);
    imshow("after", skel);

/**
*   end LINE DETECTION (Hough)
*/

    // 4. return lines
    //return allTheLinesAsCString ;
    // cout << "there are " << src.rows << " rows and " << src.cols << " columns in this pic" << endl;
    // return allTheLines;
}


/**
*   called every time trackbar is changed (re-runs the method with next frame from webcam).
*   makes sure houghRho and Theta don't cause exceptions when houghLinesP is called
*/
void onTrackbar( int, void*){ 
    if (houghRho==0)
        houghRho=1;
    if(houghTheta==0)
        houghTheta=1;

    // findHough(cannyThresh1, cannyThresh2 + cannyThresh1, 3, cannyBool, houghRho, (float) (CV_PI / 180.0f) * houghTheta, houghMinLineLength, houghMaxLineGap, houghMaxNumberOfLines, thresholdThresh /*0*//*thresholdType*/, blurAmount, erodeSize, erodeIterations);
    // int *p = findHough();
    // cout << p[0] << endl;
    findHough();
    // freeOneDee();
}

/**
*   sets up windows and GUI etc.
*/
void initialize(){


    //create a window
    namedWindow("mainWindow", CV_WINDOW_NORMAL);
    namedWindow("source", CV_WINDOW_NORMAL);
    namedWindow("after", CV_WINDOW_NORMAL);
    namedWindow("undistorted", CV_WINDOW_NORMAL);

    //create trackbars
    // createTrackbar( "cannyThresh1", "mainWindow", &cannyThresh1, cannyThresh1Max, onTrackbar );
    // createTrackbar( "cannyThresh2", "mainWindow", &cannyThresh2, cannyThresh2Max, onTrackbar );
    // createTrackbar( "cannyToggle", "mainWindow", &cannyToggle, cannyToggleMax, onTrackbar );
    createTrackbar( "houghRho", "mainWindow", &houghRho, houghRhoMax, onTrackbar );
    createTrackbar( "houghTheta", "mainWindow", &houghTheta, houghThetaMax, onTrackbar );
    createTrackbar( "houghMinLineLength", "mainWindow", &houghMinLineLength, houghMinLineLengthMax, onTrackbar );
    createTrackbar( "houghMaxLineGap", "mainWindow", &houghMaxLineGap, houghMaxLineGapMax, onTrackbar );
    createTrackbar( "houghMaxNumberOfLines", "mainWindow", &houghMaxNumberOfLines, houghMaxNumberOfLinesMax, onTrackbar );
    createTrackbar( "thresholdThresh", "mainWindow", &thresholdThresh, thresholdThreshMax, onTrackbar );
    createTrackbar( "thresholdSize", "mainWindow", &thresholdSize, thresholdSizeMax, onTrackbar );
    createTrackbar( "blurAmount", "mainWindow", &blurAmount, blurAmountMax, onTrackbar );
    // createTrackbar( "erodeSize", "mainWindow", &erodeSize, erodeSizeMax, onTrackbar );
    createTrackbar( "erodeIterations", "mainWindow", &erodeIterations, erodeIterationsMax, onTrackbar );


    // setTrackbarPos("cannyThresh1", "mainWindow",100);
    // setTrackbarPos("cannyThresh2", "mainWindow",100);
    // setTrackbarPos("cannyToggle", "mainWindow",0);
    setTrackbarPos("houghRho", "mainWindow",1);
    setTrackbarPos("houghTheta", "mainWindow",1);
    setTrackbarPos("houghMinLineLength", "mainWindow",50);
    setTrackbarPos("houghMaxLineGap", "mainWindow",5);
    setTrackbarPos("houghMaxNumberOfLines", "mainWindow",100);
    setTrackbarPos( "thresholdThresh", "mainWindow", 24);
    setTrackbarPos( "thresholdSize", "mainWindow", 150);
    setTrackbarPos( "blurAmount", "mainWindow", 20);
    // setTrackbarPos( "erodeSize", "mainWindow", 0);
    setTrackbarPos( "erodeIterations", "mainWindow", 0);


    // findHough(/*100, 200, 3, false,*/ 1.0f, (float) (CV_PI / 180.0f), 50, 5, 100, 200/*, 0*/, 0, 0, 0);
    //createOneDee(rows, columns); // don't hard-code during real deployment
    findHough();
    waitKey();
    //freeOneDee();
}

