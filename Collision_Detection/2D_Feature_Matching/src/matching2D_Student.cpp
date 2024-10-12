#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        // Hamming distance is only used on binary (ORB, BRIEF, BRISK)
        int normType = descriptorType == "DES_BINARY" ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);

        // int normType = cv::NORM_HAMMING;
        // matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F) {
            std::cout << "Changed type"; 
            descSource.convertTo(descSource, CV_32F); 
            }
            
        if (descRef.type() != CV_32F)
            descRef.convertTo(descRef, CV_32F); 

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches);
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { 
        // reference: opencv
        vector<vector<cv::DMatch>> knnMatches; 
        int k = 2; // k nearest neighbors (k=2)
        matcher->knnMatch(descSource, descRef, knnMatches, k);

        //-- Filter matches using the Lowe's ratio test
        const float threshold = 0.8f;
        for (size_t i = 0; i < knnMatches.size(); i++)
        {
            if (knnMatches[i][0].distance < threshold * knnMatches[i][1].distance)
            {
                matches.push_back(knnMatches[i][0]);
            }
        }
    }

    cout << "Number of matches " << matches.size() << endl; 
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves
        float patternScale = 1.0f; 

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(); 
    }
    else if (descriptorType.compare("ORB") == 0) {
        extractor = cv::ORB::create(); 
    }
    else if (descriptorType.compare("FREAK") == 0) {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create(); 
    }
    else if (descriptorType.compare("SIFT") == 0) {
        extractor = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}


void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter 

    double t = (double)cv::getTickCount();
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // Perform Non Maximum Supression 
    double maxOverlap = 0.0; 

    for (size_t j = 0; j < dst_norm.rows; j++) {
        for (size_t i = 0; i < dst_norm.cols; i++) {
            int response = (int)dst_norm.at<float>(j, i); 
            if (response > minResponse) {
                cv::KeyPoint newKeyPoint; 
                newKeyPoint.pt = cv::Point2f(i, j); 
                newKeyPoint.size = 2* apertureSize; 
                newKeyPoint.response = response;
            
                bool flag = false; 
                for (auto it = keypoints.begin(); it != keypoints.end(); it++) {
                    double overlap = cv::KeyPoint::overlap(newKeyPoint, *it); 
                    if (overlap > maxOverlap) {
                        flag = true; 
                        if (newKeyPoint.response > (*it).response) {
                            *it = newKeyPoint; 
                            break; 
                        }
                    }
                }

                if (!flag) {
                    keypoints.push_back(newKeyPoint); 
                }
            }
        }
    }   // end of for loop 

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris Corner Detector=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis) {
    // https://docs.opencv.org/2.4/modules/features2d/doc/feature_detection_and_description.html#brisk
    // FAST, BRISK, ORB, AKAZE, and SIFT
    // select appropriate descriptor
    cv::Ptr<cv::FeatureDetector> detector; 
    if (detectorType.compare("BRISK") == 0)
    {
        int threshold = 50;     
        int octaves = 3;   
        float patternScale = 1.0f;

        detector = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (detectorType.compare("FAST") == 0) {
        int threshold = 50; // threshold for diff between central pixel and pixels around
        bool bNMS = true;   // non-maxima suppression 
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
        // detector = cv::FastFeatureDetector::create();
    }
    else if (detectorType.compare("ORB") == 0) {
        detector = cv::ORB::create(); 
    }
    else if (detectorType.compare("AKAZE") == 0) {
        detector = cv::AKAZE::create(); 
    }
    else if (detectorType.compare("SIFT") == 0) {
        detector = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    cout << detectorType << " detect keypoints =" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
}