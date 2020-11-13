#include <numeric>
#include "matching2D.hpp"

// KeyPoints is a data structure for salient point detectors.
// The class instance stores a keypoint, i.e. a point feature found by one of many available keypoint detectors, such as Harris corner detector
// The keypoint is characterized by the 2D position, scale (proportional to the diameter of the neighborhood that needs to be taken into account), 
// orientation and some other parameters.

void callDetector(cv::Mat &imgGray, std::string detectorType, std::vector<cv::KeyPoint> &keypoints, bool bDetectorVis)
{
    if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("FAST") == 0)
        {
            detKeypointsFAST(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("BRISK") == 0)
        {
            detKeypointsBRISK(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("SIFT") == 0)
        {
            detKeypointsSIFT(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("AKAZE") == 0)
        {
            detKeypointsAKAZE(keypoints, imgGray, bDetectorVis);
        }
        else if(detectorType.compare("ORB") == 0)
        {
            detKeypointsORB(keypoints, imgGray, bDetectorVis);
        }
        else{}

}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { 
            // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { 
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "(NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << std::endl;

    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { 
        // k nearest neighbors (k=2), finds the k best matches for each descriptor from a query set.
        int k = 2;
        float DESC_distanceThreshold = 0.8;
        std::vector<std::vector<cv::DMatch>> knnMatches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knnMatches, k);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        std::cout << "(KNN) with n=" << knnMatches.size() << " matches in " << 1000 * t / 1.0 << " ms" << std::endl;

        // Filtering matches using descriptor distance ratio test
        for(auto it = knnMatches.begin(); it!= knnMatches.end(); ++it){

            double distanceRatio = (*it)[0].distance / (*it)[1].distance;
            if(distanceRatio < DESC_distanceThreshold)
                matches.push_back((*it)[0]);
        }
        std::cout << "Keypoints removed = " << knnMatches.size() - matches.size() << std::endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, std::string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if(descriptorType.compare("SIFT") == 0)
    {
        int nFeatures = 0; // The number of best features to retain. The features are ranked by their scores
        int nOctaveLayers = 3; // In SIFT, an octave is the set of images generated by progressively blurring out an image

        // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. 
        // The larger the threshold, the less features are produced by the detector.
        double contrastThreshold = 0.04;

        // The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. 
        // the larger the edgeThreshold, the less features are filtered out (more features are retained).
        double edgeThreshold = 10;

        // The sigma of the Gaussian applied to the input image at the octave
        double sigma = 1.6;

        extractor = cv::SIFT::create(nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
    }
    else if(descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if(descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if(descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else{}

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << std::endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    std::vector<cv::Point2f> corners;
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
    std::cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)


    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    auto startTime = std::chrono::steady_clock::now();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled); // Scales, calculates absolute values, and converts the result to 8-bit
    auto endTime = std::chrono::steady_clock::now();

    float IoU_Threshold = 0;
    for(int rows=0; rows<dst_norm.rows; rows++){

        for(int cols=0; cols<dst_norm.cols; cols++){
            
            int pixelIntensity = dst_norm.at<float>(rows,cols);
            
            // If pixel intensity higher than the intensity threshold ==> take into account 
            if(pixelIntensity > minResponse)
            {
                cv::KeyPoint tempKeyPoint;
                tempKeyPoint.pt = cv::Point2f(cols,rows); // Columns --> change in x coord, Rows --> change in y coord  
                tempKeyPoint.response = pixelIntensity;
                tempKeyPoint.size = 2*apertureSize;

                // NMS procedure 
                bool isOverlapOccurred = false;
                for(auto it = keypoints.begin(); it != keypoints.end(); it++){
                    
                    /* This method computes overlap for pair of keypoints. Overlap is the ratio between area of keypoint regions' 
                    * intersection and area of keypoint regions' union (considering keypoint region as circle). If they don't overlap, we get zero. 
                    * If they coincide at same location with same size, we get 1
                    */
                    float overlapArea = cv::KeyPoint::overlap(tempKeyPoint, *it);

                    if(overlapArea > IoU_Threshold)
                    {   
                        isOverlapOccurred = true;
                        if(tempKeyPoint.response > it->response)
                            *it = tempKeyPoint; // Replacing the old keypoint with the new keypoint due to higher intensity of the new keypoint 
                    }
                }

                if(! isOverlapOccurred )
                    keypoints.push_back(tempKeyPoint);
            }

        }

    }

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Harris Corner KeyPoint Detection Took: " << elapsedTime.count() << " milliseconds" << std::endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsSIFT(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    
    int nFeatures = 0; // The number of best features to retain. The features are ranked by their scores
    int nOctaveLayers = 3; // In SIFT, an octave is the set of images generated by progressively blurring out an image

    // The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. 
    // The larger the threshold, the less features are produced by the detector.
    double contrastThreshold = 0.04;

    // The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. 
    // the larger the edgeThreshold, the less features are filtered out (more features are retained).
    double edgeThreshold = 10;

    // The sigma of the Gaussian applied to the input image at the octave
    double sigma = 1.6;

    cv::Ptr<cv::FeatureDetector> SIFT_Detector = cv::SIFT::create(nFeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

    // Detecting keypoints
    double t = (double)cv::getTickCount();
    SIFT_Detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if(bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "SIFT Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    
}
void detKeypointsFAST(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Features from accelerated segment test (FAST) "Published in 2006" is a corner detection method, 
    // which could be used to extract feature points and later used to track and map objects in many computer vision tasks.
    /* 
     * Select a pixel p in the image which is to be identified as an interest point or not. Let its intensity be Ip.
     * Select appropriate threshold value t.
     * Consider a circle of 16 pixels around the pixel under test. (This is a Bresenham circle of radius 3.)
     * Now the pixel p is a corner if there exists a set of n contiguous pixels in the circle (of 16 pixels) which are all brighter than Ip + t, or all darker than Ip - t. (The authors have used n= 12 in the first version of the algorithm)
     * To make the algorithm fast, first compare the intensity of pixels 1, 5, 9 and 13 of the circle with Ip. As evident from the figure above, at least three of these four pixels should satisfy the threshold criterion so that the interest point will exist.
     * If at least three of the four-pixel values — I1, I5, I9, I13 are not above or below Ip + t, then p is not an interest point (corner). In this case reject the pixel p as a possible interest point. Else if at least three of the pixels are above or below Ip + t, then check for all 16 pixels and check if 12 contiguous pixels fall in the criterion.
     * Repeat the procedure for all the pixels in the image.
     * 
    */

    // FAST detector
    int threshold = 10;
    bool nonMaxSuppression = true; 

    double t = (double)cv::getTickCount();
    cv::Ptr<cv::FeatureDetector> fastDetector = cv::FastFeatureDetector::create(threshold, nonMaxSuppression, cv::FastFeatureDetector::TYPE_9_16);
    fastDetector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "FAST with n = " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if(bVis)
    {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = "FAST Detector Results";
    cv::namedWindow(windowName , cv::WINDOW_AUTOSIZE);
    imshow(windowName, visImage);
    cv::waitKey(0);
    }
}

void detKeypointsBRISK(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();

    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "BRISK detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if(bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "BRISK Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsORB(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if(bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "ORB Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}
void detKeypointsAKAZE(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
    
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if(bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "AKAZE Detector Results";
        cv::namedWindow(windowName, 1);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}


