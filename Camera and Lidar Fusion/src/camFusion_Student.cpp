
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

	    // STUDENTS NEED TO ADD THIS CODE - START
        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
	    // STUDENTS NEED TO ADD THIS CODE - END

    } // eof loop over all Lidar points
    for (vector<BoundingBox>::iterator it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it)
    {
        if ((*it).lidarPoints.size() == 0)
        {
            it--;
            boundingBoxes.erase(it+1);
        }
    }
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        if (it1->lidarPoints.size() > 0)
        {
            // create randomized color for current 3D object
            cv::RNG rng(it1->boxID);
            cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

            // plot Lidar points into top view image
            int top=1e8, left=1e8, bottom=0.0, right=0.0; 
            float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
            for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
            {
                // world coordinates
                float xw = (*it2).x; // world position in m with x facing forward from sensor
                float yw = (*it2).y; // world position in m with y facing left from sensor
                xwmin = xwmin<xw ? xwmin : xw;
                ywmin = ywmin<yw ? ywmin : yw;
                ywmax = ywmax>yw ? ywmax : yw;

                // top-view coordinates
                int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
                int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

                // find enclosing rectangle
                top = top<y ? top : y;
                left = left<x ? left : x;
                bottom = bottom>y ? bottom : y;
                right = right>x ? right : x;

                // draw individual point
                cv::circle(topviewImg, cv::Point(x, y), 1, currColor, -1);
            }
          
            std::priority_queue<double, std::vector<double>, std::less<double>> minXCurrs_queue;
            for (auto it = it1->lidarPoints.begin(); it != it1->lidarPoints.end(); ++it)
            {
                minXCurrs_queue.push((it->x));
                if (minXCurrs_queue.size()>2)
                {
                    minXCurrs_queue.pop();
                }
            }

            xwmin = minXCurrs_queue.top();
            bottom = (-xwmin * imageSize.height / worldSize.height) + imageSize.height;

            // draw enclosing rectangle
            cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,255,0), 1.5);

            // augment object with some key data
            string str1 = cv::format("id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
            string str2 = cv::format("xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
            putText(topviewImg, str1, cv::Point2f(left-7, bottom+11), cv::FONT_ITALIC, 0.25, currColor);
            putText(topviewImg, str2, cv::Point2f(left-7, bottom+20), cv::FONT_ITALIC, 0.25, currColor);  
        }
    }
    //
    cout << "#3.1 : Prepare image done" << endl;

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

void computeStats(vector<double> &scores, double &Q1, double &Q3, double &IQR)
{
    vector<double> lowerHalfScores, upperHalfScores;
    std::sort(scores.begin(), scores.end());
    long medIndex = floor(scores.size() / 2.0);
    double med = scores.size() % 2 == 0 ? (scores[medIndex - 1] + scores[medIndex]) / 2.0 : scores[medIndex]; 
  
    for (auto it = scores.begin(); it != scores.end(); ++it)
    {
        if ((*it) < med)
        {
            lowerHalfScores.push_back(*it);
        }
        if ((*it) > med)
        {
            upperHalfScores.push_back(*it);
        }
    }
    long q1Index = floor(lowerHalfScores.size() / 2.0);
    Q1 = lowerHalfScores.size() % 2 == 0 ? (lowerHalfScores[q1Index - 1] + lowerHalfScores[q1Index]) / 2.0 : lowerHalfScores[q1Index]; 
    
    long q3Index = floor(upperHalfScores.size() / 2.0); // + lowerHalfKptMatchDists.size()
    Q3 = upperHalfScores.size() % 2 == 0 ? (upperHalfScores[q3Index - 1] + upperHalfScores[q3Index]) / 2.0 : upperHalfScores[q3Index]; 
    IQR = Q3 - Q1;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        if (boundingBox.roi.contains((kptsCurr[it->trainIdx]).pt))
        {
            boundingBox.kptMatches.push_back(*it);
        }
    }
  
    // Remove outliers
    vector<double> kptMatchDists;
    for (auto it = boundingBox.kptMatches.begin(); it != boundingBox.kptMatches.end(); ++it)
    {
        cv::KeyPoint kpCurr = kptsCurr[it->trainIdx];
        cv::KeyPoint kpPrev = kptsPrev[it->queryIdx];
        double dist = cv::norm(kpCurr.pt - kpPrev.pt); 
        kptMatchDists.push_back(dist);
    }
    double Q1, Q3, IQR;
    computeStats(kptMatchDists, Q1, Q3, IQR);
  
    int size = kptMatchDists.size();
    for (int i=0; i<size; ++i)
    {
        if(kptMatchDists[i] < (Q1-1.5*IQR) || kptMatchDists[i] > (Q3+1.5*IQR))
        {
            size--;
            i--;
            boundingBox.kptMatches.erase(boundingBox.kptMatches.begin()+i+1);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat& imgSource, cv::Mat& imgRef)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios_gt1, distRatios_lt1; 
    int count=0;
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = it1 + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 25.0; // min. required distance
            double maxDist = 100.0;

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt); // distCurr-1.;
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist && distPrev <= maxDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                count++;
                if (distRatio>(1.+1e-7))
                {
                    distRatios_gt1.push_back(distRatio);
                }
                if (distRatio<(1.-1e-7))
                {
                    distRatios_lt1.push_back(distRatio);
                }
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts
  
    vector<double>* distRatios_pt;
    if (distRatios_gt1.size()>distRatios_lt1.size())
    {
        distRatios_pt = &distRatios_gt1;
    }
    else if (distRatios_gt1.size()<distRatios_lt1.size())
    {
        distRatios_pt = &distRatios_lt1;
    }
    else
    {
        TTC = NAN;
        return;
    }
    // only continue if list of distance ratios is not empty
    if (distRatios_pt->size() < 0.5*count)
    {
        TTC = NAN;
        return;
    }

    double dT = 1. / frameRate;

    // TODO: STUDENT TASK (replacement for meanDistRatio)
    sort(distRatios_pt->begin(), distRatios_pt->end());
    long medIndex = floor(distRatios_pt->size() / 2.0);
    double medDistRatio = distRatios_pt->size() % 2 == 0 ? ((*distRatios_pt)[medIndex - 1] + (*distRatios_pt)[medIndex]) / 2.0 : (*distRatios_pt)[medIndex]; // compute median dist. ratio to remove outlier influence
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // auxiliary variables
    double dT = 1. / frameRate;        // time between two measurements in seconds
    // double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev=1e9, minXCurr=1e9;
    priority_queue<double, vector<double>, less<double>> minXPrevs_queue, minXCurrs_queue;
    vector<double> minXPrevs, minXCurrs;
    
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        minXPrevs_queue.push((it->x));
        if (minXPrevs_queue.size()>30)
        {
            minXPrevs_queue.pop();
        }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        minXCurrs_queue.push((it->x));
        if (minXCurrs_queue.size()>30)
        {
            minXCurrs_queue.pop();
        }
    }
  
    while(minXPrevs_queue.size()>0)
    {
        minXPrevs.push_back(minXPrevs_queue.top());
        minXPrevs_queue.pop();
    }
    while(minXCurrs_queue.size()>0)
    {
        minXCurrs.push_back(minXCurrs_queue.top());
        minXCurrs_queue.pop();
    }
    
    // Remove outliers
    double minXPrevQ1, minXPrevQ3, minXPrevIQR;
    computeStats(minXPrevs, minXPrevQ1, minXPrevQ3, minXPrevIQR);
    double minXCurrQ1, minXCurrQ3, minXCurrIQR;
    computeStats(minXCurrs, minXCurrQ1, minXCurrQ3, minXCurrIQR);
    
    for (auto it = minXPrevs.begin(); it != minXPrevs.end(); ++it)
    {
        if ((*it) >= (minXPrevQ1-1.5*minXPrevIQR) && (*it) <= (minXPrevQ3+1.5*minXPrevIQR) && minXPrev > (*it))
        {
            minXPrev = (*it);
        }
    }
    for (auto it = minXCurrs.begin(); it != minXCurrs.end(); ++it)
    {
        if ((*it) >= (minXCurrQ1-1.5*minXCurrIQR) && (*it) <= (minXCurrQ3+1.5*minXCurrIQR) && minXCurr > (*it))
        {
            minXCurr = (*it);
        }
    }

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
    multimap<pair<int, int>, int> cnts;
    vector<int> prevCnt, currCnt;
    for (int i=0; i<(prevFrame.boundingBoxes).size(); i++)
    {
        prevCnt.push_back(0);
    }
    for (int i=0; i<(currFrame.boundingBoxes).size(); i++)
    {
        currCnt.push_back(0);
    }
    for (int i=0; i<(currFrame.boundingBoxes).size(); i++)
    {
        for (int j=0; j<(prevFrame.boundingBoxes).size(); j++)
        {
            
            cnts.insert(pair<pair<int, int>, 
                       int>(pair<int, int>(i, j), 0));;
        }
    }

    for (auto it = matches.begin(); it != matches.end(); ++it)
    {
        int ch_i,ch_j;
        ch_i = -1;
        ch_j = -1;
        for (int i=0; i<(currFrame.boundingBoxes).size(); ++i)
        {
            if (((currFrame.boundingBoxes)[i]).roi.contains((currFrame.keypoints)[it->trainIdx].pt))
            {
                ch_i = i;
                break;
            }
        }  
        for (int j=0; j<(prevFrame.boundingBoxes).size(); ++j)
        {
            if (((prevFrame.boundingBoxes)[j]).roi.contains((prevFrame.keypoints)[it->queryIdx].pt))
            {
                ch_j = j;
                break;
            }
        }  
      
        if (ch_i != -1)
        {
            currCnt[ch_i] = currCnt[ch_i] + 1;
        }
        if (ch_j != -1)
        {
            prevCnt[ch_j] = prevCnt[ch_j] + 1;
        }
        if ((ch_i != -1) && (ch_j != -1))
        {
            multimap<pair<int, int>, int>::iterator mm_it = cnts.find(pair<int, int>(ch_i, ch_j));
            mm_it->second = mm_it->second + 1;
        }
    }  
  
    for (int i=0; i<(currFrame.boundingBoxes).size(); i++)
    {
        int max_id = -1;
        for (int j=0; j<(prevFrame.boundingBoxes).size(); j++)
        {
            multimap<pair<int, int>, int>::iterator mm_it1 = cnts.find(pair<int, int>(i, j));
            if ((mm_it1->second) > 0)
            {
                if (max_id < 0)
                {
                    max_id = j;
                }
                else
                {
                    multimap<pair<int, int>, int>::iterator mm_it2 = cnts.find(pair<int, int>(i, max_id));
                    if ((mm_it1->second) > (mm_it2->second))
                    {
                        max_id = j;
                    }
                }
            }
        }
        multimap<pair<int, int>, int>::iterator mm_it = cnts.find(pair<int, int>(i, max_id));
        if (max_id >= 0)
        {
            if (((mm_it->second) > 0) && ((mm_it->second) >= (int(currCnt[i]*0.5)+1)) && ((mm_it->second) >= (int(prevCnt[max_id]*0.5)+1)))
            {
                bbBestMatches.insert(std::pair<int,int>(((prevFrame.boundingBoxes)[max_id]).boxID,((currFrame.boundingBoxes)[i]).boxID));
            }
        }
    }
}
