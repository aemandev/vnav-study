#include "lk_feature_tracker.h"

#include <numeric>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sy = equences we provided you by using the Harris
  //   corner detector (like here). Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  Mat gray;
  TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);

  cvtColor(frame, gray, COLOR_BGR2GRAY);
  std::vector<cv::Point2f> cur_points;
  // Reinit if length of prev_corner_ is less than 10
  if (LKFeatureTracker::prev_corners_.size() < 10){
    LKFeatureTracker::init = false;
  }
    
  if ( LKFeatureTracker::init == false){

    goodFeaturesToTrack(gray, cur_points, 500, 0.01, 10, Mat(), 3, false, 0.04);
    cornerSubPix(gray, cur_points, Size(10,10), Size(-1,-1), termcrit);
  }
  else if( !LKFeatureTracker::prev_corners_.empty()){
    // Set status output and err output
    std::vector<uchar> status;
    std::vector<float> err;
    calcOpticalFlowPyrLK(LKFeatureTracker::prev_frame_, gray,
    LKFeatureTracker::prev_corners_, cur_points, status, err, Size(31,31), 2, termcrit);
    size_t i, k;
    for (i=k=0; i< cur_points.size(); i++){
      if (!status[i])
        continue;
      cur_points[k++] = cur_points[i];
      circle(frame, cur_points[i], 3, Scalar(0,255,0), -1, 8);  
      // Draw line
      line(frame, LKFeatureTracker::prev_corners_[i], cur_points[i], Scalar(0,0,255), 1, 8);
    }
    cur_points.resize(k);
    std::vector<uchar> inlier_mask;
    inlierMaskComputation(LKFeatureTracker::prev_corners_, cur_points, &inlier_mask);
  }
  LKFeatureTracker::init = true;
  imshow(window_name_, frame);
  // Calc inlier mask

  LKFeatureTracker::prev_frame_ = gray.clone();
  LKFeatureTracker::prev_corners_ = cur_points;
  //
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //
  //     **** FILL IN HERE ***
  //
  //     Hint: look at cv::line and cv::cirle functions.
  //     Hint 2: use imshow to display the image
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}
