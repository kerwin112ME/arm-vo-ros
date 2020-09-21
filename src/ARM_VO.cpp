#include <opencv2/opencv.hpp>
#include "nav_msgs/Odometry.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <omp.h>

#include "scale.hpp"
#include "GRIC.hpp"
#include "pose.hpp"
#include "detector.hpp"
#include "utils.hpp"
#include "VO.h"
#include "geometry.hpp"

using namespace cv;
using namespace std;

inline float NORM(Mat t) {
    return sqrt(t.at<float>(0)*t.at<float>(0) + t.at<float>(1)*t.at<float>(1) + t.at<float>(2)*t.at<float>(2));
}

class VoNode {
public:

    int frame_id;
    int keypointSize;
    float scale_factor, Fcriteria, Hcriteria, GRICsigma;
    vector<Point2f> prev_keypoints;
    vector<Point2f> prev_inliers, curr_inliers;
    Parameters p;
    gridFASTdetector Detector;
    tracker KLT;
    scaleEstimator Scale;
    Viewer results;

    Mat prev_frame, R, t;
    Mat R_f, t_f;
    Vec3f euler; // euler: YXZ
    float totalOdo;

    nav_msgs::Odometry odomGT; // ground truth odom

    VoNode(ros::NodeHandle nh_, Parameters p_): Detector(p_.nFeatures, p_.threshold, p_.nRows, p_.nCols), 
                                                Scale(p_.cameraHeight, p_.cameraPitchAngle),
                                                results(p_.outputVideo, p_.fps, p_.frameWidth, p_.frameHeight, p_.drawGT),
                                                KLT(p_.winSize)                                      
    {   
        frame_id = 0;
        GRICsigma = p_.GRICsigma;

        p = p_;
        prev_inliers.reserve(p.nFeatures);
        curr_inliers.reserve(p.nFeatures);

        R_f = Mat::eye(3,3,CV_32FC1);
        t_f = Mat::zeros(3,1,CV_32FC1);

        totalOdo = 0.f;

        odomGT.pose.pose.position.x = 0.f;
        odomGT.pose.pose.position.y = 0.f;

        // initial yaw orientation //
        R_f = rotMat(2, 0*3.14/180);

        /// Camera Frame ///
        // forward: -z
        // toward right: x
        // toward the sky: y
        
    }

    void readGT(const nav_msgs::Odometry::ConstPtr& GT) {
        odomGT = *GT;
    }


    void addImage(const sensor_msgs::ImageConstPtr& msg) {
        clock_t start, finish;
        Mat curr_frame_distort, imgReadGray;
        try {
            Mat imgRead = cv_bridge::toCvShare(msg,"")->image;
            cv::cvtColor(imgRead, imgReadGray, CV_BGR2GRAY);
            imgReadGray.convertTo(curr_frame_distort, CV_8UC1);
        } 
        catch(cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // Undistort the image
        Mat curr_frame;
        undistort(curr_frame_distort, curr_frame, p.cameraMatrix, p.distCoeffs);

        if(frame_id == 0) {
            ROS_INFO("Processing Frame 0");
            prev_frame = curr_frame;
            if(prev_frame.empty()) {
                ROS_INFO("Can't read 000000.png");
                std::exit(0);
            }

            Detector.detect(prev_frame, prev_keypoints);
            keypointSize = prev_keypoints.size();
            frame_id ++;
        }
        else {
            start = clock();
            if(curr_frame.empty()) {
                ROS_INFO("Can't read %06d.png", frame_id);
            }

            vector<Point2f> curr_keypoints;
            KLT.track(prev_frame, curr_frame, prev_keypoints, curr_keypoints);
            Mat mask, F, H, _3dPoints;
            

            #pragma omp parallel sections
            {
                #pragma omp section
                F = findFundamentalMat(prev_keypoints, curr_keypoints, mask, FM_RANSAC, 1, 0.99);
                #pragma omp section
                H = findHomography(prev_keypoints, curr_keypoints, CV_RANSAC, 10);
            }

            GRIC(prev_keypoints, curr_keypoints, prev_keypoints.size(), F, H, GRICsigma, Fcriteria, Hcriteria);
            
            prev_inliers.clear();
            curr_inliers.clear();
            
            if(Fcriteria < Hcriteria) {
                ROS_INFO("Processing Frame %d", frame_id);
                fflush(stdout);

                // Refine fundamental matrix
                for(size_t j=0; j<prev_keypoints.size(); j++) {
                    if(mask.at<uchar>(j) == 1) {
                        prev_inliers.push_back(prev_keypoints[j]);
                        curr_inliers.push_back(curr_keypoints[j]);
                    }
                }

                F = findFundamentalMat(prev_inliers, curr_inliers, mask, FM_RANSAC, 0.5, 0.99);
                
                recoverPoseF(F, prev_inliers, curr_inliers, p.cameraMatrix, R, t, mask, _3dPoints);

                euler = rotationMatrixToEulerAngles(R);
                Mat pureYawR = rotMat(2, euler[1]);
                Mat projectTrans =(cv::Mat_<float>(3,1)<< t.at<float>(0), 0.0, t.at<float>(2));
                float projectRatio = NORM(projectTrans) / NORM(t);
                
                // obtain scale from the ground truth (for now)
                //scale_factor = getAbsoluteScale(frame_id, 0, t.at<double>(2));
                scale_factor = Scale.estimate(_3dPoints);
                scale_factor *= projectRatio;
                totalOdo += scale_factor;

                t_f = t_f + scale_factor*(R_f*t);
                R_f = pureYawR*R_f;
                
                // keypoint size threshold //
                if(curr_keypoints.size() < keypointSize*1.0) {
                    Detector.detect(curr_frame, prev_keypoints);
                    keypointSize = prev_keypoints.size();
                }
                else
                    prev_keypoints = curr_keypoints;
                

                //Detector.detect(curr_frame, prev_keypoints);
                curr_frame.copyTo(prev_frame);
            }
            else
                ROS_INFO("Skipping Frame %d!", frame_id);

            finish = clock();
            int FPS = 1000 / (1000*(finish-start)/CLOCKS_PER_SEC);

            if(p.drawGT)
                results.showGT(curr_frame, prev_inliers, curr_inliers, FPS, t_f, odomGT);
            else
                results.show(curr_frame, prev_inliers, curr_inliers, FPS, t_f);
            frame_id ++;

            //ROS_INFO_STREAM("total odom: " << totalOdo);
        }

    }


};


int main(int argc, char* argv[])
{   
    ros::init(argc, argv, "ARM_VO");
    ros::NodeHandle nh;
    ros::Subscriber odomSubGT; // ground truth odom subscriber

    string data_dir, paramsFileName, readMode;

    nh.getParam("calFile", paramsFileName);

    Parameters p;
    read_params(paramsFileName, p);

    ROS_INFO("create VoNode");
    ROS_INFO_STREAM("gric sigma: " << p.GRICsigma);
    VoNode vo_node(nh, p);


    image_transport::ImageTransport it(nh); 
    
    //image_transport::Subscriber it_sub = 
    //   it.subscribe("/camera/image_raw", 20, &VoNode::addImage, &vo_node);

    odomSubGT = nh.subscribe("/outdoor_waypoint_nav/odometry/filtered_map", 1, &VoNode::readGT, &vo_node);
    
    image_transport::Subscriber it_sub = 
        it.subscribe("/raspicam_node/image", 20, &VoNode::addImage, &vo_node, image_transport::TransportHints("compressed"));
    
    ros::spin();

    
    return 0;
}
