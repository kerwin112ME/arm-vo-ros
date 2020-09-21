#ifndef UTILS_HPP_INCLUDED
#define UTILS_HPP_INCLUDED

struct Parameters
{
    //Camera Parameters
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    float cameraHeight;
    float cameraPitchAngle;

    //Detector Parameters
    int nFeatures;
    int threshold;
    int nRows;
    int nCols;
    int croppingSky;

    //Tracker Parameter
    int winSize;

    //GRIC
    float GRICsigma;
};

void read_params(std::string filename, Parameters &p);


class Viewer
{
    public:
        void show(const cv::Mat &frame, std::vector<cv::Point2f> prev_keypoints, std::vector<cv::Point2f> curr_keypoints, int FPS, const cv::Mat translation);
        void readGroundtruth(std::vector<cv::Point2f> &groundTruth);
        Viewer();
    private:
        cv::Mat draw_tracked_kpts(const cv::Mat &frame, std::vector<cv::Point2f> &prev_keypoints, std::vector<cv::Point2f> &curr_keypoints);
        cv::Mat draw_trajectory(void);
        cv::Point2i convertToImageCoordinates(const cv::Point2f pointInMeters);

    float scale = 1.0f;
    float min_x = -0.5f;
    float min_y = -0.5f;
    float max_x = 0.5f;
    float max_y = 0.5f;
    const float frameScale = 5.0f;
    float imageSize = 500.0f;

    std::vector<cv::Point2f> path;
    std::vector<cv::Point2f> groundTruth;
}; 


#endif // UTILS_HPP_INCLUDED
