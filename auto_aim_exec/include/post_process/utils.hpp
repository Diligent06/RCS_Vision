#ifndef UTILS_HPP
#define UTILS_HPP
#include "cmath"
#include "opencv2/opencv.hpp"

namespace utils{
    struct pred_box{
        int _x;
        int _y;
        int w;
        int h;
        float score;
        int cls;
        int bcls;
        float dist;
    };
    typedef struct{
        int _x;
        int _y;
        int _w;
        int _h;
    }filter_box;
    float sigmoid(float x);
    int argmax(const float *ptr, int len);
    float inv_sigmoid(float x);
    bool cmp(pred_box& x, pred_box& y);
    bool track(pred_box& x, pred_box& y);
    bool target_sort(cv::Point2f& x, cv::Point2f& y);
    float max(float *head, int len);
    float calculate_iou(const pred_box& box1, const pred_box& box2);
    void rrect_sort(cv::Point2f *head, int len);
}
#endif