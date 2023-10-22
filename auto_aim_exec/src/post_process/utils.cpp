#include "post_process/utils.hpp"
namespace utils{
    float sigmoid(float x){
        return 1 / (1 + std::exp(-x));
    }
    int argmax(const float *ptr, int len){
        int max_arg = 0;
        for (int i = 1; i < len; i++){
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }
    float inv_sigmoid(float x){
        return -std::log(1 / x - 1);
    }
    float max(float *head, int len){
        float temp = 0;
        for (int i = 0; i < len; i++){
            temp = std::fmax(temp, *(head + i));
        }
        return temp;
    }
    float calculate_iou(const pred_box& box1, const pred_box& box2){
        float al = box1._x - box1.w / 2;
        float ar = box1._x + box1.w / 2;
        float af = box1._y - box1.h / 2;
        float ab = box1._y + box1.h / 2;
        float bl = box2._x - box2.w / 2;
        float br = box2._x + box2.w / 2;
        float bf = box2._y - box2.h / 2;
        float bb = box2._y + box2.h / 2;
        float horizontal_edge = std::fmin(ar, br) - std::fmax(al, bl);
        float vertial_edge = std::fmin(ab, bb) - std::fmax(af, bf);
        if(horizontal_edge < 0 || vertial_edge < 0)
            return 0;
        else{
            float intersect = horizontal_edge * vertial_edge;
            float tunion = box1.w * box1.h + box2.w * box2.h - intersect;
            return intersect / tunion;
        }
    }
    // 检测，按照分数从大到小排序
    bool cmp(pred_box& x, pred_box& y){
        return x.score > y.score;
    }
    // 追踪，按照距离从小到大排序
    bool track(pred_box& x, pred_box& y){
        return x.dist < y.dist;
    }
    bool target_sort(cv::Point2f& x, cv::Point2f& y){
        if(std::fabs(x.y - y.y) > 20){
            return x.y < y.y;
        }
        else{
            return x.x < y.x;
        }
    }
    void rrect_sort(cv::Point2f *head, int len)
    {
        for(int i = 0; i < len; i++)
        {
            for(int j = i + 1; j < len; j++)
            {
                if(head[i].y > head[j].y)
                {
                    cv::Point2f temp = head[i];
                    head[i] = head[j];
                    head[j] = temp;
                }
            }
        }
    }
}