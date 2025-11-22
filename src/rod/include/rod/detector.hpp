#pragma once

#include "yolo.hpp"

void drawRect(cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness = 1);

class RodDetector {
public:
    RodDetector() = default;

    std::vector<rod> detect(const cv::Mat& frame);

private:
    cv::Mat m_imageRaw;    // 原图
    cv::Mat m_imageBinary; // 二值图
    cv::Mat m_image;       // 处理后图片
    cv::Mat m_imageShow;   // 显示图片
};