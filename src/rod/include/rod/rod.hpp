#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>
#include <vector>

struct Param {
    inline static std::string model_path = "/home/yukino/RodSorter/rod/model/best.onnx";
    inline static std::string class_path = "/home/yukino/RodSorter/rod/model/classes.txt";
    inline static float binary_threshold = 120.0f;
    const static inline cv::Size input_shape = cv::Size(640, 640);
    const static inline float confidence_threshold = 0.6f;
};

struct rod {
    rod(float conf, cv::Rect rect, const std::vector<cv::Point2f>& pts):
        isDefective(false),
        rect(rect),
        points(pts),
        confidence(conf) {
    }
    cv::RotatedRect rotatedrect;
    cv::Rect rect;
    bool isDefective;
    std::vector<cv::Point2f> points;
    float confidence;
    cv::Point2f center;
};

struct dirty {
    dirty(float conf, const std::vector<cv::Point2f>& pts):
        points(pts) {
        center = cv::minAreaRect(pts).center;
    }
    std::vector<cv::Point2f> points;
    cv::Point2f center;
};