#pragma once

#include "rod.hpp"

class YOLOInferencer {
public:
    YOLOInferencer(const cv::Mat& img);
    std::pair<std::vector<rod>, std::vector<dirty>> inference();

private:
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_ { nullptr };
    std::vector<std::string> classes_;
    float m_widthPad, m_heightPad, m_ratio;
    cv::Mat m_imageRaw;
    cv::Mat m_imagePad;

    void load_classes(const std::string& classes_file);
    void letterbox();
    std::pair<std::vector<rod>, std::vector<dirty>> parseOutput(Ort::Value& output_tensor);
};
