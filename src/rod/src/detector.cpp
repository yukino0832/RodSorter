#include "rod/detector.hpp"

/**
 * @brief 绘制矩形框
 * @param[in] Src
 * @param[in] rect
 * @param[in] color
 * @param[in] thickness
 */
void drawRect(cv::Mat& src, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
    cv::Point2f point[4];
    rect.points(point);
    for (int i = 0; i < 4; i++)
        cv::line(src, point[i], point[(i + 1) % 4], color, thickness);
}

std::vector<rod> RodDetector::detect(const cv::Mat& frame) {
    m_imageRaw = frame.clone();
    m_imageShow = frame.clone();
    m_imageBinary = frame.clone();
    cv::cvtColor(m_imageBinary, m_imageBinary, cv::COLOR_BGR2GRAY);
    cv::adaptiveThreshold(m_imageBinary, m_imageBinary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 45, 7);
    // cv::medianBlur(m_imageBinary, m_imageBinary, 5);
    cv::morphologyEx(m_imageBinary, m_imageBinary, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    cv::bitwise_not(m_imageBinary, m_imageBinary);
    YOLOInferencer inferencer(m_imageRaw);
    auto [rods, dirty] = inferencer.inference();
    std::cout << "Detected rods: " << rods.size() << ", dirty spots: " << dirty.size() << std::endl;
    for (auto& rod: rods) {
        cv::Mat imgRodRoi = m_imageBinary(rod.rect);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgRodRoi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            double maxArea = 0.0;
            int maxIdx = -1;
            for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
                double area = std::abs(cv::contourArea(contours[i]));
                if (area > maxArea) {
                    maxArea = area;
                    maxIdx = i;
                }
            }
            if (maxIdx >= 0) {
                std::vector<cv::Point> largestContour = contours[maxIdx];
                // 把轮廓坐标从 ROI 坐标系转换回原图坐标系
                cv::Point offset = rod.rect.tl();
                for (auto& p: largestContour)
                    p += offset;
                rod.rotatedrect = cv::minAreaRect(largestContour);
                rod.center = rod.rotatedrect.center;
                // 检测是否有污点
                cv::Point2f rect_points[4];
                rod.rotatedrect.points(rect_points);
                std::vector<cv::Point2f> polygon(rect_points, rect_points + 4);
                for (const auto& d: dirty) {
                    if (cv::pointPolygonTest(polygon, d.center, false) >= 0) {
                        rod.isDefective = true;
                        break;
                    }
                }
            } else {
                std::cerr << "No valid contour found for rod." << std::endl;
            }
        }
        drawRect(m_imageShow, rod.rotatedrect, rod.isDefective ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(m_imageShow, m_imageShow, cv::Size(), 0.8, 0.8);
    cv::imshow("Rod Detection", m_imageShow);
    cv::imshow("Binary Image", m_imageBinary);
    cv::waitKey(0);
    return std::move(rods);
}