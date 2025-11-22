#include "rod/yolo.hpp"

YOLOInferencer::YOLOInferencer(const cv::Mat& img) {
    m_imageRaw = img.clone();
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YOLO_OBB_Inference");
    session_options_ = Ort::SessionOptions();
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, Param::model_path.c_str(), session_options_);
    load_classes(Param::class_path);
}

/**
 * @brief 进行推理
 *
 * @return std::pair<std::vector<rod>, std::vector<dirty>> 返回检测到的杆件和污点信息
 */
std::pair<std::vector<rod>, std::vector<dirty>> YOLOInferencer::inference() {
    letterbox();

    std::vector<int64_t> input_dims = { 1, 3, 640, 640 };

    cv::Mat image_float;
    m_imagePad.convertTo(image_float, CV_32FC3, 1.0 / 255.0); // BGR to float32 normalized

    cv::cvtColor(image_float, image_float, cv::COLOR_BGR2RGB);

    // HWC → CHW
    std::vector<float> input_tensor_values(3 * 640 * 640);
    std::vector<cv::Mat> channels(3);
    for (int i = 0; i < 3; ++i) {
        channels[i] = cv::Mat(640, 640, CV_32FC1, input_tensor_values.data() + i * 640 * 640);
    }
    cv::split(image_float, channels);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        input_tensor_values.data(),
        input_tensor_values.size(),
        input_dims.data(),
        input_dims.size()
    );

    std::vector<const char*> input_names = { "images" };
    std::vector<const char*> output_names = { "output0" };

    auto output_tensors = session_.Run(Ort::RunOptions { nullptr }, input_names.data(), &input_tensor, 1, output_names.data(), 1);

    return parseOutput(output_tensors[0]);
}

/**
 * @brief 将图片缩放到适应模型输入的大小
 * 
 */
void YOLOInferencer::letterbox() {
    m_ratio = std::min(static_cast<float>(Param::input_shape.width) / m_imageRaw.size().width, static_cast<float>(Param::input_shape.height) / m_imageRaw.size().height);
    m_widthPad = (Param::input_shape.width - static_cast<int>(std::round(m_imageRaw.size().width * m_ratio))) / 2.0f;
    m_heightPad = (Param::input_shape.height - static_cast<int>(std::round(m_imageRaw.size().height * m_ratio))) / 2.0f;

    cv::Mat imageResized = m_imageRaw.clone();
    cv::resize(m_imageRaw, imageResized, cv::Size(static_cast<int>(std::round(m_imageRaw.size().width * m_ratio)), static_cast<int>(std::round(m_imageRaw.size().height * m_ratio))), 0, 0, cv::INTER_LINEAR);

    cv::copyMakeBorder(imageResized, m_imagePad, static_cast<int>(std::round(m_heightPad - 0.1)), static_cast<int>(std::round(m_heightPad + 0.1)), static_cast<int>(std::round(m_widthPad - 0.1)), static_cast<int>(std::round(m_widthPad + 0.1)), cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

/**
 * @brief 读取标签
 *
 * @param classes_file
 */
void YOLOInferencer::load_classes(const std::string& classes_file) {
    std::ifstream ifs(classes_file);
    std::string line;
    while (std::getline(ifs, line))
        classes_.push_back(line);
}

/**
 * @brief yolo输出后处理
 * 
 * @param output_tensor 
 * @return std::pair<std::vector<rod>, std::vector<dirty>>
 */
std::pair<std::vector<rod>, std::vector<dirty>> YOLOInferencer::parseOutput(Ort::Value& output_tensor) {
    std::pair<std::vector<rod>, std::vector<dirty>> detections;
    auto* output = output_tensor.GetTensorMutableData<float>();
    auto output_dims = output_tensor.GetTensorTypeAndShapeInfo().GetShape();

    if (output_dims.size() != 3 || output_dims[1] != 6 || output_dims[0] != 1)
    {
        std::cerr << "Invalid output shape. Expecting [1,6,N]." << std::endl;
        return detections;
    }

    const int num_anchors = output_dims[2];
    const int num_classes = 2; // 0: rod, 1: dirty

    std::vector<std::vector<float>> boxes;
    std::vector<float> scores;
    std::vector<int> classes;

    for (int i = 0; i < num_anchors; ++i) {
        float x_center = output[0 * num_anchors + i];
        float y_center = output[1 * num_anchors + i];
        float width = output[2 * num_anchors + i];
        float height = output[3 * num_anchors + i];

        float max_score = -1.0f;
        int class_id = -1;
        for (int j = 0; j < num_classes; ++j) {
            float cls_score = output[(4 + j) * num_anchors + i];
            if (cls_score > max_score) {
                max_score = cls_score;
                class_id = j;
            }
        }

        if (max_score < Param::confidence_threshold)
            continue;

        // 还原到原图
        x_center = (x_center - m_widthPad) / m_ratio;
        y_center = (y_center - m_heightPad) / m_ratio;
        width /= m_ratio;
        height /= m_ratio;

        boxes.push_back({ x_center, y_center, width, height });
        scores.push_back(max_score);
        classes.push_back(class_id);
    }

    if (boxes.empty())
        return detections;

    std::vector<cv::Rect> cv_boxes;

    for (const auto& box: boxes) {
        cv_boxes.emplace_back(static_cast<int>(box[0]) - static_cast<int>(box[2] / 2), static_cast<int>(box[1]) - static_cast<int>(box[3] / 2), static_cast<int>(box[2]), static_cast<int>(box[3]));
    }

    std::vector<int> keep_indices;
    cv::dnn::NMSBoxes(cv_boxes, scores, Param::confidence_threshold, 0.7f, keep_indices);

    // 检查cv_boxes是否越界
    for (auto& box: cv_boxes) {
        box.x = std::max(0, box.x);
        box.y = std::max(0, box.y);
        box.width = std::min(box.width, m_imageRaw.cols - box.x);
        box.height = std::min(box.height, m_imageRaw.rows - box.y);
    }

    for (int idx: keep_indices) {
        if (classes[idx] == 0) {
            detections.first.emplace_back(
                scores[idx],
                cv_boxes[idx],
                std::vector<cv::Point2f> {
                    cv::Point2f(cv_boxes[idx].x, cv_boxes[idx].y),
                    cv::Point2f(cv_boxes[idx].x + cv_boxes[idx].width, cv_boxes[idx].y),
                    cv::Point2f(cv_boxes[idx].x + cv_boxes[idx].width, cv_boxes[idx].y + cv_boxes[idx].height),
                    cv::Point2f(cv_boxes[idx].x, cv_boxes[idx].y + cv_boxes[idx].height) }
            );
        } else {
            detections.second.emplace_back(
                scores[idx],
                std::vector<cv::Point2f> {
                    cv::Point2f(cv_boxes[idx].x, cv_boxes[idx].y),
                    cv::Point2f(cv_boxes[idx].x + cv_boxes[idx].width, cv_boxes[idx].y),
                    cv::Point2f(cv_boxes[idx].x + cv_boxes[idx].width, cv_boxes[idx].y + cv_boxes[idx].height),
                    cv::Point2f(cv_boxes[idx].x, cv_boxes[idx].y + cv_boxes[idx].height) }
            );
        }
    }
    return detections;
}
