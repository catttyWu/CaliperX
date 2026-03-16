#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

namespace CaliperX {

// 亚像素算法类型
enum class SubpixelMethod {
    PARABOLIC,      // 抛物线拟合
    GAUSSIAN,       // 高斯拟合
    POLYNOMIAL,     // 多项式拟合
    ZERNIKE         // Zernike矩
};

// 边缘极性
enum class EdgePolarity {
    ANY,            // 任意
    DARK_TO_LIGHT,  // 黑到白
    LIGHT_TO_DARK   // 白到黑
};

// 算法类型
enum class Algorithm {
    PARABOLIC,
    GAUSSIAN,
    POLYNOMIAL,
    ZERNIKE
};

// 检测参数
struct Params {
    int projectionWidth = 11;           // 投影宽度
    EdgePolarity polarity = EdgePolarity::ANY;
    double edgeThreshold = 20.0;        // 边缘阈值
    SubpixelMethod subpixelMethod = SubpixelMethod::ZERNIKE;
    int smoothing = 3;                  // 平滑核大小
    double caliperPairDistance = 50.0;  // 卡尺对距离
};

// 检测结果
struct Result {
    double edgePosition;        // 边缘位置（亚像素）
    double edgeStrength;        // 边缘强度
    double edgeWidth;           // 边缘宽度
    bool valid;                 // 是否有效
    std::string errorMsg;       // 错误信息
};

// 卡尺检测器类
class Caliper {
public:
    explicit Caliper(Algorithm algo = Algorithm::ZERNIKE);
    ~Caliper();

    // 设置参数
    void setParams(const Params& params);
    
    // 设置卡尺线
    void setLine(const cv::Point2f& start, const cv::Point2f& end);
    
    // 执行测量
    Result measure(const cv::Mat& image);
    
    // 批量测量
    std::vector<Result> measureBatch(const cv::Mat& image, 
                                      const std::vector<std::pair<cv::Point2f, cv::Point2f>>& lines);
    
    // 获取投影数据（用于调试）
    std::vector<double> getProjectionData() const;
    
    // 获取梯度数据（用于调试）
    std::vector<double> getGradientData() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
};

} // namespace CaliperX