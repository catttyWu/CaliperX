#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

namespace CaliperX {

// 图像预处理工具
namespace Utils {

    // 高斯平滑
    void gaussianSmooth(const cv::Mat& input, cv::Mat& output, int kernelSize) {
        if (kernelSize % 2 == 0) kernelSize++;
        cv::GaussianBlur(input, output, cv::Size(kernelSize, kernelSize), 0);
    }
    
    // 对比度增强
    void enhanceContrast(const cv::Mat& input, cv::Mat& output, double alpha, double beta) {
        input.convertTo(output, -1, alpha, beta);
    }
    
    // 自适应阈值
    cv::Mat adaptiveThreshold(const cv::Mat& input, int blockSize, double C) {
        cv::Mat output;
        cv::adaptiveThreshold(input, output, 255, 
                              cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                              cv::THRESH_BINARY, blockSize, C);
        return output;
    }
    
    // 插值获取像素值（双线性）
    double bilinearInterpolate(const cv::Mat& img, double x, double y) {
        int x0 = static_cast<int>(x);
        int y0 = static_cast<int>(y);
        int x1 = std::min(x0 + 1, img.cols - 1);
        int y1 = std::min(y0 + 1, img.rows - 1);
        
        double dx = x - x0;
        double dy = y - y0;
        
        double v00 = img.at<uchar>(y0, x0);
        double v01 = img.at<uchar>(y0, x1);
        double v10 = img.at<uchar>(y1, x0);
        double v11 = img.at<uchar>(y1, x1);
        
        double v0 = v00 * (1 - dx) + v01 * dx;
        double v1 = v10 * (1 - dx) + v11 * dx;
        
        return v0 * (1 - dy) + v1 * dy;
    }
    
    // 计算信噪比
    double calculateSNR(const std::vector<double>& signal) {
        if (signal.empty()) return 0.0;
        
        double mean = 0.0;
        for (double v : signal) mean += v;
        mean /= signal.size();
        
        double variance = 0.0;
        for (double v : signal) {
            variance += (v - mean) * (v - mean);
        }
        variance /= signal.size();
        
        return mean / std::sqrt(variance + 1e-10);
    }
    
    // 去除异常值
    void removeOutliers(std::vector<double>& data, double threshold = 3.0) {
        if (data.size() < 3) return;
        
        double mean = 0.0;
        for (double v : data) mean += v;
        mean /= data.size();
        
        double stddev = 0.0;
        for (double v : data) {
            stddev += (v - mean) * (v - mean);
        }
        stddev = std::sqrt(stddev / data.size());
        
        for (double& v : data) {
            if (std::abs(v - mean) > threshold * stddev) {
                v = mean;
            }
        }
    }

} // namespace Utils

} // namespace CaliperX