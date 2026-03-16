#include "CaliperX/CaliperX.hpp"
#include "CaliperX/algorithms/ZernikeMoments.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>

namespace CaliperX {

// 亚像素算法基类
class SubpixelAlgorithm {
public:
    virtual ~SubpixelAlgorithm() = default;
    virtual double refine(const std::vector<double>& gradient, int peakIdx) = 0;
};

// 抛物线拟合
class ParabolicFit : public SubpixelAlgorithm {
public:
    double refine(const std::vector<double>& gradient, int peakIdx) override {
        if (peakIdx <= 0 || peakIdx >= gradient.size() - 1) {
            return static_cast<double>(peakIdx);
        }
        
        double y_m1 = gradient[peakIdx - 1];
        double y_0 = gradient[peakIdx];
        double y_p1 = gradient[peakIdx + 1];
        
        // 抛物线拟合: y = ax^2 + bx + c
        // 极值点偏移: delta = (y_{-1} - y_{+1}) / (2 * (2*y_0 - y_{-1} - y_{+1}))
        double denom = 2.0 * (2.0 * y_0 - y_m1 - y_p1);
        if (std::abs(denom) < 1e-10) {
            return static_cast<double>(peakIdx);
        }
        
        double delta = (y_m1 - y_p1) / denom;
        return peakIdx + delta;
    }
};

// 高斯拟合
class GaussianFit : public SubpixelAlgorithm {
public:
    double refine(const std::vector<double>& gradient, int peakIdx) override {
        if (peakIdx <= 0 || peakIdx >= gradient.size() - 1) {
            return static_cast<double>(peakIdx);
        }
        
        // 对数转换后线性拟合
        double y_m1 = std::log(std::max(gradient[peakIdx - 1], 1e-10));
        double y_0 = std::log(std::max(gradient[peakIdx], 1e-10));
        double y_p1 = std::log(std::max(gradient[peakIdx + 1], 1e-10));
        
        double delta = 0.5 * (y_m1 - y_p1) / (y_m1 - 2.0 * y_0 + y_p1);
        
        if (std::isnan(delta) || std::abs(delta) > 1.0) {
            return static_cast<double>(peakIdx);
        }
        
        return peakIdx + delta;
    }
};

// 多项式拟合 (5点)
class PolynomialFit : public SubpixelAlgorithm {
public:
    double refine(const std::vector<double>& gradient, int peakIdx) override {
        int n = gradient.size();
        if (peakIdx < 2 || peakIdx >= n - 2) {
            return static_cast<double>(peakIdx);
        }
        
        // 取5点拟合3次多项式
        std::vector<double> x = {-2, -1, 0, 1, 2};
        std::vector<double> y;
        for (int i = -2; i <= 2; ++i) {
            y.push_back(gradient[peakIdx + i]);
        }
        
        // 解析求解极值点（简化版）
        // 使用3点抛物线作为近似
        double y_m1 = gradient[peakIdx - 1];
        double y_0 = gradient[peakIdx];
        double y_p1 = gradient[peakIdx + 1];
        
        double denom = 2.0 * (2.0 * y_0 - y_m1 - y_p1);
        if (std::abs(denom) < 1e-10) {
            return static_cast<double>(peakIdx);
        }
        
        double delta = (y_m1 - y_p1) / denom;
        return peakIdx + delta;
    }
};

// Caliper实现类
class Caliper::Impl {
public:
    Params params;
    cv::Point2f lineStart;
    cv::Point2f lineEnd;
    std::vector<double> projectionData;
    std::vector<double> gradientData;
    std::unique_ptr<SubpixelAlgorithm> subpixelAlgo;
    
    explicit Impl(Algorithm algo) {
        setAlgorithm(algo);
    }
    
    void setAlgorithm(Algorithm algo) {
        switch (algo) {
            case Algorithm::PARABOLIC:
                subpixelAlgo = std::make_unique<ParabolicFit>();
                break;
            case Algorithm::GAUSSIAN:
                subpixelAlgo = std::make_unique<GaussianFit>();
                break;
            case Algorithm::POLYNOMIAL:
                subpixelAlgo = std::make_unique<PolynomialFit>();
                break;
            case Algorithm::ZERNIKE:
                // Zernike矩特殊处理
                subpixelAlgo = nullptr;
                break;
        }
    }
    
    // 灰度投影
    std::vector<double> project(const cv::Mat& image, 
                                 const cv::Point2f& start, 
                                 const cv::Point2f& end,
                                 int width) {
        cv::Point2f direction = end - start;
        float length = cv::norm(direction);
        if (length < 1e-6) return {};
        
        direction /= length;
        cv::Point2f normal(-direction.y, direction.x);
        
        int numSamples = static_cast<int>(length);
        std::vector<double> projection(numSamples, 0.0);
        
        int halfWidth = width / 2;
        
        for (int i = 0; i < numSamples; ++i) {
            float t = static_cast<float>(i) / length;
            cv::Point2f center = start + t * (end - start);
            
            double sum = 0.0;
            int count = 0;
            
            for (int w = -halfWidth; w <= halfWidth; ++w) {
                cv::Point2f pt = center + w * normal;
                int x = static_cast<int>(pt.x);
                int y = static_cast<int>(pt.y);
                
                if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                    sum += image.at<uchar>(y, x);
                    count++;
                }
            }
            
            projection[i] = (count > 0) ? sum / count : 0.0;
        }
        
        return projection;
    }
    
    // 计算梯度
    std::vector<double> computeGradient(const std::vector<double>& projection) {
        if (projection.size() < 3) return {};
        
        std::vector<double> gradient(projection.size(), 0.0);
        
        // Sobel算子
        for (size_t i = 1; i < projection.size() - 1; ++i) {
            gradient[i] = -projection[i-1] + projection[i+1];
        }
        
        // 平滑
        if (params.smoothing > 1) {
            std::vector<double> smoothed = gradient;
            int halfSmooth = params.smoothing / 2;
            for (size_t i = halfSmooth; i < gradient.size() - halfSmooth; ++i) {
                double sum = 0.0;
                for (int j = -halfSmooth; j <= halfSmooth; ++j) {
                    sum += gradient[i + j];
                }
                smoothed[i] = sum / params.smoothing;
            }
            gradient = smoothed;
        }
        
        return gradient;
    }
    
    // 找峰值
    int findPeak(const std::vector<double>& gradient) {
        double maxVal = 0.0;
        int maxIdx = -1;
        
        for (size_t i = 1; i < gradient.size() - 1; ++i) {
            // 检查是否为局部最大值
            if (gradient[i] > gradient[i-1] && gradient[i] > gradient[i+1]) {
                if (gradient[i] > maxVal) {
                    maxVal = gradient[i];
                    maxIdx = static_cast<int>(i);
                }
            }
        }
        
        return maxIdx;
    }
    
    // 执行测量
    Result doMeasure(const cv::Mat& image) {
        Result result;
        result.valid = false;
        
        // 灰度投影
        projectionData = project(image, lineStart, lineEnd, params.projectionWidth);
        if (projectionData.empty()) {
            result.errorMsg = "Projection failed";
            return result;
        }
        
        // 计算梯度
        gradientData = computeGradient(projectionData);
        if (gradientData.empty()) {
            result.errorMsg = "Gradient computation failed";
            return result;
        }
        
        // 找峰值
        int peakIdx = findPeak(gradientData);
        if (peakIdx < 0) {
            result.errorMsg = "No edge found";
            return result;
        }
        
        // 检查阈值
        if (gradientData[peakIdx] < params.edgeThreshold) {
            result.errorMsg = "Edge strength below threshold";
            return result;
        }
        
        // 亚像素精修
        double subpixelPos;
        if (params.subpixelMethod == SubpixelMethod::ZERNIKE) {
            subpixelPos = ZernikeMoments::detectEdge(gradientData, peakIdx, 7);
        } else if (subpixelAlgo) {
            subpixelPos = subpixelAlgo->refine(gradientData, peakIdx);
        } else {
            subpixelPos = peakIdx;
        }
        
        result.edgePosition = subpixelPos;
        result.edgeStrength = gradientData[peakIdx];
        result.valid = true;
        
        return result;
    }
};

// Caliper公共接口实现
Caliper::Caliper(Algorithm algo) : pImpl(std::make_unique<Impl>(algo)) {}

Caliper::~Caliper() = default;

void Caliper::setParams(const Params& params) {
    pImpl->params = params;
}

void Caliper::setLine(const cv::Point2f& start, const cv::Point2f& end) {
    pImpl->lineStart = start;
    pImpl->lineEnd = end;
}

Result Caliper::measure(const cv::Mat& image) {
    return pImpl->doMeasure(image);
}

std::vector<Result> Caliper::measureBatch(const cv::Mat& image, 
                                          const std::vector<std::pair<cv::Point2f, cv::Point2f>>& lines) {
    std::vector<Result> results;
    results.reserve(lines.size());
    
    for (const auto& line : lines) {
        setLine(line.first, line.second);
        results.push_back(measure(image));
    }
    
    return results;
}

std::vector<double> Caliper::getProjectionData() const {
    return pImpl->projectionData;
}

std::vector<double> Caliper::getGradientData() const {
    return pImpl->gradientData;
}

} // namespace CaliperX