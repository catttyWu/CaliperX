#include <opencv2/imgproc.hpp>
#include <cmath>

namespace CaliperX {

// 高斯拟合实现
// 使用对数转换后线性拟合

double gaussianFit(const std::vector<double>& gradient, int peakIdx) {
    if (peakIdx <= 0 || peakIdx >= gradient.size() - 1) {
        return static_cast<double>(peakIdx);
    }
    
    double y_m1 = std::log(std::max(gradient[peakIdx - 1], 1e-10));
    double y_0 = std::log(std::max(gradient[peakIdx], 1e-10));
    double y_p1 = std::log(std::max(gradient[peakIdx + 1], 1e-10));
    
    double delta = 0.5 * (y_m1 - y_p1) / (y_m1 - 2.0 * y_0 + y_p1);
    
    if (std::isnan(delta) || std::abs(delta) > 1.0) {
        return static_cast<double>(peakIdx);
    }
    
    return peakIdx + delta;
}

} // namespace CaliperX