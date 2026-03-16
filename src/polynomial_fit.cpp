#include <vector>
#include <cmath>

namespace CaliperX {

// 多项式拟合实现 (5点三次多项式)
// y = ax^3 + bx^2 + cx + d
// 极值点: dy/dx = 3ax^2 + 2bx + c = 0

double polynomialFit(const std::vector<double>& gradient, int peakIdx) {
    int n = gradient.size();
    if (peakIdx < 2 || peakIdx >= n - 2) {
        return static_cast<double>(peakIdx);
    }
    
    // 5点: [-2, -1, 0, 1, 2]
    double y_m2 = gradient[peakIdx - 2];
    double y_m1 = gradient[peakIdx - 1];
    double y_0 = gradient[peakIdx];
    double y_p1 = gradient[peakIdx + 1];
    double y_p2 = gradient[peakIdx + 2];
    
    // 拟合三次多项式系数（解析解）
    double a = (y_p2 - 2*y_p1 + 2*y_m1 - y_m2) / 12.0;
    double b = (y_p1 - 2*y_0 + y_m1) / 2.0 - a;
    double c = (y_p1 - y_m1) / 2.0 - a;
    
    // 求解极值点: 3ax^2 + 2bx + c = 0
    // 使用牛顿法或近似解
    // 简化：使用抛物线近似
    double delta = (y_m1 - y_p1) / (2.0 * (2.0 * y_0 - y_m1 - y_p1));
    
    if (std::abs(delta) > 2.0) {
        return static_cast<double>(peakIdx);
    }
    
    return peakIdx + delta;
}

} // namespace CaliperX