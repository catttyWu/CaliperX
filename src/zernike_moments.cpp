#include "CaliperX/algorithms/ZernikeMoments.hpp"
#include <numeric>
#include <cmath>

namespace CaliperX {

// 静态成员初始化
std::vector<double> ZernikeMoments::kernel_z11_real;
std::vector<double> ZernikeMoments::kernel_z11_imag;
std::vector<double> ZernikeMoments::kernel_z20;
bool ZernikeMoments::kernelsComputed = false;

void ZernikeMoments::precomputeKernels(int size) {
    if (kernelsComputed && kernel_z11_real.size() == size * size) {
        return;
    }
    
    int half = size / 2;
    kernel_z11_real.resize(size * size);
    kernel_z11_imag.resize(size * size);
    kernel_z20.resize(size * size);
    
    for (int y = -half; y <= half; ++y) {
        for (int x = -half; x <= half; ++x) {
            int idx = (y + half) * size + (x + half);
            double r = std::sqrt(x*x + y*y) / half;
            
            if (r > 1.0) {
                kernel_z11_real[idx] = 0;
                kernel_z11_imag[idx] = 0;
                kernel_z20[idx] = 0;
                continue;
            }
            
            double theta = std::atan2(y, x);
            
            // Z11 = 2r * e^{i*theta}
            kernel_z11_real[idx] = 2.0 * r * std::cos(theta);
            kernel_z11_imag[idx] = 2.0 * r * std::sin(theta);
            
            // Z20 = 3r^2 - 2
            kernel_z20[idx] = 3.0 * r * r - 2.0;
        }
    }
    
    kernelsComputed = true;
}

void ZernikeMoments::computeMoments(const std::vector<double>& patch, 
                                     int size,
                                     double& z11_real, double& z11_imag,
                                     double& z20) {
    precomputeKernels(size);
    
    z11_real = 0.0;
    z11_imag = 0.0;
    z20 = 0.0;
    
    // 归一化因子
    double norm = 0.0;
    
    for (size_t i = 0; i < patch.size() && i < kernel_z11_real.size(); ++i) {
        z11_real += patch[i] * kernel_z11_real[i];
        z11_imag += patch[i] * kernel_z11_imag[i];
        z20 += patch[i] * kernel_z20[i];
        norm += patch[i];
    }
    
    if (norm > 0) {
        double factor = 1.0 / (norm * size * size);
        z11_real *= factor;
        z11_imag *= factor;
        z20 *= factor;
    }
}

void ZernikeMoments::computeEdgeParams(double z11_real, double z11_imag, double z20,
                                        double& k, double& h, double& l, double& phi) {
    // 计算边缘角度
    phi = std::atan2(z11_imag, z11_real);
    
    // 计算Z11的模
    double z11_mag = std::sqrt(z11_real * z11_real + z11_imag * z11_imag);
    
    // 计算边缘距离l (亚像素偏移)
    // 简化公式: l = Z11_mag / h
    // 更精确的公式需要求解非线性方程
    
    if (std::abs(z20) > 1e-10) {
        // 近似计算
        h = 2.0 * z11_mag;
        l = z11_mag / h;
        
        // 背景灰度
        k = -z20 / 2.0;
    } else {
        k = 0;
        h = 0;
        l = 0;
    }
}

double ZernikeMoments::detectEdge(const std::vector<double>& projection,
                                   int centerIdx,
                                   int windowSize) {
    int halfWindow = windowSize / 2;
    
    if (centerIdx < halfWindow || centerIdx + halfWindow >= projection.size()) {
        return static_cast<double>(centerIdx);
    }
    
    // 构造2D patch (简化为一维多次采样)
    std::vector<double> patch(windowSize * windowSize);
    
    for (int y = 0; y < windowSize; ++y) {
        int projIdx = centerIdx - halfWindow + y;
        projIdx = std::max(0, std::min(projIdx, static_cast<int>(projection.size()) - 1));
        
        for (int x = 0; x < windowSize; ++x) {
            patch[y * windowSize + x] = projection[projIdx];
        }
    }
    
    double z11_real, z11_imag, z20;
    computeMoments(patch, windowSize, z11_real, z11_imag, z20);
    
    double k, h, l, phi;
    computeEdgeParams(z11_real, z11_imag, z20, k, h, phi, l);
    
    // 亚像素位置
    double subpixelOffset = l * std::cos(phi);
    
    // 限制偏移范围
    if (std::abs(subpixelOffset) > 1.0) {
        subpixelOffset = 0.0;
    }
    
    return centerIdx + subpixelOffset;
}

} // namespace CaliperX