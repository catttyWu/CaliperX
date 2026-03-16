#pragma once

#include <vector>
#include <cmath>

namespace CaliperX {

// Zernike矩亚像素边缘检测
class ZernikeMoments {
public:
    // 计算Zernike矩 Z11, Z20
    static void computeMoments(const std::vector<double>& patch, 
                                int size,
                                double& z11_real, double& z11_imag,
                                double& z20);
    
    // 从矩计算边缘参数
    static void computeEdgeParams(double z11_real, double z11_imag, double z20,
                                   double& k,      // 背景灰度
                                   double& h,      // 对比度
                                   double& l,      // 边缘距离
                                   double& phi);   // 边缘角度
    
    // 单点检测
    static double detectEdge(const std::vector<double>& projection,
                             int centerIdx,
                             int windowSize = 7);
    
    // 预计算Zernike核
    static void precomputeKernels(int size);

private:
    static std::vector<double> kernel_z11_real;
    static std::vector<double> kernel_z11_imag;
    static std::vector<double> kernel_z20;
    static bool kernelsComputed;
};

} // namespace CaliperX