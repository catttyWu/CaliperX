# CaliperX

> 工业级高精度卡尺检测算法库 | Industrial-grade High-precision Caliper Detection Algorithm Library

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green.svg)](https://opencv.org/)

## ✨ 特性

- 🎯 **亚像素级精度** - 支持Zernike矩、多项式拟合，精度可达±0.01像素
- ⚡ **高性能** - OpenMP/CUDA并行加速，实时处理能力
- 🔧 **工业级** - 稳定的边缘检测，抗噪声干扰
- 📦 **易集成** - 头文件库设计，一键集成到现有项目
- 📚 **多语言绑定** - 提供Python接口

## 🚀 快速开始

### 环境要求
- C++17 或更高
- OpenCV 4.x
- CMake 3.14+

### 编译安装

```bash
git clone https://github.com/catttyWu/CaliperX.git
cd CaliperX
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 示例代码

```cpp
#include <CaliperX/CaliperX.hpp>

int main() {
    // 读取图像
    cv::Mat image = cv::imread("test.jpg", cv::IMREAD_GRAYSCALE);
    
    // 创建卡尺检测器（使用Zernike矩算法）
    CaliperX::Caliper detector(CaliperX::Algorithm::ZERNIKE);
    
    // 设置参数
    CaliperX::Params params;
    params.projectionWidth = 11;
    params.edgeThreshold = 20;
    params.subpixelMethod = CaliperX::SubpixelMethod::ZERNIKE;
    
    detector.setParams(params);
    
    // 设置卡尺线（起点和终点）
    cv::Point2f start(100, 200);
    cv::Point2f end(400, 200);
    
    // 执行检测
    auto result = detector.measure(image, start, end);
    
    // 输出结果
    std::cout << "边缘位置: " << result.edgePosition << " 像素" << std::endl;
    std::cout << "边缘强度: " << result.edgeStrength << std::endl;
    
    return 0;
}
```

## 📊 精度对比

| 算法 | 精度 | 速度 | 适用场景 |
|-----|------|------|---------|
| 抛物线拟合 | ±0.15像素 | ⚡⚡⚡ | 实时性要求高 |
| 高斯拟合 | ±0.08像素 | ⚡⚡ | 通用工业检测 |
| 多项式拟合 | ±0.04像素 | ⚡ | 高精度测量 |
| **Zernike矩** | **±0.01像素** | ⚡ | **精密测量首选** |

## 📁 项目结构

```
CaliperX/
├── include/          # 头文件
│   └── CaliperX/
│       ├── CaliperX.hpp
│       ├── algorithms/
│       │   ├── ZernikeMoments.hpp
│       │   ├── GaussianFit.hpp
│       │   └── PolynomialFit.hpp
│       └── utils/
├── src/              # 源码
│   ├── caliper.cpp
│   ├── zernike_moments.cpp
│   └── utils.cpp
├── tests/            # 单元测试
├── examples/         # 示例程序
├── docs/             # 文档
└── python/           # Python绑定
```

## 📖 文档

- [快速入门指南](docs/quickstart.md)
- [API参考文档](docs/api.md)
- [算法原理详解](docs/algorithms.md)
- [工业应用案例](docs/cases.md)

## 🔬 应用场景

- 电子元件尺寸检测（±0.01mm）
- 汽车零部件测量（±0.05mm）
- PCB板金手指宽度检测（±0.003mm）
- 机械加工件精密测量

## 🤝 贡献

欢迎提交Issue和PR！

## 📄 许可证

本项目基于 [MIT](LICENSE) 许可证开源。

## 🙏 致谢

- OpenCV团队提供的计算机视觉库
- Zernike矩算法参考：Ghosal & Mehrotra (1993)

---

**Made with ❤️ for precision measurement**