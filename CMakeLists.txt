cmake_minimum_required(VERSION 3.14)
project(CaliperX VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(OpenCV 4 REQUIRED)

# 选项
option(BUILD_TESTS "Build tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_PYTHON "Build Python bindings" OFF)
option(USE_CUDA "Enable CUDA support" OFF)
option(USE_OPENMP "Enable OpenMP support" ON)

# 编译选项
if(USE_OPENMP)
    find_package(OpenMP REQUIRED)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(USE_CUDA)
    enable_language(CUDA)
    find_package(CUDA REQUIRED)
endif()

# 库文件
add_library(caliperx STATIC
    src/caliper.cpp
    src/zernike_moments.cpp
    src/gaussian_fit.cpp
    src/polynomial_fit.cpp
    src/utils.cpp
)

target_include_directories(caliperx
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(caliperx
    PUBLIC
        ${OpenCV_LIBS}
)

if(USE_OPENMP)
    target_link_libraries(caliperx PUBLIC OpenMP::OpenMP_CXX)
endif()

# 安装
install(TARGETS caliperx
    EXPORT CaliperXTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include
)

# 导出目标
install(EXPORT CaliperXTargets
    FILE CaliperXTargets.cmake
    NAMESPACE CaliperX::
    DESTINATION lib/cmake/CaliperX
)

# 配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/CaliperXConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(cmake/CaliperXConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/CaliperXConfig.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/CaliperXConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/CaliperXConfigVersion.cmake"
    DESTINATION lib/cmake/CaliperX
)

# 测试
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# 示例
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Python绑定
if(BUILD_PYTHON)
    add_subdirectory(python)
endif()