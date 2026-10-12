#pragma once
#ifndef FILTER_PARAMS_H
#define FILTER_PARAMS_H

/**
 * @brief 滤波参数结构体
 */
struct FilterParams {
    double x_step = 1.0;           // X方向格网边长
    double y_step = 1.0;           // Y方向格网边长
    double slope_threshold = 0.1;  // 坡度阈值
    int slope_method = 0;          // 坡度计算方式：0=平均, 1=最小, 2=中值
    bool use_adaptive = false;     // 是否使用动态阈值
    int neighborhood = 3;          // 邻域范围：3(3x3) 或 5(5x5)
    double height_threshold = 0.5; // 新增：点级高度阈值（单位：米）

    // CSF 布料模拟滤波参数
    int filter_method = 0;                      // 滤波方法：0=坡度滤波, 1=CSF布料模拟
    double csf_cloth_resolution = 1.0;          // 布料分辨率（m）
    int csf_max_iterations = 500;               // 最大迭代次数
    double csf_classification_threshold = 0.5;  // 地面分类高差阈值（m）
    double csf_convergence_threshold = 0.005;   // 迭代收敛阈值（m）
    int csf_rigidness = 3;                      // 布料硬度：1=软, 2=中, 3=硬
    double csf_time_step = 0.65;                // 模拟时间步长

    // CSF 后处理参数：剔除楼顶、平台等离地面过高的连通区域
    bool csf_post_process = true;               // 是否启用后处理
    double csf_connectivity_threshold = 0.5;    // 布料格网高差连通阈值（m）
};

/**
 * @brief 滤波结果统计结构体
 */
struct FilterResult {
    int total_points = 0;
    int ground_points = 0;
    int non_ground_points = 0;
    double elapsed_time_ms = 0.0;
};

#endif