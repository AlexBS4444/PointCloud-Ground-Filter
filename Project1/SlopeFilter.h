#pragma once
#ifndef SLOPE_FILTER_H
#define SLOPE_FILTER_H

#include <vector>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FilterParams.h"

// 前向声明
class GridIndex;

/**
 * @brief 坡度滤波类
 * 基于格网邻域坡度判断地面/地物点
 */
class SlopeFilter {
public:
    SlopeFilter();
    ~SlopeFilter();

    /**
     * @brief 设置滤波参数
     */
    void setParams(const FilterParams& params);

    /**
     * @brief 设置格网索引
     */
    void setGridIndex(std::shared_ptr<GridIndex> grid);

    /**
     * @brief 设置点云数据
     */
    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief 执行坡度滤波
     * @return 地面点索引列表
     */
    std::vector<int> filter();

    /**
     * @brief 获取每个格网的坡度值（用于调试/可视化）
     */
    const std::vector<double>& getSlopeValues() const { return slope_values_; }

    /**
     * @brief 获取地面点标记（true=地面, false=地物）
     */
    const std::vector<bool>& getGroundMask() const { return ground_mask_; }

private:
    /**
     * @brief 计算两个格网之间的坡度
     * @param center_grid 中心格网号
     * @param neighbor_grid 邻域格网号
     * @return 坡度值（高差/水平距离）
     */
    double calculateSlope(int center_grid, int neighbor_grid);

    /**
     * @brief 计算动态阈值
     * @param grid_id 格网号
     * @return 自适应坡度阈值
     */
    double getAdaptiveThreshold(int grid_id);

    /**
     * @brief 从邻域坡度值中计算代表坡度
     * @param slopes 邻域坡度值列表
     * @return 根据slope_method选择的代表坡度
     */
    double computeRepresentativeSlope(const std::vector<double>& slopes);

    // 参数
    FilterParams params_;
    std::shared_ptr<GridIndex> grid_index_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 结果
    std::vector<double> slope_values_;  // 每个格网的坡度值
    std::vector<bool> ground_mask_;     // 每个点的地面标记
};

#endif // SLOPE_FILTER_H