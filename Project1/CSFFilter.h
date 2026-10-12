#pragma once
#ifndef CSF_FILTER_H
#define CSF_FILTER_H

#include <vector>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FilterParams.h"

/**
 * @brief 布料模拟滤波类 (Cloth Simulation Filter, CSF)
 *
 * 实现步骤：
 * 1. 剔除粗差并翻转原始激光点云；将布料置于点云上方。
 * 2. 对点云进行格网分化，记录每个布料格网点对应碰撞点高程。
 * 3. 对可移动布料格网点，在外力（重力）作用下下落；若高程 <= 碰撞高程，
 *    则固定该格网点高程为碰撞高程并标记为不可移动。
 * 4. 计算内力作用下的格网点位移（刚性约束）。
 * 5. 重复 3、4 直至最大高程变化小于阈值或达到最大迭代次数。
 * 6. 比较原始点云与对应布料格网点高差，小于阈值则为地面点。
 */
class CSFFilter {
public:
    CSFFilter();
    ~CSFFilter();

    /**
     * @brief 设置滤波参数
     */
    void setParams(const FilterParams& params);

    /**
     * @brief 设置原始点云
     */
    void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /**
     * @brief 执行 CSF 滤波
     * @return 地面点在原始点云中的索引列表
     */
    std::vector<int> filter();

    /**
     * @brief 获取每个点的地面标记（true=地面, false=地物/非地面）
     */
    const std::vector<bool>& getGroundMask() const { return ground_mask_; }

    /**
     * @brief 获取最终布料格网高程（翻转空间），可用于调试/可视化
     */
    const std::vector<double>& getClothZ() const { return cloth_z_; }

private:
    /**
     * @brief 单个布料格网点
     */
    struct ClothNode {
        double x = 0.0;      // 水平位置 X（固定）
        double y = 0.0;      // 水平位置 Y（固定）
        double z = 0.0;      // 当前高程（翻转空间）
        bool movable = true; // 是否可移动
    };

    /**
     * @brief 预处理：剔除无效点并翻转点云
     * @return 有效点在原始点云中的索引
     */
    std::vector<int> preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud) const;

    /**
     * @brief 建立点云碰撞格网，记录每个格网内的最大翻转高程
     */
    void buildCollisionGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud);

    /**
     * @brief 初始化布料格网
     */
    void initCloth(double x_min, double x_max, double y_min, double y_max, double top_z);

    /**
     * @brief 获取点云格网中的最大翻转高程；无点返回 -inf
     */
    double getCollisionHeight(int row, int col) const;

    /**
     * @brief 计算并执行一次内外力迭代，返回本次最大高程变化
     */
    double iterate();

    /**
     * @brief 根据布料高程对原始点云进行分类
     * @param valid_indices 有效点在原始点云中的索引
     * @param flipped_cloud 翻转后的有效点云
     */
    void classify(const std::vector<int>& valid_indices,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud);

    /**
     * @brief 根据 (x,y) 查找最近的布料格网点索引
     */
    int findNearestNode(double x, double y) const;

    /**
     * @brief 后处理：基于布料格网高度连通分量剔除楼顶/平台等高处物体
     * @param valid_indices 有效点在原始点云中的索引
     * @param flipped_cloud 翻转后的有效点云
     */
    void postProcess(const std::vector<int>& valid_indices,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud);

    FilterParams params_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // 布料格网参数
    int cloth_rows_ = 0;
    int cloth_cols_ = 0;
    double cloth_x_min_ = 0.0;
    double cloth_y_min_ = 0.0;
    double cloth_resolution_ = 1.0;
    std::vector<ClothNode> cloth_nodes_;
    std::vector<double> cloth_z_; // 最终布料高程（仅用于导出调试）

    // 点云碰撞格网（存储每个格网的最大翻转高程）
    std::vector<double> collision_grid_;
    int collision_rows_ = 0;
    int collision_cols_ = 0;
    double collision_x_min_ = 0.0;
    double collision_y_min_ = 0.0;
    bool collision_grid_empty_ = true;

    // 结果
    std::vector<bool> ground_mask_;
};

#endif // CSF_FILTER_H
