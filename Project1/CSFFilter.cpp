#pragma execution_character_set("utf-8")

#include "CSFFilter.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <iostream>
#include <queue>

CSFFilter::CSFFilter()
{
}

CSFFilter::~CSFFilter()
{
}

void CSFFilter::setParams(const FilterParams& params)
{
    params_ = params;
}

void CSFFilter::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_ = cloud;
}

std::vector<int> CSFFilter::preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud) const
{
    std::vector<int> valid_indices;
    if (!cloud_ || cloud_->empty()) {
        return valid_indices;
    }

    flipped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    flipped_cloud->reserve(cloud_->points.size());

    for (size_t i = 0; i < cloud_->points.size(); ++i) {
        const auto& pt = cloud_->points[i];
        // 剔除 NaN / Inf 粗差点
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }

        pcl::PointXYZ flipped;
        flipped.x = pt.x;
        flipped.y = pt.y;
        flipped.z = -pt.z; // 翻转 Z 轴
        flipped_cloud->points.push_back(flipped);
        valid_indices.push_back(static_cast<int>(i));
    }

    flipped_cloud->width = static_cast<uint32_t>(flipped_cloud->points.size());
    flipped_cloud->height = 1;
    return valid_indices;
}

void CSFFilter::buildCollisionGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud)
{
    collision_grid_empty_ = true;
    if (!flipped_cloud || flipped_cloud->empty()) {
        return;
    }

    double x_min = std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : flipped_cloud->points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_max = std::max(y_max, static_cast<double>(pt.y));
    }

    double res = params_.csf_cloth_resolution;
    if (res <= 0.0) res = 1.0;

    collision_cols_ = static_cast<int>(std::ceil((x_max - x_min) / res)) + 1;
    collision_rows_ = static_cast<int>(std::ceil((y_max - y_min) / res)) + 1;
    if (collision_cols_ < 1) collision_cols_ = 1;
    if (collision_rows_ < 1) collision_rows_ = 1;

    collision_x_min_ = x_min;
    collision_y_min_ = y_min;

    collision_grid_.assign(collision_rows_ * collision_cols_,
                           -std::numeric_limits<double>::infinity());

    for (const auto& pt : flipped_cloud->points) {
        int c = static_cast<int>((pt.x - x_min) / res);
        int r = static_cast<int>((pt.y - y_min) / res);
        if (c < 0) c = 0;
        if (r < 0) r = 0;
        if (c >= collision_cols_) c = collision_cols_ - 1;
        if (r >= collision_rows_) r = collision_rows_ - 1;

        int idx = r * collision_cols_ + c;
        collision_grid_[idx] = std::max(collision_grid_[idx], static_cast<double>(pt.z));
        collision_grid_empty_ = false;
    }
}

void CSFFilter::initCloth(double x_min, double x_max, double y_min, double y_max, double top_z)
{
    double res = params_.csf_cloth_resolution;
    if (res <= 0.0) res = 1.0;
    cloth_resolution_ = res;

    cloth_cols_ = static_cast<int>(std::ceil((x_max - x_min) / res)) + 1;
    cloth_rows_ = static_cast<int>(std::ceil((y_max - y_min) / res)) + 1;
    if (cloth_cols_ < 1) cloth_cols_ = 1;
    if (cloth_rows_ < 1) cloth_rows_ = 1;

    cloth_x_min_ = x_min;
    cloth_y_min_ = y_min;

    cloth_nodes_.clear();
    cloth_nodes_.reserve(cloth_rows_ * cloth_cols_);

    // 布料初始位置略高于点云最高点，保证自由落体
    double start_z = top_z + 2.0 * res;

    for (int r = 0; r < cloth_rows_; ++r) {
        for (int c = 0; c < cloth_cols_; ++c) {
            ClothNode node;
            node.x = x_min + c * res;
            node.y = y_min + r * res;
            node.z = start_z;
            node.movable = true;
            cloth_nodes_.push_back(node);
        }
    }
}

double CSFFilter::getCollisionHeight(int row, int col) const
{
    if (collision_grid_empty_) {
        return -std::numeric_limits<double>::infinity();
    }
    if (row < 0 || row >= collision_rows_ || col < 0 || col >= collision_cols_) {
        return -std::numeric_limits<double>::infinity();
    }
    return collision_grid_[row * collision_cols_ + col];
}

int CSFFilter::findNearestNode(double x, double y) const
{
    int c = static_cast<int>((x - cloth_x_min_) / cloth_resolution_);
    int r = static_cast<int>((y - cloth_y_min_) / cloth_resolution_);
    if (c < 0) c = 0;
    if (r < 0) r = 0;
    if (c >= cloth_cols_) c = cloth_cols_ - 1;
    if (r >= cloth_rows_) r = cloth_rows_ - 1;
    return r * cloth_cols_ + c;
}

double CSFFilter::iterate()
{
    const double gravity = 9.8; // 重力加速度
    const double dt = params_.csf_time_step;
    double max_change = 0.0;

    // 步骤 3：外力（重力）+ 碰撞检测
    for (auto& node : cloth_nodes_) {
        if (!node.movable) continue;

        double z_old = node.z;

        // 在外力作用下下落：z_new = z - g * dt^2
        double z_new = z_old - gravity * dt * dt;

        // 查找对应碰撞格网高程
        int c = static_cast<int>((node.x - collision_x_min_) / cloth_resolution_);
        int r = static_cast<int>((node.y - collision_y_min_) / cloth_resolution_);
        double collision_z = getCollisionHeight(r, c);

        if (z_new <= collision_z) {
            // 碰到点云，固定为碰撞高程并不可移动
            node.z = collision_z;
            node.movable = false;
        }
        else {
            node.z = z_new;
        }

        double change = std::abs(node.z - z_old);
        if (change > max_change) max_change = change;
    }

    // 步骤 4：内力（刚性约束）
    // 根据硬度等级决定向邻居平均高程收敛的强度
    double rigidness_factor = 0.5; // 默认硬
    if (params_.csf_rigidness == 1) {
        rigidness_factor = 0.1;      // 软
    }
    else if (params_.csf_rigidness == 2) {
        rigidness_factor = 0.3;      // 中
    }

    std::vector<double> new_z(cloth_nodes_.size(), 0.0);
    for (size_t i = 0; i < cloth_nodes_.size(); ++i) {
        if (!cloth_nodes_[i].movable) {
            new_z[i] = cloth_nodes_[i].z;
            continue;
        }

        int r = static_cast<int>(i / cloth_cols_);
        int c = static_cast<int>(i % cloth_cols_);

        double sum_z = 0.0;
        int count = 0;
        const int dr[4] = { -1, 1, 0, 0 };
        const int dc[4] = { 0, 0, -1, 1 };

        for (int k = 0; k < 4; ++k) {
            int nr = r + dr[k];
            int nc = c + dc[k];
            if (nr >= 0 && nr < cloth_rows_ && nc >= 0 && nc < cloth_cols_) {
                int nid = nr * cloth_cols_ + nc;
                sum_z += cloth_nodes_[nid].z;
                ++count;
            }
        }

        if (count > 0) {
            double avg_z = sum_z / count;
            new_z[i] = cloth_nodes_[i].z + (avg_z - cloth_nodes_[i].z) * rigidness_factor;
        }
        else {
            new_z[i] = cloth_nodes_[i].z;
        }
    }

    for (size_t i = 0; i < cloth_nodes_.size(); ++i) {
        if (!cloth_nodes_[i].movable) continue;
        double change = std::abs(new_z[i] - cloth_nodes_[i].z);
        if (change > max_change) max_change = change;
        cloth_nodes_[i].z = new_z[i];
    }

    return max_change;
}

void CSFFilter::classify(const std::vector<int>& valid_indices,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud)
{
    ground_mask_.assign(cloud_->points.size(), false);
    cloth_z_.clear();

    if (valid_indices.empty() || !flipped_cloud || flipped_cloud->empty()) {
        return;
    }

    // 记录最终布料高程供调试
    cloth_z_.reserve(cloth_nodes_.size());
    for (const auto& node : cloth_nodes_) {
        cloth_z_.push_back(node.z);
    }

    double threshold = params_.csf_classification_threshold;

    for (size_t i = 0; i < valid_indices.size(); ++i) {
        const auto& pt = flipped_cloud->points[i];
        int node_id = findNearestNode(pt.x, pt.y);
        double cloth_height = cloth_nodes_[node_id].z;

        // 翻转空间高差：|z_flip - z_cloth| = |-z_orig - z_cloth| = |z_orig + z_cloth|
        double diff = std::abs(static_cast<double>(pt.z) - cloth_height);

        int original_idx = valid_indices[i];
        if (diff < threshold) {
            ground_mask_[original_idx] = true;
        }
    }
}

void CSFFilter::postProcess(const std::vector<int>& valid_indices,
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud)
{
    if (!params_.csf_post_process || cloth_nodes_.empty()) {
        return;
    }

    // 在布料格网上做“高度约束连通分量”：
    // 相邻格网高差小于阈值时视为连通；否则视为断开。
    // 由于点云已经被翻转，布料高程越高 => 原始点云越低（越接近真实地面）。
    // 因此保留包含全局最高布料高程的分量，其余分量（楼顶、平台等）判为非地面。
    double connect_thresh = params_.csf_connectivity_threshold;
    if (connect_thresh <= 0.0) {
        connect_thresh = 0.5;
    }

    const int total_nodes = static_cast<int>(cloth_nodes_.size());
    std::vector<int> comp_id(total_nodes, -1);
    int comp_count = 0;

    // 4-邻域
    const int dr[4] = { -1, 1, 0, 0 };
    const int dc[4] = { 0, 0, -1, 1 };

    // BFS 找连通分量
    std::queue<int> q;
    for (int i = 0; i < total_nodes; ++i) {
        if (comp_id[i] != -1) {
            continue;
        }

        comp_id[i] = comp_count;
        q.push(i);

        while (!q.empty()) {
            int cur = q.front();
            q.pop();

            int r = cur / cloth_cols_;
            int c = cur - r * cloth_cols_;  // 比 % 更直观

            for (int k = 0; k < 4; ++k) {
                int nr = r + dr[k];
                int nc = c + dc[k];
                if (nr < 0 || nr >= cloth_rows_ || nc < 0 || nc >= cloth_cols_) {
                    continue;
                }
                int nid = nr * cloth_cols_ + nc;
                if (comp_id[nid] != -1) {
                    continue;
                }
                // 高差在阈值内才连通
                if (std::abs(cloth_nodes_[nid].z - cloth_nodes_[cur].z) > connect_thresh) {
                    continue;
                }
                comp_id[nid] = comp_count;
                q.push(nid);
            }
        }

        ++comp_count;
    }

    // 找到包含全局最高布料高程的分量（真实地面）
    double global_max_z = -std::numeric_limits<double>::infinity();
    int ground_comp = -1;
    for (int i = 0; i < total_nodes; ++i) {
        if (cloth_nodes_[i].z > global_max_z) {
            global_max_z = cloth_nodes_[i].z;
            ground_comp = comp_id[i];
        }
    }

    if (ground_comp < 0) {
        return;
    }

    // 只保留属于真实地面的点
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        int original_idx = valid_indices[i];
        if (!ground_mask_[original_idx]) {
            continue;
        }

        const auto& pt = flipped_cloud->points[i];
        int node_id = findNearestNode(pt.x, pt.y);
        if (comp_id[node_id] != ground_comp) {
            ground_mask_[original_idx] = false;
        }
    }
}

std::vector<int> CSFFilter::filter()
{
    std::vector<int> ground_indices;
    ground_mask_.clear();

    if (!cloud_ || cloud_->empty()) {
        return ground_indices;
    }

    // 步骤 1：预处理（剔除粗差 + 翻转）
    pcl::PointCloud<pcl::PointXYZ>::Ptr flipped_cloud;
    std::vector<int> valid_indices = preprocess(flipped_cloud);
    if (valid_indices.empty()) {
        ground_mask_.assign(cloud_->points.size(), false);
        return ground_indices;
    }

    // 计算翻转后点云的包围盒与最高点
    double x_min = std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_max = std::numeric_limits<double>::lowest();
    double z_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : flipped_cloud->points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_max = std::max(y_max, static_cast<double>(pt.y));
        z_max = std::max(z_max, static_cast<double>(pt.z));
    }

    // 步骤 2：建立碰撞格网
    buildCollisionGrid(flipped_cloud);

    // 步骤 2（续）：初始化布料格网
    initCloth(x_min, x_max, y_min, y_max, z_max);

    // 步骤 3-5：迭代模拟
    int max_iter = params_.csf_max_iterations;
    if (max_iter < 1) max_iter = 500;

    double convergence = params_.csf_convergence_threshold;
    if (convergence <= 0.0) convergence = 0.005;

    for (int iter = 0; iter < max_iter; ++iter) {
        double max_change = iterate();
        if (max_change < convergence) {
            break;
        }
    }

    // 步骤 6：分类
    classify(valid_indices, flipped_cloud);

    // 步骤 7：后处理，剔除楼顶/平台等高处连通分量
    postProcess(valid_indices, flipped_cloud);

    for (size_t i = 0; i < ground_mask_.size(); ++i) {
        if (ground_mask_[i]) {
            ground_indices.push_back(static_cast<int>(i));
        }
    }

    std::sort(ground_indices.begin(), ground_indices.end());
    return ground_indices;
}
