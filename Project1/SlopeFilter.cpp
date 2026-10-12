#include "SlopeFilter.h"
#include "GridIndex.h"
#include <cmath>
#include <algorithm>
#include <limits>

SlopeFilter::SlopeFilter()
{
}

SlopeFilter::~SlopeFilter()
{
}

void SlopeFilter::setParams(const FilterParams& params)
{
    params_ = params;
}

void SlopeFilter::setGridIndex(std::shared_ptr<GridIndex> grid)
{
    grid_index_ = grid;
}

void SlopeFilter::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_ = cloud;
}

double SlopeFilter::calculateSlope(int center_grid, int neighbor_grid)
{
    if (!grid_index_ || !cloud_) {
        return 0.0;
    }

    double z_center = grid_index_->getGridMinElevation(center_grid);
    double z_neighbor = grid_index_->getGridMinElevation(neighbor_grid);
    double delta_h = std::abs(z_center - z_neighbor);

    int cols = grid_index_->getCols();
    int row_c = (center_grid - 1) / cols + 1;
    int col_c = (center_grid - 1) % cols + 1;
    int row_n = (neighbor_grid - 1) / cols + 1;
    int col_n = (neighbor_grid - 1) % cols + 1;

    double dx = (col_n - col_c) * params_.x_step;
    double dy = (row_n - row_c) * params_.y_step;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 1e-6) {
        return 0.0;
    }

    return delta_h / distance;
}

double SlopeFilter::getAdaptiveThreshold(int grid_id)
{
    if (!grid_index_) {
        return params_.slope_threshold;
    }

    auto neighbors = grid_index_->getNeighbors(grid_id, 2);

    if (neighbors.empty()) {
        return params_.slope_threshold;
    }

    std::vector<double> local_slopes;
    for (int nid : neighbors) {
        double s = calculateSlope(grid_id, nid);
        if (s > 1e-6) {
            local_slopes.push_back(s);
        }
    }

    if (local_slopes.empty()) {
        return params_.slope_threshold;
    }

    double mean = 0.0;
    for (double s : local_slopes) {
        mean += s;
    }
    mean /= local_slopes.size();

    double variance = 0.0;
    for (double s : local_slopes) {
        variance += (s - mean) * (s - mean);
    }
    variance /= local_slopes.size();
    double std_dev = std::sqrt(variance);

    double adaptive = params_.slope_threshold + std_dev * 0.5;

    if (adaptive < 0.01) adaptive = 0.01;
    if (adaptive > 1.0) adaptive = 1.0;

    return adaptive;
}

double SlopeFilter::computeRepresentativeSlope(const std::vector<double>& slopes)
{
    if (slopes.empty()) {
        return 0.0;
    }

    switch (params_.slope_method) {
    case 0:
    {
        double sum = 0.0;
        for (double s : slopes) {
            sum += s;
        }
        return sum / slopes.size();
    }
    case 1:
    {
        return *std::min_element(slopes.begin(), slopes.end());
    }
    case 2:
    {
        std::vector<double> sorted = slopes;
        std::sort(sorted.begin(), sorted.end());
        size_t n = sorted.size();
        if (n % 2 == 0) {
            return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
        }
        else {
            return sorted[n / 2];
        }
    }
    default:
        return 0.0;
    }
}

std::vector<int> SlopeFilter::filter()
{
    std::vector<int> ground_indices;

    if (!grid_index_ || !cloud_ || cloud_->empty()) {
        return ground_indices;
    }

    int total_points = static_cast<int>(cloud_->points.size());
    ground_mask_.assign(total_points, false);
    slope_values_.clear();

    int neighbor_radius = (params_.neighborhood == 3) ? 1 : 2;

    for (size_t i = 0; i < cloud_->points.size(); ++i) {
        int grid_id = grid_index_->getGridId(cloud_->points[i].x, cloud_->points[i].y);
        if (grid_id < 0) {
            ground_mask_[i] = false;
            continue;
        }

        // 条件1：格网内部极差判断（检测垂直结构：树、楼）
        double grid_range = grid_index_->getGridElevationRange(grid_id);
        if (grid_range > params_.height_threshold) {
            ground_mask_[i] = false;
            continue;
        }

        // 条件2：局部高度判断（相对于格网最低点，过滤楼内部点）
        double pt_z = cloud_->points[i].z;
        double local_min_z = grid_index_->getGridMinElevation(grid_id);
        double height_above_local = pt_z - local_min_z;
        if (height_above_local > params_.height_threshold) {
            ground_mask_[i] = false;
            continue;
        }

        // 条件3：坡度判断
        auto neighbors = grid_index_->getNeighbors(grid_id, neighbor_radius);
        if (neighbors.empty()) {
            ground_mask_[i] = true;
            ground_indices.push_back(static_cast<int>(i));
            continue;
        }

        std::vector<double> slopes;
        for (int nid : neighbors) {
            double slope = calculateSlope(grid_id, nid);
            slopes.push_back(slope);
        }

        double rep_slope = computeRepresentativeSlope(slopes);
        slope_values_.push_back(rep_slope);

        double threshold = params_.use_adaptive ?
            getAdaptiveThreshold(grid_id) : params_.slope_threshold;

        bool is_ground = (rep_slope < threshold);

        ground_mask_[i] = is_ground;
        if (is_ground) {
            ground_indices.push_back(static_cast<int>(i));
        }
    }

    std::sort(ground_indices.begin(), ground_indices.end());
    ground_indices.erase(std::unique(ground_indices.begin(), ground_indices.end()), ground_indices.end());

    return ground_indices;
}