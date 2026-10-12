#include "GridIndex.h"
#include <cmath>
#include <algorithm>

GridIndex::GridIndex()
    : x_min_(0), y_min_(0), x_max_(0), y_max_(0)
    , x_step_(1.0), y_step_(1.0)
    , rows_(0), cols_(0)
{
}

GridIndex::~GridIndex()
{
}

bool GridIndex::buildGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const FilterParams& params)
{
    if (!cloud || cloud->empty()) {
        return false;
    }

    cloud_ = cloud;
    x_step_ = params.x_step;
    y_step_ = params.y_step;

    x_min_ = y_min_ = std::numeric_limits<double>::max();
    x_max_ = y_max_ = std::numeric_limits<double>::lowest();

    for (const auto& pt : cloud->points) {
        x_min_ = std::min(x_min_, static_cast<double>(pt.x));
        x_max_ = std::max(x_max_, static_cast<double>(pt.x));
        y_min_ = std::min(y_min_, static_cast<double>(pt.y));
        y_max_ = std::max(y_max_, static_cast<double>(pt.y));
    }

    rows_ = static_cast<int>(std::ceil((y_max_ - y_min_) / y_step_));
    cols_ = static_cast<int>(std::ceil((x_max_ - x_min_) / x_step_));

    if (rows_ < 1) rows_ = 1;
    if (cols_ < 1) cols_ = 1;

    grid_map_.clear();
    for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i) {
        int grid_id = getGridId(cloud->points[i].x, cloud->points[i].y);
        if (grid_id >= 0) {
            grid_map_[grid_id].push_back(i);
        }
    }

    return true;
}

int GridIndex::getGridId(double x, double y) const
{
    if (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_) {
        return -1;
    }

    int row = static_cast<int>((y - y_min_) / y_step_) + 1;
    int col = static_cast<int>((x - x_min_) / x_step_) + 1;

    if (row > rows_) row = rows_;
    if (col > cols_) col = cols_;
    if (row < 1) row = 1;
    if (col < 1) col = 1;

    return getGridId(row, col);
}

int GridIndex::getGridId(int row, int col) const
{
    if (row < 1 || row > rows_ || col < 1 || col > cols_) {
        return -1;
    }
    return (row - 1) * cols_ + col;
}

std::vector<int> GridIndex::getGridPoints(int grid_id) const
{
    auto it = grid_map_.find(grid_id);
    if (it != grid_map_.end()) {
        return it->second;
    }
    return std::vector<int>();
}

std::vector<int> GridIndex::getNeighbors(int grid_id, int range) const
{
    std::vector<int> neighbors;
    int row = (grid_id - 1) / cols_ + 1;
    int col = (grid_id - 1) % cols_ + 1;

    for (int dr = -range; dr <= range; ++dr) {
        for (int dc = -range; dc <= range; ++dc) {
            if (dr == 0 && dc == 0) continue;
            int nr = row + dr;
            int nc = col + dc;
            if (nr >= 1 && nr <= rows_ && nc >= 1 && nc <= cols_) {
                int nid = getGridId(nr, nc);
                if (nid >= 0) {
                    neighbors.push_back(nid);
                }
            }
        }
    }
    return neighbors;
}

double GridIndex::getGridElevation(int grid_id) const
{
    auto points = getGridPoints(grid_id);
    if (points.empty() || !cloud_) return 0.0;

    double sum_z = 0.0;
    for (int idx : points) {
        sum_z += cloud_->points[idx].z;
    }
    return sum_z / points.size();
}

double GridIndex::getGridMinElevation(int grid_id) const
{
    auto points = getGridPoints(grid_id);
    if (points.empty() || !cloud_) return 0.0;

    double min_z = std::numeric_limits<double>::max();
    for (int idx : points) {
        min_z = std::min(min_z, static_cast<double>(cloud_->points[idx].z));
    }
    return min_z;
}

double GridIndex::getGridMaxElevation(int grid_id) const
{
    auto points = getGridPoints(grid_id);
    if (points.empty() || !cloud_) return 0.0;

    double max_z = std::numeric_limits<double>::lowest();
    for (int idx : points) {
        max_z = std::max(max_z, static_cast<double>(cloud_->points[idx].z));
    }
    return max_z;
}

double GridIndex::getGridElevationRange(int grid_id) const
{
    auto points = getGridPoints(grid_id);
    if (points.empty() || !cloud_) return 0.0;

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (int idx : points) {
        double z = static_cast<double>(cloud_->points[idx].z);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
    }
    return max_z - min_z;
}