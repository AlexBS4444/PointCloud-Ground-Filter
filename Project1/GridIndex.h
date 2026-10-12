#ifndef GRID_INDEX_H
#define GRID_INDEX_H

#include <vector>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FilterParams.h"

class GridIndex {
public:
    GridIndex();
    ~GridIndex();

    bool buildGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const FilterParams& params);
    int getGridId(double x, double y) const;
    int getGridId(int row, int col) const;
    std::vector<int> getGridPoints(int grid_id) const;
    std::vector<int> getNeighbors(int grid_id, int range = 1) const;

    // 高程计算
    double getGridElevation(int grid_id) const;      // 平均高程
    double getGridMinElevation(int grid_id) const;   // 最低点
    double getGridMaxElevation(int grid_id) const;   // 最高点
    double getGridElevationRange(int grid_id) const; // 高程极差（新增）

    int getRows() const { return rows_; }
    int getCols() const { return cols_; }
    int getTotalGrids() const { return rows_ * cols_; }
    double getXmin() const { return x_min_; }
    double getYmin() const { return y_min_; }
    double getXmax() const { return x_max_; }
    double getYmax() const { return y_max_; }
    const std::unordered_map<int, std::vector<int>>& getGridMap() const { return grid_map_; }

private:
    double x_min_, y_min_, x_max_, y_max_;
    double x_step_, y_step_;
    int rows_, cols_;
    std::unordered_map<int, std::vector<int>> grid_map_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
};

#endif // GRID_INDEX_H