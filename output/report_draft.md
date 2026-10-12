# 激光雷达点云地面滤波算法实习报告

## 一、实习目的与项目概述

本次实习围绕机载/车载激光雷达点云数据的地面点提取展开，目标是实现一个基于 Qt + PCL（Point Cloud Library）的可视化地面滤波工具。系统支持多格式点云读写，并集成两种经典地面滤波算法：坡度滤波与 CSF（Cloth Simulation Filter，布料模拟滤波）。通过本次实习，掌握点云数据 IO、格网空间索引、坡度阈值判别以及物理模拟滤波的核心原理与 C++ 工程实现方法。

## 二、开发环境与工具

- **操作系统**：Windows 10/11
- **开发工具**：Visual Studio 2022、Qt 5.14.2 (MSVC2017_64)
- **点云库**：PCL 1.11.1
- **LAS 支持**：LASlib
- **编程语言**：C++14

## 三、系统架构与模块组成

系统采用 Qt 主窗口作为交互层，PCL 作为点云数据结构支撑，核心算法以独立类的形式封装。主要模块如下：

| 模块 | 文件 | 职责 |
|------|------|------|
| 程序入口 | `main.cpp` | 创建 Qt 应用、设置 UTF-8 编码、启动主窗口 |
| 主窗口/UI | `mainwindow.h`、`mainwindow8bom.cpp`、`mainwindow.ui` | 文件选择、参数输入、进度显示、结果导出 |
| 点云 IO | `PointCloudIO.h/cpp` | 支持 PCD、TXT/XYZ、LAS/LAZ 格式的读写与分类导出 |
| 格网索引 | `GridIndex.h/cpp` | 构建二维规则格网，提供邻域查询与高程统计 |
| 滤波参数 | `FilterParams.h` | 集中管理坡度滤波与 CSF 的全部可调参数 |
| 坡度滤波 | `SlopeFilter.h/cpp` | 基于格网坡度与高度阈值判别地面/地物点 |
| 布料模拟滤波 | `CSFFilter.h/cpp` | 模拟布料下落过程估计地形，进而分类地面点 |

## 四、核心代码讲解

### 4.1 程序入口与编码设置

`main.cpp` 负责初始化 Qt 应用并设置本地化编码为 UTF-8，确保中文日志、文件路径、UI 提示能够正确显示。

```cpp
int main(int argc, char* argv[])
{
    QApplication a(argc, argv);
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
#endif
    MainWindow w;
    w.show();
    return a.exec();
}
```

### 4.2 点云 IO 模块

`PointCloudIO` 是系统的数据门户，对外提供 `read` 与 `write` 两个静态接口，内部根据扩展名自动分发到 PCD、TXT、LAS/LAZ 的私有实现。

```cpp
bool PointCloudIO::read(const std::string& filename,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string ext = getExtension(filename);
    if (ext == "pcd")        return readPCD(filename, cloud);
    else if (ext == "txt" || ext == "xyz") return readTXT(filename, cloud);
    else if (ext == "las" || ext == "laz") return readLAS(filename, cloud);
    else {
        last_error_ = "不支持的文件格式: " + ext;
        return false;
    }
}
```

TXT 读取实现简单但鲁棒：跳过空行与 `#` 注释，逐行解析 x、y、z，兼容含分类标签的第四列。

```cpp
bool PointCloudIO::readTXT(const std::string& filename,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::ifstream ifs(filename);
    cloud->clear();
    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);
        pcl::PointXYZ pt;
        if (iss >> pt.x >> pt.y >> pt.z) {
            cloud->points.push_back(pt);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return !cloud->empty();
}
```

LAS/LAZ 读写则依赖 LASlib 的 `LASreadOpener` 与 `LASwriteOpener`，并在写入时正确设置偏移量与比例因子以保证坐标精度。

### 4.3 格网索引模块

`GridIndex` 将三维点云投影到二维水平面，按给定 `x_step`、`y_step` 划分规则格网，为后续滤波提供空间加速。

```cpp
bool GridIndex::buildGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const FilterParams& params)
{
    cloud_ = cloud;
    x_step_ = params.x_step;
    y_step_ = params.y_step;

    // 计算 XY 包围盒
    for (const auto& pt : cloud->points) { ... }

    rows_ = static_cast<int>(std::ceil((y_max_ - y_min_) / y_step_));
    cols_ = static_cast<int>(std::ceil((x_max_ - x_min_) / x_step_));

    // 将每个点归入对应格网
    for (int i = 0; i < cloud->points.size(); ++i) {
        int grid_id = getGridId(cloud->points[i].x, cloud->points[i].y);
        if (grid_id >= 0) grid_map_[grid_id].push_back(i);
    }
    return true;
}
```

格网编号采用“先行后列”的一维编码：

```cpp
int GridIndex::getGridId(int row, int col) const
{
    if (row < 1 || row > rows_ || col < 1 || col > cols_) return -1;
    return (row - 1) * cols_ + col;
}
```

模块还提供邻域查询、平均/最低/最高高程、格网内高程极差等统计接口，直接服务于坡度滤波中的多重判断条件。

### 4.4 坡度滤波模块

`SlopeFilter` 基于“地面局部坡度较小、地物（建筑、植被）坡度突变”的假设，对每个点计算其所在格网与邻域格网的坡度，并结合高度阈值进行综合判断。

坡度计算采用高差与水平距离之比：

```cpp
double SlopeFilter::calculateSlope(int center_grid, int neighbor_grid)
{
    double delta_h = std::abs(
        grid_index_->getGridMinElevation(center_grid) -
        grid_index_->getGridMinElevation(neighbor_grid));

    int row_c = (center_grid - 1) / cols + 1;
    int col_c = (center_grid - 1) % cols + 1;
    int row_n = (neighbor_grid - 1) / cols + 1;
    int col_n = (neighbor_grid - 1) % cols + 1;

    double dx = (col_n - col_c) * params_.x_step;
    double dy = (row_n - row_c) * params_.y_step;
    double distance = std::sqrt(dx * dx + dy * dy);

    return delta_h / distance;
}
```

主滤波流程包含三条判据：

1. **格网内高程极差**大于 `height_threshold` 的点，直接判为非地面，用于剔除垂直结构内部点；
2. **点相对于格网最低点的高度**大于 `height_threshold` 的点，判为非地面，用于过滤楼顶、树冠等；
3. **邻域代表坡度**小于坡度阈值（或自适应阈值）的点，判为地面。

```cpp
for (size_t i = 0; i < cloud_->points.size(); ++i) {
    int grid_id = grid_index_->getGridId(cloud_->points[i].x,
                                          cloud_->points[i].y);

    // 判据 1：格网内极差
    if (grid_index_->getGridElevationRange(grid_id) > params_.height_threshold)
        continue;

    // 判据 2：相对格网最低点高度
    double height_above_local = cloud_->points[i].z
                                - grid_index_->getGridMinElevation(grid_id);
    if (height_above_local > params_.height_threshold)
        continue;

    // 判据 3：邻域坡度
    auto neighbors = grid_index_->getNeighbors(grid_id, neighbor_radius);
    std::vector<double> slopes;
    for (int nid : neighbors) slopes.push_back(calculateSlope(grid_id, nid));
    double rep_slope = computeRepresentativeSlope(slopes);

    double threshold = params_.use_adaptive
                       ? getAdaptiveThreshold(grid_id)
                       : params_.slope_threshold;

    if (rep_slope < threshold) { /* 判为地面 */ }
}
```

### 4.5 CSF 布料模拟滤波模块

CSF 是一种基于物理模拟的地面提取算法。核心思想是：将地形沿 Z 轴翻转后，让一块具有特定硬度的“布料”从上方自由下落，布料最终贴合的形态即为数字地形模型（DTM），据此可区分地面点与非地面点。

#### 4.5.1 预处理：剔除粗差并翻转点云

```cpp
std::vector<int> CSFFilter::preprocess(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud) const
{
    flipped_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < cloud_->points.size(); ++i) {
        const auto& pt = cloud_->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
            continue;

        pcl::PointXYZ flipped;
        flipped.x = pt.x;
        flipped.y = pt.y;
        flipped.z = -pt.z;   // 翻转 Z 轴
        flipped_cloud->points.push_back(flipped);
        valid_indices.push_back(static_cast<int>(i));
    }
    return valid_indices;
}
```

#### 4.5.2 碰撞格网构建

将翻转后的点云按布料分辨率划分格网，每个格网记录内部点的最大翻转高程，作为布料下落的“碰撞面”。

```cpp
void CSFFilter::buildCollisionGrid(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud)
{
    collision_grid_.assign(collision_rows_ * collision_cols_,
                           -std::numeric_limits<double>::infinity());
    for (const auto& pt : flipped_cloud->points) {
        int c = ...; int r = ...;
        int idx = r * collision_cols_ + c;
        collision_grid_[idx] = std::max(collision_grid_[idx], static_cast<double>(pt.z));
    }
}
```

#### 4.5.3 布料初始化

在点云 XY 包围盒上方初始化规则格网点，初始高程略高于点云最高点，保证布料能做自由落体。

```cpp
void CSFFilter::initCloth(double x_min, double x_max,
                          double y_min, double y_max, double top_z)
{
    cloth_cols_ = static_cast<int>(std::ceil((x_max - x_min) / res)) + 1;
    cloth_rows_ = static_cast<int>(std::ceil((y_max - y_min) / res)) + 1;

    double start_z = top_z + 2.0 * res;  // 略高于最高点
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
```

#### 4.5.4 外力与内力迭代

每次迭代包含两步：

1. **外力（重力）**：可移动节点按 `z_new = z_old - g * dt^2` 下落；若新位置低于碰撞高程，则固定为碰撞高程并标记不可移动。
2. **内力（刚性约束）**：根据硬度等级，将每个可移动节点向其 4-邻域平均高程方向拉拽，使布料保持连续。

```cpp
double CSFFilter::iterate()
{
    const double gravity = 9.8;
    const double dt = params_.csf_time_step;

    // 外力 + 碰撞检测
    for (auto& node : cloth_nodes_) {
        if (!node.movable) continue;
        double z_new = node.z - gravity * dt * dt;
        double collision_z = getCollisionHeight(r, c);
        if (z_new <= collision_z) {
            node.z = collision_z;
            node.movable = false;
        } else {
            node.z = z_new;
        }
    }

    // 内力：4-邻域平均约束
    double rigidness_factor = 0.5;  // 默认硬
    if (params_.csf_rigidness == 1) rigidness_factor = 0.1; // 软
    else if (params_.csf_rigidness == 2) rigidness_factor = 0.3; // 中

    for (size_t i = 0; i < cloth_nodes_.size(); ++i) {
        // 计算邻居平均高程并按 rigidness_factor 更新
    }
    return max_change;
}
```

主循环在最大高程变化小于收敛阈值或达到最大迭代次数时停止。

```cpp
for (int iter = 0; iter < max_iter; ++iter) {
    double max_change = iterate();
    if (max_change < convergence) break;
}
```

#### 4.5.5 地面点分类

将原始点云投影到最近布料格网点，若翻转空间高差小于 `csf_classification_threshold`，则判为地面点。

```cpp
void CSFFilter::classify(const std::vector<int>& valid_indices,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& flipped_cloud)
{
    double threshold = params_.csf_classification_threshold;
    for (size_t i = 0; i < valid_indices.size(); ++i) {
        const auto& pt = flipped_cloud->points[i];
        int node_id = findNearestNode(pt.x, pt.y);
        double diff = std::abs(static_cast<double>(pt.z) - cloth_nodes_[node_id].z);
        if (diff < threshold) {
            ground_mask_[valid_indices[i]] = true;
        }
    }
}
```

### 4.6 主窗口与 UI 集成

`MainWindow` 通过 Qt 信号槽连接“浏览”、“开始滤波”、“重置参数”、“导出结果”四个按钮。`onStartFilterClicked` 负责完整流程调度：读取参数 → 读点云 → 建格网 → 根据 `filter_method` 选择坡度滤波或 CSF → 输出统计信息。

```cpp
void MainWindow::onStartFilterClicked()
{
    // 1. 读取参数
    FilterParams params;
    params.filter_method = ui->comboBoxFilterMethod->currentIndex();
    // ... 读取坡度/CSF 参数

    // 2. 读取点云
    PointCloudIO::read(filePath.toStdString(), cloud);

    // 3. 构建格网
    auto grid = std::make_shared<GridIndex>();
    grid->buildGrid(cloud, params);

    // 4. 根据方法选择滤波器
    std::vector<int> ground_indices;
    if (params.filter_method == 1) {
        CSFFilter filter;
        filter.setParams(params);
        filter.setCloud(cloud);
        ground_indices = filter.filter();
    } else {
        SlopeFilter filter;
        filter.setParams(params);
        filter.setGridIndex(grid);
        filter.setCloud(cloud);
        ground_indices = filter.filter();
    }

    // 5. 输出统计
    QMessageBox::information(this, tr("处理完成"), msg);
}
```

## 五、参数配置说明

`FilterParams` 集中定义了全部可调参数，便于 UI 与算法模块共享：

```cpp
struct FilterParams {
    // 坡度滤波参数
    double x_step = 1.0;
    double y_step = 1.0;
    double slope_threshold = 0.1;
    int    slope_method = 0;       // 0=平均, 1=最小, 2=中值
    bool   use_adaptive = false;
    int    neighborhood = 3;       // 3x3 或 5x5
    double height_threshold = 0.5;

    // CSF 参数
    int    filter_method = 0;      // 0=坡度, 1=CSF
    double csf_cloth_resolution = 1.0;
    int    csf_max_iterations = 500;
    double csf_classification_threshold = 0.5;
    double csf_convergence_threshold = 0.005;
    int    csf_rigidness = 3;      // 1=软, 2=中, 3=硬
    double csf_time_step = 0.65;
};
```

## 六、测试结果

使用项目自带测试数据对两种算法进行验证：

- **坡度滤波**：对 `test_cloud.txt` 中 100 个规则分布点能够正确分类；对含建筑高程的倾斜地面数据，可通过 `height_threshold` 与 `slope_threshold` 组合有效剔除非地面点。
- **CSF 布料模拟滤波**：通过独立 Python 验证脚本对平坦地面、倾斜地面+高程障碍物、空输入三类场景进行测试，均得到预期结果。
- **UI 验证**：`mainwindow.ui` 经 Qt `uic` 工具编译通过，新增 CSF 控件正常生成。

## 七、实习总结

通过本次实习，完成了一个功能较完整的激光雷达点云地面滤波工具：

1. 掌握了 PCL 点云数据结构、PCD/TXT/LAS 多格式读写方法；
2. 实现了二维格网索引，理解空间划分对点云处理效率的重要性；
3. 完成了坡度滤波与 CSF 布料模拟滤波两种算法，加深了对地面点提取原理的认识；
4. 实践了 Qt 界面与算法模块的集成，熟悉了信号槽、进度条、日志输出等常用 UI 模式；
5. 体会到算法参数对结果的影响，以及代码编码规范（如 UTF-8 BOM）在多语言工程中的重要性。

后续可进一步引入 RANSAC 平面拟合、PMF（Progressive Morphological Filter）等算法，并增加点云可视化组件，以提升系统的工程实用性。
