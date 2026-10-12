#include "PointCloudIO.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cctype>

// PCL
#include <pcl/io/pcd_io.h>

// LASLib
#include "lasreader.hpp"
#include "laswriter.hpp"

std::string PointCloudIO::last_error_ = "";

PointCloudIO::PointCloudIO()
{
}

PointCloudIO::~PointCloudIO()
{
}

std::string PointCloudIO::getExtension(const std::string& filename)
{
    size_t pos = filename.find_last_of('.');
    if (pos == std::string::npos) {
        return "";
    }
    std::string ext = filename.substr(pos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}

std::string PointCloudIO::getLastError()
{
    return last_error_;
}

bool PointCloudIO::read(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string ext = getExtension(filename);

    if (ext == "pcd") {
        return readPCD(filename, cloud);
    }
    else if (ext == "txt" || ext == "xyz") {
        return readTXT(filename, cloud);
    }
    else if (ext == "las" || ext == "laz") {
        return readLAS(filename, cloud);
    }
    else {
        last_error_ = "不支持的文件格式: " + ext;
        return false;
    }
}

bool PointCloudIO::write(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::string ext = getExtension(filename);

    if (ext == "pcd") {
        return writePCD(filename, cloud);
    }
    else if (ext == "txt" || ext == "xyz") {
        return writeTXT(filename, cloud);
    }
    else if (ext == "las" || ext == "laz") {
        return writeLAS(filename, cloud);
    }
    else {
        last_error_ = "不支持的文件格式: " + ext;
        return false;
    }
}

bool PointCloudIO::writeClassified(const std::string& filename,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<bool>& ground_mask)
{
    // 第一步：分离地面点和非地面点（所有格式统一处理）
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGround(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if (i < ground_mask.size() && ground_mask[i]) {
            ground->points.push_back(cloud->points[i]);
        }
        else {
            nonGround->points.push_back(cloud->points[i]);
        }
    }

    ground->width = ground->points.size(); ground->height = 1;
    nonGround->width = nonGround->points.size(); nonGround->height = 1;

    std::string ext = getExtension(filename);
    std::string base = filename.substr(0, filename.find_last_of('.'));

    if (ext == "las" || ext == "laz") {
        // LAS格式：分别导出地面点和非地面点
        bool ok1 = writeLAS(base + "_ground.las", ground);
        bool ok2 = writeLAS(base + "_nonGround.las", nonGround);
        if (ok1 && ok2) {
            last_error_ = "已分别保存为 " + base + "_ground.las 和 " + base + "_nonGround.las";
            return true;
        }
        return false;
    }
    else if (ext == "pcd") {
        bool ok1 = writePCD(base + "_ground.pcd", ground);
        bool ok2 = writePCD(base + "_nonGround.pcd", nonGround);
        if (ok1 && ok2) {
            last_error_ = "已分别保存为 " + base + "_ground.pcd 和 " + base + "_nonGround.pcd";
            return true;
        }
        return false;
    }
    else if (ext == "txt") {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            last_error_ = "无法打开文件: " + filename;
            return false;
        }

        ofs << "# X Y Z Class(0=NonGround, 1=Ground)" << std::endl;
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            int cls = (i < ground_mask.size() && ground_mask[i]) ? 1 : 0;
            ofs << cloud->points[i].x << " "
                << cloud->points[i].y << " "
                << cloud->points[i].z << " "
                << cls << std::endl;
        }
        ofs.close();
        return true;
    }
    else {
        last_error_ = "不支持的文件格式: " + ext;
        return false;
    }
}

// ========== PCD ==========
bool PointCloudIO::readPCD(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        last_error_ = "无法读取PCD文件: " + filename;
        return false;
    }
    return true;
}

bool PointCloudIO::writePCD(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (pcl::io::savePCDFileASCII(filename, *cloud) == -1) {
        last_error_ = "无法保存PCD文件: " + filename;
        return false;
    }
    return true;
}

// ========== TXT ==========
bool PointCloudIO::readTXT(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        last_error_ = "无法打开文件: " + filename;
        return false;
    }

    cloud->clear();
    std::string line;
    int line_num = 0;

    while (std::getline(ifs, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        pcl::PointXYZ pt;
        if (iss >> pt.x >> pt.y >> pt.z) {
            cloud->points.push_back(pt);
        }
    }

    ifs.close();
    cloud->width = cloud->points.size();
    cloud->height = 1;

    if (cloud->empty()) {
        last_error_ = "文件为空或格式错误: " + filename;
        return false;
    }
    return true;
}

bool PointCloudIO::writeTXT(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        last_error_ = "无法打开文件: " + filename;
        return false;
    }

    ofs << "# X Y Z" << std::endl;
    for (const auto& pt : cloud->points) {
        ofs << pt.x << " " << pt.y << " " << pt.z << std::endl;
    }
    ofs.close();
    return true;
}

// ========== LAS/LAZ ==========
bool PointCloudIO::readLAS(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    LASreadOpener lasreadopener;
    lasreadopener.set_file_name(filename.c_str());

    LASreader* lasreader = lasreadopener.open(false);

    if (lasreader == nullptr) {
        last_error_ = "无法打开LAS文件: " + filename;
        return false;
    }

    cloud->clear();
    while (lasreader->read_point()) {
        pcl::PointXYZ pt;
        pt.x = lasreader->point.get_x();
        pt.y = lasreader->point.get_y();
        pt.z = lasreader->point.get_z();
        cloud->points.push_back(pt);
    }

    lasreader->close();
    delete lasreader;

    cloud->width = cloud->points.size();
    cloud->height = 1;
    return true;
}

bool PointCloudIO::writeLAS(const std::string& filename,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    if (cloud->empty()) {
        last_error_ = "点云为空，无法写入LAS文件";
        return false;
    }

    LASwriteOpener laswriteopener;
    laswriteopener.set_file_name(filename.c_str());

    LASheader header;
    header.point_data_format = 1;
    header.point_data_record_length = 28;

    // 计算边界
    double x_min = std::numeric_limits<double>::max();
    double y_min = std::numeric_limits<double>::max();
    double z_min = std::numeric_limits<double>::max();
    double x_max = std::numeric_limits<double>::lowest();
    double y_max = std::numeric_limits<double>::lowest();
    double z_max = std::numeric_limits<double>::lowest();

    for (const auto& pt : cloud->points) {
        x_min = std::min(x_min, static_cast<double>(pt.x));
        y_min = std::min(y_min, static_cast<double>(pt.y));
        z_min = std::min(z_min, static_cast<double>(pt.z));
        x_max = std::max(x_max, static_cast<double>(pt.x));
        y_max = std::max(y_max, static_cast<double>(pt.y));
        z_max = std::max(z_max, static_cast<double>(pt.z));
    }

    header.min_x = x_min; header.max_x = x_max;
    header.min_y = y_min; header.max_y = y_max;
    header.min_z = z_min; header.max_z = z_max;

    // 设置偏移和比例因子（关键！保证坐标精度）
    header.x_offset = x_min;
    header.y_offset = y_min;
    header.z_offset = z_min;
    header.x_scale_factor = 0.001;
    header.y_scale_factor = 0.001;
    header.z_scale_factor = 0.001;

    LASwriter* laswriter = laswriteopener.open(&header);

    if (laswriter == nullptr) {
        last_error_ = "无法创建LAS文件: " + filename;
        return false;
    }

    // 预分配 point 并只 init 一次
    LASpoint point;
    if (!point.init(&header, header.point_data_format, header.point_data_record_length, nullptr)) {
        last_error_ = "LASpoint 初始化失败";
        laswriter->close();
        delete laswriter;
        return false;
    }

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        point.zero();  // 清零之前的数据

        point.set_x(cloud->points[i].x);
        point.set_y(cloud->points[i].y);
        point.set_z(cloud->points[i].z);
        point.set_classification(2);  // 地面点标记为2

        laswriter->write_point(&point);
        laswriter->update_inventory(&point);
    }

    laswriter->update_header(&header, TRUE);
    laswriter->close();
    delete laswriter;

    return true;
}