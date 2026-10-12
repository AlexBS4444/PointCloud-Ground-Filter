#pragma once
#ifndef POINT_CLOUD_IO_H
#define POINT_CLOUD_IO_H

#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/**
 * @brief 点云IO工具类
 * 封装PCL和LASLib的读写功能
 */
class PointCloudIO {
public:
    PointCloudIO();
    ~PointCloudIO();

    /**
     * @brief 读取点云文件（自动识别格式）
     * @param filename 文件路径
     * @param cloud 输出点云
     * @return true-成功, false-失败
     */
    static bool read(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 保存点云文件（自动识别格式）
     * @param filename 文件路径
     * @param cloud 输入点云
     * @return true-成功, false-失败
     */
    static bool write(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 按分类导出点云（分离地面点和非地面点）
     * @param filename 文件路径
     * @param cloud 原始点云
     * @param ground_mask 地面点掩码（true=地面）
     * @return true-成功, false-失败
     */
    static bool writeClassified(const std::string& filename,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<bool>& ground_mask);

    /**
     * @brief 获取文件扩展名（小写）
     */
    static std::string getExtension(const std::string& filename);

    /**
     * @brief 获取错误信息
     */
    static std::string getLastError();

private:
    // PCD格式
    static bool readPCD(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static bool writePCD(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // TXT格式（简单XYZ文本）
    static bool readTXT(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static bool writeTXT(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // LAS/LAZ格式（使用LASLib）
    static bool readLAS(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static bool writeLAS(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    static std::string last_error_;
};

#endif // POINT_CLOUD_IO_H