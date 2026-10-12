#pragma execution_character_set("utf-8")

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QTextCodec>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "FilterParams.h"
#include "GridIndex.h"
#include "SlopeFilter.h"
#include "CSFFilter.h"
#include "PointCloudIO.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 设置应用程序-wide的编码
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
#endif

    connect(ui->btnBrowse, &QPushButton::clicked, this, &MainWindow::onBrowseButtonClicked);
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::onStartFilterClicked);
    connect(ui->btnReset, &QPushButton::clicked, this, &MainWindow::onResetParamsClicked);
    connect(ui->btnExport, &QPushButton::clicked, this, &MainWindow::onExportResultClicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onBrowseButtonClicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("选择点云文件"),
        "D:/20260517_lidar_data_p/DATA",
        tr("点云文件 (*.las *.laz *.pcd *.txt)"));

    if (!fileName.isEmpty()) {
        ui->lineEditFile->setText(fileName);
    }
}

void MainWindow::onStartFilterClicked()
{
    if (ui->lineEditFile->text().isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先选择点云文件！"));
        return;
    }

    ui->btnStart->setEnabled(false);
    ui->btnBrowse->setEnabled(false);
    ui->progressBar->setValue(0);
    ui->textEditLog->clear();

    // 从GUI读取所有参数（修复：新增 height_threshold）
    FilterParams params;
    params.x_step = ui->doubleSpinBoxXStep->value();
    params.y_step = ui->doubleSpinBoxYStep->value();
    params.slope_threshold = ui->doubleSpinBoxThreshold->value();
    params.height_threshold = ui->doubleSpinBoxHeightThreshold->value();  // 新增
    params.slope_method = ui->comboBoxMethod->currentIndex();
    params.use_adaptive = ui->checkBoxAdaptive->isChecked();
    params.neighborhood = ui->comboBoxNeighborhood->currentText() == tr("3x3") ? 3 : 5;

    // CSF 参数
    params.filter_method = ui->comboBoxFilterMethod->currentIndex();
    params.csf_cloth_resolution = ui->doubleSpinBoxCSFResolution->value();
    params.csf_max_iterations = ui->spinBoxCSFMaxIter->value();
    params.csf_classification_threshold = ui->doubleSpinBoxCSFThreshold->value();
    params.csf_rigidness = ui->spinBoxCSFRigidness->value();
    params.csf_time_step = ui->doubleSpinBoxCSFTimeStep->value();

    QString filePath = ui->lineEditFile->text();
    auto start_total = std::chrono::high_resolution_clock::now();

    ui->textEditLog->append(tr("[开始] 读取点云文件..."));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!PointCloudIO::read(filePath.toStdString(), cloud)) {
        ui->textEditLog->append(tr("[错误] 读取失败: %1").arg(QString::fromStdString(PointCloudIO::getLastError())));
        ui->btnStart->setEnabled(true);
        ui->btnBrowse->setEnabled(true);
        return;
    }
    ui->progressBar->setValue(20);
    ui->textEditLog->append(tr("[完成] 读取点云，共 %1 个点").arg(cloud->points.size()));

    ui->textEditLog->append(tr("[开始] 构建格网索引..."));
    auto grid = std::make_shared<GridIndex>();
    if (!grid->buildGrid(cloud, params)) {
        ui->textEditLog->append(tr("[错误] 格网构建失败！"));
        ui->btnStart->setEnabled(true);
        ui->btnBrowse->setEnabled(true);
        return;
    }
    ui->progressBar->setValue(40);
    ui->textEditLog->append(tr("[完成] 格网构建，共 %1 个格网").arg(grid->getTotalGrids()));

    std::vector<int> ground_indices;
    if (params.filter_method == 1) {
        ui->textEditLog->append(tr("[开始] CSF 布料模拟滤波..."));
        CSFFilter filter;
        filter.setParams(params);
        filter.setCloud(cloud);
        ground_indices = filter.filter();
    }
    else {
        ui->textEditLog->append(tr("[开始] 坡度滤波..."));
        SlopeFilter filter;
        filter.setParams(params);
        filter.setGridIndex(grid);
        filter.setCloud(cloud);
        ground_indices = filter.filter();
    }

    ui->progressBar->setValue(80);
    ui->textEditLog->append(tr("[完成] 滤波完成，地面点: %1，地物点: %2")
        .arg(ground_indices.size()).arg(cloud->points.size() - ground_indices.size()));

    auto end_total = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(end_total - start_total).count();

    ui->progressBar->setValue(100);

    // 构建消息，避免复杂的.arg()嵌套
    double groundRatio = ground_indices.size() * 100.0 / cloud->points.size();
    QString totalStr = QString::number(cloud->points.size());
    QString groundStr = QString::number(ground_indices.size());
    QString ratioStr = QString::number(groundRatio, 'f', 2);
    QString nonGroundStr = QString::number(cloud->points.size() - ground_indices.size());
    QString timeStr = QString::number(total_time / 1000.0, 'f', 2);

    QString msg = tr("滤波完成！\n总点数: %1\n地面点: %2 (%3%)\n地物点: %4\n耗时: %5 秒")
        .arg(totalStr).arg(groundStr).arg(ratioStr).arg(nonGroundStr).arg(timeStr);

    QMessageBox::information(this, tr("处理完成"), msg);

    ui->btnStart->setEnabled(true);
    ui->btnBrowse->setEnabled(true);
}

void MainWindow::onResetParamsClicked()
{
    ui->doubleSpinBoxXStep->setValue(1.0);
    ui->doubleSpinBoxYStep->setValue(1.0);
    ui->doubleSpinBoxThreshold->setValue(0.1);
    ui->doubleSpinBoxHeightThreshold->setValue(0.5);  // 新增
    ui->comboBoxMethod->setCurrentIndex(0);
    ui->checkBoxAdaptive->setChecked(false);
    ui->comboBoxNeighborhood->setCurrentIndex(0);

    // 重置 CSF 参数
    ui->comboBoxFilterMethod->setCurrentIndex(0);
    ui->doubleSpinBoxCSFResolution->setValue(1.0);
    ui->spinBoxCSFMaxIter->setValue(500);
    ui->doubleSpinBoxCSFThreshold->setValue(0.5);
    ui->spinBoxCSFRigidness->setValue(3);
    ui->doubleSpinBoxCSFTimeStep->setValue(0.65);

    ui->textEditLog->clear();
    ui->progressBar->setValue(0);
}

void MainWindow::onExportResultClicked()
{
    if (ui->lineEditFile->text().isEmpty()) {
        QMessageBox::warning(this, tr("警告"), tr("请先选择点云文件并执行滤波！"));
        return;
    }

    QString defaultPath = "D:/20260517_lidar_data_p/DATA/result_ground.las";

    QString fileName = QFileDialog::getSaveFileName(this,
        tr("导出滤波结果"),
        defaultPath,
        tr("LAS文件 (*.las);;PCD文件 (*.pcd);;文本文件 (*.txt)"));

    if (fileName.isEmpty()) {
        ui->textEditLog->append(tr("[取消] 用户取消导出"));
        return;
    }

    ui->textEditLog->append(tr("[开始] 导出结果..."));
    ui->textEditLog->append(tr("保存路径: %1").arg(fileName));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!PointCloudIO::read(ui->lineEditFile->text().toStdString(), cloud)) {
        ui->textEditLog->append(tr("[错误] 读取原始文件失败！"));
        return;
    }

    // 从GUI读取所有参数（修复：新增 height_threshold）
    FilterParams params;
    params.x_step = ui->doubleSpinBoxXStep->value();
    params.y_step = ui->doubleSpinBoxYStep->value();
    params.slope_threshold = ui->doubleSpinBoxThreshold->value();
    params.height_threshold = ui->doubleSpinBoxHeightThreshold->value();  // 新增
    params.slope_method = ui->comboBoxMethod->currentIndex();
    params.use_adaptive = ui->checkBoxAdaptive->isChecked();
    params.neighborhood = ui->comboBoxNeighborhood->currentText() == tr("3x3") ? 3 : 5;

    // CSF 参数
    params.filter_method = ui->comboBoxFilterMethod->currentIndex();
    params.csf_cloth_resolution = ui->doubleSpinBoxCSFResolution->value();
    params.csf_max_iterations = ui->spinBoxCSFMaxIter->value();
    params.csf_classification_threshold = ui->doubleSpinBoxCSFThreshold->value();
    params.csf_rigidness = ui->spinBoxCSFRigidness->value();
    params.csf_time_step = ui->doubleSpinBoxCSFTimeStep->value();

    ui->textEditLog->append(tr("[处理] 重新执行滤波..."));

    std::vector<bool> mask;
    if (params.filter_method == 1) {
        CSFFilter filter;
        filter.setParams(params);
        filter.setCloud(cloud);
        filter.filter();
        mask = filter.getGroundMask();
    }
    else {
        auto grid = std::make_shared<GridIndex>();
        if (!grid->buildGrid(cloud, params)) {
            ui->textEditLog->append(tr("[错误] 格网构建失败！"));
            return;
        }

        SlopeFilter filter;
        filter.setParams(params);
        filter.setGridIndex(grid);
        filter.setCloud(cloud);
        filter.filter();
        mask = filter.getGroundMask();
    }

    if (PointCloudIO::writeClassified(fileName.toStdString(), cloud, mask)) {
        ui->textEditLog->append(tr("[完成] 结果已导出: %1").arg(fileName));
        QMessageBox::information(this, tr("导出成功"),
            tr("滤波结果已成功导出到:\n%1").arg(fileName));
    }
    else {
        ui->textEditLog->append(tr("[错误] 导出失败: %1")
            .arg(QString::fromStdString(PointCloudIO::getLastError())));
        QMessageBox::critical(this, tr("导出失败"),
            tr("保存失败，请检查路径和权限。\n错误信息: %1")
            .arg(QString::fromStdString(PointCloudIO::getLastError())));
    }
}