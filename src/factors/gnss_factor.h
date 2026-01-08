/*
 * OB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GNSS_FACTOR_H
#define GNSS_FACTOR_H

#include <Eigen/Geometry>
#include <ceres/ceres.h>

#include "src/common/rotation.h"
#include "src/common/types.h"

class GnssFactor : public ceres::SizedCostFunction<3, 7> {

public:
    explicit GnssFactor(GNSS gnss, Vector3d lever)
        : gnss_(std::move(gnss))
        , lever_(std::move(lever)) {
    }

    void updateGnssState(const GNSS &gnss) {
        gnss_ = gnss;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // 提取状态量，从 parameters 中提取位姿信息；parameters 是什么：是 statedatalist[i].pose 的地址。parameters[0][0] ~ [0][6]：就是当前这帧的 `px, py, pz, qw, qx
        Vector3d p{parameters[0][0], parameters[0][1], parameters[0][2]};
        Quaterniond q{parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]};

        // 计算残差
            /* 利用w系下IMU的初始位置改正杆臂误差，转换得到w系下GNSS的NED坐标计算值，再根据文件读取并转换的GNSS的NED坐标读取值，计算残差并乘以权阵的平方根。*/

        Eigen::Map<Eigen::Matrix<double, 3, 1>> error(residuals);
            // 误差 = (IMU位置 + 杆臂补偿) - GNSS测量位置
        error = p + q.toRotationMatrix() * lever_ - gnss_.blh;

            // 加权
            // 误差需要除以标准差 (sigma)，这也叫白化 (Whitening)。
            // 测量越准，sigma越小，weight越大，对优化的影响越大。
        Matrix3d weight = Matrix3d::Zero();
        weight(0, 0)    = 1.0 / gnss_.std[0];
        weight(1, 1)    = 1.0 / gnss_.std[1];
        weight(2, 2)    = 1.0 / gnss_.std[2];

        error = weight * error;

        // 雅可比矩阵：包括两个部分：位置参数、姿态参数的系数
        if (jacobians) { // 如果雅可比矩阵不为空，说明需要计算雅可比矩阵
            if (jacobians[0]) { // 如果第一个参数块的雅可比矩阵不为空，说明需要计算位置和姿态的雅可比矩阵；第一个参数块是位姿参数块

                Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian_pose(jacobians[0]);

                jacobian_pose.setZero();

                // 对位置 p 的导数是 Identity (因为 r = p + ...)
                jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();

                // 对旋转 theta 的导数（即四元数 q）的导数是 -R * [lever]_x
                jacobian_pose.block<3, 3>(0, 3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);

                // 加权
                jacobian_pose = weight * jacobian_pose;
            }
        }
        /* PS：
        1.我们在参数块里定义了 7 个变量（3位置+4四元数）。但是在写导数时，我们实际上是针对“3位置+3旋转失量”来写的（共6维）。
        前3列 (0,1,2) 填了位置导数。
        中间3列 (3,4,5) 填了旋转导数。
        第7列 (6)，也就是四元数的实部w，w对应的导数，被置为0了（或者说在流形切空间中不存在这一维）。
        这是因为四元数有单位模长约束，实际自由度只有3。*/

        return true;
    }

private:
    GNSS gnss_;
    Vector3d lever_;
};

#endif // GNSS_FACTOR_H
