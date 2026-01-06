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

#ifndef IMUFILELOADER_H
#define IMUFILELOADER_H

#include "fileloader.h"
#include "src/common/types.h"

class ImuFileLoader : public FileLoader {

public:
    ImuFileLoader() = delete;
    ImuFileLoader(const string &filename, int columns, int rate = 200) {
        open(filename, columns, FileLoader::TEXT);

        dt_ = 1.0 / (double) rate; // 预设时间间隔

        imu_.time = 0;
    }

    const IMU &next() {
        imu_pre_ = imu_; // 保存上一个IMU数据
 
        data_ = load(); // 读取一行数据

        imu_.time = data_[0];
        memcpy(imu_.dtheta.data(), &data_[1], 3 * sizeof(double)); // 角增量
        memcpy(imu_.dvel.data(), &data_[4], 3 * sizeof(double)); // 速度增量

        double dt = imu_.time - imu_pre_.time; // 计算时间间隔
        if (dt < 0.1) {  // 如果时间间隔正常，则使用计算的时间间隔
            imu_.dt = dt;
        } else {    // 如果时间间隔异常，则使用预设的时间间隔
            imu_.dt = dt_;
        }

        // 增量形式
        if (columns_ == 8) { // 8列数据时，最后一列为里程计增量
            imu_.odovel = data_[7] * imu_.dt;
        } else if (columns_ == 9) { // 9列数据时，最后两列为前后轮里程计增量，取平均值
            imu_.odovel = 0.5 * (data_[7] + data_[8]) * imu_.dt;
        }

        return imu_;
    }

private:
    double dt_;

    IMU imu_, imu_pre_;
    vector<double> data_;
};

#endif // IMUFILELOADER_H
