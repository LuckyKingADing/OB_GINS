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

#ifndef GNSSFILELOADER_H
#define GNSSFILELOADER_H

#include "fileloader.h"
#include "src/common/angle.h"
#include "src/common/types.h"

class GnssFileLoader : public FileLoader {

public:
    GnssFileLoader() = delete;
    explicit GnssFileLoader(const string &filename, int columns = 7) {
        open(filename, columns, FileLoader::TEXT);
    }

    const GNSS &next() {
        data_ = load(); // load()表示读取一行数据

        gnss_.time = data_[0];
        memcpy(gnss_.blh.data(), &data_[1], 3 * sizeof(double)); // 从data_的第1个元素开始拷贝3个double到gnss_.blh，表示纬度、经度、高度

        // 13列GNSS文件包含GNSS速度；7列GNSS文件不包含GNSS速度
        if (data_.size() == 7) {
            memcpy(gnss_.std.data(), &data_[4], 3 * sizeof(double)); // 从data_的第4个元素开始拷贝3个double到gnss_.std
        } else {
            memcpy(gnss_.std.data(), &data_[7], 3 * sizeof(double)); // 从data_的第7个元素开始拷贝3个double到gnss_.std，因为前面有速度信息
        }
        gnss_.blh[0] *= D2R;
        gnss_.blh[1] *= D2R;

        return gnss_;
    }

private:
    GNSS gnss_;
    vector<double> data_;
};

#endif // GNSSFILELOADER_H
