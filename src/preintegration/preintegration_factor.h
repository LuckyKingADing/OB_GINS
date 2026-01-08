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

#ifndef PREINTEGRATION_FACTOR_H
#define PREINTEGRATION_FACTOR_H

#include "preintegration_base.h"

#include <ceres/ceres.h>

class PreintegrationFactor : public ceres::CostFunction { // 继承自Ceres的CostFunction基类，代价函数

public:
    PreintegrationFactor() = delete;

    explicit PreintegrationFactor(std::shared_ptr<PreintegrationBase> preintegration)
        : preintegration_(std::move(preintegration)) {

        // parameter： 查询参数块维度，std::vector<int>{NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX};
        *mutable_parameter_block_sizes() = preintegration_->numBlocksParameters();

        // residual：查询残差维度
        set_num_residuals(preintegration_->numResiduals());
    }

    // !!!核心函数：预积分因子
    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // 构建状态量
        // construct state
        IntegrationState state0, state1;
            // 将参数块parameters提取赋值到两个时刻的状态量state0和state1
        preintegration_->constructState(parameters, state0, state1);

        // !!!硬核：计算残差residual
        preintegration_->evaluate(state0, state1, residuals);

        // 计算雅可比
        // 这里分别计算残差对 4 个参数块的偏导数。
        if (jacobians) {
            if (jacobians[0]) { // jacobians[0] 对应第一个参数块 Pose0 (上一帧位姿)
                preintegration_->residualJacobianPose0(state0, state1, jacobians[0]);
            }
            if (jacobians[1]) { // jacobians[1] 对应第二个参数块 Mix0 (上一帧混合状态)
                preintegration_->residualJacobianMix0(state0, state1, jacobians[1]);
            }
            if (jacobians[2]) { // jacobians[2] 对应第三个参数块 Pose1 (当前帧位姿)
                preintegration_->residualJacobianPose1(state0, state1, jacobians[2]);
            }
            if (jacobians[3]) { // jacobians[3] 对应第四个参数块 Mix1 (当前帧混合状态)
                preintegration_->residualJacobianMix1(state0, state1, jacobians[3]);
            }
        }

        return true;
    }

private:
    std::shared_ptr<PreintegrationBase> preintegration_;
};

#endif // PREINTEGRATION_FACTOR_H
