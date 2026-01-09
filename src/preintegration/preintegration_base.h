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

#ifndef PREINTEGRATION_BASE_H
#define PREINTEGRATION_BASE_H

#include "src/common/rotation.h"
#include "src/common/types.h"

#include "integration_state.h"

#include <memory>
#include <vector>

class PreintegrationBase {

public:
    PreintegrationBase(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0, IntegrationState state);

    virtual ~PreintegrationBase() = default;

    const IntegrationState &currentState() {
        return current_state_;
    }

    const IntegrationState &deltaState() {
        return delta_state_;
    }

    double deltaTime() const {
        return delta_time_;
    }

    double startTime() const {
        return start_time_;
    }

    double endTime() const {
        return end_time_;
    }

    const Vector3d &gravity() {
        return gravity_;
    }

    const vector<IMU> &imuBuffer() {
        return imu_buffer_;
    }

    void addNewImu(const IMU &imu);
    void reintegration(IntegrationState &state);

public:
    virtual Eigen::MatrixXd evaluate(const IntegrationState &state0, const IntegrationState &state1,
                                     double *residuals) = 0;

    virtual Eigen::MatrixXd residualJacobianPose0(const IntegrationState &state0, const IntegrationState &state1,
                                                  double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianPose1(const IntegrationState &state0, const IntegrationState &state1,
                                                  double *jacobian) = 0;
    virtual Eigen::MatrixXd residualJacobianMix0(const IntegrationState &state0, const IntegrationState &state1,
                                                 double *jacobian)  = 0;
    virtual Eigen::MatrixXd residualJacobianMix1(const IntegrationState &state0, const IntegrationState &state1,
                                                 double *jacobian)  = 0;

    virtual int numResiduals()                     = 0;
    virtual std::vector<int> numBlocksParameters() = 0;

    virtual void constructState(const double *const *parameters, IntegrationState &state0,
                                IntegrationState &state1) = 0;

    virtual int imuErrorNumResiduals()                     = 0;
    virtual std::vector<int> imuErrorNumBlocksParameters() = 0;

    virtual void imuErrorEvaluate(const double *const *parameters, double *residuals) = 0;

    virtual void imuErrorJacobian(double *jacobian) = 0;

protected:
    // need compensate bias
    virtual void updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) = 0;

    virtual void resetState(const IntegrationState &state) = 0;
    virtual void integrationProcess(unsigned long index)   = 0;

    static void stateToData(const IntegrationState &state, IntegrationStateData &data);
    static void stateFromData(const IntegrationStateData &data, IntegrationState &state);

    void integration(const IMU &imu_pre, const IMU &imu_cur);

    IMU compensationBias(const IMU &imu) const;

    IMU compensationScale(const IMU &imu) const;

public:
    static constexpr int NUM_POSE = 7;

protected:
    // 待办：可以根据你的IMU设置这些参数，参数包括陀螺和加速度计的零偏标准差、比例因子标准差，ODO比例因子标准差等
    // TODO: You can set these parameters according to your IMU

    // 这些是宽泛的、保护性的工程经验上限。而不是从配置文件读取的
    // 配置文件中的imu参数包括： IMU噪声建模参数：角速度随机游走、加速度随机游走、陀螺零偏稳定性标准差、加速度零偏稳定性标准差、相关时间、odo比例因子误差
    // 这里的参数是经验上限，用于预积分过程中的数值稳定性；注意区分两者区别和用途。详细见MyNotes.md中的说明
    static constexpr double IMU_GRY_BIAS_STD = 7200 / 3600.0 * M_PI / 180.0; // 7200 deg / hr  2 deg/s   imu陀螺零偏误差放大
    static constexpr double IMU_ACC_BIAS_STD = 2.0e4 * 1.0e-5;               // 20000 mGal = 0.2 m/s²    imu加速度计零偏误差放大
    static constexpr double IMU_SCALE_STD    = 5.0e3 * 1.0e-6;               // 5000 PPM  = 0.005  0.5%  imu比例因子误差放大
    static constexpr double IMU_ACC_Z_SCALE  = 100; // 100倍加速度计Z轴比例因子误差放大，增强数值稳定性
    static constexpr double ODO_SCALE_STD    = 2.0e4 * 1.0e-6; // 0.02 2% 20000PM  里程计比例因子误差放大
    /* PS： 
        1.要合理设置噪声参数，std最好是常值零偏(IMU 手册里给的零偏不稳定性（Bias Instability）)的3倍以上。
        2.约束零偏常量大小，相当于先验约束，对系统初始化的稳定性有较好的作用，在边缘化之后有了先验约束，这个约束的作用将会减小。
        3.这个约束因子和GVINS中的GNSS/INS紧组合优化的接收机钟差因子，功能非常类似，都是有利于系统初始化。 */


    const std::shared_ptr<IntegrationParameters> parameters_;

    IntegrationState current_state_;
    IntegrationState delta_state_;

    vector<IMU> imu_buffer_;
    double delta_time_{0};
    double start_time_;
    double end_time_;

    Vector3d gravity_;

    Eigen::MatrixXd jacobian_, covariance_;
    Eigen::MatrixXd noise_;
    Eigen::MatrixXd sqrt_information_;

    Quaterniond corrected_q_;
    Vector3d corrected_p_, corrected_v_;
};

#endif // PREINTEGRATION_BASE_H