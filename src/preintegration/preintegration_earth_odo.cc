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

#include "preintegration_earth_odo.h"
#include "src/common/earth.h"

PreintegrationEarthOdo::PreintegrationEarthOdo(std::shared_ptr<IntegrationParameters> parameters, const IMU &imu0,
                                               IntegrationState state)
    : PreintegrationBase(std::move(parameters), imu0, std::move(state)) {

    // 初始化delta_state_、jacobian_、covariance_
    // Reset state 
    resetState(current_state_, NUM_STATE);

    // 设置初始噪声矩阵：包括从配置文件读取后赋值在parameters_中的噪声参数，包括：随机游走噪声、零偏噪声等
    // Set initial noise matrix
    setNoiseMatrix();

    // 里程计参数：从配置参数中读取安装角和杆臂
    cvb_  = Rotation::euler2matrix(parameters_->abv).transpose();
    lodo_ = parameters_->lodo;
}

Eigen::MatrixXd PreintegrationEarthOdo::evaluate(const IntegrationState &state0, const IntegrationState &state1,
                                                 double *residuals) {
    sqrt_information_ =
        Eigen::LLT<Eigen::Matrix<double, NUM_STATE, NUM_STATE>>(covariance_.inverse()).matrixL().transpose();

    Eigen::Map<Eigen::Matrix<double, NUM_STATE, 1>> residual(residuals);

    Matrix3d dp_dbg   = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba   = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg   = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba   = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg   = jacobian_.block<3, 3>(6, 9);
    Vector3d ds_dsodo = jacobian_.block<3, 1>(15, 18);
    Matrix3d ds_dbg   = jacobian_.block<3, 3>(15, 9);

    // 零偏误差
    Vector3d dbg = state0.bg - delta_state_.bg;
    Vector3d dba = state0.ba - delta_state_.ba;
    double dsodo = state0.sodo - delta_state_.sodo;

    // 位置补偿项
    Vector3d p_cor{0, 0, 0};
    for (const auto &pn : pn_) {
        p_cor += (pn.second - state0.p) * pn.first;
    }
    p_cor = 2.0 * iewn_skew_ * p_cor;

    // 速度补偿项
    Vector3d v_cor;
    v_cor = 2.0 * iewn_skew_ * (state1.p - state0.p);

    // 姿态
    Vector3d dnn    = -iewn_ * delta_time_;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn);

    dpn_ = state1.p - state0.p - state0.v * delta_time_ - 0.5 * gravity_ * delta_time_ * delta_time_ + p_cor;
    dvn_ = state1.v - state0.v - gravity_ * delta_time_ + v_cor;

    // 积分校正
    corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
    corrected_v_ = delta_state_.v + dv_dba * dba + dv_dbg * dbg;
    corrected_q_ = delta_state_.q * Rotation::rotvec2quaternion(dq_dbg * dbg);
    corrected_s_ = delta_state_.s + ds_dbg * dbg + ds_dsodo * dsodo;

    Quaterniond qnb0 = state0.q.inverse();
    Matrix3d cnb0    = qnb0.toRotationMatrix();
    qb0b1_           = state1.q.inverse() * qnn * state0.q;

    // Residuals
    residual.block<3, 1>(0, 0)  = cnb0 * dpn_ - corrected_p_;
    residual.block<3, 1>(3, 0)  = cnb0 * dvn_ - corrected_v_;
    residual.block<3, 1>(6, 0)  = 2 * (qb0b1_ * corrected_q_).vec();
    residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
    residual.block<3, 1>(12, 0) = state1.ba - state0.ba;
    residual.block<3, 1>(15, 0) = cnb0 * (state1.p - state0.p) - corrected_s_;
    residual(18)                = state1.sodo - state0.sodo;

    residual = sqrt_information_ * residual;

    return residual;
}

Eigen::MatrixXd PreintegrationEarthOdo::residualJacobianPose0(const IntegrationState &state0,
                                                              const IntegrationState &state1, double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();

    jaco.block(0, 0, 3, 3) = -cnb0 - 2.0 * cnb0 * iewn_skew_ * delta_time_;
    jaco.block(0, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dpn_);
    jaco.block(3, 0, 3, 3) = -2.0 * cnb0 * iewn_skew_;
    jaco.block(3, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * dvn_);
    jaco.block(6, 3, 3, 3) =
        (Rotation::quaternionleft(qb0b1_) * Rotation::quaternionright(corrected_q_)).bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = -cnb0;
    jaco.block(15, 3, 3, 3) = Rotation::skewSymmetric(cnb0 * (state1.p - state0.p));

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationEarthOdo::residualJacobianPose1(const IntegrationState &state0,
                                                              const IntegrationState &state1, double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_POSE, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();

    jaco.block(0, 0, 3, 3)  = cnb0;
    jaco.block(3, 0, 3, 3)  = 2.0 * cnb0 * iewn_skew_;
    jaco.block(6, 3, 3, 3)  = -Rotation::quaternionright(qb0b1_ * corrected_q_).bottomRightCorner<3, 3>();
    jaco.block(15, 0, 3, 3) = cnb0;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationEarthOdo::residualJacobianMix0(const IntegrationState &state0,
                                                             const IntegrationState &state1, double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    Matrix3d dp_dbg   = jacobian_.block<3, 3>(0, 9);
    Matrix3d dp_dba   = jacobian_.block<3, 3>(0, 12);
    Matrix3d dv_dbg   = jacobian_.block<3, 3>(3, 9);
    Matrix3d dv_dba   = jacobian_.block<3, 3>(3, 12);
    Matrix3d dq_dbg   = jacobian_.block<3, 3>(6, 9);
    Vector3d ds_dsodo = jacobian_.block<3, 1>(15, 18);
    Matrix3d ds_dbg   = jacobian_.block<3, 3>(15, 9);

    Matrix3d cnb0 = state0.q.inverse().toRotationMatrix();

    jaco.block(0, 0, 3, 3)  = -cnb0 * delta_time_;
    jaco.block(0, 3, 3, 3)  = -dp_dbg;
    jaco.block(0, 6, 3, 3)  = -dp_dba;
    jaco.block(3, 0, 3, 3)  = -cnb0;
    jaco.block(3, 3, 3, 3)  = -dv_dbg;
    jaco.block(3, 6, 3, 3)  = -dv_dba;
    jaco.block(6, 3, 3, 3)  = Rotation::quaternionleft(qb0b1_ * delta_state_.q).bottomRightCorner<3, 3>() * dq_dbg;
    jaco.block(9, 3, 3, 3)  = -Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = -Eigen::Matrix3d::Identity();
    jaco.block(15, 3, 3, 3) = -ds_dbg;
    jaco.block(15, 9, 3, 1) = -ds_dsodo;
    jaco(18, 9)             = -1.0;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

Eigen::MatrixXd PreintegrationEarthOdo::residualJacobianMix1(const IntegrationState &state0,
                                                             const IntegrationState &state1, double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_STATE, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);
    jaco.setZero();

    jaco.block(3, 0, 3, 3)  = state0.q.inverse().toRotationMatrix();
    jaco.block(9, 3, 3, 3)  = Eigen::Matrix3d::Identity();
    jaco.block(12, 6, 3, 3) = Eigen::Matrix3d::Identity();
    jaco(18, 9)             = 1.0;

    jaco = sqrt_information_ * jaco;
    return jaco;
}

int PreintegrationEarthOdo::numResiduals() {
    return NUM_STATE; // 15 维（位置3 + 速度3 + 姿态3 + 陀螺零偏3 + 加计零偏3
}

vector<int> PreintegrationEarthOdo::numBlocksParameters() {
    return std::vector<int>{NUM_POSE, NUM_MIX, NUM_POSE, NUM_MIX};
}

IntegrationStateData PreintegrationEarthOdo::stateToData(const IntegrationState &state) {
    IntegrationStateData data;
    PreintegrationBase::stateToData(state, data);
    data.mix[9] = state.sodo; // 单独存储里程计尺度因子sodo

    return data;
}

IntegrationState PreintegrationEarthOdo::stateFromData(const IntegrationStateData &data) {
    IntegrationState state;
    PreintegrationBase::stateFromData(data, state);
    state.sodo = data.mix[9];

    return state;
}

void PreintegrationEarthOdo::constructState(const double *const *parameters, IntegrationState &state0,
                                            IntegrationState &state1) {
    state0 = IntegrationState{
        .p    = {parameters[0][0], parameters[0][1], parameters[0][2]},
        .q    = {parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]},
        .v    = {parameters[1][0], parameters[1][1], parameters[1][2]},
        .bg   = {parameters[1][3], parameters[1][4], parameters[1][5]},
        .ba   = {parameters[1][6], parameters[1][7], parameters[1][8]},
        .sodo = parameters[1][9],
    };

    state1 = IntegrationState{
        .p    = {parameters[2][0], parameters[2][1], parameters[2][2]},
        .q    = {parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]},
        .v    = {parameters[3][0], parameters[3][1], parameters[3][2]},
        .bg   = {parameters[3][3], parameters[3][4], parameters[3][5]},
        .ba   = {parameters[3][6], parameters[3][7], parameters[3][8]},
        .sodo = parameters[3][9],
    };
}

/* IMU 预积分 */
void PreintegrationEarthOdo::integrationProcess(unsigned long index) {
    // 零偏补偿
    IMU imu_pre = compensationBias(imu_buffer_[index - 1]);
    IMU imu_cur = compensationBias(imu_buffer_[index]);

    // 区间时间累积
    double dt = imu_cur.dt;
    delta_time_ += dt; // 累积时间间隔

    end_time_           = imu_cur.time; // 更新结束时间为当前IMU数据的时间戳
    current_state_.time = imu_cur.time; // 更新当前状态的时间戳

    // 1.连续状态积分(INS惯导解算：机械编排), 先位置速度再姿态

    // 位置速度

    // 速度：“Ref：KF-GINS github项目中docs文件夹内的《kf-gins开源代码分享-i2nav-0514.pdf》”
        /* 采用双子样假设，根据上一时刻的速度、上一时刻和当前时刻的IMU观测值，计算n系下当前时刻k的速度，包括比力积分项、重力和哥氏积分项。 
            1.考虑旋转效应、划桨效应，计算b系下比力积分项。
            2.根据投影参数、重力、上一时刻的速度，计算重力和哥氏积分项。
            3.根据投影参数、上一时刻的姿态，由b系比力积分项，得到n系下比力积分项*/
    // b系比力积分项：dvfb
    Vector3d dvfb = imu_cur.dvel + 0.5 * imu_cur.dtheta.cross(imu_cur.dvel) +
                    1.0 / 12.0 * (imu_pre.dtheta.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dtheta));
    // 哥氏项和重力项：iewn_是地球自转角速度在n系下的投影；current_state_.v表示n系下上一时刻的速度，k-1時刻
    Vector3d dv_cor_g = (gravity_ - 2.0 * iewn_.cross(current_state_.v)) * dt;

    // !!!地球自转补偿项, 省去了enwn项，（可以考虑加上)
        /* 在计算地球自转补偿项时，使用的投影参数是指地球自转角速度投影到n系的参数iewn_ ，
            但是忽略了n系相对于e系转动角速度投影到n系的补偿项，而KF-GINS都考虑了 */
    Vector3d dnn    = -iewn_ * dt;
    Quaterniond qnn = Rotation::rotvec2quaternion(dnn); // 旋转向量转换为四元数，表示n系下k-1时刻到k时刻的旋转四元数

    Vector3d dvel = 
        // 首尾平均法，和公式中的 中点法 效果一样
        0.5 * (Matrix3d::Identity() + qnn.toRotationMatrix()) * current_state_.q.toRotationMatrix() * dvfb + dv_cor_g;

    current_state_.v += dvel; // 更新速度

    // 位置
    // 前后历元平均速度计算位置
    // PS：KF-GINS在进行位置更新时，计算的是大地坐标系下的BLH坐标，所以和OB-GINS不一样。
    current_state_.p += dt * current_state_.v + 0.5 * dt * dvel;

    // 缓存IMU时刻位置, 时间间隔为两个历元的间隔
    pn_.emplace_back(std::make_pair(dt, current_state_.p));

    // 姿态
    // Ref：KF-GINS github项目中docs文件夹内的《kf-gins开源代码分享-i2nav-0514.pdf》
        /*  采用四元数进行姿态更新计算：
            根据投影参数，计算k-1时刻~k时刻n系下的旋转四元数
            基于双子样假设，根据IMU角度增量数据，计算k-1时刻~k时刻b系下的旋转四元数
            根据k-1时刻的四元数 ，计算k时刻的四元数 */
    Vector3d dtheta = imu_cur.dtheta + 1.0 / 12.0 * imu_pre.dtheta.cross(imu_cur.dtheta);
        // Rotation::rotvec2quaternion(dtheta)表示b系下k-1时刻到k时刻的旋转四元数
        // current_state_.q表示k-1时刻的四元数
        // qnn表示n系下k-1时刻到k时刻的旋转四元数
        // 最后得到current_state_.q表示k时刻的四元数
    current_state_.q = qnn * current_state_.q * Rotation::rotvec2quaternion(dtheta);
    current_state_.q.normalize();

    /* PS: 相比于KF-GINS，OB_GINS并没有计算中间时刻的速度、位置，更新投影参数，应该还是因为面向的惯导类型不同，OB_GINS面向的惯导是MEMS，可以简略一些步骤，而KF-GINS面向的惯导是高精度的。*/

    // 2.预积分（相对状态约束）：利用相邻节点之间的 IMU 数据预先积分出与积分起点位姿无关的相对位置、速度、姿态增量。
        // 具体概念，详见武汉大学i2nav团队常乐的博士论文第2章的2.4小节。

    // (1) ODO预积分:直接使用ODO测量的原始里程信息，积分得到里程增量。
    // 中间时刻的地球自转等效旋转矢量：根据投影参数iewn_、相对姿态delta_state_.q，计算中间时刻的地球自转等效旋转矢量 
        // Ref：i2nav团队唐海亮博士在IEEE发表的论文《Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization》 -- 公式11 
    dnn           = -(delta_time_ - 0.5 * dt) * iewn_;  // dnn: 从积分起点到当前时刻中间点的地球自转旋转矢量，dnn = - (Δt - 0.5*dt) * ω_ie^n；消除地球自转对惯性系假设的影响
    Matrix3d cbbe = (q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix();  // cbbe: 补偿地球自转的旋转矩阵，C_b^b_e = q0^{-1} * exp(dnn) * q0 * Δq，其中 q0 是积分起点时刻的姿态四元数，Δq 是预积分的姿态增量，表示从积分起点到当前时刻的相对姿态变化，cbbe：最终得到的矩阵，表示考虑了地球自转修正后的，从当前b系到初始b0系的旋转矩阵（将b系转换到w系(即b_e(k-1)系)下的旋转矩阵）

    // 里程增量：根据里程计比例因子、杆臂lodo_ ，计算里程增量delta_state_.s，更新delta_state_.s
        // Ref：《基于图优化的LiDAR/INS/ODO/GNSS车载组合导航算法研究》-武汉大学博士论文-常乐-公式(5.6)
    Vector3d dsodo = Vector3d(imu_cur.odovel, 0, 0);  // dsodo: 里程计原始测量向量，dsodo = [odovel, 0, 0]^T，其中 odovel 是里程计速度测量，通常是前进速度，即x轴，前右下对应xyz
    delta_state_.s += cbbe * (cvb_ * dsodo * (1 + delta_state_.sodo)
                            - Rotation::rotvec2quaternion(imu_cur.dtheta).toRotationMatrix() * lodo_ + lodo_); // cvb_: 车体坐标系到ODO安装坐标系的旋转矩阵（根据参数配置中的安装角计算得到），lodo_: 里程计安装位置的杆臂向量

    /* PS：以上ODO预积分公式涉及两个坐标系，v系和w系： v系是车辆坐标系，w系是惯导坐标系。
           v系是以ODO安装所在车轮与地面的切点为原点。
           w系的原点不变，一直在初始位置，固定在k-1时刻的e系下。*/

    // (2) 速度、位置预积分:
        // 速度预积分：利用计算的b系下的比力积分项dvfb，转为w系下
        // 位置预积分：和之前位置更新的原理相同，不过得到的应该是w系下的位置。
        // Ref：《Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization》-公式10
    // 前后历元平均速度计算位置
    dvel = cbbe * dvfb; // cbbe: 补偿地球自转的旋转矩阵，C_b^b_e，将b系下的比力积分项dvfb转为w系下
    delta_state_.p += dt * delta_state_.v + 0.5 * dt * dvel;
    delta_state_.v += dvel;

    // 姿态
        // Ref：《Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization》-公式10
        // 利用本计算的k-1时刻~k时刻b系下的旋转四元数dtheta，更新姿态的相对约束,转为w系下
    delta_state_.q *= Rotation::rotvec2quaternion(dtheta);
    delta_state_.q.normalize();

    // 3.更新系统状态雅克比和协方差矩阵
    updateJacobianAndCovariance(imu_pre, imu_cur);
}

void PreintegrationEarthOdo::resetState(const IntegrationState &state) {
    resetState(state, NUM_STATE);
}

/* 误差传播函数：根据误差传播定律，建立误差微分方程，更新雅可比矩阵和协方差矩阵。
    Ref：《Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization》-公式14~25

    总览：
    1.计算状态转移矩阵Phi（19行*19列）。
    2.计算噪声驱动矩阵G（19行*16列）。
    3.计算噪声矩阵Q（19行*19列）。
    4.更新雅可比矩阵J（19行*19列）。
    5.更新参数协方差矩阵Pk（19行*19列）

    误差传播方程 (线性化):
    delta_x_{k+1} = Phi_k * delta_x_k + G_k * n_k
    jacobian_ = phi * jacobian_;  // jacobian_实际上是 当前时刻误差 关于 初始时刻误差 的偏导数,表示：如果在预积分起点的状态（比如零偏 bg, ba）发生微小变化，当前时刻的预积分结果会发如何变化,利用该公式的递推关系，一步步累积计算出了相对于初始状态的灵敏度矩阵。这个矩阵最后在 evaluate 函数中被用来做一阶泰勒展开，修正预积分结果

    公式总结：
    1. 状态向量 (19维):
       delta_x = [dp(3), dv(3), dtheta(3), dbg(3), dba(3), ds(3), ds_odo(1)]^T
         dp, dv, dtheta: IMU预积分位置、速度、姿态误差
         dbg, dba: 陀螺仪、加速度计零偏误差
         ds: 里程计位置增量误差 (w系下)
         ds_odo: 里程计比例因子误差

    2. 状态转移矩阵 Phi (19x19):
       Phi = [ I    I*dt   0               0              0              0                0          ]
             [ 0    I      C*Sk(dv)        0              C*dt           0                0          ]
             [ 0    0      I-Sk(dtheta)   -I*dt           0              0                0          ]
             [ 0    0      0               I*(1-dt/tau)   0              0                0          ]
             [ 0    0      0               0              I*(1-dt/tau)   0                0          ]
             [ 0    0      C*Sk(stheta)    C*Sk(l)*dt     0              I            -C*cvb*dsodo ]
             [ 0    0      0               0              0              0                1          ]
       其中:
         C = C_b^b0 (当前时刻b系到初始b0系的旋转矩阵，代码中为 cbb0)
         Sk(.) 为反对称矩阵
         stheta = cvb * dsodo * (1+sodo) - dtheta x lodo
         dsodo = [odo_vel, 0, 0]^T

    3. 噪声驱动矩阵 G (19x16):
       G = [ 0     0     0     0     0     0]
           [ 0     C     0     0     0     0]  <- v 对应 acc噪声
           [-I     0     0     0     0     0]  <- theta 对应 gyr噪声
           [ 0     0     I     0     0     0]  <- bg 对应 bg噪声
           [ 0     0     0     I     0     0]  <- ba 对应 ba噪声
           [ G_sg  0     0     0     G_so  0]  <- s 对应 gyr噪声(杆臂效应) 和 odo噪声
           [ 0     0     0     0     0     1]  <- sodo 对应 sodo噪声
       其中:
         G_sg = C * Sk(lodo)
         G_so = C * cvb * (1+sodo)

    4. 传播公式:
       Jacobian: J_k = Phi_k * J_{k-1}
       Covariance: P_k = Phi_k * P_{k-1} * Phi_k^T + Q_k
       Discrete Noise Q_k: 中值积分近似 (Trapezoidal integration)
         Q_k = 0.5 * dt * (Phi * G * Q * G^T + G * Q * G^T * Phi^T)

    PS：
    雅可比矩阵不是EKF中提及的系数矩阵，系数矩阵是常数矩阵，而雅可比矩阵是包含参数的矩阵，是关于参数的一阶偏导数，会随着参数变化而变化。因此，参数更新后，雅可比矩阵也需要更新。

    误差微分方程的维数，包括：
        位置（3维）
        速度（3维）
        姿态（3维）
        陀螺仪零偏（3维）
        加速度计零偏（3维）
        ODO里程增量（3维）
        里程计比例因子（1维）。 */
void PreintegrationEarthOdo::updateJacobianAndCovariance(const IMU &imu_pre, const IMU &imu_cur) {
    // dp, dv, dq, dbg, dba

    // 状态转移矩阵 phi，初始化为零矩阵，19*19；NUM_STATE = 19
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(NUM_STATE, NUM_STATE);

    double dt = imu_cur.dt; // 时间间隔

    // 整体区间的累计旋转cbb0
    Vector3d dnn  = -iewn_ * delta_time_;  // dnn: 从积分起点到当前时刻的地球自转旋转矢量，dnn = - Δt * ω_ie^n；消除地球自转对惯性系假设的影响;delta_time_ 表示 从预积分起始时刻到当前时刻的累计时间长度,
    Matrix3d cbb0 = -(q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix(); // C_b^b0: 从当前b系到初始b0系的旋转矩阵（将b系转换到w系(即b_e(k-1)系)下的旋转矩阵）

    // 1.jacobian

    // phi = I + F * dt   计算状态转移矩阵Phi（19行*19列）。phi的具体构建应该看教材或者官方文档
    phi.block<3, 3>(0, 0)   = Matrix3d::Identity();
    phi.block<3, 3>(0, 3)   = Matrix3d::Identity() * dt;
    phi.block<3, 3>(3, 3)   = Matrix3d::Identity();
    phi.block<3, 3>(3, 6)   = cbb0 * Rotation::skewSymmetric(imu_cur.dvel);
    phi.block<3, 3>(3, 12)  = cbb0 * dt;
    phi.block<3, 3>(6, 6)   = Matrix3d::Identity() - Rotation::skewSymmetric(imu_cur.dtheta);
    phi.block<3, 3>(6, 9)   = -Matrix3d::Identity() * dt;
    phi.block<3, 3>(9, 9)   = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);
    phi.block<3, 3>(12, 12) = Matrix3d::Identity() * (1 - dt / parameters_->corr_time);

    Vector3d dsodo  = Vector3d(imu_cur.odovel, 0, 0);
    Vector3d stheta = cvb_ * dsodo * (1 + delta_state_.sodo) - imu_cur.dtheta.cross(lodo_);

    phi.block<3, 3>(15, 6)  = cbb0 * Rotation::skewSymmetric(stheta);
    phi.block<3, 3>(15, 9)  = cbb0 * Rotation::skewSymmetric(lodo_) * dt;
    phi.block<3, 3>(15, 15) = Matrix3d::Identity();
    phi.block<3, 1>(15, 18) = -cbb0 * cvb_ * dsodo;
    phi(18, 18)             = 1.0;

    // 更新雅克比矩阵，公式：Jk = phi * Jk-1，递推
    jacobian_ = phi * jacobian_; 

    // 2.covariance

    // gt 计算噪声驱动矩阵G（19行*16列）    
    Eigen::MatrixXd gt = Eigen::MatrixXd::Zero(NUM_STATE, NUM_NOISE);

    gt.block<3, 3>(3, 3)   = cbb0;
    gt.block<3, 3>(6, 0)   = -Matrix3d::Identity();
    gt.block<3, 3>(9, 6)   = Matrix3d::Identity();
    gt.block<3, 3>(12, 9)  = Matrix3d::Identity();
    gt.block<3, 3>(15, 0)  = cbb0 * Rotation::skewSymmetric(lodo_);
    gt.block<3, 3>(15, 12) = cbb0 * cvb_ * (1 + delta_state_.sodo);
    gt(18, 15)             = 1.0;

    // 计算噪声矩阵Q（19行*19列），公式：Qk = 0.5 * dt * (phi * gt * noise_ * gt.transpose() + gt * noise_ * gt.transpose() * phi.transpose())  ，近似等于
    Eigen::MatrixXd Qk =
        0.5 * dt * (phi * gt * noise_ * gt.transpose() + gt * noise_ * gt.transpose() * phi.transpose());

    // 更新参数协方差矩阵 （19行*19列），公式：Pk = phi * Pk * phi.transpose() + Qk
    covariance_ = phi * covariance_ * phi.transpose() + Qk;
}

/* 初始预积分delta_state_状态变量设置 */
void PreintegrationEarthOdo::resetState(const IntegrationState &state, int num) {
    // 初始化预积分状态
    delta_time_ = 0;
    delta_state_.p.setZero();
    delta_state_.q.setIdentity();
    delta_state_.v.setZero();
    delta_state_.s.setZero();
    delta_state_.bg   = state.bg;
    delta_state_.ba   = state.ba;
    delta_state_.sodo = state.sodo;

    jacobian_.setIdentity(num, num); //
    covariance_.setZero(num, num);

    // 预积分起点的绝对姿态
    q0_ = current_state_.q;

    // 地球自转, 近似使用初始时刻位置
        // iewn_ 投影参数是指地球自转角速度投影到n系的参数
    iewn_      = Earth::iewn(parameters_->station, current_state_.p);
        // iewn_skew_ 是 iewn_的反对称矩阵
    iewn_skew_ = Rotation::skewSymmetric(iewn_);

    pn_.clear();
}

void PreintegrationEarthOdo::setNoiseMatrix() {
    noise_.setIdentity(NUM_NOISE, NUM_NOISE);
    noise_.block<3, 3>(0, 0) *= parameters_->gyr_arw * parameters_->gyr_arw; // nw
    noise_.block<3, 3>(3, 3) *= parameters_->acc_vrw * parameters_->acc_vrw; // na
    noise_.block<3, 3>(6, 6) *=
        2 * parameters_->gyr_bias_std * parameters_->gyr_bias_std / parameters_->corr_time; // nbg
    noise_.block<3, 3>(9, 9) *=
        2 * parameters_->acc_bias_std * parameters_->acc_bias_std / parameters_->corr_time; // nba
    noise_(12, 12) *= parameters_->odo_std[0] * parameters_->odo_std[0];                    // nodo
    noise_(13, 13) *= parameters_->odo_std[1] * parameters_->odo_std[1];                    // nodo
    noise_(14, 14) *= parameters_->odo_std[2] * parameters_->odo_std[2];                    // nodo
    noise_(15, 15) *= parameters_->odo_srw * parameters_->odo_srw;                          // nsodo
}

int PreintegrationEarthOdo::imuErrorNumResiduals() {
    return NUM_ERROR_RESIDUAL;
}

vector<int> PreintegrationEarthOdo::imuErrorNumBlocksParameters() {
    return std::vector<int>{NUM_MIX};
}

void PreintegrationEarthOdo::imuErrorEvaluate(const double *const *parameters, double *residuals) {
    // bg, ba
    residuals[0] = parameters[0][3] / IMU_GRY_BIAS_STD;
    residuals[1] = parameters[0][4] / IMU_GRY_BIAS_STD;
    residuals[2] = parameters[0][5] / IMU_GRY_BIAS_STD;
    residuals[3] = parameters[0][6] / IMU_ACC_BIAS_STD;
    residuals[4] = parameters[0][7] / IMU_ACC_BIAS_STD;
    residuals[5] = parameters[0][8] / IMU_ACC_BIAS_STD;
    residuals[6] = parameters[0][9] / ODO_SCALE_STD;
}

void PreintegrationEarthOdo::imuErrorJacobian(double *jacobian) {
    Eigen::Map<Eigen::Matrix<double, NUM_ERROR_RESIDUAL, NUM_MIX, Eigen::RowMajor>> jaco(jacobian);

    jaco.setZero();

    jaco(0, 3) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(1, 4) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(2, 5) = 1.0 / IMU_GRY_BIAS_STD;
    jaco(3, 6) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(4, 7) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(5, 8) = 1.0 / IMU_ACC_BIAS_STD;
    jaco(6, 9) = 1.0 / ODO_SCALE_STD;
}
