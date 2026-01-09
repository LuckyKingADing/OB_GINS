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

#ifndef MARGINILAZATION_INFO_H
#define MARGINILAZATION_INFO_H

#include <memory>
#include <unordered_map>

#include "residual_block_info.h"

/*  如果IMU预积分因子的数量超过滑动窗口大小，使用舒尔补（Schur消元）边缘化历史信息，并将其转为先验信息。
    PS：
    1.边缘化这部分的原理在唐海亮博士的论文中介绍较少，可能边缘化操作在SLAM项目中较为常见，原理类似。
    2.边缘化的作用是，去掉滑动窗口内最老节点的状态参数，最老节点的状态参数将不会再变化，但是其和次老节点之间存在关系，下图中的因子就是两者的联系。
    因此，需要根据最老节点和次老节点的关系建立约束方程，并消元，转化为只包含次老节点的先验信息，在下一次优化的时候，对滑窗内次老节点的状态参数进行先验约束。 */
class MarginalizationInfo {

public:
    MarginalizationInfo() = default;

    ~MarginalizationInfo() {
        for (auto &block : parameter_block_data_)
            delete[] block.second;
    }

    bool isValid() const {
        return isvalid_;
    }

    static int localSize(int size) {
        return size == POSE_GLOBAL_SIZE ? POSE_LOCAL_SIZE : size;
    }

    static int globalSize(int size) {
        return size == POSE_LOCAL_SIZE ? POSE_GLOBAL_SIZE : size;
    }

    // blockinfo就是残差块信息：包含factor因子、其涉及的具体参数、需要边缘化掉的参数索引
    /* 举例：
       当你调用这个函数加入一个连接 Pose A 和 Pose B 的因子，并且说“边缘化 Pose A”时：
       1.因子被存进 factors_。
       2.Pose A (7维) 和 Pose B (7维) 都被登记在 parameter_block_size_ (全员名单)。
       3.Pose A 被单独登记在 parameter_block_index_ (死亡名单/待消元名单) 中 */
    void addResidualBlockInfo(const std::shared_ptr<ResidualBlockInfo> &blockinfo) {
        factors_.push_back(blockinfo); // 加入factors_列表
 
        const auto &parameter_blocks = blockinfo->parameterBlocks();  
        const auto &block_sizes      = blockinfo->parameterBlockSizes(); 

        for (size_t k = 0; k < parameter_blocks.size(); k++) {
            parameter_block_size_[reinterpret_cast<long>(parameter_blocks[k])] = block_sizes[k]; 
        }

        // 被边缘化的参数, 先加入表中以进行后续的排序
        for (int index : blockinfo->marginalizationParametersIndex()) {
            parameter_block_index_[reinterpret_cast<long>(parameter_blocks[index])] = 0;
        }
    }

    // !!!边缘化操作：较难理解
    bool marginalization() {

        // 对边缘化的参数和保留的参数按照local size分配索引, 边缘化参数位于前端
        if (!updateParameterBlocksIndex()) {
            isvalid_ = false;

            // 释放内存
            releaseMemory();

            return false;
        }

        // 计算每个残差块参数, 进行参数内存拷贝
        preMarginalization();

        // 构造增量线性方程
        constructEquation();

        // Schur消元
        schurElimination();

        // 求解线性化雅克比和残差
        linearization();

        // 释放内存
        releaseMemory();

        return true;
    }

    std::vector<double *> getParamterBlocks(std::unordered_map<long, double *> &address) {
        std::vector<double *> remained_block_addr;

        remained_block_data_.clear();
        remained_block_index_.clear();
        remained_block_size_.clear();

        for (const auto &block : parameter_block_index_) {
            // 保留的参数
            if (block.second >= marginalized_size_) {
                remained_block_data_.push_back(parameter_block_data_[block.first]);
                remained_block_size_.push_back(parameter_block_size_[block.first]);
                remained_block_index_.push_back(parameter_block_index_[block.first]);
                remained_block_addr.push_back(address[block.first]);
            }
        }

        return remained_block_addr;
    }

    const Eigen::MatrixXd &linearizedJacobians() {
        return linearized_jacobians_;
    }

    const Eigen::VectorXd &linearizedResiduals() {
        return linearized_residuals_;
    }

    int marginalizedSize() const {
        return marginalized_size_;
    }

    int remainedSize() const {
        return remained_size_;
    }

    const std::vector<int> &remainedBlockSize() {
        return remained_block_size_;
    }

    const std::vector<int> &remainedBlockIndex() {
        return remained_block_index_;
    }

    const std::vector<double *> &remainedBlockData() {
        return remained_block_data_;
    }

private:
    // 更新雅可比和残差矩阵（linearization函数）:根据消元后的信息矩阵H，利用SVD分解，得到消元更新后的雅可比矩阵J、残差矩阵e。
    // 线性化
    void linearization() {
        // SVD分解求解雅克比, Hp = J^T * J = V * S^{1/2} * S^{1/2} * V^T
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(Hp_);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv =
            Eigen::VectorXd((saes2.eigenvalues().array() > EPS).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt     = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        // J0 = S^{1/2} * V^T
        linearized_jacobians_ = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
        // e0 = -{J0^T}^{-1} * bp = - S^{-1/2} * V^T * bp
        linearized_residuals_ = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * -bp_;
    }

    // 更新信息矩阵（schurElimination函数）:通过舒尔补（Schur）操作，消除边缘化参数，得到消元后的信息矩阵H。
    // Schur消元, 求解 Hp * dx_r = bp
    // PS:舒尔补操作的原理和中学的二元一次方程求解非常类似，比如两个方程，两个未知数x、y，通过未知数的系数一致性，消除x，得到y的解。
    void schurElimination() {
        // H0 * dx = b0
        Eigen::MatrixXd Hmm = 0.5 * (H0_.block(0, 0, marginalized_size_, marginalized_size_) +
                                     H0_.block(0, 0, marginalized_size_, marginalized_size_).transpose());
        Eigen::MatrixXd Hmr = H0_.block(0, marginalized_size_, marginalized_size_, remained_size_);
        Eigen::MatrixXd Hrm = H0_.block(marginalized_size_, 0, remained_size_, marginalized_size_);
        Eigen::MatrixXd Hrr = H0_.block(marginalized_size_, marginalized_size_, remained_size_, remained_size_);
        Eigen::VectorXd bmm = b0_.segment(0, marginalized_size_);
        Eigen::VectorXd brr = b0_.segment(marginalized_size_, remained_size_);

        // SVD分解Amm求逆
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Hmm);
        Eigen::MatrixXd Hmm_inv =
            saes.eigenvectors() *
            Eigen::VectorXd((saes.eigenvalues().array() > EPS).select(saes.eigenvalues().array().inverse(), 0))
                .asDiagonal() *
            saes.eigenvectors().transpose();

        // Hp = Hrr - Hrm * Hmm^-1 * Hmr
        Hp_ = Hrr - Hrm * Hmm_inv * Hmr;
        // bp = br - Hrm * Hmm^-1 * bm
        bp_ = brr - Hrm * Hmm_inv * bmm;
    }

    // 计算信息矩阵（constructEquation函数）
    // 线性化 构造增量方程 H * dx = b, 计算 H 和 b
    /* PS:
       H矩阵在SLAM领域称为信息矩阵，而从测绘的间接平差的角度上解释：
       对于观测方程 v = Bx - l，H矩阵就是 (B^T P B)^-1，b就是 B^T P l，相信学过 《误差理论与测量平差》 的测绘人都很熟悉~ */
    void constructEquation() {
        H0_ = Eigen::MatrixXd::Zero(local_size_, local_size_); // 构建信息矩阵H0
        b0_ = Eigen::VectorXd::Zero(local_size_);

        for (const auto &factor : factors_) { 
            for (size_t i = 0; i < factor->parameterBlocks().size(); i++) { 
                int row0 = parameter_block_index_[reinterpret_cast<long>(factor->parameterBlocks()[i])]; // 获取当前参数块在信息矩阵中的起始行索引
                int rows = parameter_block_size_[reinterpret_cast<long>(factor->parameterBlocks()[i])]; // 获取当前参数块的全局大小
                rows     = (rows == POSE_GLOBAL_SIZE) ? POSE_LOCAL_SIZE : rows; // 转为local size

                Eigen::MatrixXd jacobian_i = factor->jacobians()[i].leftCols(rows); // 获取当前因子关于第i个参数块的雅可比矩阵J
                for (size_t j = i; j < factor->parameterBlocks().size(); ++j) {
                    int col0 = parameter_block_index_[reinterpret_cast<long>(factor->parameterBlocks()[j])]; // 获取当前参数块在信息矩阵中的起始列索引
                    int cols = parameter_block_size_[reinterpret_cast<long>(factor->parameterBlocks()[j])]; // 获取当前参数块的全局大小
                    cols     = (cols == POSE_GLOBAL_SIZE) ? POSE_LOCAL_SIZE : cols; // 转为local size

                    Eigen::MatrixXd jacobian_j = factor->jacobians()[j].leftCols(cols); // 获取当前因子关于第j个参数块的雅可比矩阵J

                    // H = J^T * J
                    if (i == j) {  // 对角线元素
                        // Hmm, Hrr
                        H0_.block(row0, col0, rows, cols) += jacobian_i.transpose() * jacobian_j; // 对角线块直接累加
                    } else { // 非对角线元素
                        // Hmr, Hrm = Hmr^T
                        H0_.block(row0, col0, rows, cols) += jacobian_i.transpose() * jacobian_j; // 非对角线块累加
                        H0_.block(col0, row0, cols, rows) = H0_.block(row0, col0, rows, cols).transpose(); // 利用对称性赋值
                    }
                }
                // b = - J^T * e，e是残差
                // 填充b向量
                b0_.segment(row0, rows) -= jacobian_i.transpose() * factor->residuals();
            }
        }
    }

    // 更新参数块索引（updateParameterBlocksIndex函数）:为每个参数块分配索引，边缘化参数在前，保留参数在后。
    /* PS： */
    bool updateParameterBlocksIndex() {
        int index = 0;
        // 只有被边缘化的参数预先加入了表
        for (auto &block : parameter_block_index_) {
            block.second = index;
            index += localSize(parameter_block_size_[block.first]);
        }
        marginalized_size_ = index;

        // 加入保留的参数, 分配索引
        for (const auto &block : parameter_block_size_) {
            if (parameter_block_index_.find(block.first) == parameter_block_index_.end()) {
                parameter_block_index_[block.first] = index;
                index += localSize(block.second);
            }
        }
        remained_size_ = index - marginalized_size_;

        local_size_ = index;

        return marginalized_size_ > 0;
    }

    // 边缘化预处理, 评估每个残差块, 拷贝参数
    void preMarginalization() {
        // 遍历所有待边缘化的因子，调用Evaluate函数，计算每个残差块的雅可比矩阵J和残差矩阵e
        for (const auto &factor : factors_) {
            // 计算边缘化、IMU预积分、GNSS残差因子的雅可比矩阵J、残差矩阵e
            factor->Evaluate(); // 调用 Ceres 的 CostFunction::Evaluate，计算出当前参数下的残差向量和雅可比矩阵

            std::vector<int> block_sizes = factor->parameterBlockSizes(); // 获取当前因子涉及的参数块的大小
            for (size_t k = 0; k < block_sizes.size(); k++) {
                long addr = reinterpret_cast<long>(factor->parameterBlocks()[k]); // 获取参数块的内存地址
                int size  = block_sizes[k];  // 参数块的全局大小

                // 拷贝参数块数据
                // 深拷贝参数块数据到 parameter_block_data_，以便后续边缘化计算使用
                /* 深拷贝 (memcpy)：如果该参数块还没备份过，就在 parameter_block_data_ 中开辟新内存，把当前的参数值原封不动地存下来。这个备份的数据（Pose, Vel, Bias 等）将作为后续先验因子的基准值 */
                if (parameter_block_data_.find(addr) == parameter_block_data_.end()) {
                    auto *data = new double[size];
                    memcpy(data, factor->parameterBlocks()[k], sizeof(double) * size);
                    parameter_block_data_[addr] = data;
                }
            }
        }
    }

    void releaseMemory() {
        // 释放因子所占有的内存, 尤其是边缘化因子及其占有的边缘化信息数据结构
        factors_.clear();
    }

private:
    // 增量线性方程参数
    Eigen::MatrixXd H0_, Hp_;
    Eigen::VectorXd b0_, bp_;

    // 以内存地址为key的无序表

    // 存放参数块的global size
    std::unordered_map<long, int> parameter_block_size_;
    // 存放参数块索引, 待边缘化参数索引在前, 保留参数索引在后, 用于构造边缘化 H * dx = b
    std::unordered_map<long, int> parameter_block_index_;
    // 存放参数快数据指针
    std::unordered_map<long, double *> parameter_block_data_;

    // 保留的参数
    std::vector<int> remained_block_size_;  // global size
    std::vector<int> remained_block_index_; // local size
    std::vector<double *> remained_block_data_;

    // local size in total
    int marginalized_size_{0};
    int remained_size_{0};
    int local_size_{0};

    // 边缘化参数相关的残差块
    std::vector<std::shared_ptr<ResidualBlockInfo>> factors_;

    const double EPS = 1e-8;

    // 边缘化求解的残差和雅克比
    Eigen::MatrixXd linearized_jacobians_;
    Eigen::VectorXd linearized_residuals_;

    // 若无待边缘化参数, 则无效
    bool isvalid_{true};
};

#endif // MARGINILAZATION_INFO_H
