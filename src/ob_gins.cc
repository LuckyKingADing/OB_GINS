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

#include "src/common/earth.h"
#include "src/common/types.h"

#include "src/fileio/filesaver.h"
#include "src/fileio/gnssfileloader.h"
#include "src/fileio/imufileloader.h"

#include "src/factors/gnss_factor.h"
#include "src/factors/marginalization_factor.h"
#include "src/factors/pose_manifold.h"
#include "src/preintegration/imu_error_factor.h"
#include "src/preintegration/preintegration.h"
#include "src/preintegration/preintegration_factor.h"

#include <absl/strings/str_format.h>
#include <absl/time/clock.h>
#include <deque>
#include <iomanip>
#include <yaml-cpp/yaml.h>

#define INTEGRATION_LENGTH 1.0
#define MINIMUM_INTERVAL 0.001

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid);

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile);

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
        return -1;
    }

    std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";

    auto ts = absl::Now();

    // 读取配置
    // load configuration
    YAML::Node config;
    std::vector<double> vec;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to read configuration file" << std::endl;
        return -1;
    }

    // 时间信息
    // processing time
    int windows   = config["windows"].as<int>();
    int starttime = config["starttime"].as<int>();
    int endtime   = config["endtime"].as<int>();

    // 迭代次数
    // number of iterations
    int num_iterations = config["num_iterations"].as<int>();

    // 进行GNSS粗差检测
    // Do GNSS outlier culling
    bool is_outlier_culling = config["is_outlier_culling"].as<bool>();

    // 初始化信息
    // initialization
    vec = config["initvel"].as<std::vector<double>>();
    Vector3d initvel(vec.data());
    vec = config["initatt"].as<std::vector<double>>();
    Vector3d initatt(vec.data());
    initatt *= D2R;

    vec = config["initgb"].as<std::vector<double>>();
    Vector3d initbg(vec.data());
    initbg *= D2R / 3600.0;
    vec = config["initab"].as<std::vector<double>>();
    Vector3d initba(vec.data());
    initba *= 1.0e-5;

    // 数据文件
    // data file
    std::string gnsspath   = config["gnssfile"].as<std::string>();
    std::string imupath    = config["imufile"].as<std::string>();
    std::string outputpath = config["outputpath"].as<std::string>();
    int imudatalen         = config["imudatalen"].as<int>();
    int imudatarate        = config["imudatarate"].as<int>();

    // 是否考虑地球自转
    // consider the Earth's rotation
    bool isearth = config["isearth"].as<bool>();

    // 创建文件对象
    GnssFileLoader gnssfile(gnsspath);
    ImuFileLoader imufile(imupath, imudatalen, imudatarate);
    FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);      // 导航结果文件，文本格式
    FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY); // IMU误差文件，二进制格式
    if (!imufile.isOpen() || !gnssfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
        std::cout << "Failed to open data file" << std::endl;
        return -1;
    }

    // 安装参数
    // installation parameters
    vec = config["antlever"].as<std::vector<double>>();
    Vector3d antlever(vec.data());
    vec = config["odolever"].as<std::vector<double>>();
    Vector3d odolever(vec.data());
    vec = config["bodyangle"].as<std::vector<double>>();
    Vector3d bodyangle(vec.data());
    bodyangle *= D2R; // rad

    // IMU噪声参数
    // IMU noise parameters
    auto parameters          = std::make_shared<IntegrationParameters>();
    parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
    parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
    parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
    parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
    parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;

    bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
    vec                 = config["odometer"]["std"].as<std::vector<double>>();
    parameters->odo_std = Vector3d(vec.data());
    parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
    parameters->lodo    = odolever;
    parameters->abv     = bodyangle;

    // GNSS仿真中断配置
    // GNSS outage parameters
    bool isuseoutage = config["isuseoutage"].as<bool>();
    int outagetime   = config["outagetime"].as<int>();
    int outagelen    = config["outagelen"].as<int>();
    int outageperiod = config["outageperiod"].as<int>();

    auto gnssthreshold = config["gnssthreshold"].as<double>();

    // 数据文件调整
    // data alignment
    IMU imu_cur, imu_pre;
    do {
        imu_pre = imu_cur; 
        imu_cur = imufile.next();
    } while (imu_cur.time < starttime); // 持续读取直到imu_cur.time超过starttime

    GNSS gnss;
    do {
        gnss = gnssfile.next();
    } while (gnss.time < starttime); // 持续读取直到gnss.time超过starttime

    // 初始位置, 求相对
    Vector3d station_origin = gnss.blh;
    parameters->gravity     = Earth::gravity(gnss.blh); // gravity() 根据当前位置经纬度，计算该位置的重力加速度
    gnss.blh                = Earth::global2local(station_origin, gnss.blh); // 将GNSS位置从大地坐标系的BLH坐标转为世界坐标系（即w系/局部坐标系）的NED坐标，即相对于站心坐标系原点的位置；可说成是导航坐标系n系，或者局部坐标系local，或者世界坐标系w系，都是一样的

    // 站心坐标系原点：即最开始的时间戳所在的gnss位置
    parameters->station = station_origin;

    std::vector<IntegrationState> statelist(windows + 1);
    std::vector<IntegrationStateData> statedatalist(windows + 1);
    std::deque<std::shared_ptr<PreintegrationBase>> preintegrationlist;
    std::deque<GNSS> gnsslist;
    std::deque<double> timelist;

    Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(isuseodo, isearth);

    // 初始状态
    // initialization
    IntegrationState state_curr = {
        .time = round(gnss.time), // 取整秒
        .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever, // 初始位置，改正GNSS天线和IMU之间的杆臂，得到w系下的IMU初始位置
        .q    = Rotation::euler2quaternion(initatt),                        // 初始姿态，将初始欧拉角转换为四元数                  
        .v    = initvel, // 初始速度，配置文件读取，n系下速度
        .bg   = initbg,  // 初始陀螺仪偏置
        .ba   = initba,  // 初始加速度计偏置
        .sodo = 0.0,     // 
        .abv  = {bodyangle[1], bodyangle[2]},  // bodyangle是IMU到载体的旋转角 (mouting angles to construct C_b^v)，这里存储的是y轴和z轴的安装角，俯仰角和横滚角， v系是载体坐标系， b系是IMU坐标系，载体坐标系中心点在哪？？？
    };
    std::cout << "Initilization at " << gnss.time << " s " << std::endl;

    statelist[0]     = state_curr; // 初始状态存入状态列表
    statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options); // 初始状态转换为数据格式存入状态数据列表，因为ceres求解时使用的是数据格式，即double格式，而不是状态向量格式，vector3d和quaterniond格式
    gnsslist.push_back(gnss); // 初始GNSS存入GNSS列表

    double sow = round(gnss.time); // sow: start of week，当前积分周期的起始时间，取整秒
    timelist.push_back(sow);       // 时间列表，存入当前积分周期起始时间

    // 初始预积分
    // Initial preintegration
    preintegrationlist.emplace_back( // 在预积分列表末尾添加一个新的预积分对象
        Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));

    // 读取下一个整秒GNSS
    gnss                = gnssfile.next();
    parameters->gravity = Earth::gravity(gnss.blh); // 每次读取新的GNSS后，更新重力加速度
    gnss.blh            = Earth::global2local(station_origin, gnss.blh); // 转为相对于站心坐标系原点的位置

    // 边缘化信息
    std::shared_ptr<MarginalizationInfo> last_marginalization_info; // 上一个边缘化信息
    std::vector<double *> last_marginalization_parameter_blocks;    // 上一个边缘化参数块

    // 下一个积分节点
    sow += INTEGRATION_LENGTH;  // 下一个积分周期起始时间，当前积分周期起始时间加上积分长度INTEGRATION_LENGTH（目前为1秒）

    while (true) {
        if ((imu_cur.time > endtime) || imufile.isEof()) { // 结束条件：当前IMU时间超过结束时间，或者IMU文件读到末尾
            break;
        }

        // （当imu_cur.time<=sow时，会一直循环执行addNewImu()函数）
        // 加入IMU数据：循环读取imu数据，直到当前IMU时间超过下一个积分节点时间sow，
        // Add new imu data to preintegration
        preintegrationlist.back()->addNewImu(imu_cur); // 将当前IMU数据加入到当前预积分对象中,back()获取deque容器中最后一个元素的引用，即当前预积分对象；队列deque，先入先出，所以back()是当前正在处理的预积分对象

        imu_pre = imu_cur;        // 
        imu_cur = imufile.next(); // 读下一个新的imu

        /* 注： 设置下一个积分的节点时刻sow（整秒），两个积分节点的间隔固定设置为1s，由于GNSS的采样间隔也是1s，因此sow也是GNSS数据的观测时刻。 */
        if (imu_cur.time > sow) { // 当前IMU时间超过下一个积分节点时间sow，说明需要进行积分了；不超过则继续往preintegration加入IMU数据
            
            // 如果当前IMU数据时间大于sow, 读取新的GNSS
            // add GNSS and read new GNSS
            if (fabs(gnss.time - sow) < MINIMUM_INTERVAL) { // MINIMUM_INTERVAL = 0.001，说明当前GNSS时间和下一个积分节点时间sow基本相等
                gnsslist.push_back(gnss);

                gnss = gnssfile.next(); // 读取下一个GNSS数据

                // 粗差检测：固定阈值GNSS抗差 (m)：0.2
                while ((gnss.std[0] > gnssthreshold) || (gnss.std[1] > gnssthreshold) ||
                       (gnss.std[2] > gnssthreshold)) {
                    gnss = gnssfile.next();
                }

                // 中断配置
                // do GNSS outage
                if (isuseoutage) {
                    if (lround(gnss.time) == outagetime) { // lround()将double类型的gnss.time四舍五入取整为long类型，与outagetime比较
                        std::cout << "GNSS outage at " << outagetime << " s" << std::endl;
                        for (int k = 0; k < outagelen; k++) { // 中断长度内，持续读取GNSS数据，直到中断结束
                            gnss = gnssfile.next();
                        }
                        outagetime += outageperiod; // 更新下一个中断时间
                    }
                }

                parameters->gravity = Earth::gravity(gnss.blh);
                gnss.blh            = Earth::global2local(station_origin, gnss.blh);

                if (gnssfile.isEof()) { // 如果GNSS文件读到末尾，重置gnss.time为0，避免后续判断出错
                    gnss.time = 0;
                }
            }

            // IMU内插处理
            // IMU interpolation
            int isneed = isNeedInterpolation(imu_pre, imu_cur, sow);
            if (isneed == -1) { // sow靠近imu_pre
            } else if (isneed == 1) { // sow靠近imu_cur
                preintegrationlist.back()->addNewImu(imu_cur); // 将imu_cur加入当前预积分对象

                // 将当前imu_cur加入当前预积分对象之后，更新imu_pre和imu_cur
                imu_pre = imu_cur; 
                imu_cur = imufile.next();
            } else if (isneed == 2) { // sow在imu_pre和imu_cur之间
                // imuInterpolation(原始数据, 输出的前半段, 输出的后半段, 分割时间点)；处理完之后imu_pre对应时间就是sow时刻，相当于从上一帧(已经在while循环开头处加入preintegration)到imu_pre(也就是sow时刻)的IMU数据已经加入当前预积分对象中了
                imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
                preintegrationlist.back()->addNewImu(imu_pre);
            }

            // 下一个积分节点
            // next time node
            timelist.push_back(sow); // 时间列表，存入当前积分周期起始时间
            sow += INTEGRATION_LENGTH; // 更新下一个积分节点时间

            // 当前整秒状态加入到滑窗中
            state_curr                               = preintegrationlist.back()->currentState();
            statelist[preintegrationlist.size()]     = state_curr;
            statedatalist[preintegrationlist.size()] = Preintegration::stateToData(state_curr, preintegration_options);

            // 构建优化问题
            // construct optimization problem
            {
                // (1) 配置优化参数
                ceres::Problem::Options problem_options;    // ceres问题选项
                problem_options.enable_fast_removal = true; // 启用快速移除功能

                ceres::Problem problem(problem_options); // 创建ceres问题实例，传入options
                ceres::Solver solver;                   // 创建求解器实例
                ceres::Solver::Summary summary;         // 求解器摘要
                ceres::Solver::Options options;         // 设置求解器选项

                // 求解器参数设置
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // 信赖域算法类型：LM算法
                options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY; // 线性求解器类型：稀疏矩阵的Cholesky分解
                options.num_threads                = 4; // 线程数

                // (2) 添加参数块（AddParameterBlock函数）
                // 参数块：将位置、姿态、速度、零偏、里程计比例因子添加到优化的参数块中。
                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) { 
                    // 位姿：位置3、姿态4,共7个状态量
                    ceres::Manifold *manifold = new PoseManifold(); // 位姿流形，因为位姿包含四元数，需要定义流形以处理四元数的单位约束
                    problem.AddParameterBlock(
                        statedatalist[k].pose,  // 参数块数据指针，pose包含位置和姿态，即double[7]
                        Preintegration::numPoseParameter(),  // 参数块维度：7
                        manifold
                    ); 
                    //  速度、零偏、里程计比例因子
                    problem.AddParameterBlock(
                        statedatalist[k].mix, // 参数块数据指针，mix包含速度、零偏和里程计比例因子等，最多double[18](此处只用到10维，如果还需要估计其他参数量，可以增加NUM_MIX的大小维度)
                        Preintegration::numMixParameter(preintegration_options) //  NUM_MIX = 10, vel + bias + sodo : 3 + 6 + 1 = 10
                    );
                }

                // (3) 添加残差块（AddResidualBlock函数）
                    /* PS：
                    1.这些残差因子类都继承于Ceres的代价函数CostFunction类，由于采用Ceres库的解析求导（即自定义求导）的方法，因此重载了Evaluate函数，用于计算雅可比矩阵和残差矩阵，详见附录。
                    2.其中，只有GNSS因子使用了LossFunction类的Huber损失函数（即核函数）HuberLoss，用于剔除异常值，降低异常值的权重；其他因子未使用损失函数。 */
                // GNSS残差因子
                // GNSS factors
                int index = 0;
                ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); // 使用Huber损失函数
                std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id; // 存储添加进去的残差块ID，方便后续做粗差剔除 (Outlier Culling)
                for (const auto &gnss : gnsslist) {
                    auto factor = new GnssFactor(gnss, antlever); // 创建GNSS因子对象，即GNSS残差因子，代价函数
                    for (size_t i = index; i <= preintegrationlist.size(); ++i) { // 遍历滑动窗口内的所有状态节点 (i)
                        if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) { // 判断GNSS时间和积分节点时间是否基本相等
                            // AddResidualBlock: 将约束加入因子图
                                // 参数1: 代价函数 (Factor)
                                // 参数2: 损失函数 (Loss Function)
                                // 参数3: 待优化变量块 (这里只约束位姿 pose，因为 GNSS 只提供位置信息) 
                                // 返回： 残差块ID (Residual Block ID)
                            auto id = problem.AddResidualBlock(factor, loss_function, statedatalist[i].pose);

                            gnss_residualblock_id.push_back(std::make_pair(gnss.time, id)); // 记录残差块ID，传入GNSS时间和残差块ID
                            index++;
                            break;
                        }
                    }
                }

                // 添加IMU预积分残差因子：预积分残差
                // preintegration factors
               for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(
                        factor,                   // 代价函数：预积分约束
                        nullptr,                  // 损失函数：不使用（认为IMU误差符合高斯分布，未做鲁棒核处理）
                        statedatalist[k].pose,    // 参数块1：第k帧的位姿（位置+姿态）
                        statedatalist[k].mix,     // 参数块2：第k帧的混合状态（速度+零偏等）
                        statedatalist[k + 1].pose, // 参数块3：第k+1帧的位姿
                        statedatalist[k + 1].mix   // 参数块4：第k+1帧的混合状态
                        /* PS：
                        1.建立了时间序列上相邻两个状态之间的约束关系，在滑动窗口中，将相邻的每一对状态帧（State k 和 State k+1）链接起来。IMU 预积分类（PreintegrationFactor）内部计算了从 k 时刻到 k+1 时刻的理论位移、速度变化和旋转变化
                        2.调用 AddResidualBlock 就像是在“连线”。
                        -节点（Node）：你传入的那 4 个 double* 指针（参数块）。
                        -边（Edge）：你创建的 factor 对象。
                        -这条边（IMU 预积分约束）是一个 4 元边（4-ary edge），它同时连接了 4 个变量节点。Ceres 完全支持这种多元约束 */
                    );
                }
                // 添加IMU零偏误差约束：仅作用于当前最新的那个状态（窗口末端的帧），边界约束，防止最新的零偏估计值因为缺乏约束而“飞，原因：防止漂移：虽然上一段代码限制了 Bias 的变化率（Random Walk），但在弱观测（如长时间缺乏有效 GNSS）或刚开始初始化时，Bias 的绝对值可能会因为缺乏全局观测而整体漂移
                // add IMU bias-constraint factors
                {   // IMU误差控制
                    // add IMU bias-constraint factors

                    // 根据预积分列表的最后一个预积分对象，即当前预积分对象，创建IMU误差因子
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin()); // rbegin()返回deque容器中最后一个元素的反向迭代器，即当前预积分对象
                    problem.AddResidualBlock(
                        factor, 
                        nullptr, 
                        statedatalist[preintegrationlist.size()].mix // 仅约束最新的那个状态的混合分量（包含Bias）
                    );
                }

                // 添加边缘化残差因子
                // prior factor 即先验因子
                /* PS
                1.作用：将上一轮被“移除”的旧状态所包含的信息，转化成一个“先验约束”，加到当前优化问题中 
                2.在系统刚启动的前几帧（窗口没满时），还没有发生过边缘化，last_marginalization_info 是空的，所以这里会跳过，一旦窗口满了，系统开始移除旧帧，并计算出了边缘化信息（存储在 last_marginalization_info 中），这里就会进入 */
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(
                        factor, 
                        nullptr, 
                        last_marginalization_parameter_blocks // 注意这里传入的是一个 std::vector<double*>。
                                                            // PS：为什么是 vector？ 和之前的 PreintegrationFactor 固定传入 4 个参数不同，边缘化约束连接的参数块数量是动态的。它连接了所有与“被移除变量”有过共视关系的“保留变量”。比如，被移除的第 0 帧可能和第 1, 2, ..., N 帧都有约束。因此，Ceres 允许直接传入一个包含所有相关参数块指针的 vector。
                    );
                }

                // !!!核心：求解最小二乘
                // solve the Least-Squares problem
                    // 第一段优化
                options.max_num_iterations = num_iterations / 4; // 最大迭代次数设置为总迭代次数的1/4
                solver.Solve(options, &problem, &summary);       // 调用求解器求解问题：Solve(options, &problem, &summary)

                // GNSS质量控制：粗差剔除(卡方检验)、重加权：使用卡方检验，判断GNSS因子的代价是否超过阈值，降低超过阈值的GNSS因子的权重。
                // Do GNSS outlier culling using chi-square test

                // TODO: Just a example, you need remodify.（TODO：只是示例，你需要重新修改。）
                if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                    // 3 degrees of freedom, 0.05
                    // 3 自由度：GNSS 提供了 x, y, z 三个维度的位置信息。
                    // 7.815：对应卡方分布在 3 自由度下，置信度为 95% (p=0.05) 的临界值
                    // 含义：如果某个 GNSS 点的计算误差平方和超过了 7.815，我们有 95% 的把握认为它不仅仅是噪声，而是一个粗差（异常值）。
                    double chi2_threshold = 7.815; // 卡方阈值，3自由度，显著性水平0.05

                    // Find GNSS outliers in the window
                    std::unordered_set<double> gnss_outlier;
                    for (size_t k = 0; k < gnsslist.size(); k++) {
                        auto time = gnss_residualblock_id[k].first;
                        auto id   = gnss_residualblock_id[k].second;

                        double cost;
                        double chi2;

                        // 在这一步之前，其实已经运行了一次求解器 solver.Solve（代码 382 行），得到了一个初步的轨迹。
                        // EvaluateResidualBlock：利用这个初步轨迹，回代计算当前这个 GNSS 因子的 Cost。
                        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);  
                        chi2 = cost * 2; // Ceres 中的 cost 是残差平方和的一半，因此乘以 2 得到实际的 chi2 值

                        // 重加权策略 (Reweighting Strategy)
                        if (chi2 > chi2_threshold) {
                            gnss_outlier.insert(time);

                            // Reweigthed GNSS 放大噪声标准差，等效于降低权重
                            double scale = sqrt(chi2 / chi2_threshold);  // 比例因子等于实际 chi2 与阈值的比值的平方根
                            gnsslist[k].std *= scale;  // 乘以比例因子，放大标准差
                        }
                    }
                    // // Log outliers
                    // if (!gnss_outlier.empty()) {
                    //     std::string log = absl::StrFormat("Reweight GNSS outlier at %g:", sow - 1);
                    //     for (const auto& time:gnss_outlier) {
                    //         absl::StrAppendFormat(&log, " %g", time);
                    //     }
                    //     std::cout << log << std::endl;
                    // }

                    // 删除旧的GNSS因子
                    // Remove all old GNSS factors
                    for (const auto &block : gnss_residualblock_id) {
                        problem.RemoveResidualBlock(block.second); // 根据残差块ID，移除对应的GNSS残差块
                    }

                    // 重新添加GNSS因子，不使用损失函数
                    // Add GNSS factors without loss function
                    index = 0;
                    for (auto &gnss : gnsslist) {
                        auto factor = new GnssFactor(gnss, antlever);
                        for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                            if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                                problem.AddResidualBlock(factor, nullptr, statedatalist[i].pose);
                                index++;
                                break;
                            }
                        }
                    }
                    /* PS： 
                    1.为什么要先删再加？
                    因为 GnssFactor 的权重矩阵（信息矩阵）是在构造函数里根据 gnss.std 计算死的。
                    我们在上一步修改了 gnss.std，必须通过 new GnssFactor 重新生成因子，新的权重才会生效。
                    2.为什么 loss_function 变成了 nullptr？
                    第一阶段使用 HuberLoss 是为了防止粗差在这个阶段把轨迹拉得太远。
                    第二阶段（当前阶段）我们已经手动完成了粗差的降权（通过修改 std），这相当于手动实施了一个极其精确的 Robust Kernel，所以不再需要 Huber Loss 这种通用的核函数了，直接用最小二乘求解即可 
                    3.旧因子不可变：旧的 GnssFactor 对象里的权重是写死的，没法改。
                    4.必须换新：必须生成新的 GnssFactor 对象才能应用新的权重。
                    5.全量替换更简单：GNSS 因子很轻量，全删全加比精细维护“哪个要改哪个不用改”的逻辑更低成本，且不易出错。*/
                }

                    // 第二段优化
                options.max_num_iterations = num_iterations * 3 / 4; // 最大迭代次数设置为总迭代次数的3/4，第一段时1/4,共加起来为num_iterations
                solver.Solve(options, &problem, &summary);           // 再次调用求解器求解问题：Solve(options, &problem, &summary)

                /* PS：
                   1.GNSS因子的代价是指基于EvaluateResidualBlock函数，计算滑动窗口内各时刻GNSS因子的验后残差平方和，每个时刻的GNSS因子残差块对应一个验后残差平方和。
                   2.之所以分成两段，一是为了控制GNSS质量；二是快速获取一个初步的解并进行粗略优化，之后再细致优化，逐步提高解的精度。
                   3.执行Solve()函数时，Ceres 实际上是在无数次地循环调用上述 4 个因子中的Evaluate 和 **1 个Plus** （PoseManifold::Plus） 函数，直到误差收敛。
                */               

                // 输出进度
                // output the percentage
                int percent            = ((int) sow - starttime) * 100 / (endtime - starttime);
                static int lastpercent = 0;
                if (abs(percent - lastpercent) >= 1) {
                    lastpercent = percent;
                    std::cout << "Percentage: " << std::setw(3) << percent << "%\r";
                    flush(std::cout);
                }
            }

            // 滑窗与边缘化处理
            if (preintegrationlist.size() == static_cast<size_t>(windows)) { // 如果预积分列表大小等于滑动窗口大小，说明滑动窗口已满，需要进行边缘化处理
                {
                    // 边缘化
                    // marginalization
                    std::shared_ptr<MarginalizationInfo> marginalization_info = std::make_shared<MarginalizationInfo>();
                    if (last_marginalization_info && last_marginalization_info->isValid()) {

                        std::vector<int> marginilized_index;
                        for (size_t k = 0; k < last_marginalization_parameter_blocks.size(); k++) {
                            if (last_marginalization_parameter_blocks[k] == statedatalist[0].pose ||
                                last_marginalization_parameter_blocks[k] == statedatalist[0].mix) {
                                marginilized_index.push_back(static_cast<int>(k));
                            }
                        }

                        auto factor   = std::make_shared<MarginalizationFactor>(last_marginalization_info);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr, last_marginalization_parameter_blocks, marginilized_index);
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // IMU残差
                    // preintegration factors
                    {
                        auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
                        auto residual = std::make_shared<ResidualBlockInfo>(
                            factor, nullptr,
                            std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, statedatalist[1].pose,
                                                  statedatalist[1].mix},
                            std::vector<int>{0, 1});
                        marginalization_info->addResidualBlockInfo(residual);
                    }

                    // GNSS残差
                    // GNSS factors
                    {
                        if (fabs(timelist[0] - gnsslist[0].time) < MINIMUM_INTERVAL) {
                            auto factor   = std::make_shared<GnssFactor>(gnsslist[0], antlever);
                            auto residual = std::make_shared<ResidualBlockInfo>(
                                factor, nullptr, std::vector<double *>{statedatalist[0].pose}, std::vector<int>{});
                            marginalization_info->addResidualBlockInfo(residual);
                        }
                    }

                    // 边缘化处理
                    // do marginalization
                    marginalization_info->marginalization();

                    // 数据指针调整
                    // get new pointers
                    std::unordered_map<long, double *> address;
                    for (size_t k = 1; k <= preintegrationlist.size(); k++) {
                        address[reinterpret_cast<long>(statedatalist[k].pose)] = statedatalist[k - 1].pose;
                        address[reinterpret_cast<long>(statedatalist[k].mix)]  = statedatalist[k - 1].mix;
                    }
                    last_marginalization_parameter_blocks = marginalization_info->getParamterBlocks(address);
                    last_marginalization_info             = std::move(marginalization_info);
                }

                // 滑窗处理
                // sliding window
                {
                    if (lround(timelist[0]) == lround(gnsslist[0].time)) {
                        gnsslist.pop_front();
                    }
                    timelist.pop_front();
                    preintegrationlist.pop_front();

                    for (int k = 0; k < windows; k++) {
                        statedatalist[k] = statedatalist[k + 1];
                        statelist[k]     = Preintegration::stateFromData(statedatalist[k], preintegration_options);
                    }
                    statelist[windows] = Preintegration::stateFromData(statedatalist[windows], preintegration_options);
                    state_curr         = statelist[windows];
                }
            } else {
                state_curr =
                    Preintegration::stateFromData(statedatalist[preintegrationlist.size()], preintegration_options);
            }

            // write result
            writeNavResult(*timelist.rbegin(), station_origin, state_curr, navfile, errfile);

            // 新建立新的预积分
            // build a new preintegration object
            preintegrationlist.emplace_back(
                Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
        } else {
            // imu_cur.time <= sow,不进入积分，只记录轨迹点，然后继续读取下一个IMU数据并加入预积分 addNewImu()
            auto integration = *preintegrationlist.rbegin(); // 获取当前预积分对象
            writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile); // 输出当前状态到文件
        }
    }

    navfile.close();
    errfile.close();
    imufile.close();
    gnssfile.close();

    auto te = absl::Now();
    std::cout << std::endl << std::endl << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

    return 0;
}

void writeNavResult(double time, const Vector3d &origin, const IntegrationState &state, FileSaver &navfile,
                    FileSaver &errfile) {
    vector<double> result;

    Vector3d pos = Earth::local2global(origin, state.p);
    pos.segment(0, 2) *= R2D;
    Vector3d att = Rotation::quaternion2euler(state.q) * R2D;
    Vector3d vel = state.v;
    Vector3d bg  = state.bg * R2D * 3600;
    Vector3d ba  = state.ba * 1e5;

    {
        result.clear();

        result.push_back(0);
        result.push_back(time);
        result.push_back(pos[0]);
        result.push_back(pos[1]);
        result.push_back(pos[2]);
        result.push_back(vel[0]);
        result.push_back(vel[1]);
        result.push_back(vel[2]);
        result.push_back(att[0]);
        result.push_back(att[1]);
        result.push_back(att[2]);
        navfile.dump(result);
    }

    {
        result.clear();

        result.push_back(time);
        result.push_back(bg[0]);
        result.push_back(bg[1]);
        result.push_back(bg[2]);
        result.push_back(ba[0]);
        result.push_back(ba[1]);
        result.push_back(ba[2]);
        result.push_back(state.sodo);
        errfile.dump(result);
    }
}

// 示例调用：imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid; // mid即sow, 下一个积分节点时间

    double scale = (imu01.time - time) / imu01.dt; // scale: 后半段占总时间的比例
    IMU buff     = imu01; // 备份原始数据imu_cur

    // 前半段imu增量
    imu00.time   = time; // 从 t_start 到 mid 的 IMU 数据（用于将状态推进到 sow/GNSS 时刻)
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);

    // 后半段imu增量
    imu11.time   = buff.time; // 从 mid 到 t_end 的 IMU 数据（用于下一个积分周期的预备)
    imu11.dt     = buff.time - time;
    imu11.dtheta = buff.dtheta * scale;
    imu11.dvel   = buff.dvel * scale;
    imu11.odovel = buff.odovel * scale;
}

int isNeedInterpolation(const IMU &imu0, const IMU &imu1, double mid) {
    double time = mid; // mid即sow, 下一个积分节点时间

    if (imu0.time < time && imu1.time > time) {
        double dt = time - imu0.time;

        // 前一个历元接近
        // close to the first epoch
        if (dt < 0.0001) {
            return -1;
        }

        // 后一个历元接近
        // close to the second epoch
        dt = imu1.time - time;
        if (dt < 0.0001) {
            return 1;
        }

        // 需内插
        // need interpolation
        return 2;
    }

    // 不需内插
    // no need interpolation
    return 0;
}
