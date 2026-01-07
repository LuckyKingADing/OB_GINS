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
                // imuInterpolation(原始数据, 输出的前半段, 输出的后半段, 分割时间点)
                // 注意这里第二个和第三个参数分别是 imu_pre 和 imu_cur
                // 实际上是将 imu_cur 这个时间段的数据拆分，
                // 前半段存入 imu_pre (作为当前积分周期的最后一帧), 
                // 后半段存入 imu_cur (作为下个周期的第一帧预备)
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
                ceres::Problem::Options problem_options;
                problem_options.enable_fast_removal = true;

                ceres::Problem problem(problem_options);
                ceres::Solver solver;
                ceres::Solver::Summary summary;
                ceres::Solver::Options options;
                options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
                options.linear_solver_type         = ceres::SPARSE_NORMAL_CHOLESKY;
                options.num_threads                = 4;

                // 参数块
                // add parameter blocks
                for (size_t k = 0; k <= preintegrationlist.size(); k++) {
                    // 位姿
                    ceres::Manifold *manifold = new PoseManifold();
                    problem.AddParameterBlock(statedatalist[k].pose, Preintegration::numPoseParameter(), manifold);

                    problem.AddParameterBlock(statedatalist[k].mix,
                                              Preintegration::numMixParameter(preintegration_options));
                }

                // GNSS残差
                // GNSS factors
                int index = 0;

                ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
                std::vector<std::pair<double, ceres::ResidualBlockId>> gnss_residualblock_id;
                for (const auto &gnss : gnsslist) {
                    auto factor = new GnssFactor(gnss, antlever);
                    for (size_t i = index; i <= preintegrationlist.size(); ++i) {
                        if (fabs(gnss.time - timelist[i]) < MINIMUM_INTERVAL) {
                            auto id = problem.AddResidualBlock(factor, loss_function, statedatalist[i].pose);
                            gnss_residualblock_id.push_back(std::make_pair(gnss.time, id));
                            index++;
                            break;
                        }
                    }
                }

                // 预积分残差
                // preintegration factors
                for (size_t k = 0; k < preintegrationlist.size(); k++) {
                    auto factor = new PreintegrationFactor(preintegrationlist[k]);
                    problem.AddResidualBlock(factor, nullptr, statedatalist[k].pose, statedatalist[k].mix,
                                             statedatalist[k + 1].pose, statedatalist[k + 1].mix);
                }
                {
                    // IMU误差控制
                    // add IMU bias-constraint factors
                    auto factor = new ImuErrorFactor(*preintegrationlist.rbegin());
                    problem.AddResidualBlock(factor, nullptr, statedatalist[preintegrationlist.size()].mix);
                }

                // 边缘化残差
                // prior factor
                if (last_marginalization_info && last_marginalization_info->isValid()) {
                    auto factor = new MarginalizationFactor(last_marginalization_info);
                    problem.AddResidualBlock(factor, nullptr, last_marginalization_parameter_blocks);
                }

                // 求解最小二乘
                // solve the Least-Squares problem
                options.max_num_iterations = num_iterations / 4;
                solver.Solve(options, &problem, &summary);

                // TODO: Just a example, you need remodify.
                // Do GNSS outlier culling using chi-square test
                if (is_outlier_culling && !gnss_residualblock_id.empty()) {
                    // 3 degrees of freedom, 0.05
                    double chi2_threshold = 7.815;

                    // Find GNSS outliers in the window
                    std::unordered_set<double> gnss_outlier;
                    for (size_t k = 0; k < gnsslist.size(); k++) {
                        auto time = gnss_residualblock_id[k].first;
                        auto id   = gnss_residualblock_id[k].second;

                        double cost;
                        double chi2;

                        problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
                        chi2 = cost * 2;

                        if (chi2 > chi2_threshold) {
                            gnss_outlier.insert(time);

                            // Reweigthed GNSS
                            double scale = sqrt(chi2 / chi2_threshold);
                            gnsslist[k].std *= scale;
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

                    // Remove all old GNSS factors
                    for (const auto &block : gnss_residualblock_id) {
                        problem.RemoveResidualBlock(block.second);
                    }

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
                }

                options.max_num_iterations = num_iterations * 3 / 4;
                solver.Solve(options, &problem, &summary);

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

            if (preintegrationlist.size() == static_cast<size_t>(windows)) {
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

void imuInterpolation(const IMU &imu01, IMU &imu00, IMU &imu11, double mid) {
    double time = mid;

    double scale = (imu01.time - time) / imu01.dt;
    IMU buff     = imu01;

    imu00.time   = time;
    imu00.dt     = buff.dt - (buff.time - time);
    imu00.dtheta = buff.dtheta * (1 - scale);
    imu00.dvel   = buff.dvel * (1 - scale);
    imu00.odovel = buff.odovel * (1 - scale);

    imu11.time   = buff.time;
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
