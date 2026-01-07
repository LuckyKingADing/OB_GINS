# OB_GINS学习笔记

## 后续跟进项目

1. [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) - 视觉惯导融合单目版本
2. [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) - 视觉惯导融合多传感器版本
3. IC_GINS - 惯导/GNSS组合导航
4. KF_GINS 与 OB_GINS 对比

## 参考学习文档

1. [知乎文章 - Iron Man](https://zhuanlan.zhihu.com/p/15928217258)（重点关注里面的论文）
2. [武汉大学i2nav研究室](http://i2nav.net/index/newListDetail_zw.do?newskind_id=f8990a24cf86440483d7821d9c2975c9&newsinfo_id=8226f3d32e8f4fa29184834c09762f5b)

## 核心概念

### 1. 轮式里程计（ODO）

**轮速计 vs 里程计的区别**

| 概念 | 定义 | 数据形式 |
|------|------|---------|
| 轮速计 | 车轮转速传感器（每个轮子） | 原始信号：转速/脉冲 |
| 里程计 | 基于轮速计计算的车辆运动信息 | 处理后：速度/距离（标量） |

**在OB_GINS中的实现**

- IMU数据文件第8列是 `odovel`（里程计速度）
- 处理流程：轮速传感器原始数据 → 处理 → 里程计模块 → 输出速度值
- 程序直接使用预处理后的里程计数据（标量）

**里程计数据属性**

- 数据来源：与IMU数据同步采集
- 存储位置：IMU结构体中的 `odovel` 字段
- 数据格式：浮点数，代表沿载体纵向的速度（m/s）
- 采样频率：与IMU同频率（配置中为200Hz）

### 2. 在组合导航中的作用

里程计主要用于补强INS在速度更新中的不足：

#### 位置更新补充
- INS只有加速度计，经过二次积分会产生累积误差
- 里程计提供直接的速度测量，有助于约束位置漂移

#### 零偏估计
- 系统状态中包含 `sodo`（里程计比例因子误差）
- 在优化中被估计和修正，提高里程计数据的可用性

#### GNSS中断补救
- 配置支持GNSS中断仿真（outage）
- 中断期间里程计成为仅有的辅助信息
- 与IMU配合维持导航解的连续性和精度

#### 特殊处理
- 进行坐标系转换：从载体系转换到导航系
- 考虑里程计杆臂 `lodo` 和安装角 `abv`
- 参与预积分计算，详见 `preintegration_odo.cc:206`

### 3. 随机游走 vs 标准差

**标准差（Standard Deviation）**
- 描述数据的离散程度
- 代表某个时刻的不确定度
- 单位与量本身相同：m, m/s, rad/s, deg/hr 等
- 是**静态**的状态不确定性度量

**随机游走（Random Walk）**
- 描述噪声随时间的累积性质
- 误差会随时间不断增长（$\propto \sqrt{t}$）
- 单位中包含时间：m/√s, m/s/√s, deg/√hr, PPM/√Hz 等
- 是**动态**的误差增长特性

**在导航系统中的应用**

| 参数 | 类型 | 单位 | 含义 |
|------|------|------|------|
| ARW (角度随机游走) | 随机游走 | deg/√hr, rad/√s | 陀螺噪声，偏差累积 |
| VRW (速度随机游走) | 随机游走 | m/s/√hr | 加速度计噪声，速度偏差累积 |
| 陀螺零偏标准差 | 标准差 | deg/hr | 零偏的初始不确定度 |
| 加表零偏标准差 | 标准差 | mGal | 零偏的初始不确定度 |
| 比例因子随机游走 | 随机游走 | PPM/√Hz | 比例因子误差增长 |

**在OB_GINS配置中的体现**

```yaml
imumodel:
    arw: 0.1           # deg/sqrt(hr) - 随机游走，陀螺噪声
    vrw: 0.1           # m/s/sqrt(hr) - 随机游走，加表噪声
    gbstd: 25.0        # deg/hr - 标准差，零偏初值不确定度
    abstd: 200.0       # mGal - 标准差，零偏初值不确定度
    corrtime: 1.0      # hr - 相关时间常数

odometer:
    odo_std: [0.05, 0.05, 0.05]  # m/s - 标准差，速度白噪声
    odo_srw: 100                  # PPM - 随机游走，比例因子误差
```

### 4. 连续状态积分与OB_GINS的特点

相比于KF_GINS，OB_GINS在连续状态积分（机械编排）中并没有计算中间时刻的速度、位置，更新投影参数。

**原因分析**
- OB_GINS面向MEMS惯导，精度相对较低，可以简略一些步骤
- KF_GINS面向高精度惯导，需要更精细的中间状态计算

### 5. 预积分（相对状态约束）

**概念**
- 利用相邻节点之间的IMU数据预先积分出与积分起点位姿无关的相对位置、速度、姿态增量
- 形成相邻时刻间的约束关系

**参考文献**
- 基于图优化的LiDAR/INS/ODO/GNSS车载组合导航算法研究
- 作者：常乐
- 学校：武汉大学
- 重点章节：第2章 2.4小节

### 6. 地球自转补偿项

#### **物理背景**

地球自转角速度：**ω_ie = 7.2921151467×10⁻⁵ rad/s**（约15°/小时）

虽然这个值很小，但在高精度惯导中会产生显著影响，特别是对于MEMS-IMU长时间积分。

#### **具体补偿内容**

##### **1. 哥氏效应（Coriolis Effect）**

在旋转参考系（地球）中，运动物体会受到哥氏力的影响：

**公式**：$-2\omega_{ie}^n \times v^n$

```cpp
// 哥氏项和重力项
Vector3d dv_cor_g = (gravity_ - 2.0 * iewn_.cross(current_state_.v)) * dt;
```

其中 `iewn_` 是地球自转角速度在导航系（n系）的投影：

$$\omega_{ie}^n = \begin{bmatrix} \omega_{ie} \cos\varphi \\ 0 \\ -\omega_{ie} \sin\varphi \end{bmatrix}$$

- $\varphi$ 是当地纬度
- 投影到北、东、天方向

##### **2. 姿态更新补偿**

导航系相对于惯性系旋转，姿态更新需要补偿这个旋转：

```cpp
// 地球自转补偿项, 省去了enwn项
Vector3d dnn    = -iewn_ * dt;                      // 旋转矢量
Quaterniond qnn = Rotation::rotvec2quaternion(dnn); // 转四元数

// 姿态更新
current_state_.q = qnn * current_state_.q * Rotation::rotvec2quaternion(dtheta);
```

##### **3. 预积分中的补偿**

在预积分计算相对位姿时，需要补偿地球自转影响：

```cpp
// 中间时刻的地球自转等效旋转矢量
dnn = -(delta_time_ - 0.5 * dt) * iewn_;
Matrix3d cbbe = (q0_.inverse() * Rotation::rotvec2quaternion(dnn) * 
                 q0_ * delta_state_.q).toRotationMatrix();
```

#### **为什么要考虑？**

| 场景 | 不考虑地球自转的影响 | 考虑地球自转的改善 |
|------|---------------------|-------------------|
| **短时间（秒级）** | 误差很小，可忽略 | 影响不大 |
| **中等时间（分钟级）** | 姿态误差累积开始显现 | 姿态精度提升 |
| **长时间（小时级）** | 姿态漂移显著，位置误差累积 | **显著提升精度** |
| **GNSS中断时** | 纯惯导误差快速增长 | **关键补偿项** |

#### **定量影响**

根据代码中引用的论文（Tang等，IEEE论文）：

对于MEMS-IMU：
- **不补偿**：姿态误差增长 ~15°/小时
- **补偿后**：姿态误差显著减小

#### **配置选项**

```yaml
isearth: true   # 启用地球自转补偿
```

- `true`：适用于长时间导航、高精度需求
- `false`：可用于短时间测试或低精度应用

#### **参考文献**

- **《Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization》**
- 作者：i2Nav团队 唐海亮博士
- 发表于：IEEE
- 该论文详细分析了地球自转补偿对MEMS惯导预积分的影响

#### **代码位置**

- 地球自转计算：`src/common/earth.h` - `iewn()` 函数
- 连续状态积分：`src/preintegration/preintegration_earth.cc:218-260`
- 预积分补偿：`src/preintegration/preintegration_earth_odo.cc:250-284`


 7. 初始状态转换为数据格式存入状态数据列表，因为ceres求解时使用的是数据格式，即double格式，而不是状态向量格式，vector3d和quaterniond格式

 8. 设置下一个积分的节点时刻sow（整秒），两个积分节点的间隔固定设置为1s，由于GNSS的采样间隔也是1s，因此sow也是GNSS数据的观测时刻。

9. 对比sow、上一历元imu_pre和当前历元imu_cur的IMU数据观测时间（isNeedInterpolation函数），分以下四种情况：

（1）sow新于imu_cur，只进行INS解算（integrationProcess函数）。

（2）sow靠近imu_pre，阈值是两个时刻小于0.0001s，先GNSS/INS松组合解算，再进行INS解算。

（3）sow靠近imu_cur，阈值同上，先INS解算，再GNSS/INS松组合解算。

（4）sow在imu_pre和imu_cur之间，先内插INS观测数据（imuInterpolation函数），再进行INS前半部分解算，进行GNSS/INS松组合解算，再进行INS后半部分解算。

Gemini Pro
**IMU内插处理逻辑（代码实现）**

`sow` 在这里是 **下一个积分节点时间** (Start of Week/Second of Week)，也就是我们需要对齐的目标时间点。`IMU` 数据是离散的，采样时间点不一定刚好落在 `sow` 上，所以需要处理。

代码中 `isNeedInterpolation` 函数判断 `sow` 落在 `imu_pre` (上一个IMU数据) 和 `imu_cur` (当前IMU数据) 之间的位置情况，返回值 `isneed` 定义了三种处理方式：

**1. `sow` 非常靠近 `imu_pre` (`isneed == -1`)**

- **情况**：目标时间点几乎就是上一帧的时间。
- **处理**：什么都不做。因为上一帧的时间点已经基本上就是我们需要的时间点了，直接使用上一帧作为结束即可（实际上上一轮循环可能已经处理了）。

**2. `sow` 非常靠近 `imu_cur` (`isneed == 1`)**

- **情况**：目标时间点几乎就是当前帧的时间。
- **处理**：直接把当前帧 `imu_cur` 加入到预积分中，并更新 `imu_pre` 和 `imu_cur` 为下一帧。这表示当前帧刚好就是我们要的积分结束点。
  ```cpp
  else if (isneed == 1) { // sow靠近imu_cur
      preintegrationlist.back()->addNewImu(imu_cur);
      imu_pre = imu_cur;
      imu_cur = imufile.next();
  }
  ```

**3. `sow` 在 `imu_pre` 和 `imu_cur` 中间 (`isneed == 2`)**

- **情况**：目标时间点夹在两帧之间，且都不靠近，需要精确分割。
- **处理**：调用 `imuInterpolation` 进行线性插值。
    - 将 `imu_cur` (这里传入参数名为 `imu01`) 拆分成两部分：
        - `imu00` (前一部分)：从 `imu_pre` 结束时刻 到 `sow`。这部分属于**当前积分周期**。
        - `imu11` (后一部分)：从 `sow` 到 `imu_cur` 结束时刻。这部分属于**下一个积分周期**。
    - 将前半部分 `imu00`（即 `imu_pre`）加入到当前的预积分对象中。
    - `imu_pre` 会被更新为后半部分 `imu11` (在函数内部通过引用修改)，以便下一个循环使用，作为下一周期的起始。

  ```cpp
  else if (isneed == 2) { // sow在imu_pre和imu_cur之间
      // imuInterpolation(原始数据, 输出的前半段, 输出的后半段, 分割时间点)
      imuInterpolation(imu_cur, imu_pre, imu_cur, sow);
      preintegrationlist.back()->addNewImu(imu_pre);
  }
  ```

**简单总结**
- **刚好切在当前帧尾巴** -> 直接用当前帧。
- **切在两帧中间** -> 把当前帧劈成两半，前半截给现在用，后半截留给下次用。

### 7. 数据处理主流程（src/ob_gins.cc）详解

整个系统基于一个主循环 `while(true)` 运行，核心逻辑是按时间顺序处理 IMU 数据，并以 `sow`（整秒时刻）为界限触发优化。

#### **核心逻辑全景图**

每次循环迭代代表处理 **一帧新的 IMU 数据**。

**步骤 1：加入 IMU 数据 (无条件)**
```cpp
preintegrationlist.back()->addNewImu(imu_cur);
```
- 不管是哪一帧，先塞进预积分器进行状态推算（机械编排）。此时系统的实时状态已更新。

**步骤 2：判断是否跨越了积分节点 (sow)**
```cpp
if (imu_cur.time > sow) {
    // 【分支A】跨越了节点 -> 触发优化、切分
} else {
    // 【分支B】没跨越节点 -> 仅记录中间轨迹
}
```

#### **【分支 A】 跨越了积分节点 (`if` 块)**
这意味着 `imu_cur` 的时间跑到了目标时刻 `sow` 后面，需要在这个整秒时刻进行“截断”和“结算”。

1. **对齐 GNSS**：读取并预处理整秒处的 GNSS 数据（粗差剔除、中断模拟等）。
2. **IMU 内插/精细处理**：调用 `isNeedInterpolation` 确保积分严格截止在 `sow` 时刻（可能需要将某一帧劈成两半）。
3. **构建与优化**：
   - 将对齐后的状态存入滑窗。
   - 构建因子图（添加位姿、GNSS、预积分、零偏等因子）。
   - 调用 Ceres Solver 进行求解。
4. **边缘化与重置**：
   - 滑窗满了则边缘化最老帧。
   - 生成新的预积分器，`sow` 增加 1.0 秒，准备下一轮。

#### **【分支 B】 没跨越节点 (`else` 块)**
处在两个整秒节点之间（例如 `sow=100`, `time=99.05`）。虽然不做优化，但需要输出**高频轨迹**。

```cpp
auto integration = *preintegrationlist.rbegin();
writeNavResult(..., integration->currentState(), ...);
```
- 直接获取当前预积分器的推算状态（包含刚才步骤1塞进去的数据）。
- 写入结果文件。这保证了输出轨迹是高频（如200Hz）的，而非仅有每秒的优化点。

#### **流程总结**

1. **一直吃数据**：不断把 IMU 数据塞进预积分器，同时输出高频推算轨迹（走 `else` 分支）。
2. **到点就算账**：一旦发现时间跨过了整秒 `sow`（走 `if` 分支），就进行内插截断、对齐 GNSS、执行优化修正，然后开启下一个周期。




