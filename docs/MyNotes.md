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




 PS：相比于KF-GINS，OB_GINS并没有计算中间时刻的速度、位置，更新投影参数，应该还是因为面向的惯导类型不同，OB_GINS面向的惯导是MEMS，可以简略一些步骤，而KF-GINS面向的惯导是高精度的。

在预积分时， ODO预积分公式涉及两个坐标系，v系和w系：
v系： vehicle车辆坐标系
w：world世界坐标系，imu坐标系
           v系是以ODO安装所在车轮与地面的切点为原点。
           w系的原点不变，一直在初始位置，固定在k-1时刻的e系下。

### 8. 预积分中的坐标系与核心公式解析

#### **核心思想**
预积分的目的是将一连串的 IMU/ODO 测量值，压缩成相对于**“积分起点时刻（$k-1$）”**的增量。所有的测量值（加速度、角速度、里程计速度）都需要投影到同一个参考系下才能进行累加。

#### **两个关键坐标系**
1. **v系 (Vehicle/Odometer Frame)**
   - **定义**：里程计安装位置所在的坐标系。
   - **原点**：车轮与地面的接触点。
   - **用途**：里程计原始读数（前进速度）是在这个坐标系下的。

2. **w系 / b_start系 (World/Inertial Frame)**
   - **定义**：**固定在积分起点时刻（$k-1$时刻）的载体坐标系**。
   - **特性**：它是一个**惯性系**。这意味着在物理空间中，它是“冻结”的，不随地球自转而转动。
   - **几何关系**：在 $k-1$ 时刻，它与当时的载体坐标系重合；随着时间推移，载体动了，地球也转了，但这个系保持惯性不动。

#### **关键代码解析**

```cpp
// 1. 计算地球自转引起的旋转矢量 (n系下)
dnn = -(delta_time_ - 0.5 * dt) * iewn_;

// 2. 构建核心投影矩阵 cbbe (Current Body -> Start Body)
Matrix3d cbbe = (q0_.inverse() * Rotation::rotvec2quaternion(dnn) * q0_ * delta_state_.q).toRotationMatrix();
```

**`dnn` 的含义**：
- 计算从积分起点到当前中间时刻，地球自转导致的角度变化。
- 负号表示这是一个补偿项（逆向旋转），目的是消除地球自转的影响，维持惯性系的假设。

**`cbbe` ($C_{b_{start}}^{b_{current}}$) 的含义**：
- 这是一个**从当前载体坐标系到积分起点惯性系**的旋转矩阵。
- **公式拆解**：
  $$ \mathbf{C}_{final} = (\mathbf{q}_{nb_0}^{-1} \cdot \mathbf{q}_{earth} \cdot \mathbf{q}_{nb_0}) \cdot \Delta \mathbf{q}_{gyro} $$
  - `delta_state_.q`: 纯陀螺仪积分得到的相对旋转。
  - `q0_inv * dnn * q0`: 将地球自转补偿量从 **n系** 变换到 **b_start系**。
- **作用**：将当前时刻测得的物理量（如加速度 `dvfb`、里程计速度 `dsodo`）投影回**b_start系**进行累加。

#### **预积分累加过程**

**1. ODO 预积分**
```cpp
// 修正杆臂效应 + 投影回 b_start 系
delta_state_.s += cbbe * (cvb_ * dsodo * (1 + sodo) + lever_arm_comp);
```
- 先把里程计速度 `dsodo` 转到 IMU 系 (`cvb_`)。
- 加上杆臂效应修正（`lever_arm_comp`）。
- 最后乘上 `cbbe`，统一投影到起点系累加。

**2. INS 速度预积分**
```cpp
dvel = cbbe * dvfb; 
delta_state_.v += dvel; // 这里的 v 是在 b_start 系下的速度增量
```
- `dvfb`: 当前 IMU 系下的比力积分（速度增量）。
- `cbbe`: 投影到起点系。

### 9. 预积分结果 (delta_state_) 的含义与作用

#### **delta_state_ 中的 PVQ 是什么？**

`delta_state_` 结构体存储了从 **预积分起始时刻 ($k-1$)** 到 **当前时刻 ($k$)** 的**相对状态增量**。

- **`delta_state_.p` (相对位移)**
  - 不是物理空间的两点距离，而是在 **$b_{start}$ (w) 系** 下累积的位移增量。
  - 物理含义：假设初始速度为0、位置为0，纯粹由加速度计推算出的位移。
  - 核心公式项：$\sum [\mathbf{v}_t \Delta t + \frac{1}{2} \mathbf{a}_t \Delta t^2]$

- **`delta_state_.v` (相对速度增量)**
  - 在 **$b_{start}$ (w) 系** 下的速度变化量。
  - 物理含义：纯惯性测量带来的速度改变。
  - 核心公式项：$\sum \mathbf{a}_t \Delta t$

- **`delta_state_.q` (相对旋转)**
  - 从 **$b_{start}$ (w) 系** 到 **$b_{current}$ 系** 的旋转四元数。
  - 物理含义：这段时间内陀螺仪积分得到的总旋转量（已剔除地球自转）。

#### **预积分结果如何使用？**

预积分的目的是构建 **优化因子 (Factor)**，充当非线性优化中的 **“测量约束”**。

在 `evaluate` 函数中，系统通过计算 **残差 (Residuals)** 引导优化：

> **残差 = 状态变量推算出的增量 - 预积分测得的增量**

**1. 位置残差**
```cpp
// 理论相对位移 (考虑了初始速度、重力影响，并转到 b_start 系)
Vector3d theory_dp = cnb0 * (p_j - p_i - v_i * dt - 0.5 * g * dt * dt);
// 残差
residual_p = theory_dp - delta_state_.p;
```

**2. 速度残差**
```cpp
// 理论相对速度变化
Vector3d theory_dv = cnb0 * (v_j - v_i - g * dt);
// 残差
residual_v = theory_dv - delta_state_.v;
```

**3. 总结**
- `delta_state_` 把成百上千次高频 IMU 数据压缩成了一个 **虚拟测量值**。
- 它充当了 **“尺子”**：告诉优化器，$i$ 时刻到 $j$ 时刻，物体根据惯性计“应该”发生了多大的相对运动。
- 优化器调整全局状态 ($p, v, q, bg, ba$)，使得所有时刻的相对运动都尽可能符合这把“尺子”的度量。

### 10. 误差传播函数 (updateJacobianAndCovariance) 解析

该函数核心作用是 **递推更新系统状态误差的雅可比矩阵（Jacobian）和协方差矩阵（Covariance）**。

#### **背景**
在后端优化时，我们需要回答两个问题：
1. **Result Correction**: 如果零偏 bias 被微调了，预积分结果需要改变量是多少？（需要 Jacobian）
2. **Weighting**: 这次预积分结果的可信度是多少？（需要 Covariance）

#### **核心原理**
基于线性化的误差传播离散方程：
$$ \delta \mathbf{x}_{k} \approx \boldsymbol{\Phi}_{k, k-1} \delta \mathbf{x}_{k-1} + \mathbf{G}_{k-1} \mathbf{n}_{k-1} $$

其中：
- $\boldsymbol{\Phi}$ (Phi)：状态转移矩阵，描述上一时刻误差如何传导到当前时刻（例如速度误差随时间变成位置误差）。
- $\mathbf{G}$：噪声驱动矩阵，描述传感器白噪声如何进入系统。

#### **函数流程**

**Step 1: 构造状态转移矩阵 $\boldsymbol{\Phi}$**
- 一个 $19 \times 19$ 的大矩阵，体现物理规律。
- 典型项：
  - $\frac{\partial \mathbf{p}}{\partial \mathbf{v}} = I \cdot \Delta t$
  - $\frac{\partial \mathbf{v}}{\partial \mathbf{b}_a} = -C_{b}^{b_0} \Delta t$

**Step 2: 递推更新雅可比 $\mathbf{J}$**
```cpp
// 链式法则累积
jacobian_ = phi * jacobian_;
```
- 作用：后端优化调整 Bias 时，直接利用 $J$ 进行一阶近似修正，避免重积分。

**Step 3: 递推更新协方差 $\mathbf{P}$**
```cpp
// 离散KF预测公式: P = Phi * P * Phi^T + Q
covariance_ = phi * covariance_ * phi.transpose() + Qk;
```
- 作用：量化预积分测量的不确定度。协方差越小，优化时该因子的权重（信息矩阵）越大。

### 11. IMU 数据插值与对齐

在 GNSS/INS 融合中，传感器的采样时间和积分周期往往不一致。代码中的 `imuInterpolation` 函数处理了这个问题。

#### **核心问题**
当一个新的 GNSS 观测到达（时刻 `sow`），我们需要把系统状态推进到这个精确时刻。然而，通常这个时刻会落在一个 IMU 采样间隔中间：
`imu_cur.start < sow < imu_cur.end`

#### **插值逻辑图解**

```text
时间轴:     t_start (上一帧结束) -------------> mid (sow) -------------> t_end (当前帧结束)
              |                                  ^                         |
              |<-------- imu00 (前半段) -------->|<------- imu11 (后半段) ------->|
              |                                  |                         |
原始数据:      |<-------------------------- imu01 (跨越帧) ------------------------->|
```

#### **代码变量复用技巧**
在 `src/ob_gins.cc` 中，作者巧妙（但容易让人困惑）地复用了变量名：

```cpp
// 此时 imu_cur 是跨越 sow 的那个帧
// imu_pre 是上一帧（已经被加入预积分了，此时是“旧值”）

// 执行插值：
// 输入：imu_cur (跨越多帧)
// 输出1：imu_pre (被覆写为前半段) -> 用于完成本轮预积分
// 输出2：imu_cur (被覆写为后半段) -> 留给下一轮预积分作为起始
imuInterpolation(imu_cur, imu_pre, imu_cur, sow);

// 将前半段加入预积分，本轮积分结束
preintegrationlist.back()->addNewImu(imu_pre);
```

#### **数学假设**
采用线性假设 (Linear Interpolation)：
$$ \text{Scale} = \frac{t_{end} - t_{mid}}{t_{end} - t_{start}} $$
$$ \Delta \theta_{front} = \Delta \theta_{total} \times (1 - \text{Scale}) $$
$$ \Delta v_{front} = \Delta v_{total} \times (1 - \text{Scale}) $$
假设物体在这一短时间内（例如 0.005s）做匀加速直线运动和匀速转动。

### 12. 边缘化 (Marginalization) 与滑动窗口管理

在 OB_GINS（以及 VINS 等基于滑窗的系统）中，边缘化是限制计算量、维持系统实时性的核心机制。它通过“Schur 补”数学操作，将移出窗口的旧状态所包含的信息，转化为对剩余状态的“先验约束”，从而避免直接丢弃旧帧导致的信息丢失。

#### **边缘化流程详解**

##### **1. 判定窗口状态**
在主循环中，系统会检查当前预积分列表的大小是否达到了设定的窗口长度（`windows`）。
```cpp
if (preintegrationlist.size() == static_cast<size_t>(windows)) {
    // 窗口已满，准备执行边缘化
}
```

##### **2. 构建边缘化信息 (Construct MarginalizationInfo)**
首先创建一个 `MarginalizationInfo` 对象，这相当于一个容器，用于收集所有与“将要被移除的状态”有关的约束。

**关键步骤：**
1.  **确定移除目标**：通常是最早的一帧（第0帧）。
    - 移除参数：`statedatalist[0].pose`, `statedatalist[0].mix`。
2.  **收集相关因子 (Factors)**：找出所有连接到第0帧的残差块。
    - **上一次的边缘化因子** (`last_marginalization_info`)：这是一个递归过程，包含了更早之前的先验信息。
    - **预积分因子** (`PreintegrationFactor`)：连接第0帧和第1帧的约束。
    - **GNSS因子** (`GnssFactor`)：直接观测第0帧位置的约束。

```cpp
auto factor   = std::make_shared<PreintegrationFactor>(preintegrationlist[0]);
auto residual = std::make_shared<ResidualBlockInfo>(
    factor, nullptr,
    std::vector<double *>{statedatalist[0].pose, statedatalist[0].mix, ...}, // 参数块列表
    std::vector<int>{0, 1} // 这里的0, 1表示参数块列表中第0和第1个参数（即第0帧状态）是需要被边缘化的
);
marginalization_info->addResidualBlockInfo(residual);
```

##### **3. 执行边缘化 (Perform Marginalization)**
调用 `marginalization_info->marginalization()`。这是最耗时的数学计算步骤。

- **构造 H 矩阵**：将收集到的所有因子线性化，拼成一个巨大的 Hessian 矩阵 ($H$) 和残差向量 ($b$)。
- **Schur 补操作**：将变量分为两部分：
    - $x_m$ (Marginalized): 即将被移除的变量（第0帧）。
    - $x_r$ (Remained): 需要保留的变量（与第0帧有联系的第1帧、第2帧...）。
    - 系统通过对 $x_m$ 求导并代入，得到一个仅关于 $x_r$ 的新线性方程：
      $$ H_{new} = H_{rr} - H_{rm} H_{mm}^{-1} H_{mr} $$
      $$ b_{new} = b_{r} - H_{rm} H_{mm}^{-1} b_{m} $$

这步操作的结果 ($H_{new}, b_{new}$) 就是我们所谓的“边缘化信息”，它作为一个先验约束，包含了 $x_m$ 曾经存在过的证据。

##### **4. 状态指针调整**
边缘化完成后，第0帧的内存会被释放。但我们的先验信息可能还引用着原来的指针地址。需要建立映射关系，将先验信息中的旧指针指向数据移动后的新指针。
- 在 `preintegrationlist.pop_front()` 调用后，原来的 `statedatalist[1]` 变成了现在的 `statedatalist[0]`。
- 必须通过 `address` map 告诉边缘化信息模块这个变动。

##### **5. 滑动窗口位移 (Shift)**
物理上移除数据：
```cpp
gnsslist.pop_front();          // 移除最老GNSS
timelist.pop_front();          // 移除最老时间戳
preintegrationlist.pop_front(); // 移除最老预积分

// 数据前移
for (int k = 0; k < windows; k++) {
    statedatalist[k] = statedatalist[k + 1]; // [1]->[0], [2]->[1]...
}
```

##### **6. 注入下一轮优化**
在下一轮循环构建 `ceres::Problem` 时，上一步计算出的 `last_marginalization_info` 会被封装成一个 `MarginalizationFactor` 加入问题中。

```cpp
if (last_marginalization_info) {
    auto factor = new MarginalizationFactor(last_marginalization_info);
    problem.AddResidualBlock(
        factor, 
        nullptr, 
        last_marginalization_parameter_blocks // 连接到幸存的参数块上
    );
}
```
**注意：** `MarginalizationFactor` 是一个**变长参数**的因子。它的维度是不固定的，取决于边缘化时这一帧到底和多少其他帧产生了联系。

### 13. GNSS 粗差剔除 (Outlier Culling)

OB_GINS 采用了一种 **“两阶段优化 + 重加权（Reweighting）”** 的策略处理 GNSS 观测中的异常值（如多路径效应、信号遮挡导致的跳变）。

#### **处理流程**

1.  **第一阶段优化**：正常构建因子图，使用 Huber 核函数防止巨大粗差拉崩系统，进行第一次求解。
2.  **卡方检验 (Chi-Square Test)**：利用第一阶段的初步解，回代计算每个 GNSS 观测的残差，判断其是否符合统计规律。
3.  **重加权 (Reweighting)**：对判定为粗差的观测值，通过放大其方差（即降低权重）来抑制其影响。
4.  **第二阶段优化**：移除旧的 GNSS 因子，使用调整权重后的新因子重建因字图，进行最终求解。

#### **关键代码逻辑**

##### **1. 设定统计阈值**
```cpp
// 3自由度 (x,y,z)，置信度 95% (p=0.05)
double chi2_threshold = 7.815; 
```
如果一个测量值的误差平方和（马氏距离）超过 7.815，我们有 95% 的把握认为它不仅仅是噪声，而是粗差。

##### **2. 验算残差 (Evaluation)**
```cpp
problem.EvaluateResidualBlock(id, false, &cost, nullptr, nullptr);
double chi2 = cost * 2; // Ceres cost is 1/2 * r^2
```
使用 `EvaluateResidualBlock` 可以在不重新优化的情况下，计算当前状态下某个因子的残差值。

##### **3. 降权策略**
```cpp
if (chi2 > chi2_threshold) {
    // 放大噪声标准差，等效于降低权重
    // Weight ~ 1/std^2
    double scale = sqrt(chi2 / chi2_threshold);
    gnsslist[k].std *= scale; 
}
```
这是一种温和的抗差手段。不直接删除数据，而是将其投影到阈值边界上。例如，如果误差是阈值的 4 倍，方差就放大 2 倍（权重减小为 1/4），限制其对系统的拉扯力。

##### **4. 重构因子图**
由于因子的权重矩阵通常在构造函数中固定，修改 `std` 后需要**销毁旧因子，创建新因子**。
```cpp
// 移除旧因子
problem.RemoveResidualBlock(block.second);

// 添加新因子 (注意：此时通常不再使用 Huber 核函数，因为已经手动处理了粗差)
auto factor = new GnssFactor(gnss, antlever);
problem.AddResidualBlock(factor, nullptr, ...);
```

### 14. Ceres Solver `Solve` 内部执行流程详解

当调用 `solver.Solve(options, &problem, &summary)` 时，Ceres 并不只是简单的“计算一下”。它在后台执行了一套严密的工业级优化流程。

#### **Phase 1: 预处理 (Preprocessing)**

在真正开始数学迭代之前，Ceres 需要将用户友好的 `Problem` 对象转换为求解器高效的内部表示。

1.  **修剪与剔除 (Pruning)**
    *   移除没有任何残差块连接的孤立参数块。
    *   移除被标记为 `Constant` 的参数块。
    *   **检查数据有效性**：验证所有指针非空，所有数据非 NaN。

2.  **构建程序 (Program Construction)**
    *   将分散在内存各处的参数块（`double*`）拷贝到一个连续的**状态向量 (State Vector)** 中。
    *   将所有高维参数（如四元数 4维）转换为流形上的切空间参数（如旋转矢量 3维）。这就是 `Manifold` 发挥作用的地方。

3.  **线性代数准备 (Linear Algebra Setup)**
    *   **重排序 (Ordering)**：对参数块进行重新排序（如使用 AMD 算法），目的是减小稀疏矩阵分解（如 Cholesky）时的填充元，提高求解速度。
    *   **Schur 消除准备**：对于 VIO 问题，通常会识别出特征点（Landmarks）和相机位姿（Poses）的稀疏结构，准备使用 Schur 补来加速求解。

#### **Phase 2: 最小化循环 (Minimizer Loop)**

OB_GINS 使用的是 **Levenberg-Marquardt (LM)** 算法，这是一个“信赖域”方法。主要循环如下：

1.  **评估当前状态 (Evaluate)**
    *   调用所有 `CostFunction::Evaluate()`（包括我们写的 `PreintegrationFactor::Evaluate`）。
    *   计算总残差 $r(x)$ 和 总代价 Cost $\frac{1}{2}\|r(x)\|^2$。
    *   计算雅可比矩阵 $J(x)$。

2.  **构建线性系统 (Linear System Construction)**
    *   构建正规方程（Normal Equations）：
        $$(J^T J + \mu I) \Delta x = -J^T r$$
    *   其中 $\mu$ 是阻尼因子（LM 核心参数），用于在高斯牛顿法（$\mu \to 0$）和梯度下降法（$\mu \to \infty$）之间切换。

3.  **求解步长 (Solve Step)**
    *   使用线性求解器（如 `SPARSE_NORMAL_CHOLESKY`）解上述方程，得到建议的更新步长 $\Delta x$。

4.  **并验证 (Update & Verify)**
    *   **试探更新**：$x_{new} = x \oplus \Delta x$（$\oplus$ 表示在流形上的加法）。
    *   **评估新状态**：计算新的 Cost。
    *   **接受/拒绝 (Trust Region Step)**：
        *   如果 Cost 下降足够多（$\rho > \text{threshold}$）：**接受更新**，$x \leftarrow x_{new}$，减小阻尼 $\mu$（更激进）。
        *   如果 Cost 没怎么降甚至升了：**拒绝更新**，保持 $x$ 不变，增大阻尼 $\mu$（更保守），重新回到第 2 步。

5.  **检查终止条件**
    *   是否达到最大迭代次数（`max_num_iterations`）？
    *   梯度是否足够小（`gradient_tolerance`）？
    *   步长是否足够小（`parameter_tolerance`）？
    *   Cost 变化是否足够小（`function_tolerance`）？
    *   满足任一条件即退出。

#### **Phase 3: 后处理 (Post-processing)**

1.  **回写数据 (Writeback)**
    *   将优化器内部连续向量中的最优解，拷贝回用户最初传入的 `double*` 参数块中。
    *   如果使用了 `Manifold`，会将切空间的增量（3维）正确地应用到原始状态（四元数 4维）上。

2.  **生成报告 (Summary)**
    *   填充 `ceres::Solver::Summary` 对象。
    *   包含：总耗时、各阶段耗时、最终 Cost、迭代次数、收敛原因等。

#### **简要总结图**

```mermaid
graph TD
    A[用户调用 Solve] --> B[预处理: 拷贝数据, 重排序]
    B --> C{LM 迭代循环}
    C -->|1. 计算 J, r| D[构建方程 (J'J + uI)dx = -J'r]
    D -->|2. 求解 dx| E[计算新Cost]
    E -->|3. 比较 Cost| F{更好了?}
    F -->|Yes| G[接受 dx, 减小 u]
    F -->|No| H[拒绝 dx, 增大 u]
    G --> I{满足终止条件?}
    H --> D
    I -->|No| C
    I -->|Yes| J[后处理: 回写 double*]
    J --> K[返回]
```

#### **关键重载函数调用 (Key Valid Overrides)**

在 `solver.Solve` 运行期间，Ceres 会不断回调（Callback）我们在代码中定义的重载函数。主要分为两类：**残差计算 (Evaluate)** 和 **流形更新 (Manifold)**。

**1. 因子评估 (Evaluate Phase)**
每当 LM 算法需要计算当前状态的残差 $r$ 或雅可比 $J$ 时（即流程图中的步骤 C），它会遍历所有添加的 ResidualBlock，并调用其对应的 `CostFunction::Evaluate`。

在 OB_GINS 中，以下函数会被高频调用：

*   **`GnssFactor::Evaluate`**
    *   **位置**：`src/factors/gnss_factor.h` Line 44
    *   **作用**：计算 $r_{gnss} = p_{meas} - p_{est}$，以及对位置 $p$ 的雅可比。
    
*   **`PreintegrationFactor::Evaluate`**
    *   **位置**：`src/preintegration/preintegration_factor.h` Line 45
    *   **作用**：计算 IMU 预积分残差（位置、速度、姿态、零偏误差），及其对前后两帧状态的雅可比。这是计算量最大的部分。

*   **`ImuErrorFactor::Evaluate`**
    *   **位置**：`src/preintegration/imu_error_factor.h` Line 40
    *   **作用**：计算零偏先验误差 $r_{bias} = b / \sigma$，用于限制零偏大小。

*   **`MarginalizationFactor::Evaluate`**
    *   **位置**：`src/factors/marginalization_factor.h` Line 47
    *   **作用**：计算边缘化先验误差 $r_{marg} = \bar{r} + J \Delta x$，引入历史约束。

**2. 状态更新 (Update Phase)**
当 LM 算法计算出更新步长 $\Delta x$ 后，准备尝试更新状态 $x_{new} = x \oplus \Delta x$ 时（即流程图中的步骤 E 和 G），它会查看该参数块是否绑定了 `Manifold`。

在 OB_GINS 中，只有 **Pose (位姿)** 参数块绑定了 `PoseManifold`，因此会调用：

*   **`PoseManifold::Plus`**
    *   **位置**：`src/factors/pose_manifold.cc` Line 34
    *   **作用**：定义位姿的加法。
        *   位置：普通加法 $p_{new} = p + \Delta p$
        *   **姿态**：四元数乘法 $q_{new} = q \otimes Exp(\Delta \theta)$（将切空间中的旋转失量增量 $\Delta \theta$ 映射回旋转群并应用）。
        *   **重要性**：保证四元数始终保持单位长度，且符合旋转群 $SO(3)$ 的几何结构。

*   **`PoseManifold::PlusJacobian`**
    *   **位置**：`src/factors/pose_manifold.cc` Line 51
    *   **作用**：计算上述加法操作相对于 $\Delta x$ 的导数。主要用于雅可比矩阵的链式法则修正（即使 Ceres 能够数值微分，提供解析解也能提高精度和速度）。

**3. 普通加法**
对于 **Mix (速度+零偏)** 参数块，代码中没有为其指定 Manifold。
*   **行为**：Ceres 默认直接进行欧氏空间加法（vector addition）。
    *   $v_{new} = v + \Delta v$
    *   $b_{new} = b + \Delta b$
*   这也是为什么你看不到 `MixManifold` 的原因。




#### **15. 核心因子 Evaluate 实现详解**

`Evaluate()` 是优化器计算“误差（Residuals）”和“梯度（Jacobians）”的最底层逻辑。在 LM 循环中被高频调用。

**1. GNSS 因子 (`GnssFactor`)**
*   **物理意义**: 几何约束。让 **IMU预测的位置 + 杆臂修正** 尽可能接近 **GNSS测量位置**。
*   **公式**: $\mathbf{r} = \mathbf{p}_{WB} + \mathbf{R}_{WB} \cdot \mathbf{l}_{G} - \mathbf{p}_{GNSS}$
*   **代码逻辑**:
    1.  提取状态 $p, q$。
    2.  计算残差 $error = p + R \cdot l - p_{gnss}$。
    3.  **白化 (Whitening)**: 乘以权重 $W = \Sigma^{-1/2}$。

**2. 预积分因子 (`PreintegrationFactor`)**
*   **物理意义**: 动力学约束。约束相邻两帧 $i, j$ 之间的相对运动（位置、速度、姿态），使其符合 IMU 积分结果。
*   **代码逻辑**:
    *   这是一个 **代理 (Proxy)** 类。实际计算委托给 `preintegration_->evaluate()`。
    *   输入涉及 4 个参数块：$Pose_i, Mix_i, Pose_j, Mix_j$。
    *   这是计算量最大的部分，涉及预积分误差求导。

**3. IMU 误差因子 (`ImuErrorFactor`)**
*   **物理意义**: 统计约束。限制零偏 (Bias) 不发生剧烈漂移（模拟高斯随机游走）。
*   **公式**: $\mathbf{r} = \mathbf{b} / \sigma_{rw}$。
*   **作用**: 防止因缺乏观测导致 Bias 估计发散。

**4. 边缘化因子 (`MarginalizationFactor`)**
*   **物理意义**: 历史信息约束。代表“被移除的旧帧对当前剩余帧的约束”。
*   **原理**: 基于线性化点 $x_0$ 的一阶泰勒展开。
*   **公式**: $\mathbf{r}(x) = \mathbf{r}_0 + \mathbf{J}_0 (x - x_0)$
*   **特点**: 它的雅可比 $\mathbf{J}$ 是常数（即 $\mathbf{J}_0$），对应一个固定的二次型 Cost。

---

#### **16. 雅可比矩阵 (Jacobian) 深度解析**

雅可比矩阵是优化器的“指南针”，告诉 Solver 应该往哪个方向修改参数才能减小误差。如果 $J$ 为正，说明参数增大误差也会增大，应减小参数；反之亦然。

以 `GnssFactor` 的雅可比实现为例：

**残差公式**: $\mathbf{r} = \mathbf{p} + \mathbf{R} \cdot \mathbf{l}_{lever} - \mathbf{p}_{gnss}$

**代码片段**:
```cpp
// 1. 对位置 p 的导数
jacobian_pose.block<3, 3>(0, 0) = Matrix3d::Identity();

// 2. 对旋转(四元数)的导数
jacobian_pose.block<3, 3>(0, 3) = -q.toRotationMatrix() * Rotation::skewSymmetric(lever_);
```

**解析**:

1. **对位置的导数 (Identity)**
    *   **数学**: $\frac{\partial (\mathbf{p} + \dots)}{\partial \mathbf{p}} = \mathbf{I}$
    *   **物理含义**: 如果位置估计偏了 1 米，残差也会偏 1 米。这是一个 1:1 的线性关系。直接平移修正即可。

2. **对旋转的导数 (杆臂效应 Lever Arm Effect)**
    *   **物理含义**: 当转动 IMU 时，虽然 IMU 中心没变，但由于**杆臂**的存在，连接在杆顶的 GNSS 天线会画弧移动。
    *   **数学推导**:
        *   旋转微扰: $\mathbf{R}_{new} \approx \mathbf{R}(\mathbf{I} + [\delta \theta]_{\times})$
        *   带入误差项: $\mathbf{R}_{new}\mathbf{l} \approx \mathbf{R}\mathbf{l} - \mathbf{R}[\mathbf{l}]_{\times} \delta \theta$
        *   求导结果: $- \mathbf{R} \cdot [\mathbf{l}]_{\times}$
    *   **作用**: 告诉优化器，“如果位置对不上，可能是姿态歪了，请根据杆臂长度算个旋转角度修正回来”。

3. **维度不匹配的处理**
    *   参数块虽然是 7 维 (3 pos + 4 quat)，但雅可比是 6 维 (3 pos + 3 rotation vector)。
    *   代码中 `jacobian_pose.setZero()` 清空矩阵后，只填充对应的 $3 \times 6$ 区块。对于四元数的第 4 维（实部），在流形切空间中不存在，故对应的导数为 0（或隐含处理）。

#### **17. 优化流程大揭秘：谁在计算修正量？**

一个常见的误区是认为 `Evaluate()` 负责计算“怎么修正状态”。事实上，**`GnssFactor` 及其 `Evaluate()` 函数只负责“告状”，不负责“判案”**。

“计算修正量”这一步是在 **Ceres 求解器的内部核心（Linear Solver）** 统一计算的。

**分工流程如下：**

**1. 告状阶段 (Callback: `Evaluate`)**
*   你的代码 (`GnssFactor`) 做两件事：
    *   **提交残差 ($r$)**：“报告长官，现在的估计位置比 GNSS 测量位置偏了 0.5 米！”
    *   **提交雅可比 ($J$)**：“报告长官，如果现在的姿态转动 1 度，会导致那个位置移动 2 米（由杆臂长度决定的变化率）！”
*   **注意**: 此时并没有计算“到底要转几度”。只提供了**当前误差**和**梯度（斜率）**。

**2. 判案阶段 (Internal: Linear Solver)**
*   当 Ceres 收集了所有因子（GNSS、预积分、IMU误差等）的 $r$ 和 $J$ 后，它会构建巨大的线性方程组（Normal Equation）：
    $$ (\mathbf{J}^T \mathbf{J} + \lambda \mathbf{I}) \Delta \mathbf{x} = -\mathbf{J}^T \mathbf{r} $$
*   **这里才是计算修正量的地方！**
*   Ceres 会综合考虑 GNSS 想让你往左转，预积分想让你往右转，最后通过求解这个方程，算出一个**全局最优的修正量 $\Delta \mathbf{x}$**（包含位置修正 $\Delta p$ 和角度修正 $\Delta \theta$）。
*   **直观理解**: 
    *   `Evaluate(J)` 说：转 1 度能移 2 米 ($Slope = 2$)。
    *   `Evaluate(r)` 说：现在偏了 0.5 米 ($Error = 0.5$)。
    *   `Solver` 算：$0.5 / 2 = 0.25$。所以决定修正 **0.25 度**。

**3. 执行阶段 (Callback: `Plus`)**
*   Solver 算出修正量 $\Delta \theta$ 后，调用 `PoseManifold::Plus`。
*   执行数学运算：`q_new = q_old * Exp(delta_theta)`，将修正量应用到状态上。

### 18. 预积分Evaluate深度解析：残差构建与线性化校正

在 `PreintegrationEarthOdo::evaluate` 函数中，我们看到了两类核心逻辑：一类是构建**物理残差**，另一类是执行**线性化校正**。

#### **1. 残差项逐行物理含义解析**

```cpp
// 1. 零偏随机游走约束 (Bias Random Walk)
residual.block<3, 1>(9, 0)  = state1.bg - state0.bg;
residual.block<3, 1>(12, 0) = state1.ba - state0.ba;
```
- **物理含义**：假设零偏随时间缓慢变化（高斯马尔可夫过程）。
- **约束**：“下一时刻的零偏”应该非常接近“上一时刻的零偏”。
- **坐标系**：无（传感器内部参数）。

```cpp
// 2. 里程计相对位移约束
residual.block<3, 1>(15, 0) = cnb0 * (state1.p - state0.p) - corrected_s_;
```
- **cnb0 * (state1.p - state0.p)**：由 **INS/GNSS 融合解** 算出的在世界系下的位移，投影回 **b0系（初始载体系）**。
- **corrected_s_**：由 **里程计** 测量累积得到的在 b0系 下的位移。
- **物理含义**：强迫“宏观导航解算的相对运动”与“微观里程计测量的相对运动”保持一致。这是 VIO/GINS 中融合不同传感器的关键耦合项。

```cpp
// 3. 里程计比例因子约束
residual(18) = state1.sodo - state0.sodo;
```
- **物理含义**：里程计比例因子（Scale Factor）也应保持稳定，不发生突变。

#### **2. 线性化校正 (Linearization Correction) 核心逻辑**

代码中最令人困惑的部分往往是这几行计算 `dbg` 及其后续修正的代码：

```cpp
Vector3d dbg = state0.bg - delta_state_.bg; 
// ...
corrected_p_ = delta_state_.p + dp_dbg * dbg + ...;
```

##### **Q1: 三个 "bg" 的身份大揭秘**

| 变量名 | 身份 | 来源 | 数值示例 (假设) |
| :--- | :--- | :--- | :--- |
| **`delta_state_.bg`** | **线性化点 (Linearization Point)** | **预积分时刻**的假设值 | 0.01 (旧值) |
| **`state0.bg`** | **优化变量 (State Variable)** | **当前优化迭代**的最新估计 | 0.012 (新值) |
| **`state1.bg`** | **下一帧变量** | **当前优化迭代**的最新估计 | 0.0121 |

##### **Q2: 为什么要算 `state0.bg - delta_state_.bg`？**

这是一个 **“补救措施”**。

1.  **问题**：我们在预积分阶段（几百毫秒前），是假设 $bg=0.01$ 来积分出位移 `delta_state_.p` 的。
2.  **现状**：现在 Ceres 优化器经过几轮迭代，认为 $bg$ 其实应该是 $0.012$。
3.  **冲突**：如果直接用旧的 `delta_state_.p` 和新的 `state0` 做残差，会导致逻辑不自洽（位移是基于旧参数算的，状态却是新参数）。
4.  **解决**：
    -   **笨办法**：用 $0.012$ 重新把这几百个 IMU 数据积分一遍。（太慢，至少不可行）
    -   **巧办法 (泰勒展开)**：利用预先算好的 **雅可比矩阵 (Jacobian)** 进行一阶修正。

##### **Q3: 校正流水线 (The Correction Pipeline)**

这几行代码实际上构成了一个完整的校正过程：

**Step 1: 准备输入 (计算 $\Delta x$)**
```cpp
// 计算当前估计值相对于当初积分假设值的偏差
Vector3d dbg = state0.bg - delta_state_.bg;
```
这是泰勒公式 $f(x) \approx f(x_0) + J(x-x_0)$ 中的 **$(x-x_0)$** 项。

**Step 2: 获取灵敏度 (提取 $J$)**
```cpp
// 从大雅可比矩阵中提取出“位置对陀螺零偏”的偏导数
Matrix3d dp_dbg = jacobian_.block<3, 3>(0, 9);
```
这是泰勒公式中的 **$f'(x_0)$** 项。它告诉我们：如果零偏变了 1 单位，位置积分结果会偏离多少米。

**Step 3: 应用校正 (计算 $f(x)$)**
```cpp
// Old_Integral + Jacobian * Delta_Bias
corrected_p_ = delta_state_.p + dp_dba * dba + dp_dbg * dbg;
```
这样得到的 `corrected_p_` 就近似等于“假如当初我们用 $0.012$ 进行积分”应该得到的结果。

通过这种机制，我们既享受了预积分带来的速度（不用重复积分），又保证了在优化参数不断调整的过程中，约束依然准确有效。
