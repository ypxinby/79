我来先了解一下这个工程的整体结构，然后分析各个部分的逻辑关系和引脚配置。
用户需要我阅读这个工程文件，输出一个开发文档。让我先了解一下项目结构。

让我读取所有核心源文件来分析项目结构。

现在我可以看到这是一个基于TI MSPM0G3507的电赛小车项目。让我读取所有的源文件来理解整个工程的结构和逻辑关系。

让我从Debug目录读取引脚配置文件：

看起来 ti_msp_dl_config.h 和 ti_msp_dl_config.c 在 Debug 目录下。让我从那里读取。

现在我已经收集了所有必要的信息。让我整理一份完整的开发文档，描述项目结构、模块关系、引脚配置等信息。

让我整理一下信息：

1. **主控芯片**: TI MSPM0G3507
2. **CPU时钟**: 32MHz
3. **项目结构**: 4个主要模块 + 主程序

**引脚配置**:
- TB6612 电机驱动A (GPIOA):
  - AIN1: GPIOA.13 (Pin 6)
  - AIN2: GPIOA.12 (Pin 5)
  
- TB6612 电机驱动B (GPIOB):
  - BIN1: GPIOB.0 (Pin 47)
  - BIN2: GPIOB.1 (Pin 48)
  - PWMA: GPIOB.15 (Pin 3)
  - PWMB: GPIOB.16 (Pin 4)

- 编码器 (GPIOA):
  - MOTOR_A_ENCA: GPIOA.15 (Pin 8) - 中断输入
  - MOTOR_A_ENCB: GPIOA.16 (Pin 9)
  - MOTOR_B_ENCA: GPIOA.17 (Pin 10) - 中断输入
  - MOTOR_B_ENCB: GPIOA.24 (Pin 25)

**模块逻辑关系**:
主程序 -> SysTick定时器 -> 产生100us节拍
  -> Motor_PwmTick100us(): 软件PWM控制电机转速
  -> 每20ms触发 StraightControl_Update()

主程序 -> GPIO中断 -> Encoder_HandleGpioInterrupt(): 读取编码器脉冲

StraightControl_Update():
  -> Encoder_GetAndClearPulseDeltas(): 获取两电机脉冲数
  -> PID_Update(): 根据误差计算修正量
  -> Motor_SetSpeed(): 输出修正后的PWM占空比

让我写一份完整的开发文档。
现在我已经完整理解了整个工程。以下是开发文档：

---

# 电赛小车开发文档

## 1. 项目概述

本项目是基于 **TI MSPM0G3507** 微控制器的电赛小车控制系统，目前实现了 **编码器反馈 + PID 直线行驶校正** 的基础控制框架。

- **主控芯片**: MSPM0G3507
- **CPU 主频**: 32 MHz
- **开发环境**: TI Code Composer Studio (CCS) + SysConfig

---

## 2. 工程文件结构

```
diansaikaishi/
├── empty.c                    # 主程序 (main)
├── encoder.h / encoder.c      # 编码器模块
├── motor.h / motor.c          # 电机驱动模块
├── pid.h / pid.c              # PID 控制器
├── straight_control.h/c       # 直线行驶控制
└── ti_msp_dl_config.h/c       # SysConfig生成的引脚配置
```

---

## 3. 模块逻辑关系图

```
┌─────────────────────────────────────────────────────────────────────┐
│                          main() [empty.c:19-39]                    │
│  ┌───────────────────────────────────────────────────────────────┐  │
│  │  1. SYSCFG_DL_init()        // 系统初始化                    │  │
│  │  2. Motor_Init()            // 电机初始化                    │  │
│  │  3. Encoder_Reset()         // 编码器清零                    │  │
│  │  4. StraightControl_Init()  // 直线控制初始化                │  │
│  │  5. SysTick_Config(10kHz)   // 配置系统节拍                  │  │
│  │  6. 开中断, 启动 StraightControl                            │  │
│  └───────────────────────────────────────────────────────────────┘  │
│                            ↓ 循环                                  │
│  g_straightControlDue ? → StraightControl_Update() [每20ms]        │
└─────────────────────────────────────────────────────────────────────┘

              ↓ 中断 ↓                               ↓ 中断 ↓
  ┌───────────────────────────┐           ┌──────────────────────────┐
  │   SysTick_Handler()       │           │  GROUP1_IRQHandler()    │
  │  [empty.c:41-58]          │           │  [empty.c:60-69]        │
  │   100us 节拍 → PWM 输出   │           │ 编码器脉冲计数          │
  │   每20ms → 标记控制标志   │           │  ↓ Encoder_Handle...    │
  └───────────────────────────┘           └──────────────────────────┘
                    │                                       │
                    ↓                                       ↓
     ┌─────────────────────────────┐         ┌──────────────────────────┐
     │   Motor_PwmTick100us()      │         │ Encoder_HandleGpio...()  │
     │   [motor.c:105-123]         │         │ [encoder.c:29-47]        │
     │   软件 PWM, 分100级输出     │         │ 读取A/B相, 判断方向     │
     └─────────────────────────────┘         └──────────────────────────┘

                     ┌──────────────────────────────────┐
                     │  StraightControl_Update()        │
                     │  [straight_control.c:50-72]      │
                     │  ┌──────────────────────────┐    │
                     │  │  读编码器脉冲差          │    │
                     │  │  Encoder_GetAndClear...  │    │
                     │  └────────────┬─────────────┘    │
                     │               ↓ 误差 = A - B    │
                     │  ┌──────────────────────────┐    │
                     │  │  PID_Update(error)       │    │
                     │  │  [pid.c:32-49]           │    │
                     │  └────────────┬─────────────┘    │
                     │               ↓ correction       │
                     │  ┌──────────────────────────┐    │
                     │  │  Motor_SetSpeed(...)     │    │
                     │  │  A: base - correction    │    │
                     │  │  B: base + correction    │    │
                     │  └──────────────────────────┘    │
                     └──────────────────────────────────┘
```

---

## 4. 各模块详细说明

### 4.1 主程序 (`empty.c`)

**功能**: 系统初始化、调度控制、中断分发

**关键时序**:
- `SysTick` 配置为 **10 kHz** (每 100 µs 触发一次)
- **PWM 更新周期**: 每 100 µs (`Motor_PwmTick100us`)
- **控制周期**: 每 **20 ms** 执行一次 `StraightControl_Update`
  (由 `STRAIGHT_CONTROL_PERIOD_MS` 定义，见 `straight_control.h:6`)

### 4.2 电机驱动模块 (`motor.c/h`)

**驱动芯片**: TB6612FNG (双路直流电机驱动)

**核心函数**:
| 函数 | 功能 |
|------|------|
| `Motor_Init()` | 初始化，停止所有输出 |
| `Motor_SetSpeed(A, B)` | 设置两电机速度（范围 -1000 ~ +1000） |
| `Motor_Stop()` | 电机制动 |
| `Motor_PwmTick100us()` | 软件 PWM 输出，需 100 µs 调用一次 |

**工作原理**:
- **方向控制**: 通过 `AIN1/AIN2` (电机A) 和 `BIN1/BIN2` (电机B) 的高低电平组合决定正反转
- **速度控制**: 通过软件 PWM 驱动 `PWMA/PWMB`，PWM 周期 = 100 × 100 µs = **10 ms** (100 Hz)
- **占空比分辨率**: 100 级 (输入值 0-1000 按 `/10` 映射到 0-100)
- **方向翻转**: 可通过定义 `MOTOR_A_INVERT_DIRECTION=1` 或 `MOTOR_B_INVERT_DIRECTION=1` 反转某一电机方向

### 4.3 编码器模块 (`encoder.c/h`)

**解码方式**: A/B 相正交编码（中断式解码）

**核心函数**:
| 函数 | 功能 |
|------|------|
| `Encoder_Reset()` | 清零脉冲计数 |
| `Encoder_HandleGpioInterrupt()` | GPIO 中断服务，检测 A/B 相判断方向并累加脉冲 |
| `Encoder_GetAndClearPulseDeltas(&A, &B)` | 读取并清零累计脉冲数（取绝对值） |

**方向判断逻辑** (`encoder.c:34-44`):
```
当 ENCA 边沿触发时:
  若 ENCA == ENCB  → 正转 (+1 脉冲)
  若 ENCA != ENCB  → 反转 (-1 脉冲)
```
> 注：`ENCA` 配置为**双边沿中断**（上升沿+下降沿），`ENCB` 仅作方向判断输入。两电机的 ENCA 共用 `GPIOA` 中断组。

### 4.4 PID 控制器 (`pid.c/h`)

**类型**: 整数型位置式 PID

```
output = (Kp × error + Ki × integral + Kd × derivative) / scale
```

**参数结构体**:
- `kp`, `ki`, `kd` — PID 增益（整数）
- `scale` — 输出缩放因子（用于将整数增益还原为合理输出范围）
- `outputLimit` — 输出限幅
- `integralLimit` — 积分项限幅（抗饱和）

**使用示例** (`straight_control.c:32`):
```c
PID_Init(&g_straightPid, 1800, 35, 600, 100, 250, 2000);
// Kp=18, Ki=0.35, Kd=6 → 除以 scale=100 得到真实增益
// 输出限幅 ±250, 积分限幅 ±2000
```

### 4.5 直线行驶控制 (`straight_control.c/h`)

**核心思想**: 比较左右轮脉冲差 → PID 输出修正量 → 调节两轮差速

**控制流程** (`straight_control.c:50-72`):
1. 读取两轮脉冲数 `motorA`, `motorB`
2. 计算误差: `error = motorA - motorB`
3. PID 计算修正量: `correction = PID_Update(error)`
4. 输出到电机:
   - 电机A: `base_duty - correction`
   - 电机B: `base_duty + correction`

**默认参数**:
- 基准占空比 `STRAIGHT_BASE_DUTY = 350` (见 `straight_control.c:8`)
- 输出范围: 180 ~ 650
- PID 参数: Kp=18, Ki=0.35, Kd=6

**调参提示**:
- 若修正方向相反，将 `STRAIGHT_REVERSE_CORRECTION` 改为 `1` (`straight_control.c:14`)

---

## 5. 引脚配置表

来源: `ti_msp_dl_config.h` (SysConfig 自动生成)

### 5.1 TB6612 电机驱动 (输出引脚)

| 信号 | 端口 | 引脚号 | 封装引脚 | IOMUX | 功能 |
|------|------|--------|----------|-------|------|
| **AIN1** (电机A方向1) | GPIOA | 13 | Pin 6 | pinCM35 | 数字输出 |
| **AIN2** (电机A方向2) | GPIOA | 12 | Pin 5 | pinCM34 | 数字输出 |
| **BIN1** (电机B方向1) | GPIOB | 0 | Pin 47 | pinCM12 | 数字输出 |
| **BIN2** (电机B方向2) | GPIOB | 1 | Pin 48 | pinCM13 | 数字输出 |
| **PWMA** (电机A速度) | GPIOB | 15 | Pin 3 | pinCM32 | 数字输出 |
| **PWMB** (电机B速度) | GPIOB | 16 | Pin 4 | pinCM33 | 数字输出 |

### 5.2 编码器 (输入引脚)

| 信号 | 端口 | 引脚号 | 封装引脚 | IOMUX | 配置 |
|------|------|--------|----------|-------|------|
| **MOTOR_A_ENCA** (中断) | GPIOA | 15 | Pin 8 | pinCM37 | 上拉输入, 双边沿中断 |
| **MOTOR_A_ENCB** | GPIOA | 16 | Pin 9 | pinCM38 | 上拉输入, 无中断 |
| **MOTOR_B_ENCA** (中断) | GPIOA | 17 | Pin 10 | pinCM39 | 上拉输入, 双边沿中断 |
| **MOTOR_B_ENCB** | GPIOA | 24 | Pin 25 | pinCM54 | 上拉输入, 无中断 |

**中断配置**:
- 两个 ENCA 引脚共用 `GPIOA_INT_IRQn` (Group 1)
- 边沿触发: **上升沿 + 下降沿** (`EDGE_RISE_FALL`, 见 `ti_msp_dl_config.c:100-101`)

### 5.3 TB6612 接线参考

```
MSPM0G3507             TB6612FNG
───────────            ───────────
GPIOA.13 (Pin6)  ─────→ AIN1
GPIOA.12 (Pin5)  ─────→ AIN2
GPIOB.0  (Pin47) ─────→ BIN1
GPIOB.1  (Pin48) ─────→ BIN2
GPIOB.15 (Pin3)  ─────→ PWMA
GPIOB.16 (Pin4)  ─────→ PWMB
                    +  VM  → 电池正 (≤12V)
                    +  VCC → 3.3V/5V
                    +  GND → 共地
                    +  STBY → 接高电平(VCC) 或 另一个GPIO
                    +  AO1/AO2 → 电机A
                    +  BO1/BO2 → 电机B
```

---

## 6. 关键时间参数汇总

| 参数 | 值 | 定义位置 |
|------|-----|----------|
| CPU 频率 | 32 MHz | `ti_msp_dl_config.h:76` |
| SysTick 频率 | 10 kHz (100 µs) | `empty.c:14` |
| PWM 频率 | 100 Hz (10 ms) | `motor.c:4` (100级 × 100µs) |
| 控制周期 | 20 ms | `straight_control.h:6` |
| 电机占空比范围 | 0 ~ 1000 | `motor.h:7` |
| 直线控制占空比范围 | 180 ~ 650 | `straight_control.c:7-8` |

---

## 7. 扩展建议

当前代码实现了**基础直行闭环**。建议后续扩展：

1. **增加转向控制**: 在 `StraightControl` 基础上增加一个转向 PID，或扩展为差速转向
2. **增加传感器输入**: 如红外循迹、超声波测距等，新增模块类似 `encoder.c` 的结构
3. **增加遥控接口**: UART/蓝牙接收目标速度指令
4. **调参接口**: 通过 UART 实时调整 PID 参数，方便现场调试