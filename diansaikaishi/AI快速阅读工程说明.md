# AI 快速阅读工程说明

> 目的：让后续 AI 或开发者快速理解当前工程结构、控制主线、任务层设计和后续改进方向。

## 1. 当前工程定位

这是一个 TI CCS / MSPM0G3507 嵌入式循迹小车工程。

当前主线：

```text
七路灰度循迹 + MPU6050 航向辅助 + 静态任务动作编排
```

底层运控仍由 `CarController` 统一控制电机输出；新增的 `MissionManager` / `MotionAction` / `MissionLibrary` 只负责描述和调度任务，不直接控制电机。

## 2. 建议阅读顺序

如果从零理解当前工程，建议按这个顺序看：

```text
1. app_features.h
2. app_config.h / app_config.c
3. app.c
4. car_controller.h / car_controller.c
5. motion_types.h
6. motion_action.h / motion_action.c
7. mission_manager.h / mission_manager.c
8. mission_library.h / mission_library.c
9. menu.c / oled_ui.c
10. track_sensor.c / imu.c / heading_control.c / motor.c
```

如果只看底层运控：

```text
car_controller.c
track_sensor.c
heading_control.c
imu.c
motor.c
```

如果只看任务层：

```text
motion_types.h
motion_action.c
mission_manager.c
mission_library.c
menu.c
```

## 3. 20ms 调度主线

入口在 `empty.c`。

```text
SysTick 每 100us 调用 Motor_PwmTick100us()
累计 20ms 后主循环调用 App_Update_20ms()
```

当前 `App_Update_20ms()` 顺序：

```text
1. Imu_Update(0.02f)
2. App_UpdateHeadingObserver()
3. Key_Update_20ms()
4. Key_GetEvent()
5. Menu_HandleKeyEvent()
6. MissionManager_Update_20ms()
7. CarController_Update_20ms()
8. 更新调试变量
9. OledUi_Update_20ms()
```

注意：

```text
MissionManager 在 CarController 前更新。
MissionManager/MotionAction 只设置动作目标。
CarController 仍是唯一实际输出电机速度的运控层。
```

## 4. 当前文件结构

核心应用层：

```text
app.c / app.h
app_config.c / app_config.h
app_features.h
car_state.c / car_state.h
```

任务层：

```text
motion_types.h
motion_action.c / motion_action.h
mission_manager.c / mission_manager.h
mission_library.c / mission_library.h
```

底层运控：

```text
car_controller.c / car_controller.h
track_sensor.c / track_sensor.h
heading_control.c / heading_control.h
imu.c / imu.h
motor.c / motor.h
encoder.c / encoder.h
pid.c / pid.h
```

人机交互：

```text
key.c / key.h
menu.c / menu.h
oled.c / oled.h
oled_ui.c / oled_ui.h
```

工程/生成文件：

```text
empty.c
empty.syscfg
.ccsproject / .cproject / .project
Debug/                 CCS 编译产物，不建议手改
targetConfigs/
```

## 5. 底层运控状态机

状态定义在 `car_controller.h`：

```c
typedef enum {
    TRACK_MODE_SEEK_LINE = 0,
    TRACK_MODE_FOLLOW_LINE,
    TRACK_MODE_TURN_LEFT_90,
    TRACK_MODE_TURN_RIGHT_90,
    TRACK_MODE_LOST_RECOVER
} TrackRunMode;
```

含义：

```text
SEEK_LINE       没看到线时按目标 yaw 直线找线
FOLLOW_LINE     正常灰度循迹
TURN_LEFT_90    左 90 度单轮转弯
TURN_RIGHT_90   右 90 度单轮转弯
LOST_RECOVER    丢线后按最后方向找线
```

关键接口：

```c
void CarController_ResetRuntime(void);
void CarController_ResetTransientState(void);
void CarController_Update_20ms(void);
void CarController_Stop(void);
void CarController_StartSeekLine(float target_yaw_deg);
void CarController_StartFollowLine(CarTurnHandlingPolicy turn_policy);
void CarController_StartTurnLeft90(void);
void CarController_StartTurnRight90(void);
void CarController_UseCurrentHeadingForSeek(void);
void CarController_SetSeekTargetYaw(float target_yaw_deg);
TrackRunMode CarController_GetRunMode(void);
const CarControllerFeedback *CarController_GetFeedback(void);
```

`CarControllerFeedback` 用于给任务层上报底层事件：

```text
line_found
line_lost
center_detected
detected_turn
turn_completed
operation_failed
```

FOLLOW 的路口处理策略：

```text
CAR_TURN_POLICY_AUTO         保持旧行为，检测到 90 度后自动转弯
CAR_TURN_POLICY_REPORT_ONLY  只上报路口，不自动转弯
CAR_TURN_POLICY_IGNORE       忽略路口检测
```

当前 90 度路口检测在 `track_sensor.c`：

```text
左/右边缘传感器看到线
中间 S3/S4/S5 至少一个看到线
黑线数量 >= TRACK_TURN_MIN_BLACK_COUNT，当前为 3
误差方向达到 TRACK_TURN_MIN_ABS_ERROR，当前为 200
对侧边缘没有看到线
```

这样比旧逻辑更不容易把普通大弯误判为 90 度路口。

## 6. SEEK 直线找线逻辑

`SEEK_LINE` 没看到黑线时：

```text
left  = search_speed + heading_correction
right = search_speed - heading_correction
```

任务层可以指定 SEEK 目标 yaw。

当前任务 SEEK 目标角计算：

```text
普通任务 SEEK:
target_yaw = mission_start_yaw + action_angle + YAW

第二次 SEEK:
target_yaw = mission_start_yaw + REV + YAW
```

其中：

```text
mission_start_yaw = 任务启动瞬间 Imu_GetYaw()
YAW = g_appConfig.seek_heading_offset_deg
REV = g_appConfig.second_seek_angle_deg
```

当前现场标定默认值：

```text
YAW = -1
REV = 215
```

SEEK 航向修正有 1 度死区：

```text
abs(heading_error_deg) <= 1.0 时，heading_correction = 0
```

这样避免目标误差接近 0 时，仅由陀螺仪 D 项造成固定偏转。

## 7. FOLLOW / TURN / LOST 行为

### FOLLOW_LINE

灰度循迹 PD 为主：

```text
line_correction = KP * error + KD * derivative
```

稳定直线段可叠加小幅航向辅助。

### TURN_90

90 度转弯继续使用当前稳定的单轮转弯方案，不接 IMU：

```text
L90: left = 0,          right = turn_speed
R90: left = turn_speed, right = 0
```

退出条件：

```text
turn_elapsed_ms >= turn_min_ms
并且中间传感器 S3/S4/S5 重新看到黑线
```

### LOST_RECOVER

丢线恢复不左右扫线，而是按最后明确方向找线：

```text
LEFT:  left = 0,             right = recover_speed
RIGHT: left = recover_speed, right = 0
NONE:  left = recover_speed, right = recover_speed
```

注意：

```text
在任务动作 FOLLOW_END_LINE_LOST 中，
CarController 进入 TRACK_MODE_LOST_RECOVER 会被 MotionAction 视为“FOLLOW 动作成功完成”，
用于表达“出循迹路段”。
```

## 8. 任务层结构

任务层分三部分：

```text
motion_types.h       定义动作类型、结果、yaw 参考系
motion_action.c      执行一个动作，并判断动作完成
mission_manager.c    管理任务、动作索引、暂停/继续/取消
mission_library.c    静态任务数组和任务注册表
```

当前动作类型：

```c
typedef enum {
    MOTION_ACTION_SEEK_LINE = 0,
    MOTION_ACTION_FOLLOW_LINE,
    MOTION_ACTION_TURN_LEFT_90,
    MOTION_ACTION_TURN_RIGHT_90,
    MOTION_ACTION_WAIT,
    MOTION_ACTION_STOP
} MotionActionType;
```

当前支持的动作：

```text
SEEK_LINE
FOLLOW_LINE，支持 FOLLOW_END_DURATION / FOLLOW_END_LINE_LOST /
             FOLLOW_END_LEFT_90_DETECTED /
             FOLLOW_END_RIGHT_90_DETECTED /
             FOLLOW_END_ANY_90_DETECTED
TURN_LEFT_90
TURN_RIGHT_90
WAIT
STOP
```

`timeout_ms = 0U` 表示不启用超时。

## 9. 当前任务库

任务 ID：

```text
MISSION_ID_LEGACY     = 0
MISSION_ID_TEST_SF    = 1
MISSION_ID_TEST_R90   = 2
MISSION_ID_TEST_RSTOP = 3
MISSION_ID_TEST_SEEK_FOLLOW = 4
MISSION_ID_TEST_SEEK_STOP   = 5
```

### LEGACY

接近旧版启动行为：

```c
static const MotionAction g_missionLegacy[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};
```

含义：

```text
按任务起点方向找线
找到线后持续循迹
```

### TEST-SF

当前现场测试任务：

```c
static const MotionAction g_missionTestSeekFollow[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_SEEK_SECOND_CONFIG(0U),
    ACTION_FOLLOW_UNTIL_LINE_LOST(0U),
    ACTION_STOP()
};
```

含义：

```text
1. 第一次直线 SEEK，目标 = mission_start_yaw + 0 + YAW
2. 循迹，直到出循迹/丢线
3. 第二次直线 SEEK，目标 = mission_start_yaw + REV + YAW
4. 再次循迹，直到出循迹/丢线
5. 停车
```

当前现场标定：

```text
YAW = -1
REV = 215
```

### TEST-R90

用于验证“检测右 90”和“执行右转”已经分层：

```c
static const MotionAction g_missionTestRight90Turn[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    ACTION_TURN_RIGHT_90(0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};
```

含义：

```text
找线
循迹到右 90 路口，只上报路口，不自动转
任务层启动右 90 转弯
转完后继续循迹
```

### TEST-RSTOP

用于验证同一个右 90 路口也可以选择停车：

```c
static const MotionAction g_missionTestRight90Stop[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_UNTIL_RIGHT_90(0U),
    ACTION_STOP()
};
```

含义：

```text
找线
循迹到右 90 路口
停车
```

### TEST-SK-L

用于粗略验证“SEEK 找到线后继续循迹”：

```c
static const MotionAction g_missionTestSeekThenFollow[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_FOLLOW_FOREVER(0U),
    ACTION_STOP()
};
```

### TEST-SK-S

用于粗略验证“SEEK 找到线后停车”：

```c
static const MotionAction g_missionTestSeekThenStop[] = {
    ACTION_SEEK_MISSION_YAW(0.0f, 0U),
    ACTION_STOP()
};
```

注意：

```text
当前 SEEK 找到线后底层仍会短暂切入 FOLLOW_LINE。
如果下一个任务动作是 STOP，下一周期会停车。
后续建议实现 SEEK_COMPLETE_REPORT_ONLY，让 SEEK 后续行为完全由任务层决定。
```

## 10. OLED / 按键操作

页面循环：

```text
STATUS -> HEAD -> STATUS
```

`SENSOR` / `IMU` 页面代码仍保留在 `oled_ui.c` 中，但默认不放进普通 K1 页面循环。

STATUS 页当前显示：

```text
TASK:<id> <mission_status>
ACT:<action> S:<step> T:<total>
RAW:<7路灰度>
TIME:<当前动作秒数> M:<car_controller_mode>
```

这页用于现场快速判断：

```text
当前任务
当前动作
灰度传感器状态
当前动作运行时间和底层运控模式
```

按键：

```text
K1 短按：切换 OLED 页面
K1 长按：进入参数菜单

K2 短按：
    READY  -> 启动当前任务
    RUNNING -> 暂停任务
    PAUSED -> 继续任务

K3 短按：取消任务，回 READY
K3 长按：重置任务和底层运行态，回 STATUS/READY
```

参数菜单：

```text
K1 短按：下一个参数
K1 长按：退出参数菜单
K2：增加
K3：减少
```

当前参数菜单项：

```text
TASK   当前任务 ID，0=LEGACY，1=TEST-SF，2=TEST-R90，3=TEST-RSTOP，4=TEST-SK-L，5=TEST-SK-S
SPD
YAW    全局 SEEK 航向微调，默认 -1，每次 1 度
REV    第二次 SEEK 反向角，默认 215，每次 5 度
KP
KD
MAX
```

`REV` 范围：

```text
120 ~ 220
```

## 11. IMU 模块

文件：

```text
imu.c / imu.h
heading_control.c / heading_control.h
```

硬件：

```text
MPU6050
I2C 地址 0x68
SDA = GPIOA.10
SCL = GPIOA.11
独立模拟 I2C，不与 OLED 共用
```

IMU 负责：

```text
初始化 MPU6050
读取 gyro z
零偏校准
计算 corrected gyro
积分 yaw
```

HeadingControl 负责：

```text
保存 target_yaw
计算 yaw error
输出 heading_correction
```

## 12. 当前可调参数

在 `app_config.c/h`：

```text
target_laps
base_speed
search_speed
recover_speed
turn_speed
max_correction
track_kp
track_kd
track_scale
start_line_threshold
lost_line_threshold
lap_cooldown_ms
lost_recover_max_ms
turn_min_ms
turn_max_ms
gyro_deadband_dps
heading_kp
heading_kd
heading_scale
heading_max_correction
heading_enable_error
heading_enable_derivative
heading_lock_delay_ms
seek_heading_offset_deg      当前默认 -1
second_seek_angle_deg        当前默认 215
```

## 13. 不要轻易改的点

- 不要恢复 `motor_balance` / `TRIM` 固定补偿。
- 不要让任务层直接调用 `Motor_SetSpeed()`。
- 不要把 IMU 航向控制接入 90 度转弯，当前 90 度转弯已稳定。
- 不要在 `LOST_RECOVER` 中恢复左右扫线。
- 不要把地图任务硬编码进 `car_controller.c`。
- 不要把现场微调角 `YAW` 和任务结构角 `REV` 混为一个概念。
- 不要把 `Debug/` 下的 CCS 编译产物当作源码手改。

## 14. 后续推荐改进方向

### P0：继续完善任务可观测性

建议下一步优先加 OLED 任务状态页：

```text
TASK : 1
STEP : 2/5
ACT  : SEEK / LINE / STOP
TIME : 当前动作运行时间
```

现在已经有 `MissionRuntime` 和 `MotionActionRuntime`，显示这些信息不难。

### P1：继续增强路口检测和转弯策略

当前已经完成第一版路口检测和转弯拆分：

```text
FOLLOW 可以选择 AUTO / REPORT_ONLY / IGNORE
任务层可以用 FOLLOW_UNTIL_LEFT_90 / RIGHT_90 / ANY_90 结束动作
转弯动作仍由 TURN_LEFT_90 / TURN_RIGHT_90 单独执行
```

后续可以继续增强：

```text
路口事件去抖
路口计数
左/右/十字路口更细分类
OLED 显示 detected_turn
```

目标是让任务层决定：

```text
检测到路口后左转 / 右转 / 继续 / 停车 / 计数
```

### P2：给任务库增加更多地图任务

建议新地图只改：

```text
mission_library.c
```

用动作数组描述：

```text
SEEK
FOLLOW_UNTIL_LINE_LOST
FOLLOW_UNTIL_LEFT_90
FOLLOW_UNTIL_RIGHT_90
FOLLOW_UNTIL_ANY_90
FOLLOW_FOR_TIME
TURN_LEFT_90
TURN_RIGHT_90
WAIT
STOP
```

### P3：参数持久化

当前 `YAW=-1`、`REV=215` 是写在默认配置里的。

后续可考虑 Flash 保存：

```text
KP / KD / SPD / YAW / REV / TASK
```

但不要在任务框架还没完全稳定时同时做 Flash。

### P4：更高级运动能力

最后再考虑：

```text
编码器速度闭环
按距离行驶
IMU 指定角度转弯
任务 Profile
运行统计和故障码页
```

这些不要和任务框架重构混在一起做，否则现场问题会很难定位。
