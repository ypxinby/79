# AI 快速阅读工程说明

> 目的：让后续 AI 或开发者快速理解当前工程结构、控制主线和关键注意事项。

## 1. 先看这些文件

建议阅读顺序：

```text
1. app_features.h
2. app_config.h / app_config.c
3. app.c
4. car_controller.h / car_controller.c
5. track_sensor.h / track_sensor.c
6. imu.h / imu.c
7. heading_control.h / heading_control.c
8. menu.c / oled_ui.c
```

如果只想理解运控，重点看：

```text
car_controller.c
track_sensor.c
heading_control.c
imu.c
```

## 2. 当前工程主线

当前系统是：

```text
七路灰度循迹 + MPU6050 航向辅助
```

控制周期：

```text
SysTick 每 100us 跑软件 PWM
累计 20ms 后主循环执行 App_Update_20ms()
```

`App_Update_20ms()` 顺序：

```text
1. Imu_Update(0.02f)
2. 按键扫描
3. 菜单处理
4. CarController_Update_20ms()
5. 更新调试变量
6. OLED 刷新
```

## 3. 状态机

运控状态在 `car_controller.h`：

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
SEEK_LINE       启动后找黑线
FOLLOW_LINE     正常循迹
TURN_LEFT_90    左 90 度单轮转弯
TURN_RIGHT_90   右 90 度单轮转弯
LOST_RECOVER    丢线后按最后明确方向找线
```

## 4. 关键控制逻辑

### SEEK_LINE

没看到黑线时，按目标 yaw 直线找线：

```text
target_yaw = 当前 yaw + seek_heading_offset_deg
left  = search_speed + heading_correction
right = search_speed - heading_correction
```

如果 IMU 不 ready，退回左右同速。

### FOLLOW_LINE

灰度循迹 PD 为主：

```text
line_correction = KP * error + KD * derivative
```

直线段额外叠加小幅航向修正：

```text
final_correction = line_correction + heading_correction
```

航向修正只在误差较小、变化较小、黑线数量较少时启用。

### TURN_90

当前 90 度转弯不使用 IMU，仍使用单轮转弯：

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

不再左右扫线。

根据 `recover_direction` 单方向找线：

```text
LEFT:  left = 0,             right = recover_speed
RIGHT: left = recover_speed, right = 0
NONE:  left = recover_speed, right = recover_speed
```

## 5. IMU 模块

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

## 6. 重要接口

### CarController

```c
void CarController_ResetRuntime(void);
void CarController_Update_20ms(void);
void CarController_UseCurrentHeadingForSeek(void);
void CarController_SetSeekTargetYaw(float target_yaw_deg);
```

说明：

- `CarController_UseCurrentHeadingForSeek()`：SEEK 使用 `当前 yaw + YAW参数`。
- `CarController_SetSeekTargetYaw()`：后续任务层可指定 SEEK 目标 yaw。

### IMU

```c
bool Imu_Init(void);
bool Imu_CalibrateGyroBias(uint16_t sample_count);
void Imu_Update(float dt_s);
float Imu_GetYaw(void);
float Imu_GetCorrectedGyroZDps(void);
bool Imu_IsReady(void);
```

### HeadingControl

```c
void HeadingControl_Reset(void);
void HeadingControl_LockCurrentYaw(float current_yaw_deg);
void HeadingControl_SetTargetYaw(float target_yaw_deg);
int16_t HeadingControl_Update(float current_yaw_deg, float gyro_z_dps,
    float dt_s);
```

## 7. 当前可调参数

在 `app_config.c/h`：

```text
base_speed
search_speed
recover_speed
turn_speed
max_correction
track_kp
track_kd
track_scale
turn_min_ms
turn_max_ms
gyro_deadband_dps
heading_kp
heading_kd
heading_max_correction
heading_enable_error
heading_enable_derivative
heading_lock_delay_ms
seek_heading_offset_deg
```

OLED 参数菜单当前可调：

```text
LAP
KP
KD
SPD
MAX
START
LOST
YAW
```

`YAW` 当前每次调整 1 度，用于 SEEK 找线目标航向微调。

## 8. OLED 调试页

页面循环：

```text
STATUS -> SENSOR -> IMU -> HEAD -> STATUS
```

重点：

```text
IMU 页：
C   IMU 错误码
b   I2C 总线状态
ID  WHO_AM_I
R   原始 gyro z
BI  零偏
G   校准后角速度
Y   yaw

HEAD 页：
Y    当前 yaw
T    目标 yaw
E10  航向误差 * 10
D10  角速度项 * 10
C    航向修正量
LK   是否锁定目标
```

## 9. 不要轻易改的点

- 不要恢复 `motor_balance` / `TRIM` 固定补偿。
- 不要把 IMU 航向控制接入 90 度转弯，当前 90 度转弯已稳定。
- 不要在 `LOST_RECOVER` 中恢复左右扫线。
- 不要把任务目标角直接硬编码进 SEEK；任务动作应单独封装。
- 不要把 `-30°` 这类任务目标关系直接等同于小车行走航向。

## 10. 后续推荐封装方向

建议新增：

```text
motion_task.c
motion_task.h
```

用于处理：

```text
任务类型
起点 yaw
目标 yaw
目标点/目标线
什么时候调用 CarController_SetSeekTargetYaw()
什么时候恢复 CarController_UseCurrentHeadingForSeek()
```

当前底层能力已经具备：

```text
SEEK 可以按目标 yaw 直走找线
FOLLOW_LINE 可以直线段航向辅助
TURN_90 保持稳定单轮转弯
LOST_RECOVER 按明确方向找线
```

后续重点应是任务层，而不是继续在底层控制逻辑里硬编码比赛动作。
