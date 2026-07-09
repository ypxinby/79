# MSPM0G3507 循迹小车完整修改方案（给 Codex 使用）

## 0. 开发目标

基于现有 MSPM0G3507 小车工程，改造成一个完整的循迹小车系统。

最终功能：

1. 使用八路循迹模块识别黑线位置。
2. 使用 PD 控制左右轮差速，实现自动循迹。
3. 使用 OLED 做实时可视化显示。
4. 使用 3 个按键控制运行状态、切换页面、修改参数。
5. 目标圈数不是固定 1/3/5，而是可以通过按键 `+1 / -1` 自由修改。
6. 支持按键实时修改 PD 参数，OLED 显示当前参数，修改后立即生效。
7. 支持自动计圈，到达目标圈数后自动停车。
8. 支持丢线保护、急停、暂停、完成状态显示。

---

## 1. 现有工程基础

当前工程已经有以下模块，请保留并复用，不要推倒重写：

```c
motor.c / motor.h
encoder.c / encoder.h
pid.c / pid.h
straight_control.c / straight_control.h
empty.c
ti_msp_dl_config.c / ti_msp_dl_config.h
```

现有功能包括：

1. TB6612 电机驱动。
2. `Motor_Init()` 初始化电机。
3. `Motor_SetSpeed(left, right)` 设置左右电机速度。
4. `Motor_Stop()` 停车。
5. `Motor_PwmTick100us()` 软件 PWM 输出。
6. 编码器中断计数。
7. SysTick 100us 调度。
8. 20ms 控制周期框架。
9. PID 基础模块。

本次开发重点是在现有工程上新增 App 层封装，而不是重写底层电机代码。

---

## 2. 总体架构

建议分为三层：

```text
底层驱动层：
motor / encoder / track_sensor / key / oled

中间数据层：
app_config / app_runtime / car_state

上层控制层：
car_controller / menu / oled_ui / app
```

目标关系：

```text
八路循迹模块负责“看线”
PD 控制负责“修正方向”
电机模块负责“执行速度”
状态机负责“当前该不该跑”
按键负责“修改状态和参数”
OLED 负责“实时可视化”
AppConfig 负责“保存可调参数”
AppRuntime 负责“保存运行数据”
```

---

## 3. 需要新增的文件

请新增以下文件：

```c
track_sensor.c / track_sensor.h
key.c / key.h
oled.c / oled.h
oled_ui.c / oled_ui.h
app_config.c / app_config.h
car_state.c / car_state.h
car_controller.c / car_controller.h
menu.c / menu.h
app.c / app.h
```

其中：

| 文件 | 作用 |
|---|---|
| `track_sensor.c/h` | 八路循迹模块读取、误差计算、丢线判断、起点线判断 |
| `key.c/h` | 3 个按键扫描、消抖、短按、长按、长按连发 |
| `oled.c/h` | OLED 底层驱动 |
| `oled_ui.c/h` | OLED 页面显示，不参与控制逻辑 |
| `app_config.c/h` | 保存可实时修改的参数 |
| `car_state.c/h` | 小车运行状态机 |
| `car_controller.c/h` | 循迹控制、计圈、丢线保护、电机输出 |
| `menu.c/h` | 根据按键事件修改状态和参数 |
| `app.c/h` | App 初始化和 20ms 调度总入口 |

---

## 4. main 文件修改要求

当前 `empty.c` 或 `main.c` 需要被简化。

主函数只负责：

1. 系统初始化。
2. App 初始化。
3. 配置 SysTick。
4. 打开中断。
5. 在主循环中每 20ms 调用一次 `App_Update_20ms()`。

示例结构：

```c
#include "ti_msp_dl_config.h"
#include "app.h"
#include "motor.h"
#include "encoder.h"

#define SYSTICK_HZ 10000u

volatile bool g_app_20ms_due = false;

int main(void)
{
    SYSCFG_DL_init();

    App_Init();

    SysTick_Config(CPUCLK_FREQ / SYSTICK_HZ);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    __enable_irq();

    while (1)
    {
        if (g_app_20ms_due)
        {
            g_app_20ms_due = false;
            App_Update_20ms();
        }
    }
}

void SysTick_Handler(void)
{
    static uint16_t tick_100us = 0;

    Motor_PwmTick100us();

    tick_100us++;
    if (tick_100us >= 200)
    {
        tick_100us = 0;
        g_app_20ms_due = true;
    }
}

void GROUP1_IRQHandler(void)
{
    Encoder_HandleGpioInterrupt();
}
```

注意：

1. `SysTick_Handler()` 中不要处理 OLED。
2. `SysTick_Handler()` 中不要扫描按键。
3. `SysTick_Handler()` 中不要执行 PD 控制。
4. `SysTick_Handler()` 只做 PWM tick 和 20ms 标志位。
5. OLED、按键、循迹控制都在主循环的 `App_Update_20ms()` 里执行。

---

## 5. AppConfig 设计

`AppConfig` 用于保存所有可以通过按键实时修改的参数。

新增 `app_config.h`：

```c
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

typedef struct {
    uint8_t target_laps;             // 目标圈数，可以通过按键 +1 / -1 修改
    uint8_t min_target_laps;         // 最小圈数，建议 1
    uint8_t max_target_laps;         // 最大圈数，建议 99

    int16_t base_speed;              // 基础速度
    int16_t min_base_speed;
    int16_t max_base_speed;

    int16_t max_correction;          // 最大修正量
    int16_t min_max_correction;
    int16_t max_max_correction;

    int16_t track_kp;                // 循迹 P 参数，整数定点
    int16_t track_kd;                // 循迹 D 参数，整数定点
    int16_t track_scale;             // 缩放比例，例如 100

    uint8_t start_line_threshold;    // 起点横线判断阈值，例如 >=6 个传感器检测到黑线
    uint8_t lost_line_threshold;     // 连续丢线次数阈值，例如 15
    uint16_t lap_cooldown_ms;        // 计圈冷却时间，建议 2000ms
} AppConfig;

extern AppConfig g_appConfig;

void AppConfig_InitDefault(void);
void AppConfig_LimitAll(void);

void AppConfig_IncreaseTargetLap(void);
void AppConfig_DecreaseTargetLap(void);

#endif
```

默认值建议：

```c
AppConfig g_appConfig;

void AppConfig_InitDefault(void)
{
    g_appConfig.target_laps = 1;
    g_appConfig.min_target_laps = 1;
    g_appConfig.max_target_laps = 99;

    g_appConfig.base_speed = 280;
    g_appConfig.min_base_speed = 100;
    g_appConfig.max_base_speed = 700;

    g_appConfig.max_correction = 200;
    g_appConfig.min_max_correction = 50;
    g_appConfig.max_max_correction = 500;

    g_appConfig.track_kp = 120;
    g_appConfig.track_kd = 35;
    g_appConfig.track_scale = 100;

    g_appConfig.start_line_threshold = 6;
    g_appConfig.lost_line_threshold = 15;
    g_appConfig.lap_cooldown_ms = 2000;
}
```

重点要求：

1. `target_laps` 不是固定 1/3/5。
2. `target_laps` 是普通参数，可以 `+1 / -1`。
3. 限制范围建议为 `1 ~ 99`。
4. 所有参数都必须有上下限保护。

---

## 6. AppRuntime 设计

`AppRuntime` 用于保存运行时数据，OLED 显示时直接读取该结构体。

可以放在 `car_controller.h` 或单独新建 `app_runtime.h`。

```c
typedef struct {
    uint8_t current_lap;         // 当前已经完成的圈数
    uint8_t sensor_raw;          // 八路循迹 raw bitmask
    uint8_t black_count;         // 当前检测到黑线的传感器数量

    int16_t line_error;          // 当前循迹误差
    int16_t last_error;          // 上一次误差
    int16_t correction;          // PD 输出修正量

    int16_t left_speed;          // 左电机速度
    int16_t right_speed;         // 右电机速度

    uint8_t lost_count;          // 连续丢线次数
    uint16_t lap_cooldown_ms;    // 计圈冷却倒计时
} AppRuntime;

extern AppRuntime g_appRuntime;
```

要求：

1. OLED 只显示 `AppConfig`、`AppRuntime`、`CarState`。
2. OLED 不参与控制计算。
3. 控制逻辑只在 `car_controller.c` 中处理。

---

## 7. CarState 状态机设计

新增 `car_state.h`：

```c
#ifndef CAR_STATE_H
#define CAR_STATE_H

typedef enum {
    CAR_STATE_MENU = 0,
    CAR_STATE_READY,
    CAR_STATE_RUNNING,
    CAR_STATE_PAUSED,
    CAR_STATE_FINISHED,
    CAR_STATE_ERROR
} CarState;

void CarState_Init(void);
void CarState_Set(CarState state);
CarState CarState_Get(void);
const char* CarState_ToString(CarState state);

#endif
```

状态含义：

| 状态 | 含义 |
|---|---|
| `CAR_STATE_MENU` | 菜单状态，可修改参数 |
| `CAR_STATE_READY` | 准备状态，等待启动 |
| `CAR_STATE_RUNNING` | 正在循迹 |
| `CAR_STATE_PAUSED` | 暂停，保留当前圈数 |
| `CAR_STATE_FINISHED` | 已完成目标圈数 |
| `CAR_STATE_ERROR` | 丢线或异常 |

要求：

1. 非 `CAR_STATE_RUNNING` 状态下必须停车。
2. 急停时进入 `CAR_STATE_READY` 或 `CAR_STATE_MENU`，并清零运行状态。
3. 完成目标圈数后进入 `CAR_STATE_FINISHED`。
4. 丢线超过阈值后进入 `CAR_STATE_ERROR`。

---

## 8. 八路循迹模块设计

新增 `track_sensor.c/h`。

接口：

```c
#ifndef TRACK_SENSOR_H
#define TRACK_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#define TRACK_BLACK_LEVEL 0

void TrackSensor_Init(void);
uint8_t TrackSensor_ReadRaw(void);
uint8_t TrackSensor_CountBlack(uint8_t raw);
int16_t TrackSensor_GetErrorFromRaw(uint8_t raw);
bool TrackSensor_IsLineLost(uint8_t raw);
bool TrackSensor_IsStartLine(uint8_t raw, uint8_t threshold);

#endif
```

约定：

```text
bit0 ~ bit7 代表从左到右 8 个传感器
bit0：最左
bit7：最右
```

权重：

```c
static const int16_t TRACK_WEIGHTS[8] = {
    -350, -250, -150, -50, 50, 150, 250, 350
};
```

误差计算：

```c
int16_t TrackSensor_GetErrorFromRaw(uint8_t raw)
{
    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (raw & (1u << i))
        {
            sum += TRACK_WEIGHTS[i];
            count++;
        }
    }

    if (count == 0)
    {
        return 0;
    }

    return (int16_t)(sum / count);
}
```

注意：

1. 如果黑线电平为低电平，`TrackSensor_ReadRaw()` 应该把低电平转换成 bit=1。
2. 如果黑线电平为高电平，`TrackSensor_ReadRaw()` 应该把高电平转换成 bit=1。
3. 无论硬件电平如何，最终 raw 中 `1` 都表示检测到黑线。
4. 这样后续逻辑就统一了。

丢线判断：

```c
bool TrackSensor_IsLineLost(uint8_t raw)
{
    return raw == 0;
}
```

起点线判断：

```c
bool TrackSensor_IsStartLine(uint8_t raw, uint8_t threshold)
{
    return TrackSensor_CountBlack(raw) >= threshold;
}
```

---

## 9. 计圈逻辑设计

赛道起点需要放一条较宽的黑色横线。

判断方法：

```text
八路传感器中检测到黑线的数量 >= start_line_threshold
认为检测到起点横线
```

计圈逻辑：

```c
static void CarController_CheckLap(void)
{
    if (g_appRuntime.lap_cooldown_ms > 0)
    {
        return;
    }

    if (TrackSensor_IsStartLine(g_appRuntime.sensor_raw,
                                g_appConfig.start_line_threshold))
    {
        g_appRuntime.current_lap++;
        g_appRuntime.lap_cooldown_ms = g_appConfig.lap_cooldown_ms;

        if (g_appRuntime.current_lap >= g_appConfig.target_laps)
        {
            Motor_Stop();
            CarState_Set(CAR_STATE_FINISHED);
        }
    }
}
```

冷却时间更新：

```c
if (g_appRuntime.lap_cooldown_ms >= 20)
{
    g_appRuntime.lap_cooldown_ms -= 20;
}
else
{
    g_appRuntime.lap_cooldown_ms = 0;
}
```

要求：

1. 不能每 20ms 重复加圈。
2. 检测到起点线后，必须进入冷却期。
3. 冷却时间默认 2000ms。
4. 计圈完成后立即停车。
5. 完成条件必须是：

```c
current_lap >= target_laps
```

不能使用固定圈数判断。

---

## 10. PD 循迹控制设计

建议第一版只用 PD，不用 I。

原因：

1. P 负责根据偏差修正方向。
2. D 负责抑制左右摇摆。
3. I 容易导致积分累积，反而让车过冲。

PD 计算：

```c
error = g_appRuntime.line_error;
derivative = error - g_appRuntime.last_error;

correction =
    (g_appConfig.track_kp * error +
     g_appConfig.track_kd * derivative)
    / g_appConfig.track_scale;
```

限幅：

```c
if (correction > g_appConfig.max_correction)
{
    correction = g_appConfig.max_correction;
}

if (correction < -g_appConfig.max_correction)
{
    correction = -g_appConfig.max_correction;
}
```

左右轮速度：

```c
left_speed = g_appConfig.base_speed - correction;
right_speed = g_appConfig.base_speed + correction;
```

调用电机：

```c
Motor_SetSpeed(left_speed, right_speed);
```

要求：

1. 不使用 `float`。
2. 使用整数定点。
3. `track_scale` 默认 100。
4. `track_kp = 120` 表示实际 Kp 约为 1.20。
5. `track_kd = 35` 表示实际 Kd 约为 0.35。
6. 每 20ms 读取一次 `g_appConfig`。
7. 因此 OLED 中修改参数后，下一次控制周期立即生效。

---

## 11. CarController 设计

新增 `car_controller.c/h`。

接口：

```c
void CarController_Init(void);
void CarController_ResetRuntime(void);
void CarController_Update_20ms(void);
```

主更新逻辑：

```c
void CarController_Update_20ms(void)
{
    if (g_appRuntime.lap_cooldown_ms >= 20)
    {
        g_appRuntime.lap_cooldown_ms -= 20;
    }
    else
    {
        g_appRuntime.lap_cooldown_ms = 0;
    }

    switch (CarState_Get())
    {
        case CAR_STATE_RUNNING:
            CarController_UpdateTracking();
            break;

        case CAR_STATE_MENU:
        case CAR_STATE_READY:
        case CAR_STATE_PAUSED:
        case CAR_STATE_FINISHED:
        case CAR_STATE_ERROR:
        default:
            Motor_Stop();
            break;
    }
}
```

循迹逻辑：

```c
static void CarController_UpdateTracking(void)
{
    g_appRuntime.sensor_raw = TrackSensor_ReadRaw();
    g_appRuntime.black_count = TrackSensor_CountBlack(g_appRuntime.sensor_raw);
    g_appRuntime.line_error = TrackSensor_GetErrorFromRaw(g_appRuntime.sensor_raw);

    if (TrackSensor_IsLineLost(g_appRuntime.sensor_raw))
    {
        g_appRuntime.lost_count++;
    }
    else
    {
        g_appRuntime.lost_count = 0;
    }

    if (g_appRuntime.lost_count >= g_appConfig.lost_line_threshold)
    {
        Motor_Stop();
        CarState_Set(CAR_STATE_ERROR);
        return;
    }

    CarController_CheckLap();

    if (CarState_Get() != CAR_STATE_RUNNING)
    {
        return;
    }

    int16_t error = g_appRuntime.line_error;
    int16_t derivative = error - g_appRuntime.last_error;

    int32_t correction =
        ((int32_t)g_appConfig.track_kp * error +
         (int32_t)g_appConfig.track_kd * derivative)
        / g_appConfig.track_scale;

    if (correction > g_appConfig.max_correction)
    {
        correction = g_appConfig.max_correction;
    }
    else if (correction < -g_appConfig.max_correction)
    {
        correction = -g_appConfig.max_correction;
    }

    g_appRuntime.correction = (int16_t)correction;

    g_appRuntime.left_speed =
        g_appConfig.base_speed - g_appRuntime.correction;

    g_appRuntime.right_speed =
        g_appConfig.base_speed + g_appRuntime.correction;

    Motor_SetSpeed(g_appRuntime.left_speed,
                   g_appRuntime.right_speed);

    g_appRuntime.last_error = error;
}
```

---

## 12. 三个按键设计

新增 `key.c/h`。

3 个按键：

```text
KEY1：MODE / SELECT
KEY2：UP / PLUS / START
KEY3：DOWN / MINUS / STOP
```

按键要求：

1. 20ms 扫描一次。
2. 软件消抖。
3. 支持短按。
4. 支持长按。
5. 支持长按连发，用于快速修改参数。
6. 不使用按键中断。

建议事件枚举：

```c
typedef enum {
    KEY_EVENT_NONE = 0,

    KEY1_SHORT,
    KEY1_LONG,
    KEY1_REPEAT,

    KEY2_SHORT,
    KEY2_LONG,
    KEY2_REPEAT,

    KEY3_SHORT,
    KEY3_LONG,
    KEY3_REPEAT
} KeyEvent;
```

接口：

```c
void Key_Init(void);
void Key_Update_20ms(void);
KeyEvent Key_GetEvent(void);
```

要求：

1. `Key_Update_20ms()` 扫描按键。
2. `Key_GetEvent()` 返回一个事件。
3. 如果同时多个事件，可以先简化为每次返回一个事件。
4. 长按阈值建议 800ms。
5. 长按连发间隔建议 150ms。

---

## 13. OLED 页面设计

OLED 页面建议由 `oled_ui.c/h` 负责。

页面枚举：

```c
typedef enum {
    OLED_PAGE_STATUS = 0,
    OLED_PAGE_PARAM,
    OLED_PAGE_SENSOR
} OledPage;
```

页面 1：状态页

```text
STATUS
State: RUN
Lap: 2/10
Err: -80
L:260 R:340
S:00111100
```

页面 2：参数页

```text
PARAM
> LAP: 10
  KP : 120
  KD : 35
  SPD: 280
  MAX: 200
```

页面 3：传感器页

```text
SENSOR
Raw:00111100
Black:4
Start:0
Lost:0
```

要求：

1. OLED 只显示数据，不做控制。
2. 参数页当前选中项前面显示 `>`。
3. raw bitmask 要显示成 8 位二进制。
4. OLED 刷新可以每 100ms 一次，不必每 20ms 全量刷新。
5. 如果 OLED 刷新较慢，可以在 `OledUi_Update_20ms()` 里做分频。

---

## 14. 参数菜单设计

新增 `menu.c/h`。

参数项：

```c
typedef enum {
    PARAM_TARGET_LAPS = 0,
    PARAM_KP,
    PARAM_KD,
    PARAM_BASE_SPEED,
    PARAM_MAX_CORRECTION,
    PARAM_START_LINE_THRESHOLD,
    PARAM_LOST_LINE_THRESHOLD,
    PARAM_COUNT
} ParamItem;
```

按键功能分两种页面。

### 14.1 状态页按键功能

| 按键 | 功能 |
|---|---|
| KEY1_SHORT | 切换 OLED 页面：状态页 / 参数页 / 传感器页 |
| KEY1_LONG | 直接进入参数页 |
| KEY2_SHORT | READY 时启动，RUNNING 时暂停，PAUSED 时继续 |
| KEY2_LONG | 强制进入 RUNNING |
| KEY3_SHORT | 停止，进入 READY |
| KEY3_LONG | 急停，清零圈数，进入 READY |

注意：

1. 状态页不再用 KEY1 切换固定 1/3/5 圈。
2. 目标圈数只在参数页通过 `+1 / -1` 修改。

### 14.2 参数页按键功能

| 按键 | 功能 |
|---|---|
| KEY1_SHORT | 切换当前参数项 |
| KEY1_LONG | 返回状态页 |
| KEY2_SHORT | 当前参数 +1 |
| KEY2_REPEAT | 当前参数快速增加 |
| KEY3_SHORT | 当前参数 -1 |
| KEY3_REPEAT | 当前参数快速减少 |

参数修改规则：

1. 选中 `LAP` 时：
   - KEY2：`target_laps +1`
   - KEY3：`target_laps -1`
2. 选中 `KP` 时：
   - KEY2：`track_kp +1`
   - KEY3：`track_kp -1`
3. 选中 `KD` 时：
   - KEY2：`track_kd +1`
   - KEY3：`track_kd -1`
4. 选中 `SPD` 时：
   - KEY2：`base_speed +10`
   - KEY3：`base_speed -10`
5. 选中 `MAX` 时：
   - KEY2：`max_correction +10`
   - KEY3：`max_correction -10`
6. 选中 `START_TH` 时：
   - KEY2：`start_line_threshold +1`
   - KEY3：`start_line_threshold -1`
7. 选中 `LOST_TH` 时：
   - KEY2：`lost_line_threshold +1`
   - KEY3：`lost_line_threshold -1`

每次修改后调用：

```c
AppConfig_LimitAll();
```

确保参数不会越界。

---

## 15. App 总调度设计

新增 `app.c/h`。

接口：

```c
void App_Init(void);
void App_Update_20ms(void);
```

实现：

```c
void App_Init(void)
{
    Motor_Init();
    Encoder_Reset();

    TrackSensor_Init();
    Key_Init();
    OLED_Init();

    AppConfig_InitDefault();
    CarState_Init();
    CarController_Init();

    OledUi_Init();

    Motor_Stop();
}

void App_Update_20ms(void)
{
    Key_Update_20ms();

    Menu_Update_20ms();

    CarController_Update_20ms();

    OledUi_Update_20ms();
}
```

要求：

1. 上电默认停车。
2. App 初始化最后必须调用 `Motor_Stop()`。
3. 非运行状态必须停车。
4. 所有状态修改都由 `menu.c` 处理。
5. 所有电机输出都由 `car_controller.c` 处理。

---

## 16. GPIO 引脚要求

所有新增 GPIO 引脚先用宏定义占位，方便后续替换成真实 SysConfig 名称。

例如：

```c
#define TRACK_S1_PORT   GPIOA
#define TRACK_S1_PIN    DL_GPIO_PIN_0

#define KEY1_PORT       GPIOA
#define KEY1_PIN        DL_GPIO_PIN_1

#define OLED_SCL_PORT   GPIOB
#define OLED_SCL_PIN    DL_GPIO_PIN_2
```

要求：

1. 不要把引脚硬编码在函数里。
2. 所有引脚都放在对应 `.h` 文件或专门的 `board_pins.h` 中。
3. 代码必须方便后续根据 `ti_msp_dl_config.h` 修改真实引脚。
4. 按键需要支持高电平按下或低电平按下的宏配置。
5. 循迹模块需要支持黑线高电平或黑线低电平的宏配置。

---

## 17. 安全保护要求

必须实现：

1. 上电默认停车。
2. 非 `CAR_STATE_RUNNING` 状态必须 `Motor_Stop()`。
3. KEY3 长按任何时候都急停。
4. 急停后清零 `current_lap`、`lost_count`、`lap_cooldown_ms`。
5. 丢线超过阈值后自动停车并进入 `CAR_STATE_ERROR`。
6. 完成目标圈数后自动停车并进入 `CAR_STATE_FINISHED`。
7. 所有速度必须限幅。
8. 所有 PD correction 必须限幅。
9. 所有参数必须限幅。
10. OLED 参数修改不能导致程序崩溃或数值溢出。

---

## 18. 推荐开发顺序

请不要一次性改完所有文件，建议按以下顺序实现和测试。

### 第 1 步：八路循迹读取

只实现：

```c
track_sensor.c / track_sensor.h
```

目标：

```text
OLED 或调试变量能看到 raw = 00111100 这种传感器状态。
```

不要让电机转。

---

### 第 2 步：OLED 显示

实现：

```c
oled.c / oled.h
oled_ui.c / oled_ui.h
```

目标：

```text
OLED 能显示 raw、black_count、error。
```

---

### 第 3 步：3 个按键扫描

实现：

```c
key.c / key.h
```

目标：

```text
能识别 KEY1 / KEY2 / KEY3 的短按、长按、长按连发。
OLED 能显示按键事件或页面变化。
```

---

### 第 4 步：参数菜单

实现：

```c
app_config.c / app_config.h
menu.c / menu.h
```

目标：

```text
通过 OLED 和按键修改：
LAP
KP
KD
BASE_SPEED
MAX_CORRECTION
START_LINE_THRESHOLD
LOST_LINE_THRESHOLD
```

此时仍然不要让电机高速运行。

---

### 第 5 步：基础循迹

实现：

```c
car_controller.c / car_controller.h
```

目标：

```text
小车低速沿黑线走。
```

初始参数：

```text
base_speed = 250 ~ 280
track_kp = 80 ~ 150
track_kd = 20 ~ 60
track_scale = 100
max_correction = 150 ~ 250
```

---

### 第 6 步：计圈和自动停车

加入：

```text
起点横线检测
current_lap 计数
current_lap >= target_laps 自动停车
```

目标：

```text
通过参数页设置 LAP=3，小车跑 3 圈后自动停车。
```

---

### 第 7 步：完善保护和调参

加入：

```text
丢线保护
急停
暂停/继续
参数限幅
OLED 刷新优化
```

---

## 19. 调参建议

一开始低速调试：

```text
base_speed = 250
track_kp = 100
track_kd = 30
track_scale = 100
max_correction = 180
```

现象和调整：

| 现象 | 调整 |
|---|---|
| 小车转弯反了 | 交换左右电机公式，或 correction 取负 |
| 小车左右剧烈摆动 | 降低 KP，增加 KD |
| 小车转弯不够 | 增加 KP 或 max_correction |
| 小车反应太慢 | 增加 KP |
| 小车高速抖动 | 增加 KD 或降低 base_speed |
| 经常丢线 | 降低 base_speed，检查传感器 raw |
| 起点线重复计圈 | 增加 lap_cooldown_ms |
| 起点线识别不到 | 降低 start_line_threshold |

---

## 20. Codex 实现要求总结

请严格遵守：

1. 代码必须是 CCS / TI MSPM0 DriverLib 风格。
2. 不要写 Arduino 风格代码。
3. 不要使用 `delay()` 阻塞主循环。
4. 不要在中断里刷新 OLED。
5. 不要在中断里扫描按键。
6. 不要在中断里做 PD 控制。
7. 不要重写已有 `motor.c`。
8. 不要把所有逻辑堆进 `main.c`。
9. 状态判断和参数全部封装。
10. 目标圈数必须支持 `+1 / -1`，不能固定为 1/3/5。
11. PD 参数必须支持运行时修改，并且修改后立即生效。
12. OLED 负责显示，不参与控制。
13. 按键负责产生事件和修改参数，不直接控制电机。
14. 只有 `car_controller.c` 可以根据状态机调用 `Motor_SetSpeed()` 或 `Motor_Stop()`。

---

## 21. 最终验收标准

完成后应满足：

1. 上电后小车不动，OLED 显示状态页。
2. OLED 可以显示八路循迹 raw 值。
3. 按键可以切换 OLED 页面。
4. 参数页可以修改目标圈数 `LAP`。
5. 参数页可以修改 `KP` 和 `KD`。
6. 参数页可以修改基础速度和最大修正量。
7. KEY2 可以启动或暂停小车。
8. KEY3 可以停止小车。
9. KEY3 长按可以急停并清零圈数。
10. 小车可以沿黑线循迹。
11. 经过起点横线时圈数增加。
12. 达到目标圈数后自动停车。
13. 丢线超过阈值后自动停车并进入 ERROR 状态。
14. 修改 PD 参数后，小车运动状态能实时变化。
15. 代码结构清晰，各模块职责明确。
