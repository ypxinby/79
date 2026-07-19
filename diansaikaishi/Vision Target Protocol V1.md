# Vision Target Protocol V1

> 文档状态：V1 协议冻结版  
> 适用发送端：K230、OpenMV 或其他能够通过 UART 输出目标信息的视觉设备  
> 适用接收端：MSPM0G3507 及后续兼容控制器  
> 更新日期：2026-07-19

## 1. 目的与边界

Vision Target Protocol V1 用于将视觉端选定的单个目标发送给 MSPM0。

协议只传递目标观测，不传递云台角度、角速度、STEP、DIR、EN 或电机控制命令：

```text
K230 / OpenMV / 其他摄像头
        ↓ Vision Target Protocol V1
vision_protocol / vision_receiver
        ↓
gimbal_vision_adapter
        ↓ GimbalTargetObservation
GimbalTracker
        ↓
Gimbal YAW / PITCH
```

协议不绑定 K230。更换视觉平台时，只需要让新的发送端生成相同的 40 字节帧，不修改 MSPM0 Tracker 和云台底层。

V1 明确禁止：

- JSON；
- 字符串字段；
- 浮点字段；
- 由视觉端直接发送 STEP/DIR/EN；
- 使用视觉端时间戳判断 MSPM0 本地通信超时。

---

## 2. 传输参数

第一版 UART 参数：

```text
波特率：115200 baud
数据位：8
校验位：None
停止位：1
流控：None
传输格式：二进制
多字节整数：小端
```

建议发送频率：

```text
正常目标帧：20～30 FPS
无目标帧：保持相同或接近的发送频率
```

40 字节帧在 115200 8N1 下的带宽：

```text
30 FPS：约 12 kbit/s
60 FPS：约 24 kbit/s
```

---

## 3. 帧总体结构

V1 的 `TARGET_REPORT` 为固定 40 字节：

```text
┌─────────────── 8 bytes ───────────────┐
│ magic/version/type/flags/length       │
├────────────── 30 bytes ───────────────┤
│ session_id ... bbox_height            │
├─────────────── 2 bytes ───────────────┤
│ CRC-16/CCITT-FALSE                    │
└───────────────────────────────────────┘
```

常量：

```text
帧头：A5 5A
协议版本：01
消息类型：01 = TARGET_REPORT
payload_length：30 = 0x001E
完整帧长度：40 bytes
```

`payload_length` 的精确定义：

```text
payload_length 表示 offset 8 的 session_id
到 offset 37 的 bbox_height 最后一个字节之间的长度。

V1 固定为 30 字节。

不包含：
- offset 0～1 帧头；
- offset 2 协议版本；
- offset 3 消息类型；
- offset 4 flags；
- offset 5 reserved；
- offset 6～7 payload_length 字段自身；
- offset 38～39 CRC16。
```

通用解析器可定义：

```text
VISION_PROTOCOL_MAX_PAYLOAD = 64 bytes
VISION_PROTOCOL_MAX_FRAME = 8 + 64 + 2 = 74 bytes
```

但 V1 `TARGET_REPORT` 只接受 `payload_length == 30`。长度不是 30 的 V1 目标帧必须丢弃。

---

## 4. 完整字段表

| Offset | 长度 | 类型 | 字段 | 含义 | 合法范围/固定值 | 异常值与处理 |
| ---: | ---: | --- | --- | --- | --- | --- |
| 0 | 1 | `uint8` | `magic_0` | 第一帧头字节 | `0xA5` | 不匹配时继续搜索帧头 |
| 1 | 1 | `uint8` | `magic_1` | 第二帧头字节 | `0x5A` | 不匹配时重新同步 |
| 2 | 1 | `uint8` | `version` | 协议版本 | V1 固定 `0x01` | 未知版本丢弃，不刷新通信时间 |
| 3 | 1 | `uint8` | `message_type` | 消息类型 | `0x01 = TARGET_REPORT` | 未知类型丢弃，不刷新通信时间 |
| 4 | 1 | `uint8` | `flags` | 目标和可选字段标志 | 见 flags 定义 | bit5～7 非零时丢弃 |
| 5 | 1 | `uint8` | `reserved` | 保留字段 | V1 必须为 `0` | 非零时丢弃 |
| 6 | 2 | `uint16` | `payload_length` | payload 字节数 | V1 固定 `30` | 不等于 30 时丢弃；大于 64 立即拒绝 |
| 8 | 4 | `uint32` | `session_id` | 本次视觉程序会话 ID | 非零，每次程序启动重新生成 | 为 0 时丢弃；不得只用 `timestamp_ms` 生成 |
| 12 | 2 | `uint16` | `sequence` | 帧序列号 | `0～65535`，自然回绕 | 重复帧和乱序旧帧丢弃 |
| 14 | 4 | `uint32` | `timestamp_ms` | 视觉程序启动后的毫秒数 | `0～0xFFFFFFFF`，允许回绕 | 仅用于日志，不用于 MSPM0 超时计算 |
| 18 | 2 | `uint16` | `frame_width` | 图像宽度，单位 px | `1～8192` | 0 或超范围时丢弃 |
| 20 | 2 | `uint16` | `frame_height` | 图像高度，单位 px | `1～8192` | 0 或超范围时丢弃 |
| 22 | 2 | `uint16` | `target_center_x` | 目标中心 X，原点在左上角 | 有效目标时 `< frame_width` | 无目标时固定 `0xFFFF` |
| 24 | 2 | `uint16` | `target_center_y` | 目标中心 Y，原点在左上角 | 有效目标时 `< frame_height` | 无目标时固定 `0xFFFF` |
| 26 | 2 | `uint16` | `confidence` | 归一化置信度 | 有置信度时 `0～1000` | 无置信度或无目标时固定 `0` |
| 28 | 2 | `uint16` | `target_id` | 目标跟踪 ID | `0～65534` | 无 ID 或无目标时固定 `0xFFFF` |
| 30 | 2 | `uint16` | `bbox_x` | bbox 左上角 X | 有 bbox 时 `< frame_width` | 无 bbox 或无目标时固定 `0` |
| 32 | 2 | `uint16` | `bbox_y` | bbox 左上角 Y | 有 bbox 时 `< frame_height` | 无 bbox 或无目标时固定 `0` |
| 34 | 2 | `uint16` | `bbox_width` | bbox 宽度 | 有 bbox 时 `> 0` | 无 bbox或无目标时固定 `0` |
| 36 | 2 | `uint16` | `bbox_height` | bbox 高度 | 有 bbox 时 `> 0` | 无 bbox或无目标时固定 `0` |
| 38 | 2 | `uint16` | `crc16` | 帧 CRC | CRC-16/CCITT-FALSE，小端保存 | 不匹配时丢弃，不刷新任何有效时间 |

所有多字节字段均为小端。例如：

```text
session_id = 0x12345678
发送字节 = 78 56 34 12
```

---

## 5. flags 定义

| Bit | 名称 | 含义 |
| ---: | --- | --- |
| 0 | `TARGET_VALID` | `target_center_x/y` 表示当前有效目标 |
| 1 | `HAS_BBOX` | bbox 四个字段有效 |
| 2 | `HAS_TARGET_ID` | `target_id` 有效 |
| 3 | `HAS_CONFIDENCE` | `confidence` 有效，范围 0～1000 |
| 4 | `SOURCE_RESTART` | 视觉程序刚启动或刚建立新会话 |
| 5 | reserved | V1 必须为 0 |
| 6 | reserved | V1 必须为 0 |
| 7 | reserved | V1 必须为 0 |

定义建议：

```c
#define VISION_FLAG_TARGET_VALID    (1U << 0)
#define VISION_FLAG_HAS_BBOX        (1U << 1)
#define VISION_FLAG_HAS_TARGET_ID   (1U << 2)
#define VISION_FLAG_HAS_CONFIDENCE  (1U << 3)
#define VISION_FLAG_SOURCE_RESTART  (1U << 4)
#define VISION_FLAG_RESERVED_MASK   (0xE0U)
```

合法组合示例：

```text
0x00：通信正常，当前无目标
0x01：有目标中心，无 bbox、ID、confidence
0x09：有目标中心和 confidence
0x0F：有目标中心、bbox、ID、confidence
0x10：新会话启动阶段的无目标帧
0x1F：新会话启动阶段，且所有目标字段有效
```

`SOURCE_RESTART` 与目标是否有效相互独立。

发送端建议在每次启动后的前 3 帧设置 `SOURCE_RESTART`，以提高首帧丢失时的可观察性。MSPM0 不得仅凭该标志绕过 sequence 检查，`session_id` 变化才是重建会话的主要依据。

---

## 6. 坐标、目标和 bbox 规则

坐标系：

```text
原点：(0, 0)，图像左上角
X 正方向：向右
Y 正方向：向下
单位：pixel
```

MSPM0 视觉适配层计算：

```c
error_x_px = (int32_t)target_center_x - (int32_t)(frame_width / 2U);
error_y_px = (int32_t)target_center_y - (int32_t)(frame_height / 2U);
```

bbox 格式：

```text
bbox_x / bbox_y：左上角
bbox_width / bbox_height：宽度和高度
```

验证 bbox 边界时使用更宽的整数类型，避免 `uint16_t` 相加溢出：

```c
(uint32_t)bbox_x + bbox_width <= frame_width
(uint32_t)bbox_y + bbox_height <= frame_height
```

### 6.1 有效目标

当 `TARGET_VALID=1`：

```text
target_center_x < frame_width
target_center_y < frame_height
```

当 `HAS_CONFIDENCE=1`：

```text
confidence <= 1000
```

当 `HAS_CONFIDENCE=0`：

```text
confidence 必须为 0
```

当 `HAS_TARGET_ID=0`：

```text
target_id 必须为 0xFFFF
```

当 `HAS_BBOX=0`：

```text
bbox_x = 0
bbox_y = 0
bbox_width = 0
bbox_height = 0
```

### 6.2 无目标

当 `TARGET_VALID=0`：

```text
HAS_BBOX = 0
HAS_TARGET_ID = 0
HAS_CONFIDENCE = 0

frame_width / frame_height 保持当前有效分辨率
target_center_x = 0xFFFF
target_center_y = 0xFFFF
confidence = 0
target_id = 0xFFFF
bbox_x = 0
bbox_y = 0
bbox_width = 0
bbox_height = 0
```

无目标帧是合法协议帧：

- 刷新 MSPM0 的 `last_valid_packet_time`；
- 不刷新 `last_valid_target_time`；
- 表示 LOST，不表示 COMM_TIMEOUT。

---

## 7. session_id 规则

`session_id` 用于区分视觉程序的不同运行会话。

发送端规则：

1. K230、OpenMV 或 PC 测试程序每次启动时生成新的非零 `uint32_t session_id`；
2. 同一次程序运行期间保持不变；
3. 禁止简单地直接使用 `timestamp_ms` 作为 `session_id`；
4. 优先使用硬件随机数、操作系统随机源或持久化启动计数与随机数的组合；
5. 如果生成结果为 0，必须重新生成。

建议实现：

```text
K230/Linux Python：os.urandom(4) 或 secrets.randbits(32)
OpenMV：硬件 RNG，例如 pyb.rng()；必要时混合持久化 boot counter
PC 测试器：secrets.randbits(32)
```

MSPM0 规则：

```text
首次收到合法帧：
    保存 session_id
    初始化 sequence

session_id 改变：
    视为视觉端新会话
    清除旧 sequence 状态
    接受当前合法帧作为新基线
    重置与旧目标相关的滤波和运动历史

session_id 相同：
    按正常 sequence 规则判断
```

`SOURCE_RESTART` 只作为启动提示和诊断信息，不能代替 `session_id`。

---

## 8. sequence 规则

发送端：

```text
每发送一帧 sequence 加 1
valid=0 的无目标帧也必须递增
65535 后自然回绕到 0
```

MSPM0 在 `session_id` 相同时使用有符号差值：

```c
int16_t delta = (int16_t)(new_sequence - last_sequence);

if (delta > 0) {
    /* 正常新帧，包括 uint16_t 自然回绕 */
} else if (delta == 0) {
    /* 重复帧，丢弃 */
} else {
    /* 乱序旧帧，丢弃 */
}
```

禁止使用：

```c
new_sequence > last_sequence
```

因为该写法无法处理 `65535 -> 0` 回绕。

只有完整帧通过以下全部检查后，才能更新 `last_sequence`：

```text
帧头
长度
CRC
版本
消息类型
reserved
flags
字段范围
session/sequence
```

重复帧和旧帧不得刷新 `last_valid_packet_time` 或 `last_valid_target_time`。

如果 MSPM0 已进入本地 `COMM_TIMEOUT`，接收层可清除 `sequence_initialized`，将下一帧合法数据重新作为 sequence 基线；如果视觉程序发生了重启，正常情况下仍应通过新的 `session_id` 识别。

---

## 9. timestamp_ms 规则

`timestamp_ms` 是视觉端程序启动后的本地毫秒计数：

- 可以从 0 开始；
- 允许 `uint32_t` 自然回绕；
- 用于日志、视觉帧间隔和问题追踪；
- 不要求与 MSPM0 时钟同步；
- 不得与 MSPM0 当前时间直接相减判断通信超时。

MSPM0 在完整帧通过全部校验后记录：

```text
local_rx_timestamp_ms = MSPM0 当前本地时间
```

所有新鲜度、LOST 和 COMM_TIMEOUT 判断均使用该本地时间。

---

## 10. CRC-16/CCITT-FALSE

固定参数：

```text
名称：CRC-16/CCITT-FALSE
poly：0x1021
init：0xFFFF
refin：false
refout：false
xorout：0x0000
```

计算范围：

```text
offset 0 ～ offset 37
共 38 字节
包含帧头、版本、类型、flags、reserved、payload_length 和完整 payload
不包含 offset 38～39 的 CRC16 自身
```

CRC 结果以小端放在帧尾：

```text
offset 38：CRC 低字节
offset 39：CRC 高字节
```

参考实现：

```python
def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
```

CRC 错误帧必须丢弃，并且：

- 不更新 Observation；
- 不更新 session 或 sequence；
- 不刷新合法协议帧时间；
- 增加 CRC 错误计数。

---

## 11. 合法目标帧示例

字段：

```text
version = 1
message_type = TARGET_REPORT
flags = 0x1F
  TARGET_VALID=1
  HAS_BBOX=1
  HAS_TARGET_ID=1
  HAS_CONFIDENCE=1
  SOURCE_RESTART=1
session_id = 0x12345678
sequence = 0
timestamp_ms = 1000
frame_width = 640
frame_height = 480
target_center_x = 400
target_center_y = 200
confidence = 875
target_id = 7
bbox = (350, 150, 100, 100)
```

前 38 字节计算出的 CRC：

```text
CRC = 0xD14D
帧尾小端字节 = 4D D1
```

完整 40 字节：

```text
A5 5A 01 01 1F 00 1E 00
78 56 34 12 00 00 E8 03 00 00
80 02 E0 01 90 01 C8 00
6B 03 07 00 5E 01 96 00 64 00 64 00
4D D1
```

单行形式：

```text
A5 5A 01 01 1F 00 1E 00 78 56 34 12 00 00 E8 03 00 00 80 02 E0 01 90 01 C8 00 6B 03 07 00 5E 01 96 00 64 00 64 00 4D D1
```

---

## 12. 无目标帧示例

字段：

```text
flags = 0x00
session_id = 0x12345678
sequence = 1
timestamp_ms = 1033
frame_width = 640
frame_height = 480
target_center_x = 0xFFFF
target_center_y = 0xFFFF
confidence = 0
target_id = 0xFFFF
bbox = (0, 0, 0, 0)
```

前 38 字节计算出的 CRC：

```text
CRC = 0xC75C
帧尾小端字节 = 5C C7
```

完整 40 字节：

```text
A5 5A 01 01 00 00 1E 00
78 56 34 12 01 00 09 04 00 00
80 02 E0 01 FF FF FF FF
00 00 FF FF 00 00 00 00 00 00 00 00
5C C7
```

单行形式：

```text
A5 5A 01 01 00 00 1E 00 78 56 34 12 01 00 09 04 00 00 80 02 E0 01 FF FF FF FF 00 00 FF FF 00 00 00 00 00 00 00 00 5C C7
```

该帧必须被识别为：

```text
协议通信正常
当前没有目标
刷新 last_valid_packet_time
不刷新 last_valid_target_time
```

---

## 13. 异常流与健壮性测试方案

### 13.1 错误 CRC

基于合法目标帧，将 `target_center_x` 低字节从 `0x90` 改为 `0x91`，但保留原 CRC `4D D1`：

```text
... E0 01 91 01 C8 00 ... 4D D1
```

修改后正确 CRC 应为：

```text
0x7969
```

因此保留 `4D D1` 的帧必须产生 CRC_ERROR，且不刷新任何有效时间。

测试还应覆盖：

- 只翻转 flags 的一个 bit；
- 只翻转 payload_length 的一个 bit；
- 只翻转 CRC 自身；
- 连续发送多个错误 CRC 后恢复合法帧。

### 13.2 错误长度

测试两种情况：

```text
payload_length = 29
payload_length = 65，大于 VISION_PROTOCOL_MAX_PAYLOAD
```

分别生成：

1. 修改长度但保留旧 CRC；
2. 修改长度并重新计算正确 CRC。

第二种可以验证接收器不是只依赖 CRC，而是会独立检查 V1 `payload_length == 30`。

### 13.3 半帧

测试序列：

```text
发送合法帧前 13 字节
等待 20～100ms
发送剩余 27 字节
```

预期：最终只解析出一个合法帧。

再测试：

```text
只发送前 13 字节
不发送剩余部分
随后直接发送一个完整合法帧
```

预期：丢弃残缺数据并重新同步到后续完整帧。

### 13.4 粘包

不加延时连续发送：

```text
合法目标帧 + 无目标帧
合法帧 + 合法帧 + 合法帧
```

预期：分别解析出 2 帧和 3 帧，sequence 依次更新。

### 13.5 随机噪声

测试：

```text
随机噪声 + 合法帧
合法帧 + 随机噪声 + 合法帧
孤立的 A5 + 噪声 + 合法帧
伪帧头 A5 5A + 错误长度 + 噪声 + 合法帧
```

预期：

- 噪声不能生成 Observation；
- 解析器能够重新找到 `A5 5A`；
- 最终合法帧正常通过；
- 100us STEP 调度不被阻塞。

### 13.6 sequence 与会话

必须包含：

```text
重复 sequence
小于当前值的旧 sequence
65534 -> 65535 -> 0 -> 1 自然回绕
session_id 改变且 sequence 从 0 开始
SOURCE_RESTART 连续设置 3 帧
session_id 不变但错误地把 sequence 清零
```

预期：

- 重复帧、旧帧丢弃；
- 正常回绕接受；
- 新 session 接受并重建 sequence 基线；
- `SOURCE_RESTART` 不绕过 session/sequence 规则。

### 13.7 字段范围

至少生成：

```text
frame_width = 0
target_center_x == frame_width
confidence = 1001 且 HAS_CONFIDENCE=1
HAS_CONFIDENCE=0 但 confidence 非0
HAS_TARGET_ID=0 但 target_id != 0xFFFF
HAS_BBOX=1 但 bbox_width=0
bbox_x + bbox_width > frame_width
TARGET_VALID=0 但 HAS_BBOX=1
flags bit5～7 非0
reserved 非0
session_id = 0
```

这些帧即使 CRC 正确，也必须按字段或协议错误丢弃。

---

## 14. PC 端 Python 测试发送器设计

本阶段只冻结设计，不在 MSPM0 中实现 UART。

建议后续新增：

```text
tools/vision_protocol_sender.py
```

依赖：

```text
Python 3.10+
pyserial
标准库：argparse、dataclasses、struct、secrets、time、random
```

### 14.1 核心结构

```python
@dataclass
class VisionTargetReport:
    session_id: int
    sequence: int
    timestamp_ms: int
    frame_width: int
    frame_height: int
    target_center_x: int
    target_center_y: int
    confidence: int
    target_id: int
    bbox_x: int
    bbox_y: int
    bbox_width: int
    bbox_height: int
    flags: int
```

核心函数：

```text
crc16_ccitt_false(data) -> int
validate_report(report) -> None
build_target_frame(report) -> bytes
build_no_target_frame(...) -> bytes
corrupt_crc(frame) -> bytes
replace_payload_length(frame, length, recalc_crc) -> bytes
split_frame(frame, split_offset) -> tuple[bytes, bytes]
make_noise(length) -> bytes
```

### 14.2 struct 格式

推荐分开打包 header 和 payload：

```python
HEADER_FORMAT = "<2sBBBBH"       # 8 bytes
PAYLOAD_FORMAT = "<IHIHHHHHHHHHH"  # 30 bytes
CRC_FORMAT = "<H"                # 2 bytes
```

构帧流程：

```python
header = struct.pack(
    HEADER_FORMAT,
    b"\xA5\x5A",
    1,
    1,
    flags,
    0,
    30,
)

payload = struct.pack(PAYLOAD_FORMAT, ...)
body = header + payload
crc = crc16_ccitt_false(body)
frame = body + struct.pack(CRC_FORMAT, crc)

assert len(header) == 8
assert len(payload) == 30
assert len(frame) == 40
```

### 14.3 session_id 与时间

```python
session_id = 0
while session_id == 0:
    session_id = secrets.randbits(32)
```

禁止：

```python
session_id = timestamp_ms
```

时间戳使用单独的程序运行时间：

```python
start = time.monotonic()
timestamp_ms = int((time.monotonic() - start) * 1000) & 0xFFFFFFFF
```

### 14.4 CLI 设计

建议命令：

```text
python vision_protocol_sender.py --port COM7 --baud 115200 --mode valid --fps 30
python vision_protocol_sender.py --port COM7 --mode no-target --count 100
python vision_protocol_sender.py --port COM7 --mode alternating --fps 20
python vision_protocol_sender.py --port COM7 --mode sticky
python vision_protocol_sender.py --port COM7 --mode half --split 13
python vision_protocol_sender.py --port COM7 --mode bad-crc
python vision_protocol_sender.py --port COM7 --mode bad-length
python vision_protocol_sender.py --port COM7 --mode noise
python vision_protocol_sender.py --port COM7 --mode sequence-wrap
python vision_protocol_sender.py --port COM7 --mode source-restart
```

建议模式：

| mode | 行为 |
| --- | --- |
| `valid` | 连续发送合法目标帧 |
| `no-target` | 连续发送合法无目标帧 |
| `alternating` | 有目标和无目标交替 |
| `sticky` | 多帧拼接后一次写入串口 |
| `half` | 一帧拆成两次写入 |
| `truncated` | 发送残帧后直接发送完整帧 |
| `bad-crc` | 字段改变但保留旧 CRC |
| `bad-length` | 生成长度错误、CRC可正确或错误的帧 |
| `noise` | 在合法帧之间插入随机字节和伪帧头 |
| `duplicate` | 重复发送相同 sequence |
| `old-sequence` | 发送乱序旧帧 |
| `sequence-wrap` | 发送 65534、65535、0、1 |
| `source-restart` | 更换 session_id，sequence 从0开始并设置 bit4 |
| `range-error` | CRC正确但字段范围非法 |

### 14.5 输出与可重复性

发送器应同时输出：

```text
session_id
sequence
timestamp_ms
flags
frame hex
CRC
发送模式
```

异常测试支持固定随机种子，便于问题复现：

```text
--seed 12345
```

正常 `session_id` 仍应使用独立随机源，测试需要固定会话时可显式传入：

```text
--session-id 0x12345678
```

---

## 15. 版本兼容策略

V1 接收器规则：

```text
version 必须为 1
message_type 必须为 1
payload_length 必须为 30
完整帧必须为 40 字节
reserved 必须为 0
flags bit5～7 必须为 0
```

兼容原则：

1. 不得在 V1 中改变现有字段偏移、类型或单位；
2. 改变现有字段语义必须升级协议版本；
3. 新增其他消息可分配新的 `message_type`；
4. 未知版本或消息类型不得刷新 V1 合法通信时间；
5. 接收器后续可以通过 `switch(version)` 同时支持多个版本，但第一版只实现 V1。

---

## 16. V1 冻结结论

Vision Target Protocol V1 冻结为：

```text
固定 40 字节二进制帧
8 字节 header
30 字节 payload
2 字节 CRC
小端整数
无字符串、无JSON、无浮点
平台无关
session_id 区分视觉程序会话
sequence 处理重复、乱序和回绕
SOURCE_RESTART 提供启动提示
CRC-16/CCITT-FALSE 覆盖 offset 0～37
MSPM0 超时只使用本地接收时间
```

在本文档确认前，不进入 MSPM0 UART Receiver 实现。协议确认后，下一阶段先实现 PC 测试发送器，再实现 UART 非阻塞接收和主循环协议解析。
