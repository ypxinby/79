# Vision Protocol Sender 使用说明

脚本：`vision_protocol_sender.py`

用途：

- 严格生成 Vision Target Protocol V1 固定 40 字节帧；
- 离线输出 hex 或原始二进制数据；
- 通过串口发送合法帧和异常字节流；
- 在连接 MSPM0 前验证字段布局、CRC 和串口回环；
- 不包含 MSPM0 UART Receiver 或云台控制代码。

## 1. 环境

离线模式只需要 Python 3.10+：

```powershell
python --version
```

串口发送需要 pyserial：

```powershell
python -m pip install pyserial
```

命令均从工程根目录运行。

## 2. PC 端自验证

### 2.1 固定向量和 CRC

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode self-test
```

预期：

```text
SELF-TEST PASS
header_size=8 payload_size=30 frame_size=40
valid_crc=0xD14D
no_target_crc=0xC75C
mutated_target_center_x_crc=0x7969
```

### 2.2 pyserial 虚拟回环

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode valid --port loop:// --loopback-verify --count 3
```

每次写入应显示：

```text
loopback_verify=PASS bytes=40
```

### 2.3 USB 转串口物理回环

1. USB 转串口模块使用 3.3V 电平；
2. 将模块 TX 与 RX 短接；
3. 使用实际 COM 口运行：

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode alternating --port COM7 --loopback-verify --count 10 --fps 20
```

连接 MSPM0 时不要使用 `--loopback-verify`，除非 MSPM0 专门实现了原样回显。

## 3. 基本使用

### 离线生成

不传 `--port` 即为离线模式：

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode valid
```

每个逻辑帧输出：

```text
session_id
sequence
flags
timestamp_ms
frame_hex
帧内CRC
重新计算的CRC
CRC状态
```

### 串口发送

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode valid --port COM7 --baud 115200 --count 100 --fps 30
```

### 保存 hex 和二进制流

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode sticky `
  --hex-file vision_sticky.txt `
  --bin-file vision_sticky.bin
```

`--hex-file` 保存每次串口写入的十六进制文本；`--bin-file` 保存实际串口字节流。

## 4. 固定测试向量

### 合法目标帧

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode valid `
  --session-id 0x12345678 `
  --sequence 0 `
  --timestamp-ms 1000 `
  --count 1
```

预期：

```text
CRC = 0xD14D
A5 5A 01 01 1F 00 1E 00 78 56 34 12 00 00 E8 03 00 00 80 02 E0 01 90 01 C8 00 6B 03 07 00 5E 01 96 00 64 00 64 00 4D D1
```

### 无目标帧

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode no-target `
  --session-id 0x12345678 `
  --sequence 1 `
  --timestamp-ms 1033 `
  --restart-frames 0 `
  --count 1
```

预期：

```text
CRC = 0xC75C
A5 5A 01 01 00 00 1E 00 78 56 34 12 01 00 09 04 00 00 80 02 E0 01 FF FF FF FF 00 00 FF FF 00 00 00 00 00 00 00 00 5C C7
```

## 5. 测试模式

| mode | 行为 | 用途 |
| --- | --- | --- |
| `self-test` | 不发送 | 固定向量、长度和 CRC 自检 |
| `valid` | 合法目标帧 | 正常目标路径 |
| `no-target` | 合法无目标帧 | LOST 与通信正常区分 |
| `alternating` | valid/no-target 交替 | 目标出现和消失 |
| `half` | 一帧拆成两次写入 | 半帧接收 |
| `truncated` | 残帧后发送完整帧 | 残帧恢复 |
| `sticky` | 两帧合并一次写入 | 粘包解析 |
| `noise` | 噪声、伪帧头、合法帧 | 帧头重新同步 |
| `bad-crc` | 改字段但保留旧 CRC | CRC 错误处理 |
| `bad-length` | 错误长度但 CRC 正确 | 独立长度检查 |
| `invalid-fields` | CRC 正确但字段非法 | 字段范围校验 |
| `duplicate` | 相同 sequence 发送两次 | 重复帧过滤 |
| `old-sequence` | 当前帧后发送旧帧 | 乱序过滤 |
| `sequence-wrap` | 65534、65535、0、1 | uint16 回绕 |
| `source-restart` | session_id 变化，sequence 从0开始 | 重启恢复 |

## 6. 异常模式示例

### 半帧

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode half --split 13 --half-delay-ms 50
```

实际执行两次串口写入：前 13 字节和后 27 字节。

### 截断帧

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode truncated --split 13
```

先发送 13 字节残帧，再发送下一 sequence 的完整合法帧。

### 粘包

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode sticky
```

一次串口写入 80 字节：一个 valid 帧和一个 no-target 帧。

### 随机噪声

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode noise --noise-length 32 --seed 12345
```

### 错误 CRC

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode bad-crc
```

输出应显示 `crc_state=BAD`。

### 错误长度

```powershell
python diansaikaishi\tools\vision_protocol_sender.py `
  --mode bad-length --bad-length 29
```

帧仍为 40 字节，CRC 与错误长度字段匹配，用于验证接收器独立检查 `payload_length == 30`。

### 非法字段

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode invalid-fields
```

依次生成：

```text
frame_width=0
target_center_x==frame_width
confidence=1001
TARGET_VALID=0但HAS_BBOX=1
flags保留bit非0
session_id=0
header reserved非0
```

### sequence 回绕与 source restart

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --mode sequence-wrap
python diansaikaishi\tools\vision_protocol_sender.py --mode source-restart
```

`source-restart` 先发送旧 session，再生成新的非零 session_id，并发送 sequence 0、1、2，三帧均设置 `SOURCE_RESTART`。

## 7. CLI 参数

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `--mode` | 必填 | 测试模式 |
| `--port` | 无 | COM口或 `loop://`；不填为离线模式 |
| `--baud` | 115200 | 波特率 |
| `--loopback-verify` | 关闭 | 读取并核对串口回显 |
| `--count` | 1 | 正常模式帧数 |
| `--fps` | 20 | 发送频率 |
| `--session-id` | 随机非零 | 固定会话ID，支持十进制或 `0x` |
| `--sequence` | 0 | 起始 sequence |
| `--timestamp-ms` | 程序运行时间 | 固定起始时间戳 |
| `--restart-frames` | 3 | 开头设置 SOURCE_RESTART 的帧数 |
| `--width` | 640 | 图像宽度 |
| `--height` | 480 | 图像高度 |
| `--x` | 400 | 目标中心 X |
| `--y` | 200 | 目标中心 Y |
| `--confidence` | 875 | 置信度 |
| `--target-id` | 7 | 目标 ID |
| `--bbox` | `350,150,100,100` | bbox：x,y,w,h |
| `--split` | 13 | half/truncated 分割位置 |
| `--half-delay-ms` | 50 | 半帧写入间隔 |
| `--noise-length` | 16 | 噪声长度 |
| `--seed` | 12345 | 噪声随机种子 |
| `--bad-length` | 29 | 错误 payload_length |
| `--hex-file` | 无 | 保存写入的 hex 文本 |
| `--bin-file` | 无 | 保存原始二进制流 |

查看完整内置帮助：

```powershell
python diansaikaishi\tools\vision_protocol_sender.py --help
```

## 8. 连接 MSPM0 前的验收

```text
1. self-test 显示 PASS
2. 固定合法帧 CRC 为 D14D
3. 固定无目标帧 CRC 为 C75C
4. loop:// 回环验证通过
5. USB转串口 TX/RX 物理回环通过
6. hex和bin文件能够离线生成
7. 所有测试模式运行无Python异常
```

完成以上验证只代表 PC 测试发送端正确，不代表已经实现 MSPM0 接收。下一阶段仍需单独实现非阻塞 UART、环形缓冲、协议解析和状态管理。
