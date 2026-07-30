# Ball Position Protocol V2发送工具

脚本：`vision_protocol_sender.py`

用途：生成和发送K230滚球V2固定40字节帧，验证CRC、粘包、乱序、无目标和异常恢复。V2坐标为小球相对O点的带符号毫米值，不再使用图像像素。

## 环境

离线生成只需要Python 3；串口发送还需要：

```powershell
python -m pip install pyserial
```

命令均从工程根目录执行。

## 协议自检

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py --mode self-test
```

预期：

```text
SELF-TEST PASS
header_size=8 payload_size=30 frame_size=40
positive_50mm_crc=0x1749
no_target_crc=0x2366
negative_50mm_crc=0xAD86
```

## 正常坐标

发送`+50mm`：

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py `
  --mode ball-mm --port COM7 `
  --span-mm 300 --position-mm 50 `
  --confidence 900 --fps 50 --count 500
```

发送`-50mm`：

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py `
  --mode ball-mm --port COM7 `
  --span-mm 300 --position-mm -50 `
  --confidence 900 --fps 50 --count 500
```

`--span-mm 300`表示水管以O为中心的有效总长度是300mm，合法位置为`-150～+150mm`。

## 无目标

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py `
  --mode ball-no-target --port COM7 `
  --span-mm 300 --fps 50 --count 500
```

MSPM0应显示`BALL:NONE`，不得继续使用最后一次位置。

## 离线与回环

不带`--port`时只打印帧：

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py `
  --mode ball-mm --span-mm 300 --position-mm 0
```

虚拟串口回环：

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py `
  --mode ball-mm --port loop:// --loopback-verify `
  --span-mm 300 --position-mm 20 --count 3
```

## 异常模式

| mode | 用途 |
| --- | --- |
| `alternating` | 有目标/无目标交替 |
| `half` | 一帧拆成两次发送 |
| `truncated` | 残帧后恢复 |
| `sticky` | 两帧粘在一次写入中 |
| `noise` | 噪声和伪帧头恢复 |
| `bad-crc` | CRC错误 |
| `bad-length` | payload长度错误 |
| `invalid-fields` | 坐标范围、flags或保留字段非法 |
| `duplicate` | 重复sequence |
| `old-sequence` | 乱序旧帧 |
| `sequence-wrap` | 65535到0回绕 |
| `source-restart` | K230新session恢复 |

示例：

```powershell
python .\diansaikaishi\tools\vision_protocol_sender.py --mode sticky
python .\diansaikaishi\tools\vision_protocol_sender.py --mode bad-crc
python .\diansaikaishi\tools\vision_protocol_sender.py --mode invalid-fields
python .\diansaikaishi\tools\vision_protocol_sender.py --mode sequence-wrap
```

## 主要参数

| 参数 | 含义 |
| --- | --- |
| `--port` | 实际串口，例如COM7；省略则离线 |
| `--baud` | 默认115200 |
| `--span-mm` | 水管有效总长度，1～5000mm |
| `--position-mm` | 相对O点的带符号毫米位置 |
| `--confidence` | 0～1000 |
| `--fps` | 发送频率，推荐50Hz |
| `--count` | 发送帧数 |
| `--session-id` | 可选固定非零uint32 |
| `--sequence` | 起始序号 |
| `--restart-frames` | 启动标志帧数，默认3 |

连接MSPM0时不要使用`--loopback-verify`，除非下位机专门实现了原样回显。
