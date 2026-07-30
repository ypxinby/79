# Ball ASCII Sender使用说明

当前滚球协议为：

```text
@B,seq,time,valid,state,pos,pred,v,conf,measured*CS\n
```

生成一帧但不打开串口：

```powershell
python .\diansaikaishi\tools\ball_ascii_sender.py `
  --pos 20 --pred 21 --velocity 5 --confidence 0.9
```

通过USB转TTL以20Hz连续发送400帧：

```powershell
python .\diansaikaishi\tools\ball_ascii_sender.py `
  --port COMx --fps 20 --count 400 `
  --state TRACK --pos 20 --pred 21 `
  --velocity 5 --confidence 0.9 --measured 1
```

电脑USB转TTL TX接MSPM0 PB3，GND共地，串口为115200、8N1。协议字段和异或校验定义以
`K230滚球视觉通信协议 V3.md`为准。
