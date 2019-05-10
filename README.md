# RaspRobot
赋予树莓派3B机器人车语音识别控制和对象检测的控制程序。Control program of speech recog., obj. detection and motion for robot on Raspberry Pi Model 3B.

## 使用硬件
* 树莓派 3B
* 搭载 TB6612 芯片的小车双路驱动板
* 小型电机
* 云台舵机
* 超声传感器
* 红外传感器
* USB 摄像头
* Intel® Movidius (1代) 神经加速棒

## 运行
```
python main.py
```
执行后等待约5秒，小车可进入基于传感器信号和视觉对象检测的漫步避障模式。

