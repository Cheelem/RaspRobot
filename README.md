# RaspRobot
赋予树莓派3B机器人车语音识别控制和对象检测的控制程序。

Control program of speech recog., obj. detection and motion for robot on Raspberry Pi Model 3B.

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

# More About RaspRobot

本项目为东北大学软件学院智能驾驶技术课程（英语课程）的实践内容之一。 

This project is for the course **Intelligent Driving Tech.** of Software College, Northeastern University (China).

## Introduction 
I have implemented a small 4-wheel vehicle with a simple autonomous driving system including an ultra-sonic, an infrared distance sensor, camera for object detection, a Movidius Neural Compute Stick VPU accelerator and a Raspberry Pi for program control.

 <div align="center">
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-Sensors.png" width="600px" >
 </div>
 
* Figure 1. The vehicle and its sensors This main functionality of this vehicle is obstacle avoiding based on depth perception and object detection. 
 
## Depth Perception 
For this part, the vehicle process the input from the ultra-sonic sensor and the infrared distance sensor, dividing the front environment into 8 discrete areas (180/8 = 22.5 degrees for each area), and for each area, the depth of it will be sensed and calculated. And finally, the vehicle will choose the direction which is farthest away from the obstacle. 
 
## Object Detection 
In this part, I use the MobileNet-SSD as the deep learning model to handle the object detection problem. The MobileNet is a light weight base network proposed by google, which uses depth-wise separable convolution to calculate traditional convolution separately, counting down the calculation difficulty for devices with less computation ability like mobile devices. And SSD is a kind of learning-based object detection model we have learnt from the class. For this vehicle, the performance of Raspberry Pi is too weak to directly run the object detection model in an acceptable speed. So I use the Movidius Neural Compute Stick to accelerate it.  To make the deep learning model run on the Neural Compute Stick, I converted the model from Caffe (a deep-learning framework) weights to the NCS graph using NCS SDK provided by Movidius. After that, I can directly inference the model on the compute stick. 
 
## Build Up All Things Together 
As the vehicle control program has 2 parts, the vehicle control and the object detection, I 
wrote the control program in multi-thread. One thread is responsible for the vehicle control (control the GPIO voltage level, the PWM cycle, etc.) and the other thread is responsible for the object detection by calling the compute stick. 

<div align="center">
<img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-Message-Queue.png" width="600px" >
</div>

* Figure 2. The multi-thread data communication in my implementation
 
When the object detection thread detected the “car” object, it will notify the vehicle control system via a message queue. So that the vehicle control will control the vehicle to “change the lane” from the left. 
 
## Brief Slides Intro.

 <div align="center">
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-1.png" width="700px" >
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-2.png" width="700px" >
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-3.png" width="700px" >
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-4.png" width="700px" >
 <img src="https://github.com/Cheelem/RaspRobot/blob/master/images/IDT-5.png" width="700px" >
 </div>

## Lisence

本项目遵循 MIT 开源协定之规定。
