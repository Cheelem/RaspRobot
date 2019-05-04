# By LLG 李林根
# -*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time
import threading

# 小车电机引脚定义
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

# 小车按键定义
key = 8

# 超声波引脚定义
EchoPin = 0
TrigPin = 1

# RGB三色灯引脚定义
LED_R = 22
LED_G = 27
LED_B = 24

# 蜂鸣器引脚定义
BUZZER = 8

# 舵机引脚定义
ServoPin = 23

# 红外避障引脚定义
AvoidSensorLeft = 12
AvoidSensorRight = 17

# 设置GPIO口为BCM编码方式
GPIO.setmode(GPIO.BCM)

# 忽略警告信息
GPIO.setwarnings(False)


class RobotObstacleAvoidControlThread(threading.Thread):

    def __init__(self, message_queue):
        super().__init__()
        self.message_queue = message_queue

    # 电机引脚初始化为输出模式
    # 按键引脚初始化为输入模式
    # 超声波,RGB三色灯,舵机引脚初始化
    # 红外避障引脚初始化
    @staticmethod
    def init():
        global pwm_ENA
        global pwm_ENB
        global pwm_servo
        GPIO.setup(ENA, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(ENB, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(IN3, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN4, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(key, GPIO.IN)
        GPIO.setup(EchoPin, GPIO.IN)
        GPIO.setup(TrigPin, GPIO.OUT)
        GPIO.setup(LED_R, GPIO.OUT)
        GPIO.setup(LED_G, GPIO.OUT)
        GPIO.setup(LED_B, GPIO.OUT)
        GPIO.setup(ServoPin, GPIO.OUT)
        GPIO.setup(AvoidSensorLeft, GPIO.IN)
        GPIO.setup(AvoidSensorRight, GPIO.IN)
        GPIO.setup(BUZZER, GPIO.OUT, initial=GPIO.HIGH)
        # 设置pwm引脚和频率为2000hz
        pwm_ENA = GPIO.PWM(ENA, 2000)
        pwm_ENB = GPIO.PWM(ENB, 2000)
        pwm_ENA.start(0)
        pwm_ENB.start(0)
        # 设置舵机的频率和起始占空比
        pwm_servo = GPIO.PWM(ServoPin, 50)
        pwm_servo.start(0)

    # 小车前进
    @staticmethod
    def go_ahead(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车后退
    @staticmethod
    def back(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车左转
    @staticmethod
    def left(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车右转
    @staticmethod
    def right(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车原地左转
    @staticmethod
    def spin_left(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车原地右转
    @staticmethod
    def spin_right(leftspeed, rightspeed):
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        pwm_ENA.ChangeDutyCycle(leftspeed)
        pwm_ENB.ChangeDutyCycle(rightspeed)

    # 小车停止
    @staticmethod
    def brake():
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.LOW)

    # 按键检测
    @staticmethod
    def key_scan():
        while GPIO.input(key):
            pass
        while not GPIO.input(key):
            time.sleep(0.01)
            if not GPIO.input(key):
                time.sleep(0.01)
                while not GPIO.input(key):
                    pass

    # 超声波函数
    @staticmethod
    def ultrasonic_distance_detect():
        GPIO.output(TrigPin, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TrigPin, GPIO.LOW)
        t0 = time.time()
        while not GPIO.input(EchoPin):
            if time.time() - t0 > 0.8:
                return 9999
        t1 = time.time()
        while GPIO.input(EchoPin):
            if time.time() - t1 > 0.8:
                return 9999
        t2 = time.time()
        time.sleep(0.01)
        return ((t2 - t1) * 340 / 2) * 100

    # 舵机旋转到指定角度
    @staticmethod
    def servo_appointed_detection(pos):
        for i in range(18):
            pwm_servo.ChangeDutyCycle(2.5 + 10 * pos / 180)

    def rotate_to(self, angle):
        temp_time = 0.28 * abs(angle / 90)
        if angle < 0:
            self.spin_right(85, 85)
        else:
            self.spin_left(85, 85)
        time.sleep(temp_time)

    @staticmethod
    def whistle():
        GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(BUZZER, GPIO.HIGH)
        time.sleep(0.001)

    # 舵机选装一整圈
    def servo_surrounding_distances_detection(self):
        self.back(20, 20)
        time.sleep(0.1)
        self.brake()
        self.servo_appointed_detection(0)
        time.sleep(0.8)
        pos = 180
        distances_list = []
        max_distance = -1
        max_location = -1
        for i in range(9):
            self.servo_appointed_detection(i * 22.5)
            time.sleep(0.8 / 8)
            temp_distance = self.ultrasonic_distance_detect()
            if max_distance < temp_distance:
                max_distance = temp_distance
                max_location = i
            distances_list.append(temp_distance)
        self.servo_appointed_detection(90)
        time.sleep(0.8)
        return distances_list, max_distance, max_location

    # 舵机旋转超声波测距避障，led根据车的状态显示相应的颜色
    def servo_color_carstate(self):
        # 开红灯
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        self.back(20, 20)
        time.sleep(0.08)
        self.brake()

        # 舵机旋转到0度，即右侧，测距
        self.servo_appointed_detection(0)
        time.sleep(0.8)
        rightdistance = self.ultrasonic_distance_detect()

        # 舵机旋转到180度，即左侧，测距
        self.servo_appointed_detection(180)
        time.sleep(0.8)
        leftdistance = self.ultrasonic_distance_detect()

        # 舵机旋转到90度，即前方，测距
        self.servo_appointed_detection(90)
        time.sleep(0.8)
        frontdistance = self.ultrasonic_distance_detect()

        if leftdistance < 30 and rightdistance < 30 and frontdistance < 30:
            # 亮品红色，掉头
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
            self.spin_right(85, 85)
            time.sleep(0.58)
        elif leftdistance >= rightdistance:
            # 亮蓝色
            GPIO.output(LED_R, GPIO.LOW)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
            self.spin_left(85, 85)
            time.sleep(0.28)
        elif leftdistance <= rightdistance:
            # 亮品红色，向右转
            GPIO.output(LED_R, GPIO.HIGH)
            GPIO.output(LED_G, GPIO.LOW)
            GPIO.output(LED_B, GPIO.HIGH)
        self.spin_right(85, 85)
        time.sleep(0.28)

    def servo_surrounding_detect(self):
        # 开红灯
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.LOW)
        GPIO.output(LED_B, GPIO.LOW)
        self.back(20, 20)
        time.sleep(0.08)
        self.brake()
        # 刹车
        surrounding_distances, max_distance, max_location = self.servo_surrounding_distances_detection()
        print("max location=", max_location, ", max distance=", max_distance)
        if max_distance <= 30:
            self.back(20, 20)
            time.sleep(0.08)
            self.rotate_to(180)
            self.servo_surrounding_detect()
        else:
            self.rotate_to((max_location - 4) * 22.5)

    def change_lane(self):
        self.whistle()
        GPIO.output(LED_R, GPIO.HIGH)
        GPIO.output(LED_G, GPIO.HIGH)
        GPIO.output(LED_B, GPIO.LOW)
        print("CHANGING LANE")
        # 执行变道
        self.brake()
        self.rotate_to(50)
        self.go_ahead(5, 5)
        time.sleep(1.4)
        self.rotate_to(-50)
        self.go_ahead(10, 10)
        self.message_queue.task_done()
        self.whistle()

    def run(self):
        try:
            self.init()
            self.whistle()
            self.servo_appointed_detection(90)
            # 延时5s
            time.sleep(5)
            # try/except语句用来检测try语句块中的错误，
            # 从而让except语句捕获异常信息并处理。
            # key_scan()
            self.whistle()
            while True:
                if self.message_queue.empty() is False:
                    print("SIGNAL=", self.message_queue.get_nowait())
                    self.change_lane()
                else:
                    distance = self.ultrasonic_distance_detect()
                    # 遇到障碍物,红外避障模块的指示灯亮,端口电平为LOW
                    # 未遇到障碍物,红外避障模块的指示灯灭,端口电平为HIGH
                    left_sensor_value = (GPIO.input(AvoidSensorLeft))
                    right_sensor_value = (GPIO.input(AvoidSensorRight))
                    if distance > 50:
                        print("Inf. left=", left_sensor_value, " / right=", right_sensor_value)
                        if left_sensor_value == 1 and right_sensor_value == 1:
                            self.go_ahead(10, 10)  # 当两侧均未检测到障碍物时调用前进函数
                        elif left_sensor_value == 1 and right_sensor_value == 0:
                            print("Inf. right has obstacle")
                            self.rotate_to(45)  # 右边探测到有障碍物，有信号返回，原地向左转
                        elif right_sensor_value == 1 and left_sensor_value == 0:
                            print("Inf. left has obstacle")
                            self.rotate_to(-45)  # 左边探测到有障碍物，有信号返回，原地向右转
                        elif right_sensor_value == 0 and left_sensor_value == 0:
                            print("Inf. both have obstacle")
                            self.back(15, 15)
                            time.sleep(0.2)
                            self.brake()
                            self.spin_right(85, 85)  # 当两侧均检测到障碍物时调用固定方向的避障(原地右转)
                            time.sleep(0.28)
                        self.go_ahead(10, 10)
                        GPIO.output(LED_R, GPIO.LOW)
                        GPIO.output(LED_G, GPIO.HIGH)
                        GPIO.output(LED_B, GPIO.LOW)
                    elif 30 <= distance <= 50:
                        if left_sensor_value == 1 and right_sensor_value == 1:
                            self.go_ahead(5, 5)  # 当两侧均未检测到障碍物时调用前进函数
                        elif left_sensor_value == 1 and right_sensor_value == 0:
                            print("Inf. right has obstacle")
                            self.rotate_to(90)  # 右边探测到有障碍物，有信号返回，原地向左转
                        elif right_sensor_value == 1 and left_sensor_value == 0:
                            print("Inf. left has obstacle")
                            self.rotate_to(-90)  # 左边探测到有障碍物，有信号返回，原地向右转
                            time.sleep(0.1)
                        elif right_sensor_value == 0 and left_sensor_value == 0:
                            print("Inf. both have obstacle")
                            self.back(20, 20)
                            time.sleep(0.2)
                            self.brake()
                            self.spin_right(85, 85)  # 当两侧均检测到障碍物时调用固定方向的避障(diaotou)
                            time.sleep(0.50)
                        self.go_ahead(5, 5)
                        GPIO.output(LED_R, GPIO.HIGH)
                        GPIO.output(LED_G, GPIO.LOW)
                        GPIO.output(LED_B, GPIO.HIGH)
                    elif distance < 30:
                        if right_sensor_value is False or left_sensor_value is False:
                            self.back(15, 15)
                            time.sleep(0.5)
                            self.brake()
                        # servo_color_carstate()
                        GPIO.output(LED_R, GPIO.HIGH)
                        GPIO.output(LED_G, GPIO.LOW)
                        GPIO.output(LED_B, GPIO.LOW)
                        self.servo_surrounding_detect()

        except KeyboardInterrupt:
            pwm_ENA.stop()
            pwm_ENB.stop()
            GPIO.cleanup()
        except BaseException:
            print("ERROR!")
        pwm_ENA.stop()
        pwm_ENB.stop()
        GPIO.cleanup()
