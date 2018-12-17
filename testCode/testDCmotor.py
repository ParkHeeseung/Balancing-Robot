# -*- coding: utf-8 -*-
#! /usr/bin/python

# 라즈베리파이 GPIO 패키지
import RPi.GPIO as GPIO
from time import sleep
import smbus
import math

########### 모터 상태 ##########
STOP  = 0
FORWARD  = 1
BACKWORD = 2

########### 모터 채널 ##########
CH1 = 0
CH2 = 1

########### PIN 입출력 설정 ###########
OUTPUT = 1
INPUT = 0

########### PIN 설정 ###########
HIGH = 1
LOW = 0

########### 실제 핀 정의 ###########
########### PWM PIN ###########
ENA = 26  #37 pin
ENB = 0   #27 pin

########### GPIO PIN ###########
IN1 = 19  #37 pin
IN2 = 13  #35 pin
IN3 = 6   #31 pin
IN4 = 5   #29 pin

########### 핀 설정 함수 ###########
def setPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100khz 로 PWM 동작 시킴
    pwm = GPIO.PWM(EN, 100)
    # 우선 PWM 멈춤.
    pwm.start(0)
    return pwm

########### 모터 제어 함수 ###########
def setMotorContorl(pwm, INA, INB, speed, stat):

    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)

    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    #뒤로
    elif stat == BACKWORD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)


########### 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈) ###########
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)



###################MPU6050#####################
# Power management registers

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	high = bus.read_byte_data(address, adr)
	low = bus.read_byte_data(address, adr + 1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)

	if(val >= 0x8000):
		return -((65535 - val) + 1)

	else:
		return val

def dist(a, b):
	return math.sqrt((a * a) + (b * b))

def get_y_rotation(x, y, z):
	radians = math.atan2(x, dist(y, z))
	return math.degrees(radians)

def get_x_rotation(x, y, z):
	radians = math.atan2(y, dist(x, z))
	return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

address = 0x68

bus.write_byte_data(address, power_mgmt_1, 0)

########### 주요 FUNC ##########
def data_transform(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


########### GPIO 모드 설정 ###########
GPIO.setmode(GPIO.BCM)

#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)


########## 제어 시작 ##########

# 앞으로 80프로 속도로
# setMotor(CH1, 80, FORWARD)
# setMotor(CH2, 80, FORWARD)
# #5초 대기
# sleep(5)
#
# # 뒤로 40프로 속도로
# setMotor(CH1, 40, BACKWORD)
# setMotor(CH2, 40, BACKWORD)
# sleep(5)
#
# # 뒤로 100프로 속도로
# setMotor(CH1, 100, BACKWORD)
# setMotor(CH2, 100, BACKWORD)
# sleep(5)
#
# #정지
# setMotor(CH1, 80, STOP)
# setMotor(CH2, 80, STOP)

kp = 60.0
ki = 200.0
kd = 1.5

P_term = 0.0
I_term = 0.0
D_term = 0.0

error = 0.0
error_prov = 0.0

output = 0.0



try:

    while True:

        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0

        print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
        print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
        print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

        x_slope = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        y_slope = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

        print "x rotation: ", x_slope
        print "y rotation: ", y_slope

        print "----------------------------------------"



        if y_slope < -50.0:
            y_slope = -50.0
        elif y_slope > 50.0:
            y_slope = 50

        speed = data_transform(y_slope, -50.0, 50.0, -100, 100)

        error = speed - 0.0

        P_term = Kp * error

        error_prov = error

        output = P_term

        print "P : ", P_term

        print "speed : ", speed

        if speed > 0.000:
            setMotor(CH1, int(speed), FORWARD)
            setMotor(CH2, int(speed), FORWARD)
        elif speed < 0.000:
            setMotor(CH1, int(-1 * speed), BACKWORD)
            setMotor(CH2, int(-1 * speed), BACKWORD)
        else:
            setMotor(CH1, 0, STOP)
            setMotor(CH2, 0, STOP)

except KeyboardInterrupt:



# 종료
GPIO.cleanup()
