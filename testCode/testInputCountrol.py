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




import sys, termios, atexit
from select import select

# save the terminal settings
fd = sys.stdin.fileno()
new_term = termios.tcgetattr(fd)
old_term = termios.tcgetattr(fd)

# new terminal setting unbuffered
new_term[3] = (new_term[3] & ~termios.ICANON & ~termios.ECHO)

# switch to normal terminal
def set_normal_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, old_term)

# switch to unbuffered terminal
def set_curses_term():
    termios.tcsetattr(fd, termios.TCSAFLUSH, new_term)

def putch(ch):
    sys.stdout.write(ch)

def getch():
    return sys.stdin.read(1)

def getche():
    ch = getch()
    putch(ch)
    return ch

def kbhit():
    dr,dw,de = select([sys.stdin], [], [], 0)
    return dr <> []

if __name__ == '__main__':
    atexit.register(set_normal_term)
    set_curses_term()

try :

    while 1:
        if kbhit():
            ch = getch()
            print ch

            if ch == 'w' or ch == 'W' :
                setMotor(CH1, int(50), FORWARD)
                setMotor(CH2, int(50), FORWARD)
            else if ch == 's'or ch == 'S':
                setMotor(CH1, int(50), BACKWORD)
                setMotor(CH2, int(50), BACKWORD)
            else if ch == 'a' or ch == 'A':
                setMotor(CH1, int(50), FORWARD)
                setMotor(CH2, int(50), BACKWORD)
            else if ch == 'd' or ch == 'D':
                setMotor(CH1, int(50), BACKWORD)
                setMotor(CH2, int(50), FORWARD)


        sys.stdout.write('balancing')

    print 'done'


except KeyboardInterrupt:

    # 종료
    GPIO.cleanup()
