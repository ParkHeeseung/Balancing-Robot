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


########### GPIO 모드 설정 ###########
GPIO.setmode(GPIO.BCM)

#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴
pwmA = setPinConfig(ENA, IN1, IN2)
pwmB = setPinConfig(ENB, IN3, IN4)


###################MPU6050#####################
# Power management registers

power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

#global tempval

def read_byte(adr):
	return bus.read_byte_data(address, adr)

def read_word(adr):
	try:
		high = bus.read_byte_data(address, adr)
		low = bus.read_byte_data(address, adr + 1)
		val = (high << 8) + low
		#tempval = val
		return val
	except:
		return 0x0000


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



#kp = 1970.0
#ki = 2300.0
#kd = 10.0

kp = 1600.0
ki = 600.0
kd = 100.0

#ki = 750.0
#kd = 5.0

P_term = 0.0
I_term = 0.0
D_term = 0.0

dT = 0.1

error = 0.0
error_prov = 0.0

output = 0.0





if __name__ == '__main__':
    atexit.register(set_normal_term)
    set_curses_term()

try :

    while 1:
        if kbhit():
            ch = getch()
            print ch

            if ch == 's' or ch == 'S' :
                accel_xout = read_word_2c(0x3b)
                accel_yout = read_word_2c(0x3d)
                accel_zout = read_word_2c(0x3f)

                accel_xout_scaled = accel_xout / 16384.0
                accel_yout_scaled = accel_yout / 16384.0
                accel_zout_scaled = accel_zout / 16384.0



            	x_slope = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
                y_slope = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

            	error = -10.0 - y_slope

                P_term = kp * error
                I_term += ki * error * dT
                D_term = kd * (error - error_prov) / dT

                error_prov = error


                if I_term > 1000:
                    I_term = 1000
                elif I_term < -1000:
                    I_term = -1000

                output = P_term + I_term + D_term


                speed = output


                if speed > 0.000:

                    if speed > 10000:
                        speed = 10000
                    speed /= 100
                    setMotor(CH1, int(speed), FORWARD)
                    setMotor(CH2, int(speed), FORWARD)
                elif speed < 0.0000:
                    if speed < -10000:
                        speed = -10000

                    speed /= 100
                    setMotor(CH1, int(-1 * speed), BACKWORD)
                    setMotor(CH2, int(-1 * speed), BACKWORD)
                else:
                	setMotor(CH1, 0, STOP)
                	setMotor(CH2, 0, STOP)


            elif ch == 'w'or ch == 'W':
                accel_xout = read_word_2c(0x3b)
                accel_yout = read_word_2c(0x3d)
                accel_zout = read_word_2c(0x3f)

                accel_xout_scaled = accel_xout / 16384.0
                accel_yout_scaled = accel_yout / 16384.0
                accel_zout_scaled = accel_zout / 16384.0



            	x_slope = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
                y_slope = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

            	error = -10.0 - y_slope

                P_term = kp * error
                I_term += ki * error * dT
                D_term = kd * (error - error_prov) / dT

                error_prov = error


                if I_term > 1000:
                    I_term = 1000
                elif I_term < -1000:
                    I_term = -1000

                output = P_term + I_term + D_term


                speed = output


                if speed > 0.000:

                    if speed > 10000:
                        speed = 10000
                    speed /= 100
                    setMotor(CH1, int(speed), FORWARD)
                    setMotor(CH2, int(speed), FORWARD)
                elif speed < 0.0000:
                    if speed < -10000:
                        speed = -10000

                    speed /= 100
                    setMotor(CH1, int(-1 * speed), BACKWORD)
                    setMotor(CH2, int(-1 * speed), BACKWORD)
                else:
                	setMotor(CH1, 0, STOP)
                	setMotor(CH2, 0, STOP)

            elif ch == 'a' or ch == 'A':
                setMotor(CH1, int(100), FORWARD)
                setMotor(CH2, int(100), BACKWORD)
		        sleep(0.2)
            elif ch == 'd' or ch == 'D':
                setMotor(CH1, int(100), BACKWORD)
                setMotor(CH2, int(100), FORWARD)
		        sleep(0.2)

        P_term = 0.0
	    I_term = 0.0
	    D_term = 0.0
	else :
		accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0



	 	x_slope = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        y_slope = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)

		error = 6.5 - y_slope

        P_term = kp * error
        I_term += ki * error * dT
        D_term = kd * (error - error_prov) / dT

        error_prov = error

        	#print "I_term : ", I_term

        if I_term > 1000:
            I_term = 1000
        elif I_term < -1000:
            I_term = -1000

        output = P_term + I_term + D_term


       	speed = output


        print "P : ", P_term

        print "speed : ", speed / 100
		if speed > 0.000:

            if speed > 10000:
                speed = 10000
            speed /= 100
            setMotor(CH1, int(speed), FORWARD)
            setMotor(CH2, int(speed), FORWARD)
        	elif speed < 0.0000:
            	if speed < -10000:
                	speed = -10000

            	speed /= 100
            	setMotor(CH1, int(-1 * speed), BACKWORD)
            	setMotor(CH2, int(-1 * speed), BACKWORD)
        	else:
            	setMotor(CH1, 0, STOP)
            	setMotor(CH2, 0, STOP)


        #sys.stdout.write('balancing')



except KeyboardInterrupt:

    # 종료
    GPIO.cleanup()
