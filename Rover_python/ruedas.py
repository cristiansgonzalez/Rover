#!/usr/bin/python

from navio.adafruit_pwm_servo_driver import PWM
import time
import math

import sys

import navio.gpio
import navio.util

navio.util.check_apm()

#drive Output Enable in PCA low
pin = navio.gpio.Pin(27)
pin.write(0)

PCA9685_DEFAULT_ADDRESS = 0x40
frequency = 50
IZQUIERDO_OUT=0
NAVIO_RCOUTPUT_3 = 5


NAVIO_RCOUTPUT_1 = 3
NAVIO_RCOUTPUT_2 = 4
SERVO_MIN_ms = 0.540 # mS
SERVO_MAX_ms = 2.40 # mS
NEUTRO = 1.5
A=502
B=1
#convert mS to 0-4096 scale:
SERVO_MIN = math.trunc((SERVO_MIN_ms * 4096.0) / (1000.0 / frequency) - 1)
SERVO_MAX = math.trunc((SERVO_MAX_ms * 4096.0) / (1000.0 / frequency) - 1)
NEUTRO = math.trunc((NEUTRO * 4096.0) / (1000.0 / frequency) - 1)

pwm = PWM(0x40, debug=False)

# Set frequency to 60 Hz
pwm.setPWMFreq(frequency)

while 1:
	
	#=======CONTROL========
#	input_value = GPIO.input(7)	
#	vector.append(input_value)
#	time.sleep(0.5);
#	grabartxt()
#	decoder_ppm()
#	print(T_ch1,T_ch2,T_ch3,T_ch4)


	## SALIDA ENCODER ------------------------------
	
	pwm.setPWM(NAVIO_RCOUTPUT_3, 0, int(IZQUIERDO_OUT)); 
	print "\nIzquierdo Teclado: %d" % (int(IZQUIERDO_OUT))
	print "Izquierdo Salida: %d" % (IZQUIERDO_OUT)
	# time.sleep(1);
	
#	pwm.setPWM(NAVIO_RCOUTPUT_2, 0, int(DERECHO_OUT));
#	print "\nDerecho Teclado: %d" % (int(DERECHO_OUT))1
#	print "Derecho Salida: %d" % (DERECHO_OUT)
	# time.sleep(1);
	# ---------------------------------------------------
	
	#  INGRESO DE LOS VALORES DE FORMA MANUAL POR TECLADO
#	DERECHO_OUT = int(input("\nVelocidad rueda Derecha: "))
	IZQUIERDO_OUT = int(input("Velocidad rueda Izquierda: "))
	