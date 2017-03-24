#!/usr/bin/python

import time, signal, sys
import spidev
import math
import navio.gpio
from navio.adafruit_ads1x15 import ADS1x15
from navio.mpu9250 import MPU9250
from navio.adafruit_pwm_servo_driver import PWM
import navio.util
import serial
import RPi.GPIO as GPIO
## posicion
from navio.Complementary_Filter2 import comp_filt
posicion = comp_filt()
posicion.__init__()
ax=0
ay=0
az=0
gx=0
gy=0
gz=0
mx=0
my=0
mz=0
first_time=1
iterm_pitch=0
iterm_roll=0
iterm_yaw=0
previous_time=0
pitch_d=0

navio.util.check_apm()



imu = MPU9250()

if imu.testConnection():
    print "Connection established: True"
else: 
    sys.exit("Connection established: False")
	
imu.initialize()

time.sleep(1)
#==================USB=============
ser = serial.Serial('/dev/ttyACM0', 115200)
d=1
state=0
#==================================
# use P1 header pin numbering convention
GPIO.setmode(GPIO.BOARD)

################################################### servos
pin = navio.gpio.Pin(27)
pin.write(0)

PCA9685_DEFAULT_ADDRESS = 0x40
frequency = 50

GPIO.setup(7, GPIO.IN)

vis=0
vector=[0]
k=0
sync=0
input_ppm_1=0
diff=0
T_ch1=0
T_ch2=0
T_ch3=0
T_ch4=0
USB_Datos=['bateria','T2','T1','A2','A1','RRpm','LRpm','ax','ay','az','gx','gy','gz','mx','my','mz']

NAVIO_RCOUTPUT_1 = 3
NAVIO_RCOUTPUT_2 = 4
NAVIO_RCOUTPUT_3 = 5
SERVO_MIN_ms = 1.250 # mS
SERVO_MAX_ms = 1.750 # mS
NEUTRO = 1.5
IZQUIERDO=2040
DERECHO=2040
#convert mS to 0-4096 scale:
SERVO_MIN = math.trunc((SERVO_MIN_ms * 4096.0) / (1000.0 / frequency) - 1)
SERVO_MAX = math.trunc((SERVO_MAX_ms * 4096.0) / (1000.0 / frequency) - 1)
NEUTRO = math.trunc((NEUTRO * 4096.0) / (1000.0 / frequency) - 1)

pwm = PWM(0x40, debug=False)

# Set frequency to 60 Hz
pwm.setPWMFreq(frequency)

###########################################################
def creartxt():
    archi=open('datos.txt','w')
    archi.close()

def grabartxt():
	global USB_Datos
	archi=open('datos.txt','a')
	archi.write(str(USB_Datos))
	archi.write('\n')
	archi.close()

	#=================== PPM ==================
def decoder_ppm():
	global sync,diff,T_ch1,input_ppm_1,T_ch2,T_ch3,T_ch4
	fin=0
#===============
	input_ppm_1=GPIO.input(7)
	while(fin==0):
		input_ppm=GPIO.input(7)
		if(input_ppm==1 and sync==0):
			tic=time.time()
			toc=time.time()
			dT=toc-tic
			while(dT<0.005):
				toc=time.time()
				dT=toc-tic
				input_ppm=GPIO.input(7)
#===================================
				if(input_ppm-input_ppm_1!=0):
					tic=time.time()
#===================================
				input_ppm_1=input_ppm
			sync=1
		diff=input_ppm-input_ppm_1
		if(diff==1 and sync==1):
			while(input_ppm==0):
				input_ppm=GPIO.input(7)
			tic=time.time()
			#input_ppm=GPIO.input(7)
			while(input_ppm==1):
				toc=time.time()
				T_ch1=toc-tic
				input_ppm=GPIO.input(7)
#=========================================================
			while(input_ppm==0):
				input_ppm=GPIO.input(7)
			tic=time.time()
			#input_ppm=GPIO.input(7)
			while(input_ppm==1):
				toc=time.time()
				T_ch2=toc-tic
				input_ppm=GPIO.input(7)
			
			while(input_ppm==0):
				input_ppm=GPIO.input(7)
			tic=time.time()
			#input_ppm=GPIO.input(7)
			while(input_ppm==1):
				toc=time.time()
				T_ch3=toc-tic
				input_ppm=GPIO.input(7)
			
			while(input_ppm==0):
				input_ppm=GPIO.input(7)
			tic=time.time()
			#input_ppm=GPIO.input(7)
			while(input_ppm==1):
				toc=time.time()
				T_ch4=toc-tic
				input_ppm=GPIO.input(7)
#==========================================================				
				
			sync=0
			fin=1
 		input_ppm_1=input_ppm
#========  FIN PPM  ================


pitch_d = None
roll_d = None
yaw_d = None

def fetch_values():
	global pitch_d, roll_d, yaw_d
	pitch_d = posicion.pitch_d
	roll_d = posicion.roll_d
	yaw_d = posicion.yaw_d

def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
#print 'Press Ctrl+C to exit'

ADS1015 = 0x00  # 12-bit ADC
ADS1115 = 0x01	# 16-bit ADC

# Select the gain
# gain = 6144  # +/- 6.144V
gain = 4096  # +/- 4.096V

sps = 250  # 250 samples per second


# Initialise the ADC using the default mode (use default I2C address)
# Set this to ADS1015 or ADS1115 depending on the ADC you are using!
adc = ADS1x15(ic=ADS1115)
grabartxt()
step = 1
pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 1000);
time.sleep(0.5);
pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 0); 
time.sleep(0.5);
pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 1000); 
time.sleep(0.5); 
pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 0);
con1 = 1
con2 = 1
con3 = 1
while d:
	
	#=======CONTROL========
	input_value = GPIO.input(7)	
#	print "\nValor entrada: %d" % (int(input_value))
	vector.append(input_value)
	time.sleep(0.07);
	#grabartxt()
	decoder_ppm()
	#print(T_ch1,T_ch2,T_ch3,T_ch4)
	#====== FIN CONTROL =====
	
	#======	MODO NEUTRO ==========
	if (T_ch3 <0.00054 and T_ch3 > 0.00050):
		con1 = 0
		con2 = 1
		con3 = 1
		pwm.setPWM(0, 0, 4095)
		pwm.setPWM(NAVIO_RCOUTPUT_1, 0, 2040); 
		pwm.setPWM(NAVIO_RCOUTPUT_2, 0, 2040);
	#===== FIN MODO NEUTRO =======
	
	#===== MODO MANUAL===================
	if (T_ch3 <0.00114 and T_ch3 > 0.00110):
		#for G in xrange(0, 4095, step):
		if (con2 == 1):
			pwm.setPWM(0, 0, 0)
			pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 1000);
			time.sleep(0.5);
			pwm.setPWM(NAVIO_RCOUTPUT_3, 0, 0); 
		con1 = 1
		con2 = 0
		con3 = 1
			#print(G)
		#=====  DESENCRIPTAR PAQUETE====
		#print "ERROR"
		state=ser.readline()
#		print(state)
		USB_Datos = state.split(",")
#		print (USB_Datos)
#		print "\n"
		#======FIN DESENCRIPTADO ======
	
		#====== DATOS IMU====
		m9a, m9g, m9m = imu.getMotion9()
		#USB_Datos.append(m9a)
		#USB_Datos.append(m9g)
		#USB_Datos.append(m9m)
		posicion.attitude3(m9a[0],m9a[1],m9a[2],m9g[0],m9g[1],m9g[2],m9m[0],m9m[1],m9m[2])
		
		fetch_values()
		
		#print(pitch1)
		USB_Datos=m9m
		vis=vis+1
		if vis==10:
			print(m9a, m9g, m9m)
			print(pitch_d,roll_d,yaw_d)
			#print(pitch_d,roll_d,yaw_d)
			vis = 0
		#===== FIN DATOS IMU ====

		# CONDICION DE MOVIMIENTO ENCODER IZQUIERDO ---------------------------	
	
		if T_ch1 < 0.00170 and T_ch1 > 0.00104: # movimiento hacia adelante
			IZQUIERDO_OUT=-3030300*T_ch1 + 5191.5

		if T_ch1 < 0.00104 and T_ch1 > 0.00051: # movimiento hacia atras
			IZQUIERDO_OUT=-3773584*T_ch1 + 5964.5
		
		if T_ch1 > 0.00170 or T_ch1 < 0.00051:
			IZQUIERDO_OUT=2040
		# --------------------------------------------------------------------
	
		# CONDICION DE MOVIMIENTO ENCODER DERECHO --------------------------

		if T_ch2 < 0.00170 and T_ch2 > 0.00113: # movimiento hacia adelante
			DERECHO_OUT=-3508800*T_ch2 + 6004.9

		if T_ch2 < 0.00113 and T_ch2 > 0.00056: # movimiento hacia atras
			DERECHO_OUT=-3508800*T_ch2 + 6004.9
		
		if T_ch1 > 0.00170 or T_ch1 < 0.00056:
			DERECHO_OUT=2040
	
		# --------------------------------------------------------------------
	
		## SALIDA ENCODER ------------------------------
	
		pwm.setPWM(NAVIO_RCOUTPUT_1, 0, int(DERECHO_OUT)); 
		#print "\nIzquierdo Teclado: %d" % (int(IZQUIERDO_OUT))
#	print "Izquierdo Salida: %d" % (IZQUIERDO_OUT)
	# time.sleep(1);
	
		pwm.setPWM(NAVIO_RCOUTPUT_2, 0, int(IZQUIERDO_OUT));
		grabartxt()
		#print "\nDerecho Teclado: %d" % (int(DERECHO_OUT))
#	print "Derecho Salida: %d" % (DERECHO_OUT)
	# time.sleep(1);
	# ---------------------------------------------------
	
	#  INGRESO DE LOS VALORES DE FORMA MANUAL POR TECLADO
#	DERECHO = int(input("\nVelocidad rueda Derecha: "))
#	IZQUIERDO = int(input("Velocidad rueda Izquierda: "))
	#========= FIN MODO MANUAL =============================
	
	#========= MODO AUTOMATICO ===========
	if (T_ch3 <0.00170 and T_ch3 > 0.00160):
		pwm.setPWM(NAVIO_RCOUTPUT_1, 0, 2040); 
		pwm.setPWM(NAVIO_RCOUTPUT_2, 0, 2040);
	#======== FIN MODO AUTOMATICO============