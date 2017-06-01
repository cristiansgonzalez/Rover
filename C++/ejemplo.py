# -*- coding: utf-8 -*-
"""============================================================================
Mobile Robot:
@authors:   Cristan Gonzalez
            Diana Rojas
            David Alejandro Zambrano Prada
            María Camila Merchán Riveros
============================================================================"""
# Librerías
from __future__ import division
#import matplotlib.pyplot as plt
import numpy as np
import cmath
import spidev
import time
import argparse 
import sys, signal
import navio.mpu9250
import navio.util
import math
import cmath

import serial

import subprocess

from numpy import array

#==================== Inicializar variables======================
usb_datos='arduino'
otro1= 'hola'
otro2= 'mundo'
datoenviado =str(475)
PI = 3.14159265359
vi = 0.0
YawAnterior = 0.0
YawActual = 0.0
T = 0.05
hokuyo = str(50)

#==================== IMU =======================
navio.util.check_apm()

parser = argparse.ArgumentParser()
parser.add_argument("-i", help = "Sensor selection: -i [sensor name]. Sensors names: mpu is MPU9250, lsm is LSM9DS1")

args = parser.parse_args()

imu = navio.mpu9250.MPU9250()

imu.initialize()
time.sleep(1)
#================= FIN IMU ====================================

#============ USB =======================================
arduino1=serial.Serial('/dev/ttyACM0',9600)
#====================== FIN USB============================

#==================== Crear Folder ========================
def crearFolder():
	folder=open('datos_leidos.txt','w')
	folder.close()	
#================== Fin Crear Folder ======================

#==================== Almacenar datos ========================
def almacenarDatos():
	folder=open('datos_leidos.txt','a')
	folder.write((usb_datos))
	folder.write('\n')
	folder.close()	
#================== Fin Crear Folder ======================

#================= Almacenar giro yaw =======================

def YawGiro(angle):
	global YawActual,YawAnterior,vi
	yawa = angle

	if (yawa < 0 and YawAnterior >0 and yawa < -150 and yawa > -180):
		vi = vi + 1.0
		YawActual = yawa + vi*360
	
	if (yawa > 0 and YawAnterior < 0 and yawa > 150 and yawa < 180):
		vi = vi - 1.0
		YawActual = yawa + vi*360
	
	YawActual =yawa + vi*360
	YawAnterior = yawa
	
	return YawActual

#================= Fin Almacenar giro yaw ====================
# Hokuyo URG-04LX-UG01
import pyurg

def Gaussian(x, mu, sig):
#    (1/np.sqrt(2.*np.pi*np.power(sig, 2.)))
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

def Hokuyo(Umbral,mu,sigma):
    "Variables"
    force=np.zeros(22)
    force_ww=np.zeros(22)
    angle=np.linspace(-120.,120.,682)
    data, timestamp = urg.capture()
    length=data[44:726]
    Fr=np.complex(0)
    Fr_ww=np.complex(0)
    "Inicialización de los vectores"
    angle_mean=np.zeros(22) 
    poly_mean=np.zeros(22)
    detection=np.zeros(22)
    weight=np.zeros(22) 
    for i in range(22):
        angle_mean[i]=np.mean(angle[0+(31*i):31+(31*i)])
        poly_mean[i]=np.mean(length[0+(31*i):31+(31*i)])
        weight[i]=Gaussian(angle_mean[i],mu,sigma)
        if poly_mean[i]<Umbral:
            detection[i]=1
        force_ww[i]=(1/np.square(poly_mean[i]))*10e6*detection[i]
        force[i]=(1/np.square(poly_mean[i]))*10e6*detection[i]*weight[i]
        Fr=Fr + cmath.rect(force[i],angle_mean[i]*cmath.pi/180)
        Fr_ww=Fr_ww + cmath.rect(force_ww[i],angle_mean[i]*cmath.pi/180)
    Fdo,phio=cmath.polar(np.abs(Fr_ww)-Fr)
    print detection
    print phio
    return phio*180/np.pi

"""============================================================================
Programa principal
============================================================================"""
"Conexión con el Hokuyo"
urg = pyurg.UrgDevice()
# Connect to the URG device.
# If could not conncet it, get False from urg.connect()
if not urg.connect():
    print 'Could not connect.'
    exit()

crearFolder()
almacenarDatos()
"Constantes"
mu=0
sigma=40
Umbral=1000

while 1:
	yaw = subprocess.check_output(["./yaw"], universal_newlines=True)
	print (yaw)

#	USB_Datos=yaw
	
#	usb_datos=arduino1.readline()
#	arduino1.write(datoenviado)

#	usb_datos= otro1 + '\t' + otro2 + usb_datos 
#	print "Arduino = %s" %(usb_datos)
	hokuyo=Hokuyo(Umbral,mu,sigma)
	if(hokuyo<5 and hokuyo>-5):
		if(yaw<5 and yaw>-5):
#			arduino1.write('1')
			time.sleep(0.5)
		else:
			arduino1.write('yaw')

#	if(hokuyo>5 and hokuyo<90 or hokuyo<-5 and hokuyo>-90):
#        time.sleep(0.5)
        
#		arduino1.write(hokuyo)

#	almacenarDatos()
#    time.sleep(0.5)