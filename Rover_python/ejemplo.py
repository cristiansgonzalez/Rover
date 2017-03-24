#!/usr/local/bin/python

USB_Datos=['roll','pitch','yaw']
def creartxt():
    archi=open('roll.txt','w')
    archi.close()

def grabartxt():
	global USB_Datos
	archi=open('roll.txt','a')
	archi.write(str(USB_Datos))
	archi.write('\n')
	archi.close()


import subprocess

grabartxt()
#while 1:
res = subprocess.check_output(["./AHRS"], universal_newlines=True)
print (res)

USB_Datos=res
grabartxt()