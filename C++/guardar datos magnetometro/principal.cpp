/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
MPU9250 inertial measurement unit over SPI on Raspberry Pi + Navio.

Navio's onboard MPU9250 is connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag
*/

#include "Navio/MPU9250.h"
#include "Navio/Util.h"
#include <stdlib.h>

#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h> 
#include <termios.h> 
#include <fcntl.h>
#include <sys/time.h>
#include <string.h> 

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "AHRS.hpp"

#include <Navio/gpio.h>
#include "Navio/PCA9685.h"

#define NAVIO_RCOUTPUT_1 3
#define NAVIO_RCOUTPUT_2 4
#define G_SI 9.80665
#define PI   3.14159

#include "Navio/rs232.h"
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif
//================================ Options =====================================

unsigned int samplingRate      = 1;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 4;      // Number of channels packed in PPM
unsigned int servoFrequency    = 50;     // Servo control frequency
bool verboseOutputEnabled      = true;   // Output channels values to console

//============================ Objects & data ==================================

float channels[4],pulse0;
float buf_datosc[15];
float salidas[5];
float ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw, YawActual,YawAnterior,yawa,dato1;
char bufc[14][250], reply[50];
float Ts = 50000.0;
FILE *pFile,*fd;
MPU9250 imu;
AHRS    ahrs;

char Cabezera = 'A',serial[50],aux1[50],aux2[50],aux3[50],aux4[50],aux5[50],aux6[50],aux7[50];
int apuntador,apuntador2,lon=50,j,salir, giro=0,cont,pos;
float girar=0.0, vi,bateria,TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda;
struct timeval t_ini, t_fin;
double secs;
int i, n;

int  cport_nr=24;        /* /dev/ttyS0 (COM1 on windows) */
int    bdrate=115200;       /* 9600 baud */

unsigned char buf[4096];
char mode[]={'8','N','1',0};
//============================== PPM decoder ===================================

// Timing data

float offset[3];
struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

// Network data

int sockfd;
struct sockaddr_in servaddr = {0};
char sendline[80];


unsigned int currentChannel = 0;
unsigned int previousTick;
unsigned int deltaTime;

int CreateFile(){

	pFile=fopen("Datos.txt","w");
	if((pFile=fopen("Datos.txt","w")) != NULL){}
	else{ 
	printf("No se pudo crear el archivo");
	}
  
}
// ALMACENAMIENTO DE LOS DATOS ----------------------
int Filerecovery()
{
 /*     buf_datosc[0]=ax;
      buf_datosc[1]=ay;
      buf_datosc[2]=az;
      buf_datosc[3]=gx;
	  buf_datosc[4]=gy;
      buf_datosc[5]=gz;*/
      buf_datosc[0]=mx;
      buf_datosc[1]=my;
	  buf_datosc[2]=mz;
      buf_datosc[9]=salidas[2];
      buf_datosc[10]=salidas[3];
	  buf_datosc[11]=salidas[4];
      buf_datosc[12]=salidas[5];
      buf_datosc[13]=roll;
      buf_datosc[14]=pitch;
	  buf_datosc[15]=YawActual;
	  buf_datosc[16]=bateria;
      buf_datosc[17]=TempDerecho;
	  buf_datosc[18]=TempIzquierdo;
      buf_datosc[19]=IDerecha;
      buf_datosc[20]=IIzquierda;
      buf_datosc[21]=RpmDerecha;
	  buf_datosc[22]=RpmIzquierda;
	  
for (int i = 0; i <= 2; ++i)
      {
//save new data in archive        
        sprintf(bufc[i], "%f", buf_datosc[i]);
        fputs(bufc[i],pFile);
        fputs("\t",pFile);
      }
//	  fputs(serial,pFile);
	  fputs("\n",pFile);
}
// FIN ALMACENAMIENTO DE LOS DATOS ------------------

double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) - (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

int YawGiro()
{
	yawa = yaw*-1;
//	printf("actual:%4.4f anterior:%4.4f yactual: %4.4f giro: %4.9f\n",yawa,YawAnterior,YawActual,vi);
//	vi = vi + 1;
	if (yawa < 0 && YawAnterior >0 && yawa < -150 && yawa > -180)
	{
		vi = vi + 1.0;	
		YawActual = yawa + vi*360;
	}
	
	if (yawa > 0 && YawAnterior < 0 && yawa > 150 && yawa < 180)
	{
			vi = vi - 1.0;	
			YawActual = yawa + vi*360;
	}
	
	YawActual =yawa + vi*360;
	YawAnterior = yawa;
//	printf("Resultado: %4.4f  Giro: %4.1f \n",YawActual,vi);
}


int Desencriptar()
{
	apuntador=999;
	apuntador2=999;
	salir=999;
	cont = -1;
	pos = 0;
	char* palabra = "\t134";
	float b = atoi(palabra);
	
	bateria = 0.0;
	TempDerecho = 0.0;
	TempIzquierdo = 0.0;
	IDerecha = 0.0;
	IIzquierda = 0.0;
	RpmDerecha = 0.0;
	RpmIzquierda = 0.0;
	
//	string str1("argue2000");
//	for (int i=0;i <= 50;i++)
//	{
//		serial[i]=' ';
//	}
	for(int i=0;i<lon;i++)
	{
		aux1[i] = ' ';
		aux2[i] = ' ';
		aux3[i] = ' ';
		aux4[i] = ' ';
		aux5[i] = ' ';
		aux6[i] = ' ';
		aux7[i] = ' ';
		serial[i]=' ';
	}

	for (int i=0;i<lon;i++)
	{
		if (buf[i]==Cabezera && apuntador == 999)
		{
			apuntador =i;
		}
		if (buf[i]==Cabezera && apuntador == i && apuntador2 == 999)
		{
			apuntador2 =i;
			i=lon;
		}
	}
	
	for (int i=0;i<lon;i++)
	{
		j =i+apuntador;
		if (buf[j]==',' || buf[j]=='A')
		{
			cont++;
			pos=0;
			serial[i]='\t';
			if (buf[j]=='A' && salir == 1)
			{
				i=lon;
			}
			if (buf[j]=='A' && salir == 999)
			{
				serial[i]=' ';
				salir = 1;
			}
		}
		else
		{	
			serial[i] =buf[j];
			
		}
		if(cont == 0)
		{
			aux1[i] = serial[i];  //Valor de la bateria
		}
		if(cont == 1)
		{
			aux2[pos] = serial[i]; // Valor de la temperatura motor derecho
			pos++;
		}
		if(cont == 2)
		{
			aux3[pos] = serial[i];  //Valor de la temperatura motor izquierdo
			pos++;
		}
		if(cont == 3)
		{
			aux4[pos] = serial[i];  //Valor de la corriente motor derecho
			pos++;
		}
		if(cont == 4)
		{
			aux5[pos] = serial[i];  //Valor de la corriente motor izquierdo
			pos++;
		}
		if(cont == 5)
		{
			aux6[pos] = serial[i];  //Valor de la Rpm motor derecho
			pos++;
		}
		if(cont == 6)
		{
			aux7[pos] = serial[i];  //Valor de la Rpm motor izquierdo
			pos++;
		}
		
	}
	//ajuste de los datos del roboteq a datos reales
	
	bateria = atoi(aux1)*0.0024+0.1;
	TempDerecho = atoi(aux2);
	TempIzquierdo = atoi(aux3);
	IDerecha = atoi(aux4);
	IIzquierda = atoi(aux5);
	RpmDerecha = atoi(aux6)*2.0*PI/60.0;
	RpmIzquierda = atoi(aux7)*2.0*PI/60.0;
	
//	printf("Baeria: %2.5f TempDerecho: %2.1f TempIzquierdo: %2.1f IDerecha: %3.1f IIzquierda: %3.1f RpmDerecha: %4.1f RpmIzquierda: %4.1f\n",bateria, TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda);
	
//	printf("Valor desencriptado: %s\n",serial);
	
}		
	
// DATOS AHRS --------------

void imuSetup()
{
	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        gx *= 180 / PI;
        gy *= 180 / PI;
        gz *= 180 / PI;
		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}

void imuLoop()
{
    //----------------------- Calculate delta time ----------------------------

	gettimeofday(&tv,NULL);
	previoustime = currenttime;
	currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;
	if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
	dt = (currenttime - previoustime) / 1000000.0;

    //-------- Read raw measurements from the MPU and update AHRS --------------

    // Accel + gyro.
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax /= G_SI;
    ay /= G_SI;
    az /= G_SI;
    gx *= 180 / PI;
    gy *= 180 / PI;
    gz *= 180 / PI;
    ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

    // Accel + gyro + mag. 
    // Soft and hard iron calibration required for proper function.
    /*
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, my, mx, -mz, dt);
    */

    //------------------------ Read Euler angles ------------------------------

    ahrs.getEuler(&roll, &pitch, &yaw);

    //------------------- Discard the time of the first cycle -----------------

    if (!isFirst)
    {
    	if (dt > maxdt) maxdt = dt;
    	if (dt < mindt) mindt = dt;
    }
    isFirst = 0;

    //------------- Console and network output with a lowered rate ------------

    dtsumm += dt;
    if(dtsumm > 0.05)
    {
        // Console output
		//yaw = yaw * -1;
        //printf("ROLL: %+05.2f PITCH: %+05.2f YAW: %+05.2f PERIOD %.4fs RATE %dHz \n", roll, pitch, yaw * -1, dt, int(1/dt));
		//printf("YAW: %+05.4f\n",yaw);
		//printf("PERIOD %.4fs \n",dt);

        // Network output
        sprintf(sendline,"%10f %10f %10f %10f %dHz\n", ahrs.getW(), ahrs.getX(), ahrs.getY(), ahrs.getZ(), int(1/dt));
        sendto(sockfd, sendline, strlen(sendline), 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

        dtsumm = 0;
    }
}
// FIN DATOS AHRS --------
void ppmOnEdge(int gpio, int level, uint32_t tick)
{
	if (level == 0) {	
		deltaTime = tick - previousTick;
		previousTick = tick;
	
		if (deltaTime >= ppmSyncLength) { // Sync
			currentChannel = 0;

/*			// RC output
			for (int i = 0; i < ppmChannelsNumber; i++)
			    pwm->setPWMuS(i + 3, channels[i]); // 1st Navio RC output is 3
*/
			// Console output
			if (verboseOutputEnabled) {
				printf("\n");
				for (int i = 0; i < ppmChannelsNumber; i++)
					printf("%4.f ", channels[i]);
			}
		}
		else
			if (currentChannel < ppmChannelsNumber)
				channels[currentChannel++] = deltaTime;
	}
}


//================================== Main ======================================

using namespace Navio;
using namespace std;
//=============================================================================

int main(int argc, char *argv[])
{
	girar = 0.0;
 if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");

    return(0);
  }
  printf("ENTRO AL RSalida");
	static const uint8_t outputEnablePin = RPI_GPIO_27;

    if (check_apm()) {
        return 1;
    }
    
	//--------------------------- Network setup -------------------------------

    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    servaddr.sin_family = AF_INET;

    if (argc == 3)  {
        servaddr.sin_addr.s_addr = inet_addr(argv[1]);
        servaddr.sin_port = htons(atoi(argv[2]));
    } else {
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr.sin_port = htons(7000);
    }
	
    Pin pin(outputEnablePin);

    if (pin.init()) {
        pin.setMode(Pin::GpioModeOutput);
        pin.write(0); /* drive Output Enable low */
    } else {
        fprintf(stderr, "Output Enable not set. Are you root?");
		return 1;
    }

	// Servo controller setup

	PCA9685 pwm;
	pwm.initialize();
	pwm.setFrequency(servoFrequency);
	gpioSetAlertFunc(ppmInputGpio, ppmOnEdge);

	// GPIO setup

	gpioCfgClock(samplingRate, PI_DEFAULT_CLK_PERIPHERAL, 0); /* last parameter is deprecated now */
	gpioInitialise();
	previousTick = gpioTick();

	//-------------------------------------------------------------------------
	
	imu.initialize();
	imuSetup();
	
    //-------------------------------------------------------------------------
	imuLoop();
	YawActual=0;
	YawAnterior=0;
	YawGiro();
	CreateFile();
	
    while(1) 
	{
		gettimeofday(&t_ini, NULL);
		gpioSetAlertFunc(ppmInputGpio, ppmOnEdge);
		imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
		/*printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
		printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
		printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
		printf("%4.f %4.d", channels[1], salidas[1]);*/
		
//		Rs232();
		Desencriptar();
		imuLoop();
		YawGiro();
		
	//  NEUTRO----------------------
	if (channels[2]<930 && channels[2]>926)
	{
		pwm.setPWMmS(NAVIO_RCOUTPUT_1, 10);
		pwm.setPWMmS(NAVIO_RCOUTPUT_2, 10);
	}
	//---FIN NEUTRO-----------------
	
	// MANUAL-----------------------------
	
	if (channels[2]<1529 && channels[2]>1520)
	{
		//    IZQUIERDO -------------
		if (channels[0] < 2100 && channels[0] > 1565) //ADELANTE
		{
			salidas[0]=-0.0103*channels[0]+22.0888;   
			salidas[2]=0.1869*channels[0]-292.5234;   //porcentaje palanca 0-100%
			salidas[4]=0.0654*channels[0]-52.3832;   //duty cycle
		}
		if (channels[0] < 1518 && channels[0] > 930) //ATRAS
		{
			salidas[0]=-0.0102*channels[0]+27.4898;
			salidas[2]= 0.1701*channels[0]-258.1633;  //porcentaje palanca 0-(-100)%
			salidas[4]=0.0765*channels[0]-66.1735;   //duty cycle
		}
		if (channels[0] < 930 && channels[0] > 2100 || channels[0] < 1565 && channels[0] > 1518)  //FUERA DEL RANGO. SE DETIENE
		{
			salidas[0]=10;
			salidas[2]=0;
			salidas[4]=50;  //duty cycle
		}
		//// FIN IZQUIERDO ----------
		//	   DERECHO	
		if (channels[1] < 2085 && channels[1] > 1535)  //ADELANTE
		{
			salidas[1]=-0.0127*channels[1]+29.8390;
			salidas[3]=0.1818*channels[1]-279.0909;
			salidas[5]=0.0545*channels[1]-28.7273;   //duty cycle
		}
		if (channels[1] < 1524 && channels[1] > 950)   //ATRAS
		{
			salidas[1]=-0.0126*channels[1]+30.2216;
			salidas[3]=0.1802*channels[1]-274.5946;
			salidas[5]=0.0631*channels[1]-46.1081;   //duty cycle
		}
		if (channels[1] < 965 && channels[1] > 2085 || channels[0] < 1535 && channels[0] > 1524)  ////FUERA DEL RANGO. SE DETIENE
		{
			salidas[1]=10;
			salidas[3]=0;
			salidas[5]=50;  //duty cycle
		}
		// FIN DERECHO ------------
		//  SALIDAS ----------------
		pwm.setPWMmS(NAVIO_RCOUTPUT_1, salidas[1]); //salida izquierdo
		pwm.setPWMmS(NAVIO_RCOUTPUT_2, salidas[0]);	//salida derecho
		//printf("Izquierdo: %2.4f Derecho: %2.4f Izquierdo: %2.4f Derecho: %2.4f\n",salidas[1],salidas[0],salidas[3],salidas[2]);
		
		//printf("Derecho: %4d , %4d, \n",d-5,d-5);
		// FIN SALIDAS -------------
	}
	
	//  MODO AUTOMATICO  ------------------------
	if (channels[2]<2098 && channels[2]> 2095)
	{
		pwm.setPWMmS(NAVIO_RCOUTPUT_1, 10);
		pwm.setPWMmS(NAVIO_RCOUTPUT_2, 10);
	}
	// FIN AUTOMATICO   -------------------------
	
	//Modos de grabacion y seguridad
	
	// FRENO DE EMERGENCIA Y CREACION DEL ARCHIVO
	
	// GGRABAR DATOS
	Filerecovery();
	
	gettimeofday(&t_fin, NULL); // final del contador, para una iteracion.

	secs = timeval_diff(&t_fin, &t_ini);
    	while((secs*1000000.0)<Ts)
	{
   	gettimeofday(&t_fin,NULL);
		secs = timeval_diff(&t_fin, &t_ini);
		
	}
//	printf("Tiempo calculado %.4fs\n",secs);
	
    }
	
	//sleep(1);
}
