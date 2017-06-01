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
#include <Navio/I2Cdev.h>

#define NAVIO_RCOUTPUT_1 3
#define NAVIO_RCOUTPUT_2 4
#define G_SI 9.80665
#define PI   3.14159

// librerias i2c 
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>
#include <math.h>

const int HMC5883L_I2C_ADDR = 0x1E;

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
float buf_datosc[40];
float salidas[5];
float ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw, YawActual,YawAnterior,yawa,dato1;
char bufc[14][250], reply[50];
float Ts = 50000.0,angle;
FILE *pFile,*fd,*fpa;
MPU9250 imu;
AHRS    ahrs;

char Cabezera = 'A',serial[50],aux1[50],aux2[50],aux3[50],aux4[50],aux5[50],aux6[50],aux7[50],aux8[100],aux9[100];
int fd2;
unsigned char buf2[16];
int apuntador,apuntador2,lon=50,j,salir, giro=0,cont,pos,correccion=0;
float girar=0.0, vi,bateria,TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda,ajusteyaw=0.0,UL,UR,Velocidad,Giro;
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

// CONSTANTES PID ==================================================
float 	/*Kp=72.0, Ti=0.75, */Td=0.1667, Umin=-100.0, Umax=100.0, errorR=0.0, WtR=3.0, UpR=0.0, error_1R=0.0,YtR=0.0,UiR=0.0,UdR=0.0,Ui1R=0.0,UtR=0.0;
float 	errorL=0.0, WtL=3.0, UpL=0.0, error_1L=0.0,YtL=0.0,UiL=0.0,UdL=0.0,Ui1L=0.0,UtL=0.0;
float	KpR=1.8,Ti=0.1,t=0.0,KpL=1.8,conta=3.0;
// PID ====================================================================
int PIDMotor()
{
	if(conta<t)
	{
		KpL=KpL-0.2;
		conta=conta+1.0;
	}
	
	YtR = RpmDerecha;
	YtL = RpmIzquierda;
	
	errorR=(WtR-YtR);
	UpR=KpR*errorR; //proportional action
	UiR=Ui1R+(KpR*(0.05)/Ti)*errorR; //integral action
	UdR=(KpR*Td/(Ts/1000000.0))*(errorR-error_1R); //differential action
	UtR=UpR+UiR; //total control action  ESTA ES LA ACCIÓN QUE SE ESCRIBE SOBRE EL MOTOR
	
	if (UtR<Umin)
	{
		UtR=Umin;
	}
	if (UtR>Umax)
	{
		UtR=Umax;  //constraints
	}
//	UiR=UtR-UpR-UdR; //Anti Reset-Windup

	error_1R=errorR;
	Ui1R=UiR; //store for next sampling time
	
	
	errorL=(WtL-YtL);
	UpL=KpL*errorL; //proportional action
	UiL=Ui1L+(KpL*0.05/Ti)*errorL; //integral action
	UdL=(KpL*Td/(Ts/1000000.0))*(errorL-error_1L); //differential action
	UtL=UpL+UiL; //total control action  ESTA ES LA ACCIÓN QUE SE ESCRIBE SOBRE EL MOTOR
	
	if (UtL<Umin)
	{
		UtL=Umin;
	}
	if (UtL>Umax)
	{
		UtL=Umax;  //constraints
	}
//	UiL=UtL-UpL-UdL; //Anti Reset-Windup

	error_1L=errorL;
	Ui1L=UiL; //store for next sampling time
}
//FIN PID =================================================================


// i2c inicio--------------------------
void selectDevice(int fd2, int addr, char * name)
{
    if (ioctl(fd2, I2C_SLAVE, addr) < 0)
    {
        fprintf(stderr, "%s not present\n", name);
        //exit(1);
    }
}

void writeToDevice(int fd2, int reg, int val)
{
    buf2[0]=reg;
    buf2[1]=val;

    if (write(fd2, buf2, 2) != 2)
    {
        fprintf(stderr, "Can't write to ADXL345\n");
        //exit(1);
    }
}

int DatosI2C()
{
	buf2[0] = 0x03;

	if ((write(fd2, buf2, 1)) != 1)
	{
		// Send the register to read from
		fprintf(stderr, "Error writing to i2c slave\n");
	}

	if (read(fd2, buf2, 6) != 6) 
	{
		fprintf(stderr, "Unable to read from HMC5883L\n");
	}
	else
	{
		short x = (buf2[0] << 8) | buf2[1];
        short y = (buf2[4] << 8) | buf2[5];
		short z = (buf2[2] << 8) | buf2[3];
           
		angle = atan2(y, x) * 180 / M_PI;

		//for (int b=0; b<6; ++b)
		//{
			//printf("%02x ",buf2[b]);
		//}
		//printf("\n");
            
//		printf("x=%d, y=%d, z=%d\n", x, y, z);
//		printf("angle = %0.1f\n\n", angle);
	}
}

//-i2c fin--------

int Rs232()
{
	
n = RS232_PollComport(cport_nr, buf, 100);

    if(n > 0)
    {
      buf[n] = 0;   /* always put a "null" at the end of a string! */

      for(i=0; i < n; i++)
      {
        if(buf[i] < 32)  /* replace unreadable control-codes by dots */
        {
          buf[i] = '.';
        }
      }
// printf("received %i bytes: %s\n", n, buf);
  //    printf("received %i bytes: %s\n", n, (char *)buf);
	  
    }
#ifdef _WIN32
//    Sleep(10);
#else
//    usleep(100000);  /* sleep for 100 milliSeconds */
#endif
 // RS232_CloseComport(cport_nr)
}

int CreateFile(){

	pFile=fopen("Datos.txt","w");
	if((pFile=fopen("Datos.txt","w")) != NULL){}
	else{ 
	printf("No se pudo crear el archivo");
	}
	fputs("ax\t",pFile);
	fputs("ay\t",pFile);
	fputs("az\t",pFile);
	fputs("gx\t",pFile);
	fputs("gy\t",pFile);
	fputs("gz\t",pFile);
	fputs("mx\t",pFile);
	fputs("my\t",pFile);
	fputs("mz\t",pFile);
	fputs("MotorIzquierdo\t",pFile);
	fputs("MotorDerecho\t",pFile);
	fputs("DutyMotorIzquierdo\t",pFile);
	fputs("DutyMotorDerecho\t",pFile);
	fputs("roll\t",pFile);
	fputs("pitch\t",pFile);
	fputs("yaw\t",pFile);
	fputs("yaw_Gps\t",pFile);
	fputs("Bateria\t",pFile);
	fputs("TDerecho\t",pFile);
	fputs("TIzquierdo\t",pFile);
	fputs("IDerecho\t",pFile);
	fputs("IIzquierdo\t",pFile);
	fputs("WDerecho\t",pFile);
	fputs("WIzquierdo\t",pFile);
	fputs("U\t",pFile);
	fputs("UR\t",pFile);
	fputs("UL\t",pFile);
	fputs("UtR\t",pFile);
	fputs("UtL\t",pFile);
	fputs("errorR\t",pFile);
	fputs("UiR\t",pFile);
	fputs("UdR\t",pFile);
	fputs("UpR\t",pFile);
	fputs("Ui1R\t",pFile);
	fputs("errorL\t",pFile);
	fputs("UiL\t",pFile);
	fputs("UdL\t",pFile);
	fputs("UpL\t",pFile);
	fputs("Ui1L\t",pFile);
	fputs("t\t",pFile);
	fputs("KpL\n",pFile);
	
	//  ABRIR ARCHIVO DE LECTURA =======================================
	fpa = fopen("ruta.txt","r");

	//Checks if file is empty
	if( fpa == NULL )
	{                       
		return 1;
	}
	// =================================================================
	
	t=0.0;
	
}
// ALMACENAMIENTO DE LOS DATOS ----------------------
int Filerecovery()
{
      buf_datosc[0]=ax;
      buf_datosc[1]=ay;
      buf_datosc[2]=az;
      buf_datosc[3]=gx;
	  buf_datosc[4]=gy;
      buf_datosc[5]=gz;
      buf_datosc[6]=mx;
      buf_datosc[7]=my;
	  buf_datosc[8]=mz;
      buf_datosc[9]=salidas[2];
      buf_datosc[10]=salidas[3];
	  buf_datosc[11]=salidas[4];
      buf_datosc[12]=salidas[5];
      buf_datosc[13]=roll;
      buf_datosc[14]=pitch;
	  buf_datosc[15]=YawActual;
	  buf_datosc[16]=angle;
	  buf_datosc[17]=bateria;
      buf_datosc[18]=TempDerecho;
	  buf_datosc[19]=TempIzquierdo;
      buf_datosc[20]=IDerecha;
      buf_datosc[21]=IIzquierda;
      buf_datosc[22]=RpmDerecha;
	  buf_datosc[23]=RpmIzquierda;
	  buf_datosc[24]=Velocidad;
      buf_datosc[25]=UR;
	  buf_datosc[26]=UL;
	  buf_datosc[27]=UtR;
	  buf_datosc[28]=UtL;
	  
	  buf_datosc[29]=errorR;
	  buf_datosc[30]=UiR;
	  buf_datosc[31]=UdR;
	  buf_datosc[32]=UpR;
	  buf_datosc[33]=Ui1R;
	  buf_datosc[34]=errorL;
	  buf_datosc[35]=UiL;
	  buf_datosc[36]=UdL;
	  buf_datosc[37]=UpL;
	  buf_datosc[38]=Ui1L;
	  
	  buf_datosc[39]=t;
	  buf_datosc[40]=KpL;
	  
for (int i = 0; i <= 40; ++i)
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

int Automatico()
{
/*	float EntradaL = UtL;
	float EntradaR = UtR;
*/	float EntradaL = Velocidad;
	float EntradaR = Velocidad;
	
	if(EntradaL>0 && EntradaL <=100)
	{
		UL = -0.0550*EntradaL + 6.0;
	}
	if(EntradaL<0 && EntradaL >=-100)
	{
		UL = -0.0580*EntradaL + 12.0;
	}
	if(EntradaL==0)
	{
		UL = 10;
	}
	
	if(EntradaR>0 && EntradaR <=100)
	{
		UR = -0.0650*(EntradaR*0.5689 + 0.2808) + 7.0;
	}
	if(EntradaR<0 && EntradaR >=-100)
	{
		UR = -0.0680*EntradaR + 11.0;
	}
	if(EntradaR==0)
	{
		UR = 10;
	}
//	printf("Velocidad: %f UL: %f UR: %f\n",Velocidad,UL,UR);
}

int RutaRover()
{
	char line[100];
	
	for(int i=0;i<100;i++)
	{
		aux8[i] = ' ';
		aux9[i] = ' ';
		serial[i]=' ';
	}

//	while( fgets(line,100,fpa) )
//	{
	fgets(line,100,fpa);
		cont = 0;
		salir = 0;
		for(int i=0;i<100;i++)
		{
			if (line[i]=='\t')
			{
				pos=0;
				cont++;
			}
			
			if(cont == 0)
			{
				aux8[i] = line[i];  //Valor de la Velocidad
			}
			if(cont == 1)
			{
				aux9[pos] = line[i]; // Valor del giro
				pos++;
				salir++;
			}
			if(salir > 10)
			{
				
				i = 100;
			}	
		}
		
		Velocidad = atoi(aux8);
		Giro = atoi(aux9);
		printf("Velocidad: %f, Giro: %f\n",Velocidad,Giro);
//		printf("Linea: %s\n",line);
//	}

}


int YawGiro()
{
//	yawa = yaw*-1;
	yawa = angle;
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
	
/*	if (correccion == 150)
	{
		ajusteyaw = -yawa;
		vi = 0.0;
		//printf("yaw ajustado: %4.4f\n",ajusteyaw);
	}
	YawActual =yawa + vi*360 +ajusteyaw;     */
	YawActual =yawa + vi*360;
	YawAnterior = yawa;
	
//	printf("Yaw: %4.4f Vueltas: %f \n",YawActual,vi);
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
	IDerecha = atoi(aux4)/10;
	IIzquierda = atoi(aux5)/10;
	RpmDerecha = atoi(aux6)*2.0*PI/60.0;
	RpmIzquierda = atoi(aux7)*2.0*PI/60.0;
	
	printf("Baeria: %2.5f TempDerecho: %2.1f TempIzquierdo: %2.1f IDerecha: %3.1f IIzquierda: %3.1f RpmDerecha: %4.1f RpmIzquierda: %4.1f\n",bateria, TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda);
	
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
//	offset[2]/=100.0;
	//offset[2]=0.000498;
	offset[2]=0.00648999999999999;

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
			/*	printf("\n");
				for (int i = 0; i < ppmChannelsNumber; i++)
					printf("%4.f ", channels[i]);*/
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
	// i2c inicializacion ==============================================


    if ((fd2 = open("/dev/i2c-1", O_RDWR)) < 0)
    {
        // Open port for reading and writing
        fprintf(stderr, "Failed to open i2c bus\n");

        return 1;
    }

    /* initialise ADXL345 */

    selectDevice(fd2, HMC5883L_I2C_ADDR, "HMC5883L");

    //writeToDevice(fd2, 0x01, 0);
    writeToDevice(fd2, 0x01, 32);
    writeToDevice(fd2, 0x02, 0);

	// i2c fin ===================================================	
	
	girar = 0.0;
	if(RS232_OpenComport(cport_nr, bdrate, mode))
	{
		printf("Can not open comport\n");
		return(0);
	}
	static const uint8_t outputEnablePin = RPI_GPIO_27;

    if (check_apm())
	{
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
	DatosI2C();
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
		
		Rs232();// REcibe datos desde ROBOTEQ
		Desencriptar(); // Adecua datos de RoboteQ
		
		imuLoop();
		// datos i2c ============
		DatosI2C();
		// datos i2c fin ===============
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
			pwm.setPWMmS(NAVIO_RCOUTPUT_1, salidas[1]); //salida derecha
			pwm.setPWMmS(NAVIO_RCOUTPUT_2, salidas[0]);	//salida izquierda
			//printf("Izquierdo: %2.4f Derecho: %2.4f Izquierdo: %2.4f Derecho: %2.4f\n",salidas[1],salidas[0],salidas[3],salidas[2]);
		
			//printf("Derecho: %4d , %4d, \n",d-5,d-5);
			// FIN SALIDAS -------------
		}
	
		//  MODO AUTOMATICO  ------------------------
		if (channels[2]<2100 && channels[2]> 2080)
		{
			RutaRover();
			//PIDMotor();
			Automatico();		
			pwm.setPWMmS(NAVIO_RCOUTPUT_1, UR); //salida derecha
			pwm.setPWMmS(NAVIO_RCOUTPUT_2, UL);	//salida izquierda

		}
		// FIN AUTOMATICO   -------------------------
	
		//Modos de grabacion y seguridad
	
		// FRENO DE EMERGENCIA Y CREACION DEL ARCHIVO
		if (channels[3]<937 && channels[3]> 917)
		{
			CreateFile();
			vi = 0.0;
		}
	
		// GGRABAR DATOS
		if (channels[3]<1535 && channels[3]> 1515)
		{
			Filerecovery();
			t=(Ts/1000000)+t;
		}
		correccion++;
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
