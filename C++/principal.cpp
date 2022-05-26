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
#include "Navio/Ublox.h"
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
//	GPS====================================
std::vector<double> pos_data;
Ublox gps;
float Latitud, Longitud,LatitudRTK, LongitudRTK;
// GPS FIN ======================
short x,y,z;
//================================ Options =====================================

unsigned int samplingRate      = 1;      // 1 microsecond (can be 1,2,4,5,10)
unsigned int ppmInputGpio      = 4;      // PPM input on Navio's 2.54 header
unsigned int ppmSyncLength     = 4000;   // Length of PPM sync pause
unsigned int ppmChannelsNumber = 4;      // Number of channels packed in PPM
unsigned int servoFrequency    = 50;     // Servo control frequency
bool verboseOutputEnabled      = true;   // Output channels values to console

//============================ Objects & data ==================================

float channels[4],pulse0;
double Xm,Ym,yawc;
float buf_datosc[70];
float salidas[5];
float ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, yaw, YawActual,YawAnterior,yawa,dato1;
char bufc[14][250], reply[50];
float Ts = 50000.0,angle;
float deltatiempo,contador,contadorant = 0, TsRoboteQ=0.048;
FILE *pFile,*fd,*fpa;
MPU9250 imu;
AHRS    ahrs;

char Cabezera = 'A',serial[50],aux1[50],aux2[50],aux3[50],aux4[50],aux5[50],aux6[50],aux7[50],aux8[100],aux9[100];
int fd2;
unsigned char buf2[16];
int apuntador,apuntador2,lon=50,j,salir, giro=0,cont,pos,correccion=0;
float girar=0.0, vi,bateria,TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda,ajusteyaw=0.0,UL,UR,Velocidad,Giro;
float WDerecha,WIzquierda,RpmAntDerecha,RpmAntIzquierda,VR,VL,IAntDerecha,IAntIzquierda,bateriaAnt;
struct timeval t_ini, t_fin;
double secs;
int i, n,n3,n4;

int  cport_nr=24;        /* /dev/ttyACM0 roboteq */
int  cport_nr3=16;        /* /dev/ttyACM1 arduino */
int  cport_nr4=25;        /* /dev/ttyACM1 GPS */
int    bdrate=115200;       /* 115200 baud */
int    bdrate3=115200;       /* 115200 baud  9600 GPS*/
int    bdrate4=9600;       /* 115200 baud  9600 RTK*/

unsigned char buf[4096];
unsigned char buf3[4096];
unsigned char buf4[4096];
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
float 	/*Kp=72.0, Ti=0.75, */Td=0.1667, Umin=-100.0, Umax=100.0, errorR=0.0,Referencia = 6, WtR=Referencia, WtL=Referencia, 
		UpR=0.0, error_1R=0.0,YtR=0.0,UiR=0.0,UdR=0.0,Ui1R=0.0,UtR=0.0;
float 	errorL=0.0, UpL=0.0, error_1L=0.0,YtL=0.0,UiL=0.0,UdL=0.0,Ui1L=0.0,UtL=0.0;
float	KpR=0.51,KpL=2.3,TiR=0.75,TiL=0.75,t=0.0,conta=3.0,DeltaReferencia = 3000000/Ts;
// PID ====================================================================

// CONTASTEN PID YAW =====================================================
float yawt=-(103*PI)/180, Uminy=-0.2, Umaxy=0.2, errory= 0.0, error_1y=0.0,Uty=0.0, Upy= 0.0,
	  Kpy = 0.1, Uiy = 0.0,Udy = 0.0,Ui1y = 0.0,Tiy=0.75, V=0.0, L = 0.6, Vr = 0.0, Vi = 0.0,
	  Tdy =0.1667;
char teclado;

//===== FIN PID ===================================================================
int PIDMotor(float YtR,float YtL)
{

/*	if(WtR<Referencia && WtL<Referencia)
	{
		WtR = WtR + Referencia/DeltaReferencia;
		WtL = WtL + Referencia/DeltaReferencia;
//		printf("WtR: %3.3f WtL: %3.3f Delta: %3.3f Referencia: %3.3f\n",WtR,WtL,DeltaReferencia,Referencia);
	}
*/
	errorR=(WtR-YtR);
	UpR=KpR*errorR; //proportional action
	UiR=Ui1R+(KpR*(Ts/1000000.0)/TiR)*errorR; //integral action
	UdR=(KpR*Td/(Ts/1000000.0))*(errorR-error_1R); //differential action
	UtR=UpR+UiR+UdR; //total control action  ESTA ES LA ACCIÓN QUE SE ESCRIBE SOBRE EL MOTOR
	
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
	UiL=Ui1L+(KpL*(Ts/1000000.0)/TiL)*errorL; //integral action
	UdL=(KpL*Td/(Ts/1000000.0))*(errorL-error_1L); //differential action
	UtL=UpL+UiL+UdL; //total control action  ESTA ES LA ACCIÓN QUE SE ESCRIBE SOBRE EL MOTOR
	
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

// PID Yaw ================================================================

int PIDYaw(float yaws)
{
/*	std::cin >> teclado;
	if(teclado =='i')
	{
		Kpy=Kpy+0.01;
	}
	if(teclado == 'k')
	{
		Kpy = Kpy -0.01;
	}
	if(teclado == 'o')
	{
		Kpy = Kpy;
	}
*/
	errory=(yawt-(yaws*PI)/180);

	Upy=Kpy*errory; //proportional action
	Uiy=Ui1y+(Kpy*(Ts/1000000.0)/Tiy)*errory; //integral action
	Udy=(Kpy*Tdy/(Ts/1000000.0))*(errory-error_1y); //differential action
	Uty=Upy+Uiy+Udy; //total control action  ESTA ES LA ACCIÓN QUE SE ESCRIBE SOBRE EL MOTOR
	
	if (Uty<Uminy)
	{
		Uty=Uminy;
	}
	if (Uty>Umaxy)
	{
		Uty=Umaxy;  //constraints
	}
//	UiR=UtR-UpR-UdR; //Anti Reset-Windup
//	printf("Error: %2.5f Kpy: %2.5f Referencia: %2.5f Salida: %2.5f Upy: %2.5f\n", errory,Kpy,yawt,Uty,Upy);

	error_1y=errory;
	Ui1y=Uiy; //store for next sampling time

	if(Uty>0)
	{
		V = (1/0.2)*Uty - 1;
	}
	if(Uty<0)
	{
		V = (1/0.2)*Uty + 1;
	}
	
	Vr = -Uty*L + V;
	Vi =  Uty*L + V;

}
// PID Yaw FIN ============================================================

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
		x = (buf2[0] << 8) | buf2[1];
        y = (buf2[4] << 8) | buf2[5];
		z = (buf2[2] << 8) | buf2[3];
           
		angle = atan2(y, x) * 180 / M_PI;

		//for (int b=0; b<6; ++b)
		//{
			//printf("%02x ",buf2[b]);
		//}
		//printf("\n");
            
//		printf("x=%d, y=%d, z=%d\n", x, y, z);
//		printf("angle = %0.1f\n", angle);
	}
}

//-i2c fin--------

int Rs232() 
{
	//LECTURA ROBOTEQ====================================
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
//	printf("received %i bytes: %s\n", n, buf);
//	printf("received %i bytes: %s\n", n, (char *)buf);
	  
    }
	//FIN LECTURA ROBOTEQ ==================================

	n3 = RS232_PollComport(cport_nr3, buf3, 50);

    if(n3 > 0)
    {
    	buf3[n3] = 0;   /* always put a "null" at the end of a string! */
		for(i=0; i < n3; i++)
      	{
        	if(buf3[i] < 46)  /* replace unreadable control-codes by dots 48*/
        {
        	buf3[i] = '.';
        }
	}
//	printf("received %i bytes: %s\n", n3, buf3);
//	printf("received %i bytes: %s\n", n3, (char *)buf3);
	  
    }
/*

	n4 = RS232_PollComport(cport_nr4, buf4, 100);

    if(n4 > 0)
    {
    	buf4[n] = 0;   /* always put a "null" at the end of a string! */
/*		for(i=0; i < n4; i++)
      	{
        	if(buf4[i] < 46)  /* replace unreadable control-codes by dots 48*/
 /*       {
        	buf4[i] = '.';
        }
	}
//	printf("received %i bytes: %s\n", n4, buf4);
//	printf("received %i bytes: %s\n", n3, (char *)buf3);
	  
    }
*/


#ifdef _WIN32
//    Sleep(10);
#else
//    usleep(100000);  /* sleep for 100 milliSeconds */
#endif
 // RS232_CloseComport(cport_nr)
}

int CreateFile()
{

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
	fputs("T\t",pFile);
	fputs("KpL\t",pFile);
	fputs("Referencia\t",pFile);
	fputs("WtR\t",pFile);
	fputs("WtL\t",pFile);
	fputs("VR\t",pFile);
	fputs("VL\t",pFile);
	fputs("Latitud\t",pFile);
	fputs("Longitud\t",pFile);
	fputs("x\t",pFile);
	fputs("y\t",pFile);
	fputs("Latitudbruto\t",pFile);
	fputs("Longitudbruto\t",pFile);

	fputs("yawt\t",pFile);
	fputs("errory\t",pFile);
	fputs("Uiy\t",pFile);
	fputs("Udy\t",pFile);
	fputs("Upy\t",pFile);
	fputs("Uty\t",pFile);
	fputs("Vr\t",pFile);
	fputs("Vi\t",pFile);
	fputs("IMUmx\t",pFile);
	fputs("IMUmy\t",pFile);
	fputs("IMUmz\n",pFile);

	
	//  ABRIR ARCHIVO DE LECTURA =======================================
	fpa = fopen("ruta.txt","r");

	//Checks if file is empty
	if( fpa == NULL )
	{                       
		return 1;
	}
	// =================================================================
	
	t=0.0;
	vi = 0.0;
	
}
// ALMACENAMIENTO DE LOS DATOS ----------------------
int Guardar()
{

      buf_datosc[6]=x;
      buf_datosc[7]=y;
	  buf_datosc[8]=z;
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
	  
	  buf_datosc[39]=secs;
	  buf_datosc[40]=KpL;
	  buf_datosc[41]=Referencia;
	  buf_datosc[42]=WtR;
	  buf_datosc[43]=WtL;
	  buf_datosc[44]=VR;
	  buf_datosc[45]=VL;

//	  buf_datosc[46]=Latitud;
//	  buf_datosc[47]=Longitud;

	  buf_datosc[48]=Xm;
	  buf_datosc[49]=Ym;
	  
	  buf_datosc[52]=yawt;
	  buf_datosc[53]=errory;
	  buf_datosc[54]=Uiy;
	  buf_datosc[55]=Udy;
	  buf_datosc[56]=Upy;
	  buf_datosc[57]=Uty;
	  buf_datosc[58]=Vr;
	  buf_datosc[59]=Vi;

for (int i = 0; i <= 62; ++i)
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

int Automatico(float EntradaR,float EntradaL)
{
/*	float EntradaL = UtL;
	float EntradaR = UtR;
	float EntradaL = Velocidad;
	float EntradaR = Velocidad;
*/	
	if(EntradaL>0 && EntradaL <=100)
	{
		UL = -0.0550*EntradaL + 6.0;
	}
	if(EntradaL<0 && EntradaL >=-100)
	{
		UL = -0.0580*EntradaL + 6.0;
	}
	if(EntradaL==0)
	{
		UL = 10;
	}
	
	if(EntradaR>0 && EntradaR <=100)
	{
		//UR = -0.0650*(EntradaR*0.5689 + 0.2808) + 7.0;
		UR = -0.0163*EntradaR + 7.0;
	}
	if(EntradaR<0 && EntradaR >=-100)
	{
		UR = -0.0163*EntradaR + 7.0;
	}
	if(EntradaR==0)
	{
		UR = 10;
	}
//	printf("Velocidad: %f UL: %f UR: %f\n",Velocidad,UL,UR);
}

int conversion(float EntradaR,float EntradaL)
{
	if(EntradaL>0 && EntradaL <=10)
	{
		UL = -0.5500*EntradaL + 6.0;
	}
	if(EntradaL<0 && EntradaL >=-10)
	{
		UL = -0.580*EntradaL + 12.0;
	}
	if(EntradaL==0)
	{
		UL = 10;
	}
	
	if(EntradaR>0 && EntradaR <=10)
	{
		//UR = -0.0650*(EntradaR*0.5689 + 0.2808) + 7.0;
		UR = -0.1750*EntradaR	 + 7.0;
		//UR = -0.0650*EntradaR + 7.0;
	}
	if(EntradaR<0 && EntradaR >=-10)
	{
		UR = -0.680*EntradaR + 11.0;
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
//		printf("Velocidad: %f, Giro: %f\n",Velocidad,Giro);
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
	
	YawActual =yawa + vi*360;
	YawAnterior = yawa;
	
//	printf("Yaw: %4.4f Vueltas: %f \n",YawActual,vi);
}

int gps2utm(float Latitud,float Longitud,char lat_)
{

    /*!
     * Transformación de las coordenadas geográficas a UTM
     */
    /// Sobre la geometría del delipsoide WGS84
//    double a = 6378137.0;
//    double b = 6356752.3142;

    double a = 6378388.0;
    double b = 6356911.946130;
    
// Sobre la geometria del elipsoide
// excentricidad = e1; segunda excentricidad = e2
 
    double e1, e2;
 
    e1 = sqrt (pow(a,2) - pow(b, 2)) / a;
    e2 = sqrt (pow(a,2) - pow(b, 2)) / b;
 
// Radio polar de curvatura y aplanamiento
// radio polar de curvatura = c; aplanamiento = alpha
 
    double c, alpha;
 
    c = (pow(a,2))/b;
 
    alpha = (a -b) / a;
 
    double long_gd_rad, lat_gd_rad;

    long_gd_rad = (Longitud * PI)/180;
    lat_gd_rad = (Latitud * PI) / 180;
// Determinacion del Huso
 
    double huso_dec;
    int huso;
    huso_dec = Longitud/6 + 31;
 
    huso = int(huso_dec);
 
// Obtencion del meridiano central del uso = lambda0
 
    double lambda0, delta_lambda;
 
    lambda0 = (huso * 6.0 - 183.0)*(PI/180.0);
 
// Determinacion de la distancia angular que existe entre la longitud del punto (long_gd_rad) y
// el meridiano central del huso (lamda0)
 
    delta_lambda = long_gd_rad - lambda0;
 
// Ecuaciones de Coticchia-Surace para el Problema Directo (Paso de Geograficas a UTM)
// Calculo de Parametros
 
    double A, xi, eta, nu, zeta, A1, A2, J2, J4, J6, alpha2, beta, gamma, B_phi;
 
    A = cos(lat_gd_rad) * sin(delta_lambda); 
    xi = 0.5 * log((1+A)/(1-A)); 
    eta = atan(tan(lat_gd_rad)/cos(delta_lambda)) - lat_gd_rad; 
    nu = (c*0.9996)/sqrt((1 + e2*e2*cos(lat_gd_rad)*cos(lat_gd_rad))); 
    zeta = (e2*e2/2)*(xi*xi)*(cos(lat_gd_rad)*cos(lat_gd_rad)); 
    A1 = sin(2.0*lat_gd_rad); 
    A2 = A1 * (cos(lat_gd_rad)*cos(lat_gd_rad)); 
    J2 = lat_gd_rad + A1/2.0; 
    J4 = (3*J2 + A2)/4; 
    J6 = (5*J4 + A2 * (cos(lat_gd_rad)*cos(lat_gd_rad)))/3; 
    alpha2 = (3.0/4.0)*(e2*e2); 
    beta = (5.0/3.0)*(alpha2*alpha2); 
    gamma = (35.0/27.0)*(pow(alpha2,3)); 
    B_phi = 0.9996 * c * (lat_gd_rad - alpha2 * J2 + beta * J4 - gamma * J6); 
    Xm = xi*nu*(1+zeta/3.0)+500000.0; 
    Ym = eta*nu*(1+zeta)+B_phi;

    if (lat_ == 'S' || lat_ == 's')
 	{
 		Ym += 10000000.0;
	}
        
//   printf("X = %f Y = %f\n",Xm,Ym);
}


int DesencriptarGPS()
{
	apuntador=999;
	int cambio = 0;
	cambio = 0;
	float LongitudAnt, LatitudAnt;

	for(int i=0;i<lon;i++)
	{
		aux1[i] = ' ';
		aux2[i] = ' ';
		aux3[i] = ' ';
		aux4[i] = ' ';
		serial[i]=' ';
	}

	for(int i=0;i<200;i++)
	{
		//if(buf3[i]==',' && buf3[i+1]=='A' && buf3[i+2]==',' && buf3[i+13]==',' && buf3[i+14]=='N' && buf3[i+15]==',' && buf3[i+27]==',' && buf3[i+28]=='W' && buf3[i+29]==',')
		if(buf3[i]=='.' && buf3[i+1]=='A' && buf3[i+2]=='.' && buf3[i+13]=='.' && buf3[i+14]=='N' && buf3[i+15]=='.' && buf3[i+27]=='.' && buf3[i+28]=='W' && buf3[i+29]=='.')
		{
			cambio = 1;
			apuntador = i+4;
			i = 200;
		}
	}

	for(int i=0;i<11;i++)
	{
		if(buf3[apuntador]=='.' && buf3[apuntador+1]=='N')
		{
			apuntador = apuntador + 3;
			i=200;
		}
		else
		{
			aux1[i]=buf3[apuntador];
		}
		apuntador++;
	}

	for(int i=0;i<11;i++)
	{
		if(buf3[apuntador]=='.' && buf3[apuntador+1]=='W')
		{
			i=200;
		}
		else
		{
			aux2[i]=buf3[apuntador];
		}
		apuntador++;
	}

	if(cambio==1)
	{
/*		aux2[0]='7';
		aux2[1]='5';
*///		aux2[3]='1';
//		aux1[0]='4';
		aux3[0]=aux1[0];
		aux4[0]=aux2[0];
		aux4[1]=aux2[1];
		Latitud = ((atof(aux1) - atof(aux3)*100)/60.0) + atof(aux3);
		Longitud = (((atof(aux2) - atof(aux4)*100)/60.0) + atof(aux4))*-1;

		if(Longitud>-70.0 || Longitud<-80.0 || Longitud==0.0)
		{
			Longitud=LongitudAnt;
		}
		if(Latitud<3.0 || Latitud>5.0 || Latitud==0.0)
		{
			Latitud=LatitudAnt;
		}
//		printf("\nMuestra: %d Latitud: %f Longitud: %f Latituddd: %s\n\n", correccion, Latitud, Longitud,aux2);
//		printf("X = %f Y = %f\n",Xm,Ym);
		LongitudAnt=Longitud;
		LatitudAnt=Latitud;
	}
	buf_datosc[46]=Latitud;
	buf_datosc[47]=Longitud;
	buf_datosc[50]=atof(aux1);
	buf_datosc[51]=atof(aux2);
//	printf("longitud = %f\n",buf_datosc[50]);
}

int DesencriptarRTK()
{
	apuntador=999;
	int cambio = 0;
	cambio = 0;
	float LongitudRTKAnt, LatitudRTKAnt;

	for(int i=0;i<lon;i++)
	{
		aux1[i] = ' ';
		aux2[i] = ' ';
		aux3[i] = ' ';
		aux4[i] = ' ';
		serial[i]=' ';
	}

	for(int i=0;i<200;i++)
	{
		//if(buf3[i]==',' && buf3[i+1]=='A' && buf3[i+2]==',' && buf3[i+13]==',' && buf3[i+14]=='N' && buf3[i+15]==',' && buf3[i+27]==',' && buf3[i+28]=='W' && buf3[i+29]==',')
		if(buf4[i]=='.' && buf4[i+1]=='A' && buf4[i+2]=='.' && buf4[i+15]=='.' && buf4[i+16]=='N' && buf4[i+17]=='.' && buf4[i+31]=='.' && buf4[i+32]=='W' && buf4[i+33]=='.')
		{
			cambio = 1;
			apuntador = i+4;
			i = 200;
		}
	}

	for(int i=0;i<14;i++)
	{
		if(buf4[apuntador]=='.' && buf4[apuntador+1]=='N')
		{
			apuntador = apuntador + 3;
			i=200;
		}
		else
		{
			aux1[i]=buf4[apuntador];
		}
		apuntador++;
	}

	for(int i=0;i<14;i++)
	{
		if(buf4[apuntador]=='.' && buf4[apuntador+1]=='W')
		{
			i=200;
		}
		else
		{
			aux2[i]=buf4[apuntador];
		}
		apuntador++;
	}

	if(cambio==1)
	{
/*		aux2[0]='7';
		aux2[1]='5';
		aux1[0]='4';
*/		aux3[0]=aux1[0];
		aux4[0]=aux2[0];
		aux4[1]=aux2[1];
		LatitudRTK = ((atof(aux1) - atof(aux3)*100)/60.0) + atof(aux3);
		LongitudRTK = ((atof(aux2) - atof(aux4)*100)/60.0) + atof(aux4);
/*		if(LongitudRTK<70.0 || LongitudRTK>80.0 || LongitudRTK==0.0)
		{
			LongitudRTK=LongitudAnt;
		}
		if(LatitudRTK<3.0 || LatitudRTK>5.0 || LatitudRTK==0.0)
		{
			LatitudRTK=LatitudAnt;
		}
*///		printf("\nMuestra: %d Latitud: %f Longitud: %f\n\n", correccion, LatitudRTK, LongitudRTK);
		LongitudRTKAnt=LongitudRTK;
		LatitudRTKAnt=LatitudRTK;
	}
//	buf_datosc[46]=LatitudRTK;
//	buf_datosc[47]=LongitudRTK;
}


int DesencriptarRoboteq()
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
	contador = 0.0;
	
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
		aux8[i] = ' ';
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
		if(cont == 7)
		{
			aux8[pos] = serial[i];  //Valor de la contador
			pos++;
		}
		
	}
	//ajuste de los datos del roboteq a datos reales
	
	bateria = atoi(aux1)*0.0024+0.1;
	contador = atoi(aux8);
	TempDerecho = atoi(aux2);
	TempIzquierdo = atoi(aux3);
	IDerecha = atoi(aux4)/10;
	IIzquierda = atoi(aux5)/10;

	deltatiempo = contador - contadorant;

	RpmDerecha = (atoi(aux6)-RpmAntDerecha)*(2*3.1416)/(360*TsRoboteQ*deltatiempo*3.888888);
	RpmIzquierda = (atoi(aux7)-RpmAntIzquierda)*(2*3.1416)/(360*TsRoboteQ*deltatiempo*3.888888);

	if(bateria> 20 || bateria < 3)
	{
		bateria = bateriaAnt;
		IIzquierda =IAntIzquierda;
		IDerecha =IAntDerecha;
		RpmAntDerecha = RpmDerecha;
		RpmAntIzquierda = RpmIzquierda;
	}
	
//	RpmDerecha = (atoi(aux6)-RpmAntDerecha)*(2*3.1416)/((360*Ts*3.888888)/1E6);
//	RpmIzquierda = (atoi(aux7)-RpmAntIzquierda)*(2*3.1416)/((360*Ts*3.888888)/1E6);
	
	RpmAntDerecha = atoi(aux6);
	RpmAntIzquierda = atoi(aux7);
	IAntDerecha = atoi(aux4)/10;
	IAntIzquierda = atoi(aux5)/10;
	bateriaAnt = atoi(aux1)*0.0024+0.1;
	contadorant = contador;
	
//	printf("Baeria: %2.5f TempDerecho: %2.1f TempIzquierdo: %2.1f IDerecha: %3.1f IIzquierda: %3.1f RpmDerecha: %4.1f RpmIzquierda: %4.1f\n",bateria, TempDerecho,TempIzquierdo,IDerecha,IIzquierda,RpmDerecha,RpmIzquierda);
//	printf("contador: %4.4f RpmDerecha: %4.4f RpmIzquierda: %4.4f\n RpmDerecha: %4.4f RpmIzquierda: %4.4f\n",contador,RpmDerecha,RpmIzquierda,RpmAntDerecha,RpmAntIzquierda);
//	printf("Bateria %f\n", bateria);
	
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

	if(RS232_OpenComport(cport_nr3, bdrate3, mode))
	{
		printf("Can not open comport\n");
		return(0);
	}
/*
	if(RS232_OpenComport(cport_nr4, bdrate4, mode))
	{
		printf("Can not open comport\n");
		return(0);
	}
*/
	printf("paso las usb\n");
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
printf("paso el gpio\n");
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
	printf("paso imu loop\n");
	YawActual=0;
	YawAnterior=0;
	DatosI2C();
	printf("paso i2c\n");
	YawGiro();
	printf("paso giro yaw\n");
	CreateFile();
    while(1) 
	{
		gettimeofday(&t_ini, NULL);
		
		//LecturaGPS();		
		gpioSetAlertFunc(ppmInputGpio, ppmOnEdge);
		imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
		//printf("Acc: %+7.3f %+7.3f %+7.3f  \n", ax, ay, az);
		buf_datosc[0]=ax;
		buf_datosc[1]=ay;
		buf_datosc[2]=az;
		buf_datosc[3]=gx;
		buf_datosc[4]=gy;
		buf_datosc[5]=gz;
		buf_datosc[60]=mx;
	  	buf_datosc[61]=my;
	  	buf_datosc[62]=mz;

		//printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
		//printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
		//printf("%4.f %4.d", channels[1], salidas[1]);
		
		Rs232();// Recibe datos desde ROBOTEQ y del GPS
		DesencriptarGPS();     //Adecua datos del GPS
		gps2utm(Latitud,Longitud,'n');
//		DesencriptarRTK();
		DesencriptarRoboteq(); // Adecua datos de RoboteQ
		
		imuLoop();


		yawc=pow(atan( (-my*cos(roll) + mz*sin(roll) ) / (mx*cos(pitch) + my*sin(pitch)*sin(roll)+ mz*sin(pitch)*cos(roll)) ) ,2);
		//printf("Yawc: %f\n", yawc);
		// datos i2c ============
		DatosI2C();	
		// datos i2c fin ===============
		YawGiro();
				
		//  NEUTRO----------------------
		if (channels[2]<930 && channels[2]>920)
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
			//	   DERECHO	-------------
			if (channels[1] < 2085 && channels[1] > 1535)  //ADELANTE
			{

//				salidas[1]=-0.0127*channels[1]+29.8390;	
				salidas[1]=-0.0032*channels[1]+11.8841;	
				salidas[3]=0.1818*channels[1]-279.0909;
				salidas[5]=0.0545*channels[1]-28.7273;   //duty cycle
			}
			if (channels[1] < 1524 && channels[1] > 950)   //ATRAS
			{
				salidas[1]=-0.0126*channels[1]+30.2216;
				salidas[3]=0.1802*channels[1]-274.5946;
				salidas[5]=0.0631*channels[1]-46.1081;   //duty cycle
			}
			if (channels[1] < 950 && channels[1] > 2085 || channels[1] < 1535 && channels[1] > 1524)  ////FUERA DEL RANGO. SE DETIENE
			{
				salidas[1]=10;
				salidas[3]=0;
				salidas[5]=50;  //duty cycle
			}
			// FIN DERECHO ------------
			//  SALIDAS ----------------
			pwm.setPWMmS(NAVIO_RCOUTPUT_1, salidas[1]); //salida derecha
			pwm.setPWMmS(NAVIO_RCOUTPUT_2, salidas[0]);	//salida izquierda
	//		printf("\nDerecha: %2.4f Izquierda: %2.4f Derecha: %2.4f Izquierda: %2.4f\n",salidas[1],salidas[0],salidas[3],salidas[2]);
		
			//printf("Derecho: %4d , %4d, \n",d-5,d-5);
			// FIN SALIDAS -------------
		}

		//  MODO AUTOMATICO  ------------------------
		PIDYaw(angle);
		if (channels[2]<2100 && channels[2]> 2080)
		{
			//RutaRover(); //salidas Velocidad,Velocidad
			PIDMotor(RpmDerecha,RpmIzquierda); //salida UtR,UtL; Entradas Velocidades angulares FUNCIONA
/*			
			if (channels[0] < 2100 && channels[0] > 1600) //ADELANTE
			{
				WtL = 10;
				VL = 100;
			}
			if (channels[0] < 1400 && channels[0] > 900) //ATRAS
			{
				WtL = -5;
				VL = -55;
			}
			if (channels[0] < 900 && channels[0] > 2100 || channels[0] < 1600 && channels[0] > 1400)  //FUERA DEL RANGO. SE DETIENE
			{
				WtL = 0;
				VL = 20;
			}
			//// FIN IZQUIERDO ----------
			//	   DERECHO	-------------
			if (channels[1] < 2100 && channels[1] > 1600)  //ADELANTE
			{
				WtR = 10;
				VR = 100;
			}
			if (channels[1] < 1400 && channels[1] > 900)   //ATRAS
			{
				WtR = -5;
				VR = -100;
			}
			if (channels[1] < 900 && channels[1] > 2100 || channels[1] < 1600 && channels[1] > 1400)  ////FUERA DEL RANGO. SE DETIENE
			{
				WtR = 0;
				VR = 20;
			}
*/
			//Automatico(VR,VL);
			//conversion(UtR,UtL);
			//Automatico(UtR,UtL);  fuciona pasa de porcentaje a salida de los motores
			PIDYaw(angle);
			conversion(Vr,Vi);
			pwm.setPWMmS(NAVIO_RCOUTPUT_1, UR); //salida derecha
			pwm.setPWMmS(NAVIO_RCOUTPUT_2, UL);	//salida izquierda

		}
		// FIN AUTOMATICO   -------------------------
	
		//Modos de grabacion y seguridad
		
		// GGRABAR DATOS

		if (channels[3]<1535 && channels[3]> 1515)
		{
			correccion = 0;
			Guardar();
		}

		// FRENO DE EMERGENCIA Y CREACION DEL ARCHIVO
		if (channels[3]<937 && channels[3]> 917)
		{
			if(correccion>=2)
			{
//				CreateFile();
			}
			correccion=correccion+1;
		}
		
		gettimeofday(&t_fin, NULL); // final del contador, para una iteracion.

		secs = timeval_diff(&t_fin, &t_ini);
		
		while((secs*1000000.0)<Ts)
		{
			gettimeofday(&t_fin,NULL);
			secs = timeval_diff(&t_fin, &t_ini);
		}
//	printf("Tiempo de muestreo %.4fs\n",secs);
	
    }
	
	//sleep(1);
}





