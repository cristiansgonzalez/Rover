CC = g++
CFLAGS=-c -Wall
NAVIO = ../../Navio
INCLUDES = -I ../..

all:
	sudo  $(CC) principal.cpp $(NAVIO)/rs232.c $(NAVIO)/MPU9250.cpp $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/gpio.cpp -fpermissive -Wall -Wextra -o2 -o principal -lrt -lpthread
clean:
	rm principal
