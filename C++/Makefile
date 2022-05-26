CC = g++
NAVIO = ../../Navio

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)
INCLUDES = -I ../.. -I$(PIGPIO_PATH)

all:
	sudo $(CC) $(INCLUDES) $(LIB) principal.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/gpio.cpp $(NAVIO)/Util.cpp $(NAVIO)/rs232.c -o2 -o principal -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm principal

