CC = g++
NAVIO = ../../Navio

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)
INCLUDES = -I ../.. -I$(PIGPIO_PATH)

all:
	$(CC) $(INCLUDES) $(LIB) principal.cpp $(NAVIO)/MPU9250.cpp $(NAVIO)/rs232.c $(NAVIO)/PCA9685.cpp $(NAVIO)/I2Cdev.cpp $(NAVIO)/gpio.cpp $(NAVIO)/Util.cpp -o principal -lrt -lpthread -lpigpio || $(MAKE) pigpio

clean:
	rm principal

pigpio:
	@echo ""
	@echo "Note that pigpio library is required."
	@echo "Warning! Use with caution. Consider to remove after trying this example out."
	@echo "To install:"
	@echo "sudo apt-get update"
	@echo "sudo apt-get install pigpio"
	@echo ""
	@echo "Visit http://abyz.co.uk/rpi/pigpio/ for details."
	@echo ""