# Makefile for BeagleTrainer project
# Jon Escombe
# jone@dresco.co.uk

CC = $(CROSS_COMPILE)gcc

C_FLAGS =  -std=gnu99 -Wall 
C_FLAGS += -I /usr/include/libusb-1.0
C_FLAGS += -I ~/am335x_pru_package/pru_sw/app_loader/include

L_FLAGS =  -lpthread -lrt -lusb-1.0 -lgsl -lgslcblas -lncurses -lprussdrv
L_FLAGS += -L ~/am335x_pru_package/pru_sw/app_loader/lib

SOURCES    = BeagleTrainer.c
DEPS       = BeagleTrainer.h
OBJECTS    = $(SOURCES:.c=.o)
EXECUTABLE = BeagleTrainer

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(L_FLAGS) $(OBJECTS) -o $@

$(OBJECTS): $(DEPS)

.c.o:
	$(CC) -c $(C_FLAGS) $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

	