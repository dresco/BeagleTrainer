# Top level makefile for BeagleTrainer project
# Jon Escombe
# jone@dresco.co.uk

USERSPACE = src
PRU       = pru

.PHONY: all
.PHONY: clean

all:
	$(MAKE) -C $(USERSPACE)
	$(MAKE) -C $(PRU)
	
clean:
	$(MAKE) -C $(USERSPACE) clean
	$(MAKE) -C $(PRU) clean
	