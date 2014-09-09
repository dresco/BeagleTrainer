PRU_ASM = ~/am335x_pru_package/pru_sw/utils/pasm
DTC = ~/am335x_pru_package/pru_sw/utils/dtc

all:
	@echo ">> Generating PRU binary"
	$(PRU_ASM) -b BeagleTrainer.p
	
	@echo ">> Compiling device tree overlay"
	$(DTC) -O dtb -o BeagleTrainer-00A0.dtbo -b 0 -@ BeagleTrainer.dts
	
clean:
	rm -rf BeagleTrainer.bin BeagleTrainer-00A0.dtbo
    